/*
 * NOR FLASH (Intel/ST CFI)
 * */
#define PMB887X_TRACE_ID		FLASH
#define PMB887X_TRACE_PREFIX	"pmb887x-flash"

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "system/memory.h"
#include "cpu.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "hw/qdev-properties.h"
#include "hw/block/block.h"

#include "hw/arm/pmb887x/trace.h"
#include "hw/arm/pmb887x/flash.h"
#include "hw/arm/pmb887x/flash-internal.h"
#include "hw/arm/pmb887x/flash-blk.h"

#define TYPE_PMB887X_FLASH	"pmb887x-flash"
#define PMB887X_FLASH(obj)	OBJECT_CHECK(pmb887x_flash_t, (obj), TYPE_PMB887X_FLASH)

static void flash_trace(pmb887x_flash_t *flash, const char *format, ...) G_GNUC_PRINTF(2, 3);
static void flash_error(pmb887x_flash_t *flash, const char *format, ...) G_GNUC_PRINTF(2, 3);

void pmb887x_flash_reset(pmb887x_flash_part_t *p) {
	pmb887x_flash_trace_part(p, "back to read array mode");
	p->cmd = 0;
	p->wcycle = 0;
	// Read-Array (0xFF) dismisses any pending operation status: subsequent plain
	// reads return array data, not status (see op_pending in flash-internal.h).
	p->op_pending = false;
	// NVRAM/FFS partitions stay pinned to io-mode (see nvram_mode); array reads
	// are served by the command handler, so we skip the costly romd transaction.
	if (!p->nvram_mode)
		memory_region_rom_device_set_romd(&p->mem, true);
}

pmb887x_flash_block_t *pmb887x_flash_part_find_block(pmb887x_flash_part_t *p, uint32_t offset) {
	offset -= p->offset;
	for (uint32_t i = 0; i < p->blocks_n; i++) {
		pmb887x_flash_block_t *blk = &p->blocks[i];
		if (offset >= blk->offset && offset < blk->offset + blk->size)
			return blk;
	}
	pmb887x_flash_error_part(p, "[data] Unknown addr %08X", p->flash->offset + p->offset + offset);
	exit(1);
}

uint32_t pmb887x_flash_find_sector_size(pmb887x_flash_part_t *p, uint32_t offset) {
	offset -= p->offset;

	for (int i = 0; i < p->cfg->erase_regions_cnt; i++) {
		const pmb887x_flash_erase_region_t *region = &p->cfg->erase_regions[i];
		if (offset >= region->offset && offset < region->offset + region->size)
			return region->sector;
	}

	pmb887x_flash_error_part(p, "[data] Unknown sector size for addr %08X", p->flash->offset + p->offset + offset);
	exit(1);
}

void pmb887x_flash_data_write(pmb887x_flash_part_t *p, uint32_t offset, uint32_t value, unsigned size) {
	uint8_t *data = p->storage;

	if (offset < p->offset || (offset + size) > p->offset + p->size) {
		pmb887x_flash_error_part(p, "[data] Unknown write addr %08X [part %08X-%08X]", offset, p->offset, p->offset + p->size - 1);
		exit(1);
	}

	offset -= p->offset;

	switch (size) {
		case 1:
			data[offset] &= value & 0xFF;
			break;

		case 2:
			data[offset] &= value & 0xFF;
			data[offset + 1] &= (value >> 8) & 0xFF;
			break;

		case 4:
			data[offset] &= value & 0xFF;
			data[offset + 1] &= (value >> 8) & 0xFF;
			data[offset + 2] &= (value >> 16) & 0xFF;
			data[offset + 3] &= (value >> 24) & 0xFF;
			break;

		default:
			pmb887x_flash_error_part(p, "[data] Unknown write size %d", size);
			exit(1);
	}

	if (pmb887x_flash_blk_is_rw(p->flash->blk)) {
		int ret = pmb887x_flash_blk_pwrite(p->flash->blk, p->flash->offset + p->offset + offset, size, p->storage + offset);
		if (ret < 0) {
			pmb887x_flash_error_part(p, "Can't read to flash file: %d, %s", ret, strerror(ret));
			exit(1);
		}
	}
}

// Command-mode reads/writes are delegated to the chip's command-set handler
// (pmb887x_flash_cmd_ops_t). Read-Array reads never reach flash_io_read: the
// region is in romd mode and served directly from storage.
static uint64_t flash_io_read(void *opaque, hwaddr part_offset, unsigned size) {
	pmb887x_flash_part_t *p = (pmb887x_flash_part_t *) opaque;
	return p->cmd_ops->cmd_read(p, p->offset + part_offset, size);
}

static void flash_io_write(void *opaque, hwaddr part_offset, uint64_t value, unsigned size) {
	pmb887x_flash_part_t *p = opaque;
	hwaddr offset = p->offset + part_offset;

	if (!p->cmd_ops->cmd_write(p, offset, value, size)) {
		pmb887x_flash_error_part(p, "not implemented %d cycle for command %02X [addr: %08"PRIX64", value: %08"PRIX64"]", p->wcycle, p->cmd, p->flash->offset + offset, value);
		exit(1);
	}
}

static uint64_t flash_io_unaligned_read(void *opaque, hwaddr offset, unsigned size) {
	// Native read
	if (size == 2 && (offset & 0x1) == 0)
		return flash_io_read(opaque, offset, 2);
	
	// Unaligned read
	uint32_t value = 0;
	if ((offset & 0x1) == 0) {
		for (int i = 0; i < size; i += 2)
			value |= flash_io_read(opaque, offset + i, 2) << (i * 8);
	} else {
		value |= (flash_io_read(opaque, offset - 1, 2) >> 8) & 0xFF;
		for (int i = 1; i < size; i += 2)
			value |= flash_io_read(opaque, offset + i + 1, 2) << (i * 8);
	}
	
	value &= ((1 << (size * 8)) - 1);
	// pmb887x_flash_part_t *p = (pmb887x_flash_part_t *) opaque;
	// flash_trace_part(p, "unaligned %08"PRIX64"[%d] = %08"PRIX64"", p->flash->offset + offset, size, value);
	return value;
}

static const MemoryRegionOps io_ops = {
	.read			= flash_io_unaligned_read,
	.write			= flash_io_write,
	.endianness		= DEVICE_NATIVE_ENDIAN
};

static bool fill_data_from_hex(uint8_t *dst, uint32_t max_size, const char *src_hex) {
	uint32_t len = strlen(src_hex);
	
	if (!len)
		return true;
	
	if (len % 2 != 0)
		return false;
	
	if (len > max_size * 2)
		return false;
	
	for (int i = 0; i < len; i += 2) {
		uint8_t tmp[2];
		
		for (int j = 0; j < 2; j++) {
			char c = src_hex[i + j];
			if (c >= 'A' && c <= 'F') {
				tmp[j] = (c - 'A') + 0x0A;
			} else if (c >= 'a' && c <= 'f') {
				tmp[j] = (c - 'a') + 0x0A;
			} else if (c >= '0' && c <= '9') {
				tmp[j] = c - '0';
			} else {
				// Invalid hex
				return false;
			}
		}
		
		dst[i / 2] = (tmp[0] << 4) | tmp[1];
	}
	
	return true;
}

static void flash_init_part(pmb887x_flash_t *flash, const pmb887x_flash_cfg_part_t *part_cfg) {
	pmb887x_flash_part_t *p = g_new0(pmb887x_flash_part_t, 1);
	p->n = flash->parts_n++;
	p->flash = flash;
	p->offset = part_cfg->offset;
	p->size = part_cfg->size;
	p->cfg = part_cfg;
	p->cmd_ops = flash->cmd_ops;
	p->status = 0x80; // SR7 ready: power-up default per datasheet (0x0080)
	
	char *name = g_strdup_printf("pmb887x-flash[%s][%d]", p->flash->name, p->n);
	memory_region_init_rom_device(&p->mem, OBJECT(p->flash->dev), &io_ops, p, name, p->size, NULL);
	memory_region_rom_device_set_romd(&p->mem, true);
	memory_region_add_subregion(&flash->mmio, p->offset, &p->mem);
	g_free(name);
	
	p->storage = memory_region_get_ram_ptr(&p->mem);

	pmb887x_flash_trace_part(p, "hw partition 0x%08X ... 0x%08X", p->flash->offset + p->offset, p->flash->offset + p->offset + p->size - 1);
	
	int ret = pmb887x_flash_blk_pread(p->flash->blk, flash->offset + p->offset, p->size, p->storage);
	if (ret < 0) {
		flash_error(p->flash, "failed to read the initial flash content [offset=%08X, size=%08X]", p->flash->offset + p->offset, p->size);
		exit(1);
	}

	p->blocks_n = 0;
	for (uint32_t i = 0; i < p->cfg->erase_regions_cnt; i++)
		p->blocks_n += p->cfg->erase_regions[i].size / p->cfg->erase_regions[i].sector;
	
	p->blocks = g_new0(pmb887x_flash_block_t, p->blocks_n);
	
	uint32_t block_id = 0;
	uint32_t block_offset = 0;
	
	for (uint32_t i = 0; i < p->cfg->erase_regions_cnt; i++) {
		const pmb887x_flash_erase_region_t *region = &p->cfg->erase_regions[i];
		uint32_t sectors = region->size / region->sector;
		for (uint32_t j = 0; j < sectors; j++) {
			p->blocks[block_id].offset = block_offset;
			p->blocks[block_id].size = region->sector;
			p->blocks[block_id].locked = true;
			block_offset += region->sector;
			block_id++;
		}
	}
}

static void flash_realize(DeviceState *dev, Error **errp) {
	pmb887x_flash_t *flash = PMB887X_FLASH(dev);
	flash->dev = dev;
	
	const pmb887x_flash_cfg_t *cfg = pmb887x_flash_find(flash->vid, flash->pid);
	if (!cfg) {
		flash_error(flash, "unimplemented %04X:%04X", flash->vid, flash->pid);
		exit(1);
	}
	
	flash->cfg = cfg;
	flash->size = cfg->size;
	flash->cmd_ops = pmb887x_flash_cmd_ops_for(flash->vid, flash->pid);

	flash_trace(flash, "FLASH %04X:%04X, 0x%08X ... 0x%08X [cmds: %s]", flash->vid, flash->pid, flash->offset, flash->offset + flash->size - 1, flash->cmd_ops->name);
	
	char *mmio_name = g_strdup_printf("pmb887x-flash[%s]", flash->name);
	memory_region_init(&flash->mmio, OBJECT(flash->dev), mmio_name, flash->size);
	g_free(mmio_name);
	
	// OTP0
	if (cfg->otp0_size > 0) {
		flash->otp0_data = g_new(uint16_t, cfg->otp0_size / 2);
		memset(flash->otp0_data, 0xFF, cfg->otp0_size);
		flash->otp0_data[0] = 0x0002;
		
		if (!fill_data_from_hex((uint8_t *) flash->otp0_data, cfg->otp0_size, flash->hex_otp0_data)) {
			flash_error(flash, "Invalid OTP0 hex data: %s [max_size=%d, len=%"PRIu64"]", flash->hex_otp0_data, cfg->otp0_size, strlen(flash->hex_otp0_data) / 2);
			exit(1);
		}
	}
	
	// OTP1
	if (cfg->otp1_size > 0) {
		flash->otp1_data = g_new(uint16_t, cfg->otp1_size / 2);
		memset(flash->otp1_data, 0xFF, cfg->otp1_size);
		flash->otp1_data[0] = 0xFFFF;
		
		if (!fill_data_from_hex((uint8_t *) flash->otp1_data, cfg->otp1_size, flash->hex_otp1_data)) {
			flash_error(flash, "Invalid OTP1 hex data: %s [max_size=%d, len=%"PRIu64"]", flash->hex_otp1_data, cfg->otp1_size, strlen(flash->hex_otp1_data) / 2);
			exit(1);
		}
	}
	
	// Init hw partitions
	for (size_t i = 0; i < cfg->parts_count; i++)
		flash_init_part(flash, &cfg->parts[i]);
	
	sysbus_init_mmio(SYS_BUS_DEVICE(flash->dev), &flash->mmio);
}

static void flash_error(pmb887x_flash_t *flash, const char *format, ...) {
	g_autoptr(GString) s = g_string_new("");
	
	va_list args;
	va_start(args, format);
	g_string_append_vprintf(s, format, args);
	va_end(args);
	
	error_report("[%s] %s %s", PMB887X_TRACE_PREFIX, flash->name, s->str);
}

void pmb887x_flash_error_part(pmb887x_flash_part_t *p, const char *format, ...) {
	g_autoptr(GString) s = g_string_new("");

	va_list args;
	va_start(args, format);
	g_string_append_vprintf(s, format, args);
	va_end(args);

	error_report("[%s] %s <%d> %s", PMB887X_TRACE_PREFIX, p->flash->name, p->n, s->str);
}

void pmb887x_flash_trace_part(pmb887x_flash_part_t *p, const char *format, ...) {
	if (!pmb887x_trace_log_enabled(PMB887X_TRACE_FLASH))
		return;
	
	g_autoptr(GString) s = g_string_new("");
	
	va_list args;
	va_start(args, format);
	g_string_append_vprintf(s, format, args);
	va_end(args);
	
	qemu_log_mask(LOG_TRACE, "[%s] %s <%d> %s\n", PMB887X_TRACE_PREFIX, p->flash->name, p->n, s->str);
}

static void flash_trace(pmb887x_flash_t *flash, const char *format, ...) {
	if (!pmb887x_trace_log_enabled(PMB887X_TRACE_FLASH))
		return;
	
	g_autoptr(GString) s = g_string_new("");
	
	va_list args;
	va_start(args, format);
	g_string_append_vprintf(s, format, args);
	va_end(args);
	
	qemu_log_mask(LOG_TRACE, "[%s] %s %s\n", PMB887X_TRACE_PREFIX, flash->name, s->str);
}

static const Property flash_properties[] = {
	DEFINE_PROP_LINK("blk", struct pmb887x_flash_t, blk, "pmb887x-flash-blk", struct pmb887x_flash_blk_t *),
	
	DEFINE_PROP_STRING("name", pmb887x_flash_t, name),
	DEFINE_PROP_UINT16("vid", pmb887x_flash_t, vid, 0),
	DEFINE_PROP_UINT16("pid", pmb887x_flash_t, pid, 0),
	DEFINE_PROP_UINT32("offset", pmb887x_flash_t, offset, 0),
	DEFINE_PROP_UINT32("size", pmb887x_flash_t, size, 0),
	
	/* OTP0 Initial Data */
	DEFINE_PROP_STRING("otp0-data", pmb887x_flash_t, hex_otp0_data),
	
	/* OTP1 Initial Data */
	DEFINE_PROP_STRING("otp1-data", pmb887x_flash_t, hex_otp1_data),
};

static void flash_class_init(ObjectClass *klass, const void *data) {
	DeviceClass *dc = DEVICE_CLASS(klass);
	device_class_set_props(dc, flash_properties);
	dc->realize = flash_realize;
	set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
}

static const TypeInfo flash_info = {
    .name          	= TYPE_PMB887X_FLASH,
    .parent        	= TYPE_SYS_BUS_DEVICE,
    .instance_size 	= sizeof(pmb887x_flash_t),
    .class_init    	= flash_class_init,
};

static void flash_register_types(void) {
	type_register_static(&flash_info);
}
type_init(flash_register_types)
