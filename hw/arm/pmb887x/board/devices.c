#include "hw/arm/pmb887x/board/devices.h"

#include "hw/arm/pmb887x/flash-blk.h"
#include "hw/arm/pmb887x/board/board.h"
#include "hw/arm/pmb887x/board/gpio.h"
#include "hw/arm/pmb887x/board/memory.h"
#include "hw/arm/pmb887x/utils/regexp.h"
#include "hw/arm/pmb887x/utils/toml.h"

#include "hw/arm/pmb887x/utils/tomlc17.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/qdev-core.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "qom/object.h"
#include "hw/qdev-properties.h"
#include "hw/i2c/i2c.h"
#include "qapi/error.h"

// --- TEMP instrumentation: detect whether firmware ever accesses flash-tail (0xA8000000) ---
static uint8_t *g_flash_tail_buf;
static uint32_t g_flash_tail_size;
static uint64_t g_flash_tail_reads;
static uint64_t flash_tail_dbg_read(void *opaque, hwaddr addr, unsigned size) {
	uint64_t v = 0;
	if (addr + size <= g_flash_tail_size)
		memcpy(&v, g_flash_tail_buf + addr, size);
	if (g_flash_tail_reads < 128)
		fprintf(stderr, "[FLASH-TAIL] READ  off=0x%05x sz=%u val=0x%llx\n",
			(uint32_t)addr, size, (unsigned long long)v);
	g_flash_tail_reads++;
	return v;
}
static void flash_tail_dbg_write(void *opaque, hwaddr addr, uint64_t val, unsigned size) {
	if (addr + size <= g_flash_tail_size)
		memcpy(g_flash_tail_buf + addr, &val, size);
	fprintf(stderr, "[FLASH-TAIL] WRITE off=0x%05x sz=%u val=0x%llx\n",
		(uint32_t)addr, size, (unsigned long long)val);
}
static const MemoryRegionOps flash_tail_dbg_ops = {
	.read = flash_tail_dbg_read,
	.write = flash_tail_dbg_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
};

typedef enum pmb887x_dev_prop_type_t pmb887x_dev_prop_type_t;
typedef enum pmb887x_dev_bus_type_t pmb887x_dev_bus_type_t;
typedef struct pmb887x_dev_prop_t pmb887x_dev_prop_t;
typedef struct pmb887x_dev_t pmb887x_dev_t;

enum pmb887x_dev_bus_type_t {
	DEV_BUS_NONE,
	DEV_BUS_I2C,
	DEV_BUS_SSI,
	DEV_BUS_EBU,
};

enum pmb887x_dev_prop_type_t {
	DEV_PROP_INT,
	DEV_PROP_UINT,
	DEV_PROP_STRING,
	DEV_PROP_BOOL,
};

struct pmb887x_dev_prop_t {
	char name[32];
	pmb887x_dev_prop_type_t type;
	bool required;
};

struct pmb887x_dev_t {
	char name[32];
	pmb887x_dev_prop_t props[32];
};

static int global_cs_index = 0;

static pmb887x_dev_t devices_meta[] = {
	// LCD
	{
		.name = "jbt6k71",
		.props = {
			{ "width", DEV_PROP_UINT, true },
			{ "height", DEV_PROP_UINT, true },
			{ "rotation", DEV_PROP_UINT, false },
			{ "flip_horizontal", DEV_PROP_BOOL, false },
			{ "flip_vertical", DEV_PROP_BOOL, false },
			{ "bgr_filter", DEV_PROP_BOOL, false },
		},
	},
	{
		.name = "ssd1286",
		.props = {
			{ "width", DEV_PROP_UINT, true },
			{ "height", DEV_PROP_UINT, true },
			{ "rotation", DEV_PROP_UINT, false },
			{ "flip_horizontal", DEV_PROP_BOOL, false },
			{ "flip_vertical", DEV_PROP_BOOL, false },
			{ "bgr_filter", DEV_PROP_BOOL, false },
		},
	},
	{
		.name = "hx5050a",
		.props = {
			{ "width", DEV_PROP_UINT, true },
			{ "height", DEV_PROP_UINT, true },
			{ "rotation", DEV_PROP_UINT, false },
			{ "flip_horizontal", DEV_PROP_BOOL, false },
			{ "flip_vertical", DEV_PROP_BOOL, false },
			{ "bgr_filter", DEV_PROP_BOOL, false },
		},
	},
	{
		.name = "l2f50333t",
		.props = {
			{ "width", DEV_PROP_UINT, true },
			{ "height", DEV_PROP_UINT, true },
			{ "rotation", DEV_PROP_UINT, false },
			{ "flip_horizontal", DEV_PROP_BOOL, false },
			{ "flip_vertical", DEV_PROP_BOOL, false },
			{ "bgr_filter", DEV_PROP_BOOL, false },
		},
	},
	{
		.name = "ili9320",
		.props = {
			{ "width", DEV_PROP_UINT, true },
			{ "height", DEV_PROP_UINT, true },
			{ "rotation", DEV_PROP_UINT, false },
			{ "flip_horizontal", DEV_PROP_BOOL, false },
			{ "flip_vertical", DEV_PROP_BOOL, false },
			{ "bgr_filter", DEV_PROP_BOOL, false },
		},
	},

	// PMIC
	{
		.name = "d1094xx",
		.props = {
			{ "revision", DEV_PROP_UINT, true },
		},
	},
	{
		.name = "pmb6812",
		.props = {},
	},

	// FM Radio
	{
		.name = "tea5761uk",
		.props = {},
	},

	// Gimmick
	{
		.name = "s1d13705",
		.props = {},
	},

	// Audio Codec
	{
		.name = "b00b10b",
		.props = {},
	},

	// NOR FLASH
	{
		.name = "cfi-flash",
		.props = {},
	},

	// SDRAM
	{
		.name = "sdram",
		.props = {},
	},

	// Flash tail region (e.g. NVRAM/caldata mapped past the NOR, as-flashed)
	{
		.name = "flash-tail",
		.props = {},
	},
};

static const pmb887x_dev_t *dev_get_metadata(const char *name) {
	for (size_t i = 0; i < ARRAY_SIZE(devices_meta); i++) {
		if (strcasecmp(devices_meta[i].name, name) == 0)
			return &devices_meta[i];
	}
	hw_error("Unknown device: %s", name);
	return NULL;
}

static Object *bus_find_by_id(const char *name) {
	DeviceState *dev = qdev_find_recursive(sysbus_get_default(), name);
	if (!dev)
		hw_error("Bus not found: %s", name);

	if (object_property_find(OBJECT(dev), "bus")) {
		Object *bus_obj = object_property_get_link(OBJECT(dev), "bus", &error_fatal);
		g_assert(bus_obj);
		return bus_obj;
	}

	return OBJECT(dev);
}

static pmb887x_dev_bus_type_t bus_get_type(Object *dev) {
	const char *typename = object_get_typename(dev);
	if (strcmp(typename, "i2c-bus") == 0)
		return DEV_BUS_I2C;
	if (strcmp(typename, "SSI") == 0)
		return DEV_BUS_SSI;
	if (strcmp(typename, "pmb887x-ebu") == 0)
		return DEV_BUS_EBU;
	hw_error("Unknown bus type: %s", typename);
	return DEV_BUS_NONE;
}

static void device_init_props_from_config(DeviceState *dev, const pmb887x_dev_t *meta, toml_datum_t table) {
	for (int i = 0; i < ARRAY_SIZE(meta->props); i++) {
		const pmb887x_dev_prop_t *prop = &meta->props[i];
		if (!prop->required && toml_seek(table, prop->name).type == TOML_UNKNOWN)
			continue;

		switch (prop->type) {
			case DEV_PROP_INT: {
				int value = toml_table_get_int32(table, prop->name, -1, true);
				qdev_prop_set_uint32(dev, prop->name, value);
				break;
			}
			case DEV_PROP_UINT: {
				uint32_t value = toml_table_get_uint32(table, prop->name, 0, true);
				qdev_prop_set_uint32(dev, prop->name, value);
				break;
			}
			case DEV_PROP_STRING: {
				const char *value = toml_table_get_string(table, prop->name, NULL, true);
				qdev_prop_set_string(dev, prop->name, value);
				break;
			}
			case DEV_PROP_BOOL: {
				bool value = toml_table_get_bool(table, prop->name, false, true);
				object_property_set_bool(OBJECT(dev), prop->name, value, &error_fatal);
				break;
			}
		}
	}
}

static void device_init_gpios_from_config(DeviceState *dev, toml_datum_t table) {
	toml_datum_t gpio_in_tab = toml_table_get(table, TOML_TABLE, "in", false);
	toml_datum_t gpio_out_tab = toml_table_get(table, TOML_TABLE, "out", false);

	if (gpio_in_tab.type != TOML_UNKNOWN) {
		for (size_t i = 0; i < gpio_in_tab.u.tab.size; i++) {
			char dev_pin_name[64];
			sprintf(dev_pin_name, "%s:%s", dev->id, gpio_in_tab.u.tab.key[i]);
			const char *per_pin_name = toml_table_get_string(gpio_in_tab, gpio_in_tab.u.tab.key[i], NULL, true);
			pmb887x_gpio_connect(per_pin_name, dev_pin_name);
		}
	}

	if (gpio_out_tab.type != TOML_UNKNOWN) {
		for (size_t i = 0; i < gpio_out_tab.u.tab.size; i++) {
			char dev_pin_name[64];
			sprintf(dev_pin_name, "%s:%s", dev->id, gpio_out_tab.u.tab.key[i]);
			const char *per_pin_name = toml_table_get_string(gpio_out_tab, gpio_out_tab.u.tab.key[i], NULL, true);
			pmb887x_gpio_connect(dev_pin_name, per_pin_name);
		}
	}
}

static DeviceState *device_create_from_config(DeviceState *ebuc, const char *id, toml_datum_t table) {
	pmb887x_board_t *board = pmb887x_board();
	const char *type = toml_table_get_string(table, "type", NULL, true);
	const char *bus_id = toml_table_get_string(table, "bus", NULL, true);
	const pmb887x_dev_t *meta = dev_get_metadata(type);

	DeviceState *dev = NULL;
	Object *bus = strcmp(bus_id, "EBU") == 0 ? OBJECT(ebuc) : bus_find_by_id(bus_id);
	pmb887x_dev_bus_type_t bus_type = bus_get_type(bus);

	switch (bus_type) {
		case DEV_BUS_I2C: {
			uint32_t addr = toml_table_get_uint32(table, "addr", 0, true);
			dev = DEVICE(i2c_slave_new(type, addr));
			dev->id = g_strdup(id);
			device_init_props_from_config(dev, meta, table);
			i2c_slave_realize_and_unref(I2C_SLAVE(dev), I2C_BUS(bus), &error_fatal);
			device_init_gpios_from_config(dev, table);
			break;
		}
		case DEV_BUS_SSI: {
			dev = qdev_new(type);
			qdev_prop_set_uint8(DEVICE(dev), "cs", global_cs_index++);
			dev->id = g_strdup(id);
			device_init_props_from_config(dev, meta, table);
			qdev_realize_and_unref(dev, BUS(bus), &error_fatal);
			device_init_gpios_from_config(dev, table);
			break;
		}
		case DEV_BUS_EBU: {
			if (strcmp(type, "sdram") == 0) {
				uint32_t cs = toml_table_get_uint32(table, "ebu.cs", 0, true);
				uint32_t size = toml_table_get_uint32(table, "ram.size", 0, true);
				pmb887x_board_ebu_connect(DEVICE(bus), cs, pmb887x_board_create_sdram(id, size));
			} else if (strcmp(type, "cfi-flash") == 0) {
				uint32_t cs = toml_table_get_uint32(table, "ebu.cs", 0, true);
				uint32_t vid = toml_table_get_uint32(table, "flash.vid", 0, true);
				uint32_t pid = toml_table_get_uint32(table, "flash.pid", 0, true);
				uint32_t bank_size;
				MemoryRegion *flash_mr = pmb887x_board_create_nor_flash(id, vid, pid, board->flash_offset, &bank_size);
				pmb887x_board_ebu_connect(DEVICE(bus), cs, flash_mr);
				board->flash_offset += bank_size;

				// A flash chip larger than a single 64MB EBU window is wired across
				// consecutive even chip-selects (e.g. 128MB NOR: CS_n = low 64MB,
				// CS_n+2 = high 64MB). Alias the remaining halves onto those CSes.
				static const uint32_t EBU_WINDOW = 0x4000000; // 64MB
				for (uint32_t off = EBU_WINDOW, extra_cs = cs + 2; off < bank_size; off += EBU_WINDOW, extra_cs += 2) {
					MemoryRegion *alias = g_new(MemoryRegion, 1);
					char alias_name[64];
					sprintf(alias_name, "%s.cs%u", id, extra_cs);
					memory_region_init_alias(alias, NULL, alias_name, flash_mr, off, MIN(EBU_WINDOW, bank_size - off));
					pmb887x_board_ebu_connect(DEVICE(bus), extra_cs, alias);
				}
			} else if (strcmp(type, "flash-tail") == 0) {
				// A region of the flash image that lives past the NOR main array and
				// is not wired through an EBU chip-select (the firmware accesses it at
				// a fixed physical address). Map it directly into system memory,
				// preloaded as-flashed from the tail of the fullflash image. Used for
				// the KE970 NVRAM/caldata bank at 0xA8000000.
				uint32_t address = toml_table_get_uint32(table, "address", 0, true);
				uint32_t size = toml_table_get_uint32(table, "size", 0, true);

				pmb887x_flash_blk_t *flash_blk =
					pmb887x_flash_blk_self(qdev_find_recursive(sysbus_get_default(), "FULLFLASH"));
				int64_t total = pmb887x_flash_blk_size(flash_blk);
				int64_t tail = total - board->flash_offset;
				if (tail < 0)
					tail = 0;

				MemoryRegion *region = g_new(MemoryRegion, 1);
				g_flash_tail_size = size;
				g_flash_tail_buf = g_malloc(size);
				memset(g_flash_tail_buf, 0xFF, size);
				if (tail > 0)
					pmb887x_flash_blk_pread(flash_blk, board->flash_offset, MIN((int64_t)size, tail), g_flash_tail_buf);
				memory_region_init_io(region, NULL, &flash_tail_dbg_ops, NULL, id, size);
				memory_region_add_subregion(get_system_memory(), address, region);

				// Consume the remaining image bytes so the fullflash size check passes.
				board->flash_offset += tail;
			} else {
				error_report("Invalid dev type: %s", type);
				exit(EXIT_FAILURE);
			}
			break;
		}
		default:
			error_report("Unknown bus type: %s", bus_id);
			exit(EXIT_FAILURE);
	}

	return dev;
}

void pmb887x_board_init_devices(DeviceState *ebuc) {
	pmb887x_board_t *board = pmb887x_board();

	toml_datum_t value = toml_table_get(board->config, TOML_TABLE, "peripheral", false);
	if (value.type == TOML_UNKNOWN)
		return;

	for (int i = 0; i < value.u.tab.size; i++) {
		const char *id = value.u.tab.key[i];
		device_create_from_config(ebuc, id, toml_table_get(value, TOML_TABLE, id, true));
	}

	DeviceState *flash_blk = qdev_find_recursive(sysbus_get_default(), "FULLFLASH");
	g_assert(flash_blk != NULL);

	int64_t total_flash_size = pmb887x_flash_blk_size(pmb887x_flash_blk_self(flash_blk));
	if (board->flash_offset != total_flash_size)
		hw_error("Invalid fullflash size=0x%08"PRIX64". Please, specify fullflash with size=0x%08X", total_flash_size, board->flash_offset);

	sysbus_realize_and_unref(SYS_BUS_DEVICE(ebuc), &error_fatal);
}
