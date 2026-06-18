#pragma once

#include "qemu/osdep.h"
#include "system/memory.h"
#include "hw/sysbus.h"

#include "hw/arm/pmb887x/flash.h"
#include "hw/arm/pmb887x/flash-blk.h"

#define PMB887X_FLASH_CFI_ADDR	0x10

typedef struct pmb887x_flash_t pmb887x_flash_t;
typedef struct pmb887x_flash_part_t pmb887x_flash_part_t;
typedef struct pmb887x_flash_buffer_t pmb887x_flash_buffer_t;
typedef struct pmb887x_flash_block_t pmb887x_flash_block_t;
typedef struct pmb887x_flash_cmd_ops_t pmb887x_flash_cmd_ops_t;

struct pmb887x_flash_buffer_t {
	uint32_t offset;
	uint32_t value;
	uint8_t size;
};

struct pmb887x_flash_block_t {
	uint32_t offset;
	uint32_t size;
	bool locked;
};

struct pmb887x_flash_part_t {
	uint16_t n;
	uint32_t size;
	uint32_t offset;

	uint8_t wcycle;
	uint8_t cmd;
	uint32_t cmd_addr;
	uint8_t status;

	// Once a partition is accessed via the LG NVRAM read command (0x94) it is the
	// FFS/NVRAM data partition, which is never executed from. The firmware reads
	// it one byte at a time through a 0x70/0x94/0xFF command sequence; toggling
	// the rom_device romd flag per access (TLB/flatview rebuilds) makes the FFS
	// scan glacial. We instead pin such a partition to io-mode and serve its
	// array reads from the command handler, so no per-byte region transactions.
	bool nvram_mode;

	// True while a program/erase/lock operation has been issued and its status
	// has not yet been dismissed (by a Read-Array 0xFF or a Clear-Status 0x50).
	// On the real part, reads return Status only while the WSM is active / its
	// result is pending; a bare Read-Status (0x70) with no operation in flight
	// does NOT make a plain array read return status. This matters for the FFS
	// data partition, which is pinned to io-mode (nvram_mode): the LG read
	// primitive issues 0x70 as part of its preamble (0x94 -> 0xFF -> 0x70 ->
	// array read), and the subsequent header read must still see array bytes.
	bool op_pending;

	void *storage;

	uint32_t buffer_size;
	uint32_t buffer_index;
	uint32_t buffer_base;	// fill address of the first word in the active buffer
	pmb887x_flash_buffer_t *buffer;
	const pmb887x_flash_cfg_part_t *cfg;

	uint32_t blocks_n;
	pmb887x_flash_block_t *blocks;

	MemoryRegion mem;
	pmb887x_flash_t *flash;

	const pmb887x_flash_cmd_ops_t *cmd_ops;
};

struct pmb887x_flash_t {
	SysBusDevice parent_obj;
	DeviceState *dev;
	MemoryRegion mmio;

	pmb887x_flash_blk_t *blk;
	const pmb887x_flash_cfg_t *cfg;

	char *name;

	uint16_t vid;
	uint16_t pid;

	uint16_t hex_otp0_lock;
	char *hex_otp0_data;

	uint16_t hex_otp1_lock;
	char *hex_otp1_data;

	uint32_t size;
	uint32_t offset;

	uint16_t *otp0_data;
	uint16_t *otp1_data;

	uint32_t parts_n;

	const pmb887x_flash_cmd_ops_t *cmd_ops;
};

struct pmb887x_flash_cmd_ops_t {
	const char *name;
	uint32_t (*cmd_read)(pmb887x_flash_part_t *p, hwaddr offset, unsigned size);
	bool (*cmd_write)(pmb887x_flash_part_t *p, hwaddr offset, uint64_t value, unsigned size);
};

/* Core helpers shared with the vendor command handlers (defined in flash.c). */
void pmb887x_flash_reset(pmb887x_flash_part_t *p);
void pmb887x_flash_data_write(pmb887x_flash_part_t *p, uint32_t offset, uint32_t value, unsigned size);
uint32_t pmb887x_flash_find_sector_size(pmb887x_flash_part_t *p, uint32_t offset);
pmb887x_flash_block_t *pmb887x_flash_part_find_block(pmb887x_flash_part_t *p, uint32_t offset);
void pmb887x_flash_trace_part(pmb887x_flash_part_t *p, const char *format, ...) G_GNUC_PRINTF(2, 3);
void pmb887x_flash_error_part(pmb887x_flash_part_t *p, const char *format, ...) G_GNUC_PRINTF(2, 3);

/* Available vendor command sets. */
extern const pmb887x_flash_cmd_ops_t pmb887x_flash_numonyx_cmd_ops;

/* Select the command set for a given JEDEC id (defined in flash-numonyx.c). */
const pmb887x_flash_cmd_ops_t *pmb887x_flash_cmd_ops_for(uint16_t vid, uint16_t pid);
