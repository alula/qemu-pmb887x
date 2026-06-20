/*
 * Intel / Numonyx (M18 StrataFlash) command set.
 *
 * This is the command interpreter for the Intel-style CFI command set used by
 * the Numonyx M18 parts (and their Intel/Micron equivalents): Read Array (FFh),
 * Read Status (70h), Clear Status (50h), Read Device ID (90h) / CFI Query (98h),
 * Word/Buffered/BEFP program, Block Erase, Suspend/Resume, Block Lock and OTP
 * programming. See the M18 datasheet "Device Command Codes" table.
 *
 * Anything NOT in that datasheet is grouped under "VENDOR/FIRMWARE QUIRKS"
 * below: the LG/APOXI firmware on the KE970 relies on two non-standard
 * behaviours of this part.
 *
 * The generic device plumbing (MMIO, storage, CFI/OTP config, init) lives in
 * flash.c; this file only decodes commands via pmb887x_flash_cmd_ops_t.
 */
#include "qemu/osdep.h"
#include "system/memory.h"
#include "cpu.h"

#include "hw/arm/pmb887x/flash-internal.h"
#include "hw/arm/pmb887x/flash-blk.h"

/*
 * VENDOR/FIRMWARE QUIRK #1 - undocumented command 0x94.
 *
 * The LG config-NVRAM read primitive (firmware sub_A25C8194, copied to TCM)
 * primes the array with command 0x94 before reading config bytes:
 *     0x70 (read status, erase-suspend aware) -> [0xB0/0x50] -> 0x94 ->
 *     LDRH data -> 0xFF (read array) -> [0xD0]
 * 0x94 is NOT in the M18 datasheet (real Read Array is 0xFF). On this part it
 * behaves as a read-array for the config store, which is plain NOR data in
 * partition 3 (there is NO NAND). We model it by returning stored array bytes.
 * (A romd switch here would not take effect on the same translation block as
 * the following load, so it must be served via the io handler.)
 */
#define NUMONYX_CMD_LG_NVRAM_READ	0x94

static uint32_t numonyx_cmd_read(pmb887x_flash_part_t *p, hwaddr offset, unsigned size) {
	const pmb887x_flash_cfg_t *cfg = p->flash->cfg;
	uint16_t index;
	uint32_t value = 0;
	uint32_t _dbg_abs = p->flash->offset + offset;
	int _dbg = getenv("KE970_RD_LOG") && _dbg_abs >= 0x3000000 && (_dbg_abs & 0x3FFFF) < 0x18;
	switch (p->cmd) {
		case 0x90:	// Read Device Information
		case 0x98:	// CFI Query
			index = (offset >> 1) & 0xFFF;

			// CFI
			if (index >= PMB887X_FLASH_CFI_ADDR && index < PMB887X_FLASH_CFI_ADDR + cfg->cfi_size) {
				value = cfg->cfi[index - PMB887X_FLASH_CFI_ADDR];
				pmb887x_flash_trace_part(p, "CFI %02X: %02X", index, value);
			}
			// PRI
			else if (index >= cfg->pri_addr && index < cfg->pri_addr + cfg->pri_size) {
				value = cfg->pri[index - cfg->pri_addr];
				pmb887x_flash_trace_part(p, "PRI %02X: %02X", index - cfg->pri_addr, value);
			}
			// OTP0
			else if (index >= cfg->otp0_addr && index < cfg->otp0_addr + (cfg->otp0_size / 2)) {
				value = p->flash->otp0_data[index - cfg->otp0_addr];
				pmb887x_flash_trace_part(p, "OTP0 %02X: %04X", index - cfg->otp0_addr, value);
			}
			// OTP1
			else if (index >= cfg->otp1_addr && index < cfg->otp1_addr + (cfg->otp1_size / 2)) {
				value = p->flash->otp1_data[index - cfg->otp1_addr];
				pmb887x_flash_trace_part(p, "OTP1 %02X: %04X", index - cfg->otp1_addr, value);
			}
			// Other info
			else {
				switch (index) {
					case 0x00:
						value = p->flash->vid;
						pmb887x_flash_trace_part(p, "vendor id: %04X", value);
						break;

					case 0x01:
						value = p->flash->pid;
						pmb887x_flash_trace_part(p, "device id: %04X", value);
						break;

					case 0x02: {
						pmb887x_flash_block_t *blk = pmb887x_flash_part_find_block(p, offset);
						value = blk->locked ? cfg->lock : 0;
						pmb887x_flash_trace_part(p, "lock status: %02X", value);
						break;
					}

					case 0x05:
						value = cfg->cr;
						pmb887x_flash_trace_part(p, "configuration register: %02X", value);
						break;

					case 0x06:
						value = cfg->ehcr;
						pmb887x_flash_trace_part(p, "enhanced configuration register: %02X", value);
						break;

					default:
						value = 0xFFFF;
						pmb887x_flash_error_part(p, "%08"PRIX64": read unknown cfi index 0x%02X", offset, index);
						break;
				}
			}
			break;

		case 0x20:	// Erase
		case 0x70:	// Status
		case 0xe8:	// buffered program
		case 0xE9:	// buffered program
		case 0x41:	// program word
		case 0x40:	// program word
		case 0x10:	// program word
			// Returning Status here for cmd 0x70/program/erase makes the firmware
			// misread FFS block headers on the io-pinned (nvram_mode) partition: the
			// LG NVRAM read primitive (RAM, flash_lg_nvram_read_half_0x94) does
			// 0xB0(suspend,->cmd=0x70) -> poll status -> 0x94 -> read -> 0xD0, and
			// the GC's bulk flash_read_words() then does RAW multi-word reads that
			// assume the partition still serves array data. A plain status-follows-cmd
			// model returns status for those header reads -> the GC/allocator
			// sub_A26A5308 sees garbage, finds no free block, and fgc:1 deadlocks.
			//
			// Disambiguation: a NOR status poll is always directed at the command
			// address (p->cmd_addr); FFS header/data reads target other block
			// addresses. Serve ARRAY data for any read whose offset differs from
			// cmd_addr. Genuine status polls (offset == cmd_addr, e.g. the 0xB0
			// suspend poll and erase/program completion polling) still get status, so
			// erase-suspend RWW loops are unaffected. This applies to every partition
			// of the FFS flash: the store spans several partitions (0xA3.., 0xA4..,
			// ...) but only the partition first hit by 0x94 is nvram-pinned; the GC
			// erases/relocates across all of them, and the header reads it performs
			// on the *non-pinned* partitions while an erase/program is in flight must
			// likewise see array bytes (else the GC mis-reads a just-erased block's
			// header as status 0x80 and re-erases forever).
			if (offset != p->cmd_addr) {
				uint8_t *data = (uint8_t *)p->storage + (offset - p->offset);
				switch (size) {
					case 1:  value = data[0]; break;
					case 2:  value = data[0] | (data[1] << 8); break;
					default: value = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24); break;
				}
			} else {
				value = p->status;
			}
			// pmb887x_flash_trace_part(p, "%08"PRIX64": status 0x%02X", offset, value);
			break;

		case 0x00:	// read array (served here while the partition is pinned to io-mode)
		case NUMONYX_CMD_LG_NVRAM_READ: {	// VENDOR QUIRK #1: return stored array bytes
			uint8_t *data = p->storage + (offset - p->offset);
			switch (size) {
				case 1:	value = data[0]; break;
				case 2:	value = data[0] | (data[1] << 8); break;
				default: value = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24); break;
			}
			break;
		}

		default:
			pmb887x_flash_error_part(p, "not implemented read for command %02X [addr: %08"PRIX64"]", p->cmd, offset);
			exit(1);
	}

	if (_dbg)
		fprintf(stderr, "[KE970_RD] abs=%08X blkoff=%X size=%d cmd=%02X nvram=%d -> %08X\n",
			_dbg_abs, (uint32_t)(_dbg_abs & 0x3FFFF), size, p->cmd, p->nvram_mode, value);

	return value;
}

static bool numonyx_cmd_write(pmb887x_flash_part_t *p, hwaddr offset, uint64_t value, unsigned size) {
	bool valid_cmd = false;

	if (p->wcycle == 0) {
		// A pinned NVRAM partition already stays in io-mode; avoid the per-command
		// romd transaction that otherwise makes the FFS byte-scan glacial.
		if (!p->nvram_mode)
			memory_region_rom_device_set_romd(&p->mem, false);

		valid_cmd = true;
		p->cmd_addr = offset;

		/*
		 * VENDOR/FIRMWARE QUIRK #2 - bit 2 set on setup commands.
		 *
		 * The LG/APOXI firmware issues some setup commands with bit 2 set: 0x44 =
		 * 0x40|4 and 0x45 = 0x41|4 (program word), 0x14 = 0x10|4 (program word),
		 * 0x24 = 0x20|4 (block erase), 0x64 = 0x60|4 (block lock). No standard M18
		 * setup command uses bit 2, so the chip ignores it. These are handled as
		 * explicit case labels alongside their base commands below (rather than a
		 * blanket pre-decode rewrite, which risked mangling unrelated commands).
		 * (The undocumented 0x94 = 0x90|4 is NOT a stray-bit 0x90 - it returns
		 * array data, not device-id/CFI - so it is its own command, QUIRK #1.)
		 */
		switch (value) {
			case 0xFF:
				pmb887x_flash_reset(p);
				break;

			case 0x00:
			case 0xAA:
			case 0x55:
			case 0xF0:
				pmb887x_flash_trace_part(p, "cmd AMD probe (%02"PRIX64")", value);
				pmb887x_flash_reset(p);
				break;

			case 0x70:
				pmb887x_flash_trace_part(p, "cmd read status (%02"PRIX64")", value);
				p->cmd = value;
				break;

			case 0x90:
				pmb887x_flash_trace_part(p, "cmd read devid (%02"PRIX64")", value);
				p->cmd = value;
				break;

			case NUMONYX_CMD_LG_NVRAM_READ:	// VENDOR QUIRK #1: see top of file
				pmb887x_flash_trace_part(p, "cmd LG NVRAM read 0x94 (%02"PRIX64")", value);
				p->cmd = value;
				// This is the FFS/NVRAM data partition: pin it to io-mode so the
				// firmware's per-byte command-sequence reads don't thrash romd.
				p->nvram_mode = true;
				if (getenv("KE970_SCAN")) {
					static long _cnt = 0;
					if ((p->flash->offset + p->offset) == 0x3000000 && (_cnt++ % 200000) == 0) {
						int nfree = 0, nerasecnt = 0, nother = 0;
						for (int b = 0; b < 0xff; b++) {
							uint32_t o = (uint32_t)b * 0x40000;
							if (o + 4 > p->size) break;
							uint8_t *st = (uint8_t *)p->storage;
							uint32_t w0 = st[o] | (st[o+1]<<8) | (st[o+2]<<16) | (st[o+3]<<24);
							uint32_t hi = (w0 >> 16) & 0xf000;
							int has_ec = (w0 & 0x300000) != 0;
							if (hi == 0x3000) nfree++;
							else if (has_ec) nerasecnt++;
							else nother++;
						}
						fprintf(stderr, "[KE970_SCAN] at nvram-pin: free(0x3xxx)=%d erasecnt(0x300000)=%d other=%d\n",
							nfree, nerasecnt, nother);
					}
				}
				break;

			case 0x98:
				pmb887x_flash_trace_part(p, "cmd read cfi (%02"PRIX64")", value);
				p->cmd = value;
				break;

			case 0x50:
				// Clear Status Register: single-cycle command. Reset the status bits to
				// the default (ready) and stay in read-status mode; does not start a
				// multi-cycle sequence.
				pmb887x_flash_trace_part(p, "cmd clear status (%02"PRIX64")", value);
				p->status = 0x80;
				p->cmd = 0x70;
				p->op_pending = false;
				break;

			case 0x45:	// 0x41|4 (QUIRK #2)
			case 0x44:	// 0x40|4 (QUIRK #2)
			case 0x14:	// 0x10|4 (QUIRK #2)
				value &= ~0x04u;	// normalise so p->cmd is the base program code
				/* fallthrough */
			case 0x41:
			case 0x40:
			case 0x10:
				pmb887x_flash_trace_part(p, "cmd program word (%02"PRIX64")", value);
				p->cmd = value;
				p->op_pending = true;
				p->wcycle++;
				break;

			case 0xE9:
			case 0xE8:
				pmb887x_flash_trace_part(p, "cmd buffered program (%02"PRIX64")", value);
				p->cmd = value;
				p->op_pending = true;
				p->wcycle++;
				p->status |= 0x80;
				break;

			case 0x80:
				pmb887x_flash_trace_part(p, "cmd buffered EFP (%02"PRIX64")", value);
				p->cmd = value;
				p->op_pending = true;
				p->wcycle++;
				break;

			case 0x24:	// 0x20|4 (QUIRK #2)
				value = 0x20;	// normalise so p->cmd is the base erase code
				/* fallthrough */
			case 0x20:
				pmb887x_flash_trace_part(p, "cmd block erase (%02"PRIX64")", value);
				p->cmd = value;
				p->op_pending = true;
				p->wcycle++;
				p->status |= 0x80;
				break;

			case 0xB0:
				pmb887x_flash_trace_part(p, "cmd suspend (%02"PRIX64")", value);
				p->status |= 0x80 | 0x40 | 0x04;
				p->op_pending = true;
				p->cmd = 0x70;
				break;

			case 0xD0:
				pmb887x_flash_trace_part(p, "cmd resume (%02"PRIX64")", value);
				p->status |= 0x80;
				p->status &= ~(0x40 | 0x04);
				p->cmd = 0;
				break;

			case 0x64:	// 0x60|4 (QUIRK #2)
				value = 0x60;	// normalise so p->cmd is the base lock code
				/* fallthrough */
			case 0x60:
				pmb887x_flash_trace_part(p, "cmd block lock or read configuration (%02"PRIX64")", value);
				p->cmd = value;
				p->op_pending = true;
				p->wcycle++;
				break;


			case 0xC0:
				pmb887x_flash_trace_part(p, "cmd protection program (%02"PRIX64")", value);
				p->cmd = value;
				p->op_pending = true;
				p->wcycle++;
				break;

			default:
				pmb887x_flash_trace_part(p, "cmd unknown (%02"PRIX64") at %08"PRIX64"", value, p->flash->offset + offset);
				pmb887x_flash_reset(p);
				// pmb887x_flash_error_part(p, "cmd unknown (%02"PRIX64") at %08"PRIX64"", value, p->flash->offset + offset);
				// exit(1);
				break;
		}
	} else if (p->wcycle == 1) {
		switch (p->cmd) {
			case 0x70:	// read status
			case 0x90:	// read devid
			case 0x98:	// read cfi
				if (value == 0xFF) {
					valid_cmd = true;
					pmb887x_flash_reset(p);
				}
				break;

			case 0x60:	// lock or configuration
				if (value == 0xFF) {
					valid_cmd = true;
					pmb887x_flash_reset(p);
				} else if (value == 0x03) {
					valid_cmd = true;
					pmb887x_flash_trace_part(p, "program read configuration register (%02"PRIX64")", p->flash->offset + offset);
					pmb887x_flash_reset(p);
				} else if (value == 0x04) {
					valid_cmd = true;
					pmb887x_flash_trace_part(p, "program read enhanced configuration register (%02"PRIX64")", p->flash->offset + offset);
					pmb887x_flash_reset(p);
				} else if (value == 0x01) {
					valid_cmd = true;

					pmb887x_flash_trace_part(p, "lock block %08"PRIX64"", p->flash->offset + offset);
					pmb887x_flash_block_t *blk = pmb887x_flash_part_find_block(p, offset);
					blk->locked = true;

					p->wcycle = 0;
					p->status |= 0x80;
				} else if (value == 0xD0) {
					valid_cmd = true;

					pmb887x_flash_trace_part(p, "unlock block %08"PRIX64"", p->flash->offset + offset);
					pmb887x_flash_block_t *blk = pmb887x_flash_part_find_block(p, offset);
					blk->locked = false;

					p->wcycle = 0;
					p->status |= 0x80;
				} else if (value == 0x2F) {
					valid_cmd = true;
					pmb887x_flash_trace_part(p, "lock-down block %08"PRIX64"", p->flash->offset + offset);
					p->wcycle = 0;
					p->status |= 0x80;
				}
				break;

			case 0x20:	// erase
				if (value == 0xD0) {
					uint32_t sector_size = pmb887x_flash_find_sector_size(p, offset);
					uint32_t mask = ~(sector_size - 1);
					uint32_t base = (p->cmd_addr & mask);

					pmb887x_flash_trace_part(p, "confirm erase block %08X...%08X (sector: %08X)", p->flash->offset + base, p->flash->offset + base + sector_size - 1, sector_size);

					if ((offset & mask) != (p->cmd_addr & mask)) {
						pmb887x_flash_error_part(p, "erase sector mismatch: %08"PRIX64" != %08X", p->flash->offset + offset, p->flash->offset + p->cmd_addr);
						exit(1);
					}

					// fill sector with 0xFF's
					uint32_t erase_offset = (base - p->offset);
					if (getenv("KE970_FFS_LOG")) {
						uint32_t eabs = p->flash->offset + p->offset + erase_offset;
						if (eabs >= 0x3000000 && eabs < 0x8000000)
							fprintf(stderr, "[KE970_FFS] ERASE block abs=%08X size=%X nvram_mode=%d\n",
								eabs, sector_size, p->nvram_mode);
					}
					memset(p->storage + erase_offset, 0xFF, sector_size);

					if (pmb887x_flash_blk_is_rw(p->flash->blk)) {
						int ret = pmb887x_flash_blk_pwrite(p->flash->blk, p->flash->offset + p->offset + erase_offset, sector_size, p->storage + erase_offset);
						if (ret < 0) {
							pmb887x_flash_error_part(p, "Can't read to flash file: %d, %s", ret, strerror(ret));
							exit(1);
						}
					}

					valid_cmd = true;
					p->wcycle = 0;
					p->status |= 0x80;
					// Resume pinned NVRAM array-read after the erase completes (see
					// the program-word completion above for rationale).
					if (p->nvram_mode)
						p->cmd = NUMONYX_CMD_LG_NVRAM_READ;
				}
				break;

			case 0xE9:	// buffered program
			case 0xE8:	// buffered program
				valid_cmd = true;
				p->buffer_size = (value & 0xFFFF) + 1;
				p->buffer_index = 0;
				p->buffer = g_new0(pmb887x_flash_buffer_t, p->buffer_size);

				pmb887x_flash_trace_part(p, "buffered program %d words", p->buffer_size);

				p->wcycle++;
				break;

			case 0x10:	// program word
			case 0x40:	// program word
			case 0x41:	// program word
				valid_cmd = true;
				pmb887x_flash_trace_part(p, "program single word [%d]: %08"PRIX64" to %08"PRIX64"", size, value, p->flash->offset + offset);
				pmb887x_flash_data_write(p, offset, value, size);
				p->wcycle = 0;
				p->status |= 0x80;
				// On the io-pinned NVRAM/FFS partition the firmware does not always
				// re-issue Read-Array/0x94 before reading back block headers; on the
				// real LG part the pinned NVRAM-read mode resumes once the transient
				// program completes. Mirror that: restore array-read so subsequent
				// header reads return array data, not the (now stale) program status.
				// (Genuine in-progress status polls keep cmd 0x70 / op_pending and
				// are unaffected; this only fires at single-word program completion.)
				if (p->nvram_mode)
					p->cmd = NUMONYX_CMD_LG_NVRAM_READ;
				break;

			case 0xC0: {	// program OTP / lock register
				valid_cmd = true;
				const pmb887x_flash_cfg_t *otp_cfg = p->flash->cfg;
				uint16_t otp_index = (offset >> 1) & 0xFFF;
				// OTP is one-time-programmable: programming only clears bits (AND).
				if (otp_index >= otp_cfg->otp0_addr && otp_index < otp_cfg->otp0_addr + (otp_cfg->otp0_size / 2))
					p->flash->otp0_data[otp_index - otp_cfg->otp0_addr] &= value;
				else if (otp_index >= otp_cfg->otp1_addr && otp_index < otp_cfg->otp1_addr + (otp_cfg->otp1_size / 2))
					p->flash->otp1_data[otp_index - otp_cfg->otp1_addr] &= value;
				pmb887x_flash_trace_part(p, "program OTP/lock reg [%03X]: %04"PRIX64"", otp_index, value & 0xFFFF);
				p->wcycle = 0;
				p->status |= 0x80;
				break;
			}
		}
	} else if (p->wcycle == 2) {
		switch (p->cmd) {
			case 0xE9:	// buffered program
			case 0xE8:	// buffered program
			{
				uint32_t sector_size = pmb887x_flash_find_sector_size(p, offset);
				uint32_t mask = ~(sector_size - 1);

				valid_cmd = true;

				pmb887x_flash_trace_part(p, "program word [%d]: %08"PRIX64" to %08"PRIX64"", size, value, p->flash->offset + offset);

				/*
				 * Per the M18 datasheet (9.6.2 Buffered Programming): "User-data is
				 * programmed into the flash array at the address issued when filling
				 * the write buffer." The fill address - NOT the setup/word-count
				 * address (p->cmd_addr) - is authoritative. The LG/APOXI FFS
				 * provisioning legitimately issues the setup command and the data
				 * stream against different blocks, which the part honours (data lands
				 * at the fill address). The only true constraint is that the write
				 * buffer cannot cross a block boundary, so every fill word must share
				 * the block of the first fill word. Validate against that, not the
				 * stale setup address.
				 */
				if (p->buffer_index == 0)
					p->buffer_base = offset;

				if ((offset & mask) != (p->buffer_base & mask)) {
					pmb887x_flash_error_part(p, "program buffer crosses block boundary: %08"PRIX64" != %08X", offset, p->buffer_base);
				//	exit(1);
				}

				if (size != 2 && size != 4) {
					pmb887x_flash_error_part(p, "invalid write size: %d", size);
					exit(1);
				}

				for (int i = 0; i < size; i += 2) {
					int overwrite_buffer_index = -1;
					for (int j = 0; j < p->buffer_size; j++) {
						if (p->buffer[j].offset == offset + i && p->buffer[j].size == 2) {
							overwrite_buffer_index = j;
							break;
						}
					}

					if (overwrite_buffer_index >= 0) {
						p->buffer[overwrite_buffer_index].value = (value >> (i * 8)) & 0xFFFF;
					} else {
						p->buffer[p->buffer_index].offset = offset + i;
						p->buffer[p->buffer_index].value = (value >> (i * 8)) & 0xFFFF;
						p->buffer[p->buffer_index].size = 2;
						p->buffer_index++;
					}

					if (p->buffer_index == p->buffer_size)
						break;
				}

				if (p->buffer_index == p->buffer_size) {
					pmb887x_flash_trace_part(p, "buffered program finished");
					p->wcycle++;
				}
				break;
			}
		}
	} else if (p->wcycle == 3) {
		switch (p->cmd) {
			case 0xE9:	// buffered program
			case 0xE8:	// buffered program
				if (value == 0xD0) {
					for (uint32_t i = 0; i < p->buffer_size; i++)
						pmb887x_flash_data_write(p, p->buffer[i].offset, p->buffer[i].value, p->buffer[i].size);

					g_free(p->buffer);
					p->buffer = NULL;

					valid_cmd = true;
					pmb887x_flash_trace_part(p, "confirm buffered program");
					p->wcycle = 0;
					p->status |= 0x80;
					// Resume pinned NVRAM array-read after buffered program completes.
					if (p->nvram_mode)
						p->cmd = NUMONYX_CMD_LG_NVRAM_READ;
				}
				break;
		}
	}

	return valid_cmd;
}

const pmb887x_flash_cmd_ops_t pmb887x_flash_numonyx_cmd_ops = {
	.name		= "numonyx",
	.cmd_read	= numonyx_cmd_read,
	.cmd_write	= numonyx_cmd_write,
};

const pmb887x_flash_cmd_ops_t *pmb887x_flash_cmd_ops_for(uint16_t vid, uint16_t pid) {
	// Only the Intel/Numonyx command set is modelled today. The KE970 uses a
	// Numonyx M18 (vid 0x0089). Add AMD/Spansion etc. here when needed.
	(void)vid;
	(void)pid;
	return &pmb887x_flash_numonyx_cmd_ops;
}
