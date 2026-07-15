/*
 * SIM card interface
 */
#define PMB887X_TRACE_ID        SIM
#define PMB887X_TRACE_PREFIX    "pmb887x-sim"

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "qemu/timer.h"

#include "hw/arm/pmb887x/gen/cpu_regs.h"
#include "hw/arm/pmb887x/regs_dump.h"
#include "hw/arm/pmb887x/mod.h"
#include "hw/arm/pmb887x/trace.h"

#define TYPE_PMB887X_SIM "pmb887x-sim"
#define PMB887X_SIM(obj) OBJECT_CHECK(pmb887x_sim_t, (obj), TYPE_PMB887X_SIM)

#define SIM_CHARACTER_DELAY_NS (NANOSECONDS_PER_SECOND / 1000)

enum {
    IRQ_SIM_ERR,
    IRQ_SIM_IN,
    IRQ_SIM_OK,
};

typedef struct pmb887x_sim_t {
    SysBusDevice parent_obj;
    MemoryRegion mmio;

    pmb887x_clc_reg_t clc;
    qemu_irq irq[3];
    qemu_irq dmac_tx_sreq;
    qemu_irq dmac_tx_lsreq;
    QEMUTimer *character_timer;

    uint32_t regs[SIM_IO_SIZE / sizeof(uint32_t)];
    bool card_present;
    bool atr_active;
    bool t0_pending;
    unsigned int atr_index;
    uint8_t command[256];
    unsigned int command_len;
    unsigned int command_index;
    unsigned int command_bytes;
    uint8_t response[256];
    unsigned int response_len;
    unsigned int response_index;
    bool t0_direction_rx;
    uint16_t selected_file;
    int dmac_tx_clr;
} pmb887x_sim_t;

static const uint8_t sim_atr[] = { 0x3B, 0x00 };
static const uint8_t sim_directory_response[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0A, 0xB3, 0x01, 0x04,
    0x00, 0x00, 0x83, 0x8A, 0x83, 0x8A,
};
static const uint8_t sim_iccid[] = {
    0x98, 0x23, 0x40, 0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0xF2,
};
static const uint8_t sim_imsi[] = {
    0x08, 0x29, 0x06, 0x20, 0x21, 0x43, 0x65, 0x87, 0x09,
};
static const uint8_t sim_phase[] = { 0x03 };
static const uint8_t sim_language[] = { 0x00 };
static const uint8_t sim_admin_data[] = { 0x00, 0x00, 0x00, 0x02 };
static const uint8_t sim_service_table[10];
static const uint8_t sim_forbidden_plmn[12] = {
    [0 ... 11] = 0xFF,
};
static const uint8_t sim_emergency_call_codes[] = { 0x11, 0xF2, 0xFF };
static const uint8_t sim_kc[9] = { [0 ... 8] = 0xFF };
static const uint8_t sim_hplmn[] = { 0x0A };
static const uint8_t sim_access_control[] = { 0x00, 0x00 };
static const uint8_t sim_location_info[11] = {
    [0 ... 9] = 0xFF, [10] = 0x01,
};

typedef struct sim_file_t {
    uint16_t id;
    const uint8_t *data;
    size_t size;
} sim_file_t;

static const sim_file_t sim_files[] = {
    { 0x2FE2, sim_iccid, ARRAY_SIZE(sim_iccid) },
    { 0x2F05, sim_language, ARRAY_SIZE(sim_language) },
    { 0x6F05, sim_language, ARRAY_SIZE(sim_language) },
    { 0x6F07, sim_imsi, ARRAY_SIZE(sim_imsi) },
    { 0x6F20, sim_kc, ARRAY_SIZE(sim_kc) },
    { 0x6F31, sim_hplmn, ARRAY_SIZE(sim_hplmn) },
    { 0x6F38, sim_service_table, ARRAY_SIZE(sim_service_table) },
    { 0x6F78, sim_access_control, ARRAY_SIZE(sim_access_control) },
    { 0x6F7B, sim_forbidden_plmn, ARRAY_SIZE(sim_forbidden_plmn) },
    { 0x6F7E, sim_location_info, ARRAY_SIZE(sim_location_info) },
    { 0x6FAD, sim_admin_data, ARRAY_SIZE(sim_admin_data) },
    { 0x6FAE, sim_phase, ARRAY_SIZE(sim_phase) },
    { 0x6FB7, sim_emergency_call_codes,
        ARRAY_SIZE(sim_emergency_call_codes) },
};

static const sim_file_t *sim_find_file(uint16_t id)
{
    for (size_t i = 0; i < ARRAY_SIZE(sim_files); i++) {
        if (sim_files[i].id == id)
            return &sim_files[i];
    }

    return NULL;
}

static bool sim_is_directory(uint16_t id)
{
    return id == 0x3F00 || id == 0x7F20;
}

static void sim_prepare_get_response(pmb887x_sim_t *p, unsigned int len)
{
    const sim_file_t *file = sim_find_file(p->selected_file);

    if (sim_is_directory(p->selected_file)) {
        p->response_len = MIN(len, ARRAY_SIZE(sim_directory_response));
        memcpy(p->response, sim_directory_response, p->response_len);
        p->response[4] = p->selected_file >> 8;
        p->response[5] = p->selected_file;
        if (p->selected_file == 0x3F00)
            p->response[6] = 1;
        return;
    }

    p->response_len = MIN(len, 15);
    memset(p->response, 0, p->response_len);
    if (p->response_len > 2)
        p->response[2] = (file ? file->size : 1) >> 8;
    if (p->response_len > 3)
        p->response[3] = file ? file->size : 1;
    if (p->response_len > 4)
        p->response[4] = p->selected_file >> 8;
    if (p->response_len > 5)
        p->response[5] = p->selected_file;
    if (p->response_len > 6)
        p->response[6] = 4;
    if (p->response_len > 11)
        p->response[11] = 1;
    if (p->response_len > 12)
        p->response[12] = 2;
}

static void sim_prepare_file_data(pmb887x_sim_t *p, unsigned int len)
{
    const sim_file_t *file = sim_find_file(p->selected_file);

    p->response_len = MIN(len, ARRAY_SIZE(p->response));
    memset(p->response, 0xFF, p->response_len);
    if (file)
        memcpy(p->response, file->data, MIN(p->response_len, file->size));
}

static void sim_log_response(pmb887x_sim_t *p, const char *command,
    unsigned int requested_len)
{
    char data[3 * 32 + 1];
    size_t offset = 0;
    unsigned int dump_len = MIN(p->response_len, 32);

    for (unsigned int i = 0; i < dump_len; i++) {
        offset += snprintf(data + offset, sizeof(data) - offset,
            i ? " %02X" : "%02X", p->response[i]);
    }
    DPRINTF("HLE %s fid=%04X requested=%u returned=%u data=%s%s\n",
        command, p->selected_file, requested_len, p->response_len, data,
        p->response_len > dump_len ? " ..." : "");
}

static uint32_t *sim_reg(pmb887x_sim_t *p, hwaddr haddr)
{
    return &p->regs[haddr / sizeof(uint32_t)];
}

static void sim_update_irq(pmb887x_sim_t *p)
{
    uint32_t status = *sim_reg(p, SIM_STATUS);
    uint32_t irqen = *sim_reg(p, SIM_IRQEN);
    bool err = ((status & SIM_STATUS_PARINT) && (irqen & SIM_IRQEN_PAR)) ||
        ((status & SIM_STATUS_OVRRUN) && (irqen & SIM_IRQEN_OVR)) ||
        ((status & SIM_STATUS_T0END) && (irqen & SIM_IRQEN_T0END)) ||
        ((status & SIM_STATUS_CHTIMEOUT) && (irqen & SIM_IRQEN_CHTIMER));

    qemu_set_irq(p->irq[IRQ_SIM_ERR], err);
    qemu_set_irq(p->irq[IRQ_SIM_IN], 0);
    qemu_set_irq(p->irq[IRQ_SIM_OK],
        (status & SIM_STATUS_UARTOK) && (irqen & SIM_IRQEN_OK));
}

static void sim_schedule_character(pmb887x_sim_t *p)
{
    timer_mod(p->character_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
        SIM_CHARACTER_DELAY_NS);
}

static void sim_trigger_rx_dma(pmb887x_sim_t *p)
{
    bool last = p->response_index + 1 >= p->response_len;

    // DPRINTF("RX DMA request %u/%u%s\n", p->response_index + 1,
    //     p->response_len, last ? " last" : "");
    qemu_set_irq(p->dmac_tx_sreq, !last);
    qemu_set_irq(p->dmac_tx_lsreq, last);
}

static void sim_trigger_tx_dma(pmb887x_sim_t *p)
{
    bool last = p->command_index + 1 >= p->command_len;

    // DPRINTF("TX DMA request %u/%u%s\n", p->command_index + 1,
    //     p->command_len, last ? " last" : "");
    qemu_set_irq(p->dmac_tx_sreq, !last);
    qemu_set_irq(p->dmac_tx_lsreq, last);
}

static void sim_character_ready(void *opaque)
{
    pmb887x_sim_t *p = opaque;

    if (p->atr_active && p->atr_index < ARRAY_SIZE(sim_atr)) {
        *sim_reg(p, SIM_RX) = sim_atr[p->atr_index++];
        *sim_reg(p, SIM_STATUS) |= SIM_STATUS_UARTOK;
        sim_update_irq(p);
        return;
    }

    if (p->t0_pending) {
        uint8_t ins = *sim_reg(p, SIM_INS);

        p->t0_pending = false;
        if (ins == 0xA4) {
            uint16_t file = 0;

            if (p->command_bytes >= 2) {
                file =
                    (p->command[p->command_bytes - 2] << 8) |
                    p->command[p->command_bytes - 1];
            }
            if (sim_is_directory(file) || sim_find_file(file)) {
                p->selected_file = file;
                *sim_reg(p, SIM_SW1) = 0x9F;
                *sim_reg(p, SIM_SW2) =
                    sim_is_directory(file) ? 0x16 : 0x0F;
                DPRINTF("HLE SELECT fid=%04X -> %02X%02X\n", file,
                    *sim_reg(p, SIM_SW1), *sim_reg(p, SIM_SW2));
            } else {
                *sim_reg(p, SIM_SW1) = 0x94;
                *sim_reg(p, SIM_SW2) = 0x04;
                DPRINTF("HLE SELECT fid=%04X -> %02X%02X\n", file,
                    *sim_reg(p, SIM_SW1), *sim_reg(p, SIM_SW2));
            }
        } else {
            *sim_reg(p, SIM_SW1) = 0x90;
            *sim_reg(p, SIM_SW2) = 0x00;
            DPRINTF("HLE INS=%02X fid=%04X -> %02X%02X\n", ins,
                p->selected_file, *sim_reg(p, SIM_SW1),
                *sim_reg(p, SIM_SW2));
        }
        *sim_reg(p, SIM_STATUS) |= SIM_STATUS_T0END;
        sim_update_irq(p);
    }
}

static void sim_start_atr(pmb887x_sim_t *p)
{
    if (!p->card_present)
        return;

    p->atr_active = true;
    p->atr_index = 0;
    sim_schedule_character(p);
}

static void sim_start_t0(pmb887x_sim_t *p, uint32_t value)
{
    uint8_t ins = value;
    unsigned int len = *sim_reg(p, SIM_P3) & 0xFF;

    p->t0_pending = true;
    DPRINTF("T=0 INS=%02X P3=%u direction=%s\n", ins, len,
        value & 0x100 ? "RX" : "TX");
    p->t0_direction_rx = (value & 0x100) != 0;
    p->command_len = p->t0_direction_rx ? 0 : 5 + len;
    p->command_index = 0;
    p->command_bytes = 0;

    if (p->t0_direction_rx && ins == 0xC0) {
        sim_prepare_get_response(p, len);
        sim_log_response(p, "GET_RESPONSE", len);
        p->response_index = 0;
    } else if (p->t0_direction_rx && (ins == 0xB0 || ins == 0xB2)) {
        sim_prepare_file_data(p, len ? len : 256);
        sim_log_response(p, ins == 0xB0 ? "READ_BINARY" : "READ_RECORD",
            len ? len : 256);
        p->response_index = 0;
    } else {
        p->response_len = 0;
        p->response_index = 0;
    }

    if (p->t0_direction_rx)
        sim_trigger_rx_dma(p);
    else
        sim_trigger_tx_dma(p);
}

static uint64_t sim_io_read(void *opaque, hwaddr haddr, unsigned size)
{
    pmb887x_sim_t *p = opaque;
    hwaddr access_addr = haddr;
    uint64_t value;

    if (p->response_len && access_addr >= SIM_RX &&
        access_addr < SIM_RX + ARRAY_SIZE(p->response)) {
        value = p->response[p->response_index];
        IO_DUMP(access_addr + p->mmio.addr, size, value, false);
        return value;
    }

    haddr &= ~3;
    switch (haddr) {
    case SIM_CLC:
        value = pmb887x_clc_get(&p->clc);
        break;
    case SIM_ID:
        value = 0xF000C032;
        break;
    case SIM_STATUS:
        value = *sim_reg(p, haddr);
        if (p->card_present)
            value |= SIM_STATUS_SIMDET;
        else
            value &= ~SIM_STATUS_SIMDET;
        break;
    case SIM_RX:
        if (p->response_index < p->response_len)
            value = p->response[p->response_index];
        else
            value = *sim_reg(p, haddr);
        break;
    case SIM_CTRL:
    case SIM_BRF:
    case SIM_IRQEN:
    case SIM_RXSPC:
    case SIM_TXSPC:
    case SIM_CHTIMER1:
    case SIM_UNK3C:
    case SIM_BWT:
    case SIM_TX:
    case SIM_INS:
    case SIM_P3:
    case SIM_SW1:
    case SIM_SW2:
    case SIM_UNK70:
    case SIM_UNK7C:
    case SIM_UNK84:
        value = *sim_reg(p, haddr);
        break;
    default:
        if ((haddr & 3) || haddr >= SIM_IO_SIZE) {
            EPRINTF("invalid reg access: %02" PRIX64 "\n", haddr);
            exit(1);
        }
        value = *sim_reg(p, haddr);
        break;
    }

    IO_DUMP(access_addr + p->mmio.addr, size, value, false);
    return value;
}

static void sim_io_write(void *opaque, hwaddr haddr, uint64_t value,
    unsigned size)
{
    pmb887x_sim_t *p = opaque;
    hwaddr access_addr = haddr;

    IO_DUMP(access_addr + p->mmio.addr, size, value, true);

    haddr &= ~3;
    switch (haddr) {
    case SIM_CLC:
        pmb887x_clc_set(&p->clc, value);
        break;
    case SIM_CTRL: {
        uint32_t old = *sim_reg(p, haddr);
        *sim_reg(p, haddr) = value;

        if (!(value & SIM_CTRL_EN)) {
            p->atr_active = false;
            p->t0_pending = false;
            timer_del(p->character_timer);
        } else if (!(old & SIM_CTRL_RST) && (value & SIM_CTRL_RST)) {
            sim_start_atr(p);
        }
        break;
    }
    case SIM_STATUS:
        *sim_reg(p, haddr) &= SIM_STATUS_SIMDET;
        sim_update_irq(p);
        break;
    case SIM_IRQEN:
        *sim_reg(p, haddr) = value;
        sim_update_irq(p);
        break;
    case SIM_UNK7C:
        *sim_reg(p, haddr) = value;
        if (value & 4) {
            *sim_reg(p, SIM_STATUS) &= ~SIM_STATUS_UARTOK;
            sim_update_irq(p);
            if (p->atr_active && p->atr_index < ARRAY_SIZE(sim_atr)) {
                sim_schedule_character(p);
            } else {
                p->atr_active = false;
            }
        }
        if (value & 1) {
            *sim_reg(p, SIM_STATUS) &= ~(SIM_STATUS_PARINT |
                SIM_STATUS_OVRRUN | SIM_STATUS_T0END |
                SIM_STATUS_CHTIMEOUT);
            sim_update_irq(p);
        }
        break;
    case SIM_BRF:
    case SIM_RXSPC:
    case SIM_TXSPC:
    case SIM_CHTIMER1:
    case SIM_UNK3C:
    case SIM_BWT:
    case SIM_RX:
    case SIM_P3:
    case SIM_SW1:
    case SIM_SW2:
    case SIM_UNK70:
    case SIM_UNK84:
        *sim_reg(p, haddr) = value;
        break;
    case SIM_TX:
        *sim_reg(p, haddr) = value;
        if (p->command_bytes < ARRAY_SIZE(p->command))
            p->command[p->command_bytes++] = value;
        break;
    case SIM_INS:
        *sim_reg(p, haddr) = value;
        if (p->card_present)
            sim_start_t0(p, value);
        break;
    default:
        if ((haddr & 3) || haddr >= SIM_IO_SIZE) {
            EPRINTF("invalid reg access: %02" PRIX64 "\n", haddr);
            exit(1);
        }
        *sim_reg(p, haddr) = value;
        break;
    }
}

static void sim_handle_dmac_rx_clr(void *opaque, int id, int level)
{
    pmb887x_sim_t *p = opaque;
    int old_level = p->dmac_tx_clr;

    p->dmac_tx_clr = level;
    // DPRINTF("RX DMA CLR=%d index=%u/%u\n", level, p->response_index,
    //     p->response_len);
    if (level) {
        if (p->response_index + 1 >= p->response_len)
            qemu_set_irq(p->dmac_tx_lsreq, 0);
        else
            qemu_set_irq(p->dmac_tx_sreq, 0);
    } else if (!old_level) {
        return;
    } else if (++p->response_index < p->response_len) {
        sim_trigger_rx_dma(p);
    } else {
        p->response_len = 0;
        p->response_index = 0;
        sim_schedule_character(p);
    }
}

static void sim_handle_dmac_tx_clr(void *opaque, int id, int level)
{
    pmb887x_sim_t *p = opaque;
    int old_level = p->dmac_tx_clr;

    if (!p->command_len && p->response_len) {
        sim_handle_dmac_rx_clr(opaque, id, level);
        return;
    }

    p->dmac_tx_clr = level;
    // DPRINTF("TX DMA CLR=%d index=%u/%u\n", level, p->command_index,
    //     p->command_len);
    if (level) {
        if (p->command_index + 1 >= p->command_len)
            qemu_set_irq(p->dmac_tx_lsreq, 0);
        else
            qemu_set_irq(p->dmac_tx_sreq, 0);
    } else if (!old_level) {
        return;
    } else if (++p->command_index < p->command_len) {
        sim_trigger_tx_dma(p);
    } else {
        p->command_len = 0;
        p->command_index = 0;
        if (p->t0_direction_rx && p->response_len)
            sim_trigger_rx_dma(p);
        else
            sim_schedule_character(p);
    }
}

static const MemoryRegionOps sim_io_ops = {
    .read = sim_io_read,
    .write = sim_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void sim_init(Object *obj)
{
    pmb887x_sim_t *p = PMB887X_SIM(obj);

    memory_region_init_io(&p->mmio, obj, &sim_io_ops, p, TYPE_PMB887X_SIM,
        SIM_IO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &p->mmio);

    for (int i = 0; i < ARRAY_SIZE(p->irq); i++)
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &p->irq[i]);

    qdev_init_gpio_in_named(DEVICE(obj), sim_handle_dmac_tx_clr,
        "DMAC_TX_CLR", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &p->dmac_tx_sreq,
        "DMAC_TX_SREQ", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &p->dmac_tx_lsreq,
        "DMAC_TX_LSREQ", 1);
}

static void sim_realize(DeviceState *dev, Error **errp)
{
    pmb887x_sim_t *p = PMB887X_SIM(dev);

    pmb887x_clc_init(&p->clc);
    *sim_reg(p, SIM_BRF) = 0x5D;
    if (p->card_present)
        *sim_reg(p, SIM_STATUS) = SIM_STATUS_SIMDET;
    p->character_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
        sim_character_ready, p);
}

static const Property sim_properties[] = {
    DEFINE_PROP_BOOL("card-present", pmb887x_sim_t, card_present, false),
};

static void sim_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, sim_properties);
    dc->realize = sim_realize;
}

static const TypeInfo sim_info = {
    .name = TYPE_PMB887X_SIM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(pmb887x_sim_t),
    .instance_init = sim_init,
    .class_init = sim_class_init,
};

static void sim_register_types(void)
{
    type_register_static(&sim_info);
}
type_init(sim_register_types)
