/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          3Com EtherLink III 3C509B ISA PnP emulation
 *
 * Authors: wavz217 <wavz217@tuta.io>
 *
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include <wchar.h>
#include <time.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/io.h>
#include <86box/dma.h>
#include <86box/pic.h>
#include <86box/nvr.h>
#include <86box/mem.h>
#include <86box/random.h>
#include <86box/device.h>
#include <86box/thread.h>
#include <86box/timer.h>
#include <86box/network.h>
#include <86box/plat_unused.h>

#define MAX_3C509B_CARDS    4
#define EEPROM_WORDS_3C509B 64

static int g_3c509b_next_id = 0;

enum {
    ID_WAIT  = 0,
    ID_CMD   = 1,
    ACTIVE   = 2
};

typedef struct threec509b_t {
    int             state;
    bool            activated;

    int             card_id;

    int             io_port;

    int             waitcmd_id_port;
    bool            is_seq_validated;
    int             current_seq_byte;
    int             confirmed_seq_bytes;

    uint16_t        status;
    uint8_t         tag;

    uint8_t         window;

    uint16_t        interrupt_mask;
    uint16_t        rz_mask;

    /* Window 0 registers */                        // <port offset>
    uint16_t        eeprom_data;                    // 0C
    uint16_t        eeprom_command;                 // 0A
    uint16_t        resource_configuration;         // 08
    uint16_t        address_configuration;          // 06
    uint16_t        configuration_control;          // 04
    uint16_t        product_id;                     // 02

    /* Window 1 registers */
    /* RX */
    uint16_t        rx_status;

    /* Window 3 registers */
    uint32_t        internal_configuration;         // 02

    /* EEPROM */
    uint16_t        eeprom[EEPROM_WORDS_3C509B];
    char            nvr_fn[64];
    bool            eeprom_ewen;

    pc_timer_t      reset_timer;
} threec509b_t;

typedef struct threec509b_bus_t {
    threec509b_t    *cards[MAX_3C509B_CARDS];
    int             card_count;
    bool            handlers_registered;
} threec509b_bus_t;

static threec509b_bus_t g_3c509b_bus = {0};

/*
 * threec509b_log
 *
 * ARGUMENTS:
 *  - (variable)
 *
 * RETURNS: nothing
 *
 * This routine prints in the log when the 'ENABLE_3C509B_LOG' compiler flag is enabled.
 */
static void
threec509b_log(const char *fmt, ...)
{
    #ifdef ENABLE_3C509B_LOG
    va_list ap;
    va_start(ap, fmt);
    pclog_ex(fmt, ap);
    va_end(ap);
    #endif
}

static uint16_t
threec509b_eeprom_checksum(threec509b_t *dev)
{
    uint8_t hi = 0, lo = 0;

    for (int i = 0x00; i <= 0x0E; i++) {
        uint8_t b_hi = (dev->eeprom[i] >> 8) & 0xFF;
        uint8_t b_lo =  dev->eeprom[i]       & 0xFF;

        if (i == 0x08 || i == 0x09 || i == 0x0D) {
            lo ^= b_hi;
            lo ^= b_lo;
        } else {
            hi ^= b_hi;
            hi ^= b_lo;
        }
    }

    return ((uint16_t) hi << 8) | lo;
}

static void
threec509b_eeprom_set_defaults(threec509b_t *dev)
{
    uint8_t mac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x19, 0x22 };

    memset(dev->eeprom, 0xff, sizeof(dev->eeprom));

    dev->eeprom[0x00] = (mac[0] << 8) | mac[1];
    dev->eeprom[0x01] = (mac[2] << 8) | mac[3];
    dev->eeprom[0x02] = (mac[4] << 8) | mac[5];

    dev->eeprom[0x03] = 0x9150;
    dev->eeprom[0x04] = 0x0000;
    dev->eeprom[0x05] = 0x0000;
    dev->eeprom[0x06] = 0x0000;
    dev->eeprom[0x07] = 0x6D50;
    dev->eeprom[0x08] = 0x0000; // Address Configuration
    dev->eeprom[0x09] = 0x0000; // Resource Configuration
    dev->eeprom[0x0A] = dev->eeprom[0x00];
    dev->eeprom[0x0B] = dev->eeprom[0x01];
    dev->eeprom[0x0C] = dev->eeprom[0x02];
    dev->eeprom[0x0D] = 0x0000;   /* Software Information */
    dev->eeprom[0x0E] = 0x0000;   /* Compatibility Word */

    dev->eeprom[0x0F] = threec509b_eeprom_checksum(dev);

    dev->eeprom[0x10] = 0x2083;
    dev->eeprom[0x11] = 0x0000;
    dev->eeprom[0x12] = 0x0000;
    dev->eeprom[0x13] = 0x0000;
    dev->eeprom[0x14] = 0x0001;
    dev->eeprom[0x15] = 0x0000;
    dev->eeprom[0x16] = 0x0000;
    dev->eeprom[0x17] = 0x0000;
}

static void
threec509b_eeprom_save(threec509b_t *dev)
{
    FILE *fp = nvr_fopen(dev->nvr_fn, "wb");

    if (!fp) {
        threec509b_log("3C509B: error writing to the NVR file\n");
        return;
    }

    fwrite(dev->eeprom, sizeof(uint16_t), EEPROM_WORDS_3C509B, fp);
    fclose(fp);
}

static void
threec509b_eeprom_load(threec509b_t *dev)
{
    FILE *fp = nvr_fopen(dev->nvr_fn, "rb");

    if (!fp) {
        threec509b_log("3C509B: no NVR file, generating one\n");
        threec509b_eeprom_set_defaults(dev);
        threec509b_eeprom_save(dev);
        return;
    }

    size_t n = fread(dev->eeprom, sizeof(uint16_t), EEPROM_WORDS_3C509B, fp);
    fclose(fp);
}

static uint16_t
threec509b_eeprom_read(threec509b_t *dev, uint8_t addr)
{
    if (addr >= EEPROM_WORDS_3C509B)
        return 0xFFFF;

    return dev->eeprom[addr];
}

static void
threec509b_eeprom_handler(threec509b_t *dev)
{
    uint8_t cmd = dev->eeprom_command & 0xFF;
    uint8_t op = cmd >> 6;
    uint8_t addr = cmd & 0x3F;

    switch (op) {
        case 0x00:
            if ((addr >> 4) == 0x3) dev->eeprom_ewen = true;        // Erase/Write Enable
            else if ((addr >> 4) == 0x00) dev->eeprom_ewen = false; // Erase/Write Disable
            else if ((addr >> 4) == 0x02 && dev->eeprom_ewen)       // Erase All
                for (int i = 0; i < EEPROM_WORDS_3C509B; i++) dev->eeprom[i] = 0xFFFF;
            else if ((addr >> 4) == 0x01 && dev->eeprom_ewen)       // Write All
                for (int i = 0; i < EEPROM_WORDS_3C509B; i++) dev->eeprom[i] &= dev->eeprom_data;
            break;
        case 0x01:                                                  // Write
            if (dev->eeprom_ewen)
                dev->eeprom[addr] &= dev->eeprom_data;
            break;
        case 0x02:                                                  // Read
            dev->eeprom_data = dev->eeprom[addr];
            break;
        case 0x03:                                                  // Erase
            if (dev->eeprom_ewen)
                dev->eeprom[addr] = 0xFFFF;
            break;
    }
}

static void
threec509b_nic_status_irq(threec509b_t* dev)
{
    int irq = (dev->resource_configuration >> 12) & 0xF;
    int valid = (irq == 3 || irq == 5 || irq == 7 || irq == 9 ||
                irq == 10 || irq == 11 || irq == 12 || irq == 15);

    if (!valid)
        return;

    if (dev->status & dev->rz_mask & dev->interrupt_mask) {
        dev->status |= 0x41;
        picint(1 << irq);
    } else {
        dev->status &= ~0x41;
        picintc(1<< irq);
    }
}

static void
threec509b_reset_timer_cb(void *priv)
{
    threec509b_t *dev = (threec509b_t *) priv;
    dev->status &= ~0x1000;
}

static void
threec509b_nic_isa_bootstrap(threec509b_t* dev)
{
    dev->address_configuration = dev->eeprom[0x08];
    dev->resource_configuration = dev->eeprom[0x09];
    dev->product_id = dev->eeprom[0x03];
    dev->internal_configuration = ((uint32_t) dev->eeprom[0x13] << 16) | dev->eeprom[0x12];
}

static void
threec509b_nic_command_handler(threec509b_t *dev, uint16_t val)
{
    uint8_t cmd = val >> 11;
    uint16_t arg = val & 0x7FF;

    switch (cmd) {
        case 0x0:                                                   // Global Reset (16 clock)
            threec509b_log("3C509B: Global Reset.\n");
            dev->status |= 0x1000;
            threec509b_nic_isa_bootstrap(dev);
            dev->state = ID_WAIT;
            dev->tag = 0;
            dev->window = 0;
            dev->activated = false;
            dev->eeprom_ewen = false;
            dev->current_seq_byte = 0xFF;
            dev->confirmed_seq_bytes = 0;
            dev->is_seq_validated = 0;
            dev->interrupt_mask = 0;
            dev->rz_mask = 0;
            timer_set_delay_u64(&dev->reset_timer, 16);
            return;
        case 0x1:                                                   // Select Register Window
            threec509b_log("3C509B: Moving to window %d\n", arg & 0x7);
            dev->window = arg & 0x7;
            return;
        case 0xC:                                                   // Request Interrupt
            threec509b_log("3C509B: Interrupt requested\n");
            dev->status |= 0x40;
            threec509b_nic_status_irq(dev);
            return;
        case 0xD:                                                   // Acknowledge Interrupt
            threec509b_log("3C509B: Acknowledged interrupt\n");
            dev->status &= ~(arg & 0xFF);
            threec509b_nic_status_irq(dev);
            return;
        case 0xE:                                                   // Set Interrupt Mask
            threec509b_log("3C509B: Set interrupt mask\n");
            dev->interrupt_mask = arg & 0xFF;
            threec509b_nic_status_irq(dev);
            return;
        case 0xF:                                                   // Set Read Zero Mask
            threec509b_log("3C509B: Set RZ Mask\n");
            dev->rz_mask = arg & 0xFF;
            threec509b_nic_status_irq(dev);
            return;
        default:
            threec509b_log("3C509B: Command not implemented: cmd: %X arg: %X\n", cmd, arg);
            return;
    }
}

static uint16_t
threec509b_nic_read(threec509b_t *dev, uint8_t off)
{
    if (off == 0x0E)
        return (dev->status & dev->rz_mask) | ((dev->window & 0x7) << 13);

    switch (dev->window) {
        case 0:
            switch (off) {
                case 0x00: return 0x6D50;
                case 0x02: return dev->product_id;
                case 0x04: return dev->configuration_control;
                case 0x06: return dev->address_configuration;
                case 0x08: return dev->resource_configuration;
                case 0x0A: return (dev->eeprom_command & ~(0x7 << 8)) | ((dev->tag & 0x7) << 8);
                case 0x0C: return dev->eeprom_data;
            }
        default: return 0xFFFF;
    }
}

static void
threec509b_nic_write(threec509b_t *dev, uint16_t val, uint8_t off)
{
    if (off == 0x0E) {
        threec509b_nic_command_handler(dev, val);
        return;
    }

    switch (dev->window) {
        case 0:
            switch (off) {
                case 0x04: dev->configuration_control = val; break;
                case 0x06: dev->address_configuration = val; break;
                case 0x08: dev->resource_configuration = val; break;
                case 0x0A: dev->eeprom_command = val; threec509b_eeprom_handler(dev); break;
                case 0x0C: dev->eeprom_data = val; break;
            }
    }
}

static uint8_t
threec509b_nic_readb(uint16_t addr, void *priv)
{
    threec509b_log("3C509B: INB at 0x%X\n", addr);
    threec509b_t *dev = (threec509b_t *) priv;
    return threec509b_nic_read(dev, addr & 0xFF) & 0xFF;
}

static uint16_t
threec509b_nic_readw(uint16_t addr, void *priv)
{
    threec509b_log("3C509B: INW at 0x%X\n", addr);
    threec509b_t *dev = (threec509b_t *) priv;
    return threec509b_nic_read(dev, addr & 0x0F);
}

static uint32_t
threec509b_nic_readl(uint16_t addr, void *priv)
{
    threec509b_log("3C509B: INL at 0x%X\n", addr);
    return 0xFFFFFFFF;
}

static void
threec509b_nic_writeb(uint16_t addr, uint8_t val, void *priv)
{
    threec509b_log("3C509B: OUTB 0x%X at 0x%X\n", val, addr);
    threec509b_t *dev = (threec509b_t *) priv;
    threec509b_nic_write(dev, val, addr & 0x0F);
}

static void
threec509b_nic_writew(uint16_t addr, uint16_t val, void *priv)
{
    threec509b_log("3C509B: OUTW 0x%X at 0x%X\n", val, addr);
    threec509b_t *dev = (threec509b_t *) priv;
    threec509b_nic_write(dev, val, addr & 0x0F);
}

static void
threec509b_nic_writel(uint16_t addr, uint32_t val, void *priv)
{
    threec509b_log("3C509B: OUTL 0x%X at 0x%X\n", val, addr);
}

static void
threec509b_nic_activate(threec509b_t* dev)
{
    dev->activated = true;

    io_sethandler(dev->io_port, 0x10,
                  threec509b_nic_readb, threec509b_nic_readw, threec509b_nic_readl,
                  threec509b_nic_writeb, threec509b_nic_writew, threec509b_nic_writel, dev);
}

/*
 * threec509b_classic_id_seq_byte
 *
 * ARGUMENTS:
 *  - n: int -> the index of the sequence number to compute
 *
 * RETURNS: the sequence number
 *
 * This routine computes the sequence number at index 'n' to use by the
 * ISA proprietary detection mechanism.
 */
static uint8_t
threec509b_classic_id_seq_byte(int n)
{
    uint8_t val = 0xFF;

    for (int i = 0; i < n; i++) {
        bool carry = val & 0x80;
        val <<= 1;
        if (carry)
            val ^= 0xCF;
    }

    return val;
}

static void
threec509b_classic_idwait_handler(uint16_t addr, uint8_t val, threec509b_t* dev)
{
    if (val == 0) {
        threec509b_log("3C509B: assigning %Xh and listening for the sequence\n", addr);
        dev->waitcmd_id_port = addr;
        dev->current_seq_byte = 0xFF;
        dev->confirmed_seq_bytes = 0;
        return;
    }

    if (addr != dev->waitcmd_id_port)
        return;

    // threec509b_log("3C509B: sequence: was given %X (expected %X)\n", val, dev->current_seq_byte);

    if (val == dev->current_seq_byte) {
        dev->confirmed_seq_bytes++;
        if (dev->confirmed_seq_bytes >= 255) {
            threec509b_log("3C509B: sequence validated, entering ID_CMD\n");
            dev->is_seq_validated = true;
            dev->state = ID_CMD;
            return;
        }
        dev->current_seq_byte = threec509b_classic_id_seq_byte(dev->confirmed_seq_bytes);
        // threec509b_log("3C509B: sequence: next value should be %X\n", dev->current_seq_byte);
        return;
    }

    dev->current_seq_byte = 0xFF;
    dev->confirmed_seq_bytes = 0;
    // threec509b_log("3C509B: sequence: wrong value. resetting sequence. next value should be FF\n");
}

static void
threec509b_classic_idcmd_write(uint16_t addr, uint8_t val, threec509b_t* dev)
{
    if (addr != dev->waitcmd_id_port)
        return;

    if (val <= 0x7f) {
        /* 00 to 7F: Go to ID_WAIT state. Wait for next ID sequence. */
        threec509b_log("3C509B: received cmd %X, returning to ID_WAIT\n", val);
        dev->state = ID_WAIT;
        return;
    } else if (val <= 0xbf) {
        threec509b_log("3C509B: received cmd %X, loading EEPROM word %X\n", val, val & 0x3F);
        dev->eeprom_data = threec509b_eeprom_read(dev, val & 0x3F);
        return;
    } else if (val <= 0xcf) {
        threec509b_log("3C509B: received cmd %X, global reset\n", val);
        return;
    } else if (val <= 0xd7) {
        threec509b_log("3C509B: received cmd %X, setting tag adapter to %X\n", val, val & 0x7);
        uint8_t n = val & 0x07;
        if (n == 0) {
            dev->tag = 0;
            return;
        }
        if (dev->tag)
            return;
        dev->tag = n;
        return;
    } else if (val <= 0xdf) {
        threec509b_log("3C509B: received cmd %X, testing adapter %X\n", val, val & 0x7);
        uint8_t n = val & 0x7;
        if (dev->tag != n)
            dev->state = ID_WAIT;
        return;
    } else if (val <= 0xfe) {
        threec509b_log("3C509B: received cmd %X, activating at IO base address: %X\n", val, val & 0x1F);
        uint8_t n = val & 0x1F;
        dev->address_configuration = (dev->address_configuration & ~0x1F) | n;
        dev->io_port = 0x200 + (n << 4);
        dev->state = ID_WAIT;
        threec509b_nic_activate(dev);
        return;
    } else if (val == 0xff) {
        threec509b_log("3C509B: received cmd %X, activating at EEPROM IO base address\n", val);
        return;
    }
    threec509b_log("3C509B: received cmd %X, unknown\n", val);
    /* TODO: 80-BF (EEPROM read), C0-CF (global reset), D0-DF (tag/test), E0-FF (activate). */
}

static uint8_t
threec509b_classic_bus_read(uint16_t addr, void *priv)
{
    threec509b_bus_t *bus = (threec509b_bus_t *) priv;
    uint8_t bus_value = 0xFF;

    for (int i = 0; i < bus->card_count; i++) {
        threec509b_t *dev = bus->cards[i];
        if (dev->state != ID_CMD || addr != dev->waitcmd_id_port || dev->tag)
            continue;
        bus_value &= ((dev->eeprom_data >> 15) & 1) ? 0x01 : 0x00;
    }

    for (int i = 0; i < bus->card_count; i++) {
        threec509b_t *dev = bus->cards[i];
        if (dev->state != ID_CMD || addr != dev->waitcmd_id_port || dev->tag)
            continue;
        int driven = (dev->eeprom_data >> 15) & 1;
        dev->eeprom_data <<= 1;
        if (driven && !(bus_value & 1))
            dev->state = ID_WAIT;
    }

    return bus_value;
}

static void
threec509b_classic_bus_write(uint16_t addr, uint8_t val, void *priv)
{
    // threec509b_log("3C509B: OUT called! addr: 0x%X, val: 0x%X\n", addr, val);
    threec509b_bus_t *bus = (threec509b_bus_t *) priv;

    for (int i = 0; i < bus->card_count; i++) {
        threec509b_t *dev = bus->cards[i];

        switch (dev->state) {
            case ID_WAIT:
                threec509b_classic_idwait_handler(addr, val, dev);
                break;
            case ID_CMD:
                threec509b_classic_idcmd_write(addr, val, dev);
                break;
            default:
                continue;
        }
    }
}

static void *
threec509b_nic_init(UNUSED(const device_t *info))
{
    threec509b_t *dev = calloc(1, sizeof(threec509b_t));
    threec509b_log("3C509B: init called!\n");

    dev->card_id = g_3c509b_next_id;
    g_3c509b_next_id++;

    if (dev->card_id < MAX_3C509B_CARDS) {
        g_3c509b_bus.cards[dev->card_id] = dev;
        g_3c509b_bus.card_count++;
    } else
        threec509b_log("3C509B: too many cards, %d not added to the bus.\n", dev->card_id);

    snprintf(dev->nvr_fn, sizeof(dev->nvr_fn), "3c509b_%d.nvr", dev->card_id);
    threec509b_eeprom_load(dev);

    threec509b_nic_isa_bootstrap(dev);

    timer_add(&dev->reset_timer, threec509b_reset_timer_cb, dev, 0);

    dev->state = ID_WAIT;
    threec509b_log("3C509B: state set to wait.\n");

    if (!g_3c509b_bus.handlers_registered) {

        int isa_activation_select = (dev->internal_configuration >> 18) & 0x3;

        if (isa_activation_select != 0x2) {
            /* The 3C509B operates with an activation mechanism which consists of 3 phases.
             * The first one sets the ID_WAIT state: the card is waiting for the driver to
             * write to any 01x0h I/O port (where the x is any hex value) to lock that port
             * for normal use later on.
             *
             * We're skipping the 1F0h port as, on most hardware configurations, it conflicts
             * with the IDE controller, making the whole emulation much slower and barely usable
             * in certain cases. ~99% of the time the driver just uses port 110h.
             */

            for (int port = 0x100; port <= 0x1E0; port += 0x10) {
                io_sethandler(port, 1,
                              threec509b_classic_bus_read, NULL, NULL,
                              threec509b_classic_bus_write, NULL, NULL, &g_3c509b_bus);
            }
            g_3c509b_bus.handlers_registered = true;
            threec509b_log("3C509B: classic: registered the IO ports with the emu.\n");
        }
    }

    return dev;
}

static void
threec509b_nic_close(void *priv)
{
    threec509b_t *dev = (threec509b_t *) priv;

    for (int i = 0; i < g_3c509b_bus.card_count; i++) {
        if (g_3c509b_bus.cards[i] == dev) {
            g_3c509b_bus.cards[i] = g_3c509b_bus.cards[g_3c509b_bus.card_count - 1];
            g_3c509b_bus.cards[g_3c509b_bus.card_count - 1] = NULL;
            g_3c509b_bus.card_count--;
            break;
        }
    }

    threec509b_eeprom_save(dev);

    timer_stop(&dev->reset_timer);

    free(dev);
}

const device_t threec509b_device = {
    .name               = "3Com EtherLink III (3C509B)",
    .internal_name      = "3c509b",
    .flags              = DEVICE_ISA,
    .local              = 0,
    .init               = threec509b_nic_init,
    .close              = threec509b_nic_close,
    .reset              = NULL,
    .available          = NULL,
    .speed_changed      = NULL,
    .force_redraw       = NULL,
    .config             = NULL
};
