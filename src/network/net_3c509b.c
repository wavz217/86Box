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
#include <86box/mem.h>
#include <86box/random.h>
#include <86box/device.h>
#include <86box/thread.h>
#include <86box/timer.h>
#include <86box/network.h>
#include <86box/plat_unused.h>

enum {
    ID_WAIT  = 0,
    ID_CMD   = 1,
    ACTIVE   = 2
};

typedef struct threec509b_t {
    int         state;

    int         waitcmd_id_port;
    int         is_seq_validated;
    int         current_seq_byte;
    int         confirmed_seq_bytes;
} threec509b_t;

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

/*
 * threec509b_id_seq_byte
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
threec509b_id_seq_byte(int n)
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
threec509b_nic_idwait_handler(uint16_t addr, uint8_t val, threec509b_t* dev)
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

    threec509b_log("3C509B: sequence: was given %X (expected %X)\n", val, dev->current_seq_byte);

    if (val == dev->current_seq_byte) {
        dev->confirmed_seq_bytes++;
        if (dev->confirmed_seq_bytes >= 255) {
            threec509b_log("3C509B: sequence validated, entering ID_CMD");
            dev->is_seq_validated = true;
            dev->state = ID_CMD;
            return;
        }
        dev->current_seq_byte = threec509b_id_seq_byte(dev->confirmed_seq_bytes);
        threec509b_log("3C509B: sequence: next value should be %X\n", dev->current_seq_byte);
        return;
    }

    dev->current_seq_byte = 0xFF;
    dev->confirmed_seq_bytes = 0;
    threec509b_log("3C509B: sequence: wrong value. resetting sequence. next value should be FF\n");
}

// BUG: I valori del controller IDE a 1F0h e 170h vengono sovrascritti.
static void
threec509b_nic_idcmd_write(uint16_t addr, uint8_t val, threec509b_t* dev)
{
    if (addr != dev->waitcmd_id_port)
        return;

    switch (val) {
        case 0x00 ... 0x7f:
            break;
        default:
            break;
    }
}

static uint8_t
threec509b_nic_waitcmd_read(uint16_t addr, void *priv)
{
    threec509b_t* dev = (threec509b_t *) priv;

    // threec509b_log("3C509B: IN called! addr: 0x%X\n", addr);

    return 0xFF;
}

static void
threec509b_nic_waitcmd_write(uint16_t addr, uint8_t val, void *priv)
{
    threec509b_t* dev = (threec509b_t *) priv;

    // threec509b_log("3C509B: OUT called! addr: 0x%X, val: 0x%X\n", addr, val);

    switch (dev->state) {
        case ID_WAIT:
            threec509b_nic_idwait_handler(addr, val, dev);
        case ID_CMD:
            threec509b_nic_idcmd_write(addr, val, dev);
        default:
            return;
    }
}

static void *
threec509b_nic_init(UNUSED(const device_t *info))
{
    threec509b_t *dev = calloc(1, sizeof(threec509b_t));
    threec509b_log("3C509B: init called!\n");

    /* The 3C509B operates with an activation mechanism which consists of 3 phases.
     * The first one sets the ID_WAIT state: the card is waiting for the driver to
     * write to any 01x0h I/O port (where the x is any hex value) to lock that port
     * for normal use later on.
     *
     * We're skipping the 1F0h port as, on most hardware configurations, it conflicts
     * with the IDE controller, making the whole emulation much slower and barely usable
     * in certain cases. ~99% of the time the driver just uses port 110h.
     */

    dev->state = ID_WAIT;
    threec509b_log("3C509B: state set to wait.\n");

    for (int port = 0x100; port <= 0x1E0; port += 0x10)
        io_sethandler(port, 1,
                      threec509b_nic_waitcmd_read, NULL, NULL,
                      threec509b_nic_waitcmd_write, NULL, NULL, dev);
        threec509b_log("3C509B: registered the IO ports with the emu.\n");

    return dev;
}

const device_t threec509b_device = {
    .name               = "3Com EtherLink III (3C509B)",
    .internal_name      = "3c509b",
    .flags              = DEVICE_ISA,
    .local              = 0,
    .init               = threec509b_nic_init,
    .close              = NULL,
    .reset              = NULL,
    .available          = NULL,
    .speed_changed      = NULL,
    .force_redraw       = NULL,
    .config             = NULL
};
