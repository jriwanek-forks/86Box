/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of the WD33c93 series of SCSI Host Adapters
 *          made by Western Digital. These controllers were designed for the ISA (PC)
 *          and C-BUS (PC-98x1) buses.
 *
 *
 *
 * Authors: Sarah Walker, <http://pcem-emulator.co.uk/>
 *          TheCollector1995, <mariogplayer@gmail.com>
 *
 *          Copyright 2017-2020 Sarah Walker.
 *          Copyright 2017-2020 TheCollector1995.
 */
 
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <wchar.h>
#include <86box/86box.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/dma.h>
#include <86box/pic.h>
#include <86box/timer.h>
#include <86box/device.h>
#include <86box/plat.h>
#include <86box/scsi.h>
#include <86box/scsi_device.h>
#include <86box/scsi_wd33c93.h>

/* WD commands */
#define WD_CMD_RESET                0x00
#define WD_CMD_ABORT                0x01
#define WD_CMD_ASSERT_ATN           0x02
#define WD_CMD_NEGATE_ACK           0x03
#define WD_CMD_DISCONNECT           0x04
#define WD_CMD_RESELECT             0x05
#define WD_CMD_SEL_ATN              0x06
#define WD_CMD_SEL                  0x07
#define WD_CMD_SEL_ATN_XFER         0x08
#define WD_CMD_SEL_XFER             0x09
#define WD_CMD_RESEL_RECEIVE        0x0a
#define WD_CMD_RESEL_SEND           0x0b
#define WD_CMD_WAIT_SEL_RECEIVE     0x0c
#define WD_CMD_SSCC                 0x0d
#define WD_CMD_SND_DISC             0x0e
#define WD_CMD_SET_IDI              0x0f
#define WD_CMD_RCV_CMD              0x10
#define WD_CMD_RCV_DATA             0x11
#define WD_CMD_RCV_MSG_OUT          0x12
#define WD_CMD_RCV                  0x13
#define WD_CMD_SND_STATUS           0x14
#define WD_CMD_SND_DATA             0x15
#define WD_CMD_SND_MSG_IN           0x16
#define WD_CMD_SND                  0x17
#define WD_CMD_TRANS_ADDR           0x18
#define WD_CMD_XFER_PAD             0x19
#define WD_CMD_TRANS_INFO           0x20
#define WD_CMD_TRANSFER_PAD         0x21
#define WD_CMD_SBT_MODE             0x80

/* ASR register */
#define ASR_INT                     0x80
#define ASR_LCI                     0x40
#define ASR_BSY                     0x20
#define ASR_CIP                     0x10
#define ASR_PE                      0x02
#define ASR_DBR                     0x01

/* SCSI Bus Phases */
#define PHS_DATA_OUT                0x00
#define PHS_DATA_IN                 0x01
#define PHS_COMMAND                 0x02
#define PHS_STATUS                  0x03
#define PHS_MESS_OUT                0x06
#define PHS_MESS_IN                 0x07

/* Command Status Register definitions */

    /* reset state interrupts */
#define CSR_RESET                   0x00
#define CSR_RESET_AF                0x01

    /* successful completion interrupts */
#define CSR_RESELECT                0x10
#define CSR_SELECT                  0x11
#define CSR_SEL_XFER_DONE           0x16
#define CSR_XFER_DONE               0x18

    /* paused or aborted interrupts */
#define CSR_MSGIN                   0x20
#define CSR_SDP                     0x21
#define CSR_SEL_ABORT               0x22
#define CSR_RESEL_ABORT             0x25
#define CSR_RESEL_ABORT_AM          0x27
#define CSR_ABORT                   0x28

    /* terminated interrupts */
#define CSR_INVALID                 0x40
#define CSR_UNEXP_DISC              0x41
#define CSR_TIMEOUT                 0x42
#define CSR_PARITY                  0x43
#define CSR_PARITY_ATN              0x44
#define CSR_BAD_STATUS              0x45
#define CSR_UNEXP                   0x48

    /* service required interrupts */
#define CSR_RESEL                   0x80
#define CSR_RESEL_AM                0x81
#define CSR_DISC                    0x85
#define CSR_SRV_REQ                 0x88

    /* Own ID/CDB Size register */
#define OWNID_EAF                   0x08
#define OWNID_EHP                   0x10
#define OWNID_RAF                   0x20
#define OWNID_FS_8                  0x00
#define OWNID_FS_12                 0x40
#define OWNID_FS_16                 0x80

    /* Control register */
#define CTRL_HSP                    0x01
#define CTRL_HA                     0x02
#define CTRL_IDI                    0x04
#define CTRL_EDI                    0x08
#define CTRL_HHP                    0x10
#define CTRL_POLLED                 0x00
#define CTRL_BURST                  0x20
#define CTRL_BUS                    0x40
#define CTRL_DMA                    0x80

    /* Synchronous Transfer Register */
#define STR_FSS                     0x80

    /* Destination ID register */
#define DSTID_DPD                   0x40
#define DATA_OUT_DIR                0
#define DATA_IN_DIR                 1
#define DSTID_SCC                   0x80

    /* Source ID register */
#define SRCID_MASK                  0x07
#define SRCID_SIV                   0x08
#define SRCID_DSP                   0x20
#define SRCID_ES                    0x40
#define SRCID_ER                    0x80

enum {
    WD_OWN_ID               = 0x00,
    WD_CONTROL              = 0x01,
    WD_TIMEOUT_PERIOD       = 0x02,
    WD_CDB_1                = 0x03,
    WD_CDB_2                = 0x04,
    WD_CDB_3                = 0x05,
    WD_CDB_4                = 0x06,
    WD_CDB_5                = 0x07,
    WD_CDB_6                = 0x08,
    WD_CDB_7                = 0x09,
    WD_CDB_8                = 0x0a,
    WD_CDB_9                = 0x0b,
    WD_CDB_10               = 0x0c,
    WD_CDB_11               = 0x0d,
    WD_CDB_12               = 0x0e,
    WD_TARGET_LUN           = 0x0f,
    WD_COMMAND_PHASE        = 0x10,
    WD_SYNCHRONOUS_TRANSFER = 0x11,
    WD_TRANSFER_COUNT_MSB   = 0x12,
    WD_TRANSFER_COUNT       = 0x13,
    WD_TRANSFER_COUNT_LSB   = 0x14,
    WD_DESTINATION_ID       = 0x15,
    WD_SOURCE_ID            = 0x16,
    WD_SCSI_STATUS          = 0x17,
    WD_COMMAND              = 0x18,
    WD_DATA                 = 0x19,
    WD_QUEUE_TAG            = 0x1a,
    WD_AUXILIARY_STATUS     = 0x1f
};

typedef struct wd33c93_s {
    uint8_t    latch;
    uint8_t    regs[WD_AUXILIARY_STATUS + 1];
    uint8_t    dev_id;
    uint8_t    lun;
    uint8_t    phase;
    int        fifo_pos;
    uint8_t    fifo[12];
    int        data_pos;
    uint8_t    wdstatus;
    uint8_t    dma_status;
    int        read_pending;
    int        identify;
    uint32_t   dma_address;
    int        irq;
    int        dma_channel;
    pc_timer_t cmd_callback;
    pc_timer_t service_req_timer;
    pc_timer_t cip_timer;
} wd33c93_t;

// #define ENABLE_WDSCSI_LOG 1

#ifdef ENABLE_WDSCSI_LOG
int wd33c93_do_log = ENABLE_WDSCSI_LOG;

static void
wd33c93_log(const char *fmt, ...)
{
    va_list ap;

    if (wd33c93_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define wd33c93_log(fmt, ...)
#endif

static void 
wd33c93_set_xfer_count(wd33c93_t *wd33c93, int count)
{
    /* set the count */
    wd33c93->regs[WD_TRANSFER_COUNT_LSB] = count & 0xff;
    wd33c93->regs[WD_TRANSFER_COUNT] = (count >> 8) & 0xff;
    wd33c93->regs[WD_TRANSFER_COUNT_MSB] = (count >> 16) & 0xff;
}

static int 
wd33c93_get_xfer_count(wd33c93_t *wd33c93)
{
    /* get the count */
    int count = wd33c93->regs[WD_TRANSFER_COUNT_MSB];

    count <<= 8;
    count |= wd33c93->regs[WD_TRANSFER_COUNT];
    count <<= 8;
    count |= wd33c93->regs[WD_TRANSFER_COUNT_LSB];

    return count;
}

static void 
wd33c93_complete_immediate(wd33c93_t *wd33c93, uint8_t status)
{
    wd33c93_log("WD33c93: status %x\n", status & 0xff);

    /* set the new status */
    wd33c93->regs[WD_SCSI_STATUS] = status & 0xff;

    /* set interrupt pending */
    wd33c93->regs[WD_AUXILIARY_STATUS] |= ASR_INT;

    /* check for error conditions */
    if (wd33c93_get_xfer_count(wd33c93) > 0) {
        wd33c93_log("WD33c93: Data buffer ready\n");
        /* set data buffer ready */
        wd33c93->regs[WD_AUXILIARY_STATUS] |= ASR_DBR;
    } else {
        /* clear data buffer ready */
        wd33c93->regs[WD_AUXILIARY_STATUS] &= ~ASR_DBR;
    }

    /* clear command in progress and bus busy */
    wd33c93->regs[WD_AUXILIARY_STATUS] &= ~(ASR_CIP | ASR_BSY);

    /* if we have a callback, call it */
    if ((wd33c93->regs[WD_AUXILIARY_STATUS] & ASR_INT) != 0) {
        wd33c93_log("WD33c93: Interrupt set\n");
        picint(1 << wd33c93->irq);
    }
}

static void 
wd33c93_command_poll(void *priv)
{
    wd33c93_t *wd33c93 = (wd33c93_t *) priv;

    /* reset our timer */
    wd33c93->cmd_callback = 0LL;

    wd33c93_complete_immediate(wd33c93, wd33c93->wdstatus);
}

static void 
wd33c93_service_poll(void *priv)
{
    wd33c93_t *wd33c93 = (wd33c93_t *) priv;

    /* reset our timer */
    wd33c93->service_req_timer = 0LL;

    wd33c93_complete_immediate(wd33c93, CSR_SRV_REQ | wd33c93->phase);
}

static void 
wd33c93_cip_poll(void *priv)
{
    wd33c93_t *wd33c93 = (wd33c93_t *) priv;

    /* reset our timer */
    wd33c93->cip_timer = 0LL;

    wd33c93->regs[WD_AUXILIARY_STATUS] &= ~ASR_CIP;
}

static void 
wd33c93_complete_cmd(wd33c93_t *wd33c93)
{
    /* fire off a timer to complete the command */
    wd33c93->cmd_callback = 30LL * TIMER_USEC;
}

/* command handlers */
static void 
wd33c93_unimplemented_cmd(wd33c93_t *wd33c93)
{
    wd33c93_log("WD33c93: Unimplemented SCSI controller command: %02x\n", wd33c93->regs[WD_COMMAND]);

    /* complete the command */
    wd33c93->wdstatus = CSR_INVALID;
    wd33c93_complete_cmd(wd33c93);
}

static void 
wd33c93_invalid_cmd(wd33c93_t *wd33c93)
{
    wd33c93_log("WD33c93: Invalid SCSI controller command: %02x\n", wd33c93->regs[WD_COMMAND]);

    /* complete the command */
    wd33c93->wdstatus = CSR_INVALID;
    wd33c93_complete_cmd(wd33c93);
}

static void 
wd33c93_abort_cmd(wd33c93_t *wd33c93)
{
    /* complete the command */
    wd33c93->wdstatus = CSR_ABORT;
    wd33c93_complete_cmd(wd33c93);
}

static void 
wd33c93_xferinfo_cmd(wd33c93_t *wd33c93)
{
    /* make the buffer available right away */
    wd33c93->regs[WD_AUXILIARY_STATUS] |= ASR_DBR;
    wd33c93->regs[WD_AUXILIARY_STATUS] |= ASR_CIP;

    /* the command will be completed once the data is transferred */
    wd33c93->cip_timer = TIMER_USEC;
}

/* Handle pending commands */
static void 
wd33c93_execute_command(wd33c93_t *wd33c93, uint8_t val)
{
    switch(val) {
    case WD_CMD_RESET:
    {
        int advanced = 0;

        /* see if it wants us to reset with advanced features */
        if (wd33c93->regs[WD_OWN_ID] & OWNID_EAF) {
            wd33c93_log("WD33c93: Advanced features enabled\n");
            advanced = 1;
        }

        memset(wd33c93->regs, 0, sizeof(wd33c93->regs));

        wd33c93_log("WD33c93: Reset completed\n");

        /* complete the command */
        wd33c93->wdstatus = advanced ? CSR_RESET_AF : CSR_RESET;
        wd33c93_complete_cmd(wd33c93);
    }
    break;

    case WD_CMD_ABORT:
        wd33c93_abort_cmd(wd33c93);
        break;

    case WD_CMD_NEGATE_ACK:
        wd33c93_log("WD33c93: ACK Negated\n");

        /* complete the command */
        wd33c93->regs[WD_AUXILIARY_STATUS] &= ~(ASR_CIP | ASR_BSY);
        break;

    case WD_CMD_DISCONNECT:
        /* complete the command */
        wd33c93->regs[WD_AUXILIARY_STATUS] &= ~(ASR_CIP | ASR_BSY);
        break;

    case WD_CMD_SEL_ATN:
    case WD_CMD_SEL:
    {
        uint8_t newstatus;
        wd33c93->dev_id = wd33c93->regs[WD_DESTINATION_ID] & SRCID_MASK;

        /* see if we can select that device */
        if ((wd33c93->dev_id != -1) && scsi_device_present(sd)) {
            /* device is available - signal selection done */
            newstatus = CSR_SELECT;

            /* determine the next bus phase depending on the command */
            if ((wd33c93->regs[WD_COMMAND] & 0x7f) == WD_CMD_SEL_ATN) {
                wd33c93_log("WD33c93: ATN asserted\n");

                /* /ATN asserted during select: Move to Message Out Phase to read identify */
                wd33c93->phase = PHS_MESS_OUT;
            } else {
                wd33c93_log("WD33c93: No ATN asserted\n");

                /* No /ATN asserted: Move to Command Phase */
                wd33c93->phase = PHS_COMMAND;
            }

            /* queue up a service request out in the future */
            wd33c93->service_req_timer = 50LL * TIMER_USEC;
        } else {
            /* device is not available */
            newstatus = CSR_TIMEOUT;
        }

        /* complete the command */
        wd33c93->wdstatus = newstatus;
        wd33c93_complete_cmd(wd33c93);
    }
    break;

    case WD_CMD_SEL_ATN_XFER:
    case WD_CMD_SEL_XFER:
    {
        uint8_t newstatus;
        wd33c93->dev_id = wd33c93->regs[WD_DESTINATION_ID] & SRCID_MASK;
        wd33c93->lun = (wd33c93->regs[WD_CDB_2] >> 5) & 7;

        wd33c93_log("WD33c93: id %i\n", wd33c93->dev_id);

        /* see if we can select that device */
        if ((wd33c93->dev_id != -1) && scsi_device_present(sd)) {
            scsi_device_t *sd = &scsi_devices[wd33c93->dev_id][wd33c93->lun];
//          scsi_device_t *sd = &scsi_devices[scsi->bus][scsi->cdb_id];

            wd33c93_log("SCSI media type enumerator is %i\n", sd->LunType);

            if (wd33c93->regs[WD_COMMAND_PHASE] < 0x45) {
                wd33c93->regs[WD_OWN_ID] = scsi_device_cdb_length(sd);

                /* device is available */
                wd33c93_log("WD33c93: cdb len %i, command %02x\n", wd33c93->regs[WD_OWN_ID], wd33c93->regs[WD_CDB_1]);

                sd->buffer_length = -1;

                /* do the request */
                scsi_device_command_phase0(sd, &wd33c93->regs[WD_CDB_1]);

                if (wd33c93->phase == SCSI_PHASE_DATA_IN)
                    sd->cmd_buffer = (uint8_t *) malloc(sd->buffer_length);

                wd33c93_log("WD33c93: data length %i\n", sd->buffer_length);

                /* set transfer count */
                wd33c93_set_xfer_count(wd33c93, sd->buffer_length);

                switch (wd33c93->phase) {
                    case SCSI_PHASE_DATA_IN:
                        wd33c93->read_pending = 1;
                        scsi_device_command_phase1(sd);
                        wd33c93_log("WD33c93: Command Phase 1 in Data In\n");
                        break;
                }
            }

            if (wd33c93->read_pending) {
                dma_bm_write(wd33c93->dma_address,
                        (char *) sd->cmd_buffer, sd->buffer_length);

                wd33c93->read_pending = 0;
            }

            wd33c93->regs[WD_CONTROL] |= CTRL_EDI;
            wd33c93->regs[WD_COMMAND_PHASE] = 0x60;

            /* signal transfer ready */
            newstatus = CSR_SEL_XFER_DONE;

            wd33c93_set_xfer_count(wd33c93, 0);
            wd33c93->regs[WD_AUXILIARY_STATUS] &= ~ASR_DBR;

            /* if allowed disconnect, queue a service request */
            if (wd33c93->identify & 0x40) {
                /* queue disconnect message in */
                wd33c93->phase = PHS_MESS_IN;

                /* queue up a service request out in the future */
                wd33c93->service_req_timer = 50LL * TIMER_USEC;
            }
        } else {
            /* device is not available */
            newstatus = CSR_TIMEOUT;

            wd33c93_set_xfer_count(wd33c93, 0);
        }

        /* complete the command */
        wd33c93->wdstatus = newstatus;
        wd33c93_complete_cmd(wd33c93);
    }
    break;

    case WD_CMD_TRANS_INFO:
        wd33c93_xferinfo_cmd(wd33c93);
        break;

    case WD_CMD_ASSERT_ATN:
    case WD_CMD_RESELECT:
    case WD_CMD_RESEL_RECEIVE:
    case WD_CMD_RESEL_SEND:
    case WD_CMD_WAIT_SEL_RECEIVE:
    case WD_CMD_SSCC:
    case WD_CMD_SND_DISC:
    case WD_CMD_SET_IDI:
    case WD_CMD_RCV_CMD:
    case WD_CMD_RCV_DATA:
    case WD_CMD_RCV_MSG_OUT:
    case WD_CMD_RCV:
    case WD_CMD_SND_STATUS:
    case WD_CMD_SND_DATA:
    case WD_CMD_SND_MSG_IN:
    case WD_CMD_SND:
    case WD_CMD_TRANS_ADDR:
    case WD_CMD_XFER_PAD:
    case WD_CMD_TRANSFER_PAD:
        wd33c93_unimplemented_cmd(wd33c93);
        break;

    default:
        wd33c93_invalid_cmd(wd33c93);
        break;
    }
}

static void
wd33c93_write(uint16_t port, uint8_t val, void *priv)
{
    wd33c93_t *wd33c93 = (wd33c93_t *) priv;
    wd33c93_log("WD33c93: Write port %03x, val %x\n", port, val);

    switch (port & 3) {
        case 0:
        {
            /* update register select */
            wd33c93->latch = val & 0x1f;
        }
        break;

        case 1:
        {
            wd33c93_log("WD33c93: Write REG=%02x, data %02x\n", wd33c93->latch, val);

            /* update the register */
            wd33c93->regs[wd33c93->latch] = val;

            /* if we receive a command, schedule to process it */
            if (wd33c93->latch == WD_COMMAND) {
                wd33c93_log("WD33c93: Executing command %08x\n", val);

                /* signal we're processing it */
                wd33c93->regs[WD_AUXILIARY_STATUS] |= ASR_CIP;

                /* process the command */
                wd33c93_execute_command(wd33c93, val & 0x7f);
            } else if (wd33c93->latch == WD_CDB_1)
                wd33c93->regs[WD_COMMAND_PHASE] = 0;
            else if (wd33c93->latch == WD_DATA) {
                wd33c93_log("WD33c93: Write Data register\n");

                wd33c93->dev_id = wd33c93->regs[WD_DESTINATION_ID] & SRCID_MASK;
                wd33c93->lun = (wd33c93->regs[WD_CDB_2] >> 5) & 7;
                scsi_device_t *sd = &scsi_devices[wd33c93->dev_id][wd33c93->lun];

                /* if data was written, and we have a count, send to device */
                if (wd33c93->regs[WD_COMMAND] & 0x80)
                    sd->buffer_length = 1;

                if (sd->buffer_length-- > 0) {
                    /* write to FIFO */
                    if (wd33c93->fifo_pos <= 12)
                        wd33c93->fifo[wd33c93->fifo_pos++] = val;

                    /* update count */
                    wd33c93_set_xfer_count(wd33c93, sd->buffer_length);

                    /* if we're done with the write, see where we're at */
                    if (sd->buffer_length == 0) {
                        wd33c93->regs[WD_AUXILIARY_STATUS] |= ASR_INT;
                        wd33c93->regs[WD_AUXILIARY_STATUS] &= ~ASR_DBR;

                        switch (wd33c93->phase) {
                            case PHS_MESS_OUT:
                            /* Message out phase. Data is probably SCSI Identify. Move to command phase. */
                            wd33c93->phase = PHS_COMMAND;

                            /* reset fifo */
                            wd33c93->fifo_pos = 0;

                            wd33c93->identify = wd33c93->fifo[0];
                            break;

                            case PHS_COMMAND:
                            wd33c93_log("WD33c93: FIFO phase command\n");

                            wd33c93->fifo_pos = 0;

                            wd33c93->regs[WD_OWN_ID] = scsi_device_cdb_length(sd);

                            scsi_device_command_phase0(sd, wd33c93->regs[WD_OWN_ID]);

                            if (wd33c93->phase == SCSI_PHASE_STATUS)
                                goto skip_write_phase1;

                            if ((wd33c93->phase == SCSI_PHASE_DATA_IN) ||
                                (wd33c93->phase == SCSI_PHASE_DATA_OUT)) {
                                wd33c93_log("sd->cmd_buffer = %08X\n", sd->cmd_buffer);
                                sd->cmd_buffer = (uint8_t *) malloc(sd->buffer_length);
                                wd33c93_log("sd->cmd_buffer = %08X\n", sd->cmd_buffer);
                            }

                            /* reset fifo */
                            wd33c93->fifo_pos = 0;

                            /* set the new count */
                            wd33c93_set_xfer_count(wd33c93, sd->buffer_length);

                            switch (wd33c93->phase) {
                                case SCSI_PHASE_STATUS:
                                    wd33c93->phase = PHS_STATUS;
                                    break;

                                case SCSI_PHASE_DATA_IN:
                                    scsi_device_command_phase1(sd);
                                    wd33c93->phase = PHS_DATA_IN;
                                    wd33c93->read_pending = 1;
                                    break;

                                case SCSI_PHASE_DATA_OUT:
                                    wd33c93->phase = PHS_DATA_OUT;
                                    break;
                            }
                            break;

                            case PHS_DATA_OUT:
                                /* write data out to device */
                                dma_bm_read(wd33c93->dma_address,
                                            (char *)sd->cmd_buffer, sd->buffer_length);

                            wd33c93->phase = PHS_STATUS;
                            break;
                        }

skip_write_phase1:
                        /* complete the command */
                        wd33c93_complete_immediate(wd33c93, CSR_XFER_DONE | wd33c93->phase);
                    }
                } else
                    wd33c93_log("WD33c93: Sending data to device with transfer count = 0!. Ignoring...\n");
            }

            /* auto-increment register select if not on special registers */
            if (wd33c93->latch != WD_COMMAND && wd33c93->latch != WD_DATA && wd33c93->latch != WD_AUXILIARY_STATUS)
                wd33c93->latch = (wd33c93->latch + 1) & 0x1f;
        }
        break;

        case 2:
            wd33c93->dma_address = val & 0x1f;
            if((val & 0x02) != 0) {
                wd33c93->dma_address &= 0xfe;
                wd33c93->dma_status = 0;
            }
        break;

        case 3:
            break;
    }
}

static uint8_t
wd33c93_read(uint16_t port, void *priv)
{
    wd33c93_t *wd33c93 = (wd33c93_t *) priv;
    uint8_t ret;

    switch(port & 3) {
        case 0:
        {
            /* read aux status */
            ret = wd33c93->regs[WD_AUXILIARY_STATUS];
        }
        break;

        case 1:
        {
            ret = wd33c93->regs[wd33c93->latch];

            if (wd33c93->latch == WD_TARGET_LUN) {
                if (wd33c93->status == 0x00)
                    ret = 0x00;
                else if (wd33c93->status == 0x02)
                    ret = 0x02;
            } else if (wd33c93->latch == WD_SCSI_STATUS) {
                wd33c93->regs[WD_AUXILIARY_STATUS] &= ~ASR_INT;
                wd33c93_log("WD33c93: Interrupt cleared\n");

                picintc(1 << wd33c93->irq);
            } else if (wd33c93->latch == WD_DATA) {
                wd33c93_log("WD33c93: Read Data register\n");

                wd33c93->dev_id = wd33c93->regs[WD_DESTINATION_ID] & SRCID_MASK;
                wd33c93->lun = (wd33c93->regs[WD_CDB_2] >> 5) & 7;
                scsi_device_t *sd = &scsi_devices[wd33c93->dev_id][wd33c93->lun];

                /* we're going to be doing synchronous reads */
                sd->buffer_length = wd33c93_get_xfer_count(wd33c93);

                /* initialize the return value */
                wd33c93->regs[WD_DATA] = 0;

                if (sd->buffer_length <= 0 && wd33c93->phase == PHS_MESS_IN) {
                    /* move to disconnect */
                    wd33c93->wdstatus = CSR_DISC;
                    wd33c93_complete_cmd(wd33c93);
                } else if (sd->buffer_length == 1 && wd33c93->phase == PHS_STATUS) {
                    /* update the count */
                    wd33c93_set_xfer_count(wd33c93, 0);

                    /* move to message in phase */
                    wd33c93->phase = PHS_MESS_IN;

                    /* complete the command */
                    wd33c93->wdstatus = CSR_XFER_DONE | wd33c93->phase;
                    wd33c93_complete_cmd(wd33c93);
                } else if (sd->buffer_length-- > 0) { /* make sure we still have data to send */
                    if (wd33c93->read_pending) {
                        wd33c93->regs[WD_OWN_ID] = scsi_device_cdb_length(sd);

                        scsi_device_command_phase0(sd, &wd33c93->regs[WD_OWN_ID]);

                        if ((wd33c93->phase == SCSI_PHASE_DATA_IN) ||
                            (wd33c93->phase == SCSI_PHASE_DATA_OUT)) {
                            wd33c93_log("sd->cmd_buffer = %08X\n", sd->cmd_buffer);
                            sd->cmd_buffer = (uint8_t *) malloc(sd->buffer_length);
                            wd33c93_log("sd->cmd_buffer = %08X\n", sd->cmd_buffer);
                        }

                        if (wd33c93->phase == SCSI_PHASE_STATUS)
                            goto skip_read_phase1;

                        scsi_device_command_phase1(sd);

                        wd33c93->read_pending = 0;
                    }

                    wd33c93->regs[WD_AUXILIARY_STATUS] &= ~ASR_INT;

                    dma_bm_write(wd33c93->dma_address,
                            (char *) sd->cmd_buffer, sd->buffer_length);

                    wd33c93->regs[WD_DATA] = wd33c93->dma_address;

skip_read_phase1:
                    /* update the count */
                    wd33c93_set_xfer_count(wd33c93, sd->buffer_length);

                    wd33c93->dma_status |= 0x40;

                    /* transfer finished, see where we're at */
                    if (sd->buffer_length == 0) {
                        if (wd33c93->regs[WD_COMMAND_PHASE] != 0x60) {
                            wd33c93->phase = PHS_STATUS;

                            wd33c93->wdstatus = CSR_XFER_DONE | wd33c93->phase;
                            wd33c93_complete_cmd(wd33c93);
                        } else {
                            wd33c93->regs[WD_AUXILIARY_STATUS] |= ASR_INT;
                            wd33c93->regs[WD_AUXILIARY_STATUS] &= ~ASR_DBR;
                        }
                    }
                }
            }

            wd33c93_log("WD33c93: Read REG=%02x, data %02x\n", wd33c93->latch, ret);

            /* auto-increment register select if not on special registers */
            if (wd33c93->latch != WD_COMMAND && wd33c93->latch != WD_DATA && wd33c93->latch != WD_AUXILIARY_STATUS) {
                wd33c93->latch = (wd33c93->latch + 1) & 0x1f;
            }
        }
        break;

        case 2:
            ret = wd33c93->dma_status | wd33c93->dma_channel;
            break;

        case 3:
            ret = 0xff;
            break;
    }

    wd33c93_log("WD33c93: Read port %03x, ret %x\n", port, ret);

    return (ret);
}

static void *
wd33c93_init(const device_t *info)
{
    /* Allocate control block and set up basic stuff. */
    wd33c93_t *wd33c93 = calloc(1, sizeof(wd33c93_t));

    wd33c93->dma_channel = 7;
    wd33c93->irq         = 11;
    
    io_sethandler(0x0360, 4,
                  wd33c93_read, NULL,NULL,
                  wd33c93_write, NULL,NULL,
                  wd33c93);

// TODO
#if 0
    timer_add(wd33c93_command_poll, &wd33c93->cmd_callback,      &wd33c93->cmd_callback,      wd33c93);
    timer_add(wd33c93_service_poll, &wd33c93->service_req_timer, &wd33c93->service_req_timer, wd33c93);
    timer_add(wd33c93_cip_poll,     &wd33c93->cip_timer,         &wd33c93->cip_timer,         wd33c93);
#else
    timer_add(&wd33c93->cmd_callback,      wd33c93_command_poll, wd33c93, 1);
    timer_add(&wd33c93->service_req_timer, wd33c93_service_poll, wd33c93, 1);
    timer_add(&wd33c93->cip_timer,         wd33c93_cip_poll,     wd33c93, 1);
#endif
    
    return(wd33c93);
}

static void 
wd33c93_close(void *priv)
{
    wd33c93_t *wd33c93 = (wd33c93_t *) priv;

    free(wd33c93);
}

const device_t scsi_wd33c93_device = {
    .name          = "Western Digital WD33c93",
    .internal_name = "wd33c93",
    .flags         = DEVICE_ISA,
    .local         = 0,
    .init          = wd33c93_init,
    .close         = wd33c93_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
