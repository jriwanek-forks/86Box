/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Universal Host Controller Interface (UHCI) implementation.
 *
 *
 *
 * Authors: Fabrice Bellard
 *          Max Krasnyansky
 *          Cacodemon345
 *
 *          Copyright (c) 2005 Fabrice Bellard.
 *          Copyright (c) 2008 Max Krasnyansky
 *          Copyright (c) 2023 Cacodemon345
 */

/* TODO
  1. Implement proper wakeup support.
  2. Implement hardware part of KBC emulation.
*/

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include "cpu.h"
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/dma.h>
#include <86box/fifo8.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/plat_unused.h>
#include <86box/timer.h>
#include <86box/pci.h>
#include <86box/usb.h>

void uhci_detach(UHCIState *s, int index);
void uhci_attach(UHCIState *s, usb_device_t* device, int index);

#define UHCI_CMD_FGR (1 << 4)
#define UHCI_CMD_EGSM (1 << 3)
#define UHCI_CMD_GRESET (1 << 2)
#define UHCI_CMD_HCRESET (1 << 1)
#define UHCI_CMD_RS (1 << 0)

#define UHCI_STS_HCHALTED (1 << 5)
#define UHCI_STS_HCPERR (1 << 4)
#define UHCI_STS_HSERR (1 << 3)
#define UHCI_STS_RD (1 << 2)
#define UHCI_STS_USBERR (1 << 1)
#define UHCI_STS_USBINT (1 << 0)

#define TD_CTRL_SPD (1 << 29)
#define TD_CTRL_ERROR_SHIFT 27
#define TD_CTRL_IOS (1 << 25)
#define TD_CTRL_IOC (1 << 24)
#define TD_CTRL_ACTIVE (1 << 23)
#define TD_CTRL_STALL (1 << 22)
#define TD_CTRL_BABBLE (1 << 20)
#define TD_CTRL_NAK (1 << 19)
#define TD_CTRL_TIMEOUT (1 << 18)

#define UHCI_PORT_SUSPEND (1 << 12)
#define UHCI_PORT_RESET (1 << 9)
#define UHCI_PORT_LSDA (1 << 8)
#define UHCI_PORT_RSVD1 (1 << 7)
#define UHCI_PORT_RD (1 << 6)
#define UHCI_PORT_ENC (1 << 3)
#define UHCI_PORT_EN (1 << 2)
#define UHCI_PORT_CSC (1 << 1)
#define UHCI_PORT_CCS (1 << 0)

#define UHCI_PORT_READ_ONLY (0x1bb)
#define UHCI_PORT_WRITE_CLEAR (UHCI_PORT_CSC | UHCI_PORT_ENC)

#define FRAME_TIMER_FREQ 1000

#define FRAME_MAX_LOOPS 256

/* Must be large enough to handle 10 frame delay for initial isoc requests */
#define QH_VALID 32

typedef struct UHCI_TD {
  uint32_t link;
  uint32_t ctrl; /* see TD_CTRL_xxx */
  uint32_t token;
  uint32_t buffer;
} UHCI_TD;

typedef struct UHCI_QH {
  uint32_t link;
  uint32_t el_link;
} UHCI_QH;

static int is_valid(uint32_t link) { return (link & 1) == 0; }

static int is_qh(uint32_t link) { return (link & 2) != 0; }

static int depth_first(uint32_t link) { return (link & 4) != 0; }

/* QH DB used for detecting QH loops */
#define UHCI_MAX_QUEUES 128
typedef struct {
  uint32_t addr[UHCI_MAX_QUEUES];
  int count;
} QhDb;

static void qhdb_reset(QhDb *db) { db->count = 0; }

/* Add QH to DB. Returns 1 if already present or DB is full. */
static int qhdb_insert(QhDb *db, uint32_t addr) {
  int i;
  for (i = 0; i < db->count; i++)
    if (db->addr[i] == addr)
      return 1;

  if (db->count >= UHCI_MAX_QUEUES)
    return 1;

  db->addr[db->count++] = addr;
  return 0;
}

static void uhci_update_irq(UHCIState *s) {
  usb_t* dev = s->dev;
  int level = 0;
  if (((s->status2 & 1) && (s->intr & (1 << 2))) ||
      ((s->status2 & 2) && (s->intr & (1 << 3))) ||
      ((s->status & UHCI_STS_USBERR) && (s->intr & (1 << 0))) ||
      ((s->status & UHCI_STS_RD) && (s->intr & (1 << 1))) ||
      (s->status & UHCI_STS_HSERR) || (s->status & UHCI_STS_HCPERR)) {
    level = 1;
  }
  pci_irq(dev->params.pci_slot, PCI_INTD, 0, level, &s->irq_state);
}

static void uhci_resume (void *opaque)
{
    UHCIState *s = (UHCIState *)opaque;

    if (!s)
        return;

    if (s->cmd & UHCI_CMD_EGSM) {
        s->cmd |= UHCI_CMD_FGR;
        s->status |= UHCI_STS_RD;
        uhci_update_irq(s);
    }
}

void uhci_reset(usb_t *dev) {
  UHCIState *s = &dev->uhci_state;
  int i = 0;

  s->cmd = 0;
  s->status = UHCI_STS_HCHALTED;
  s->status2 = 0;
  s->intr = 0;
  s->fl_base_addr = 0;
  s->sof_timing = 64;

  for (i = 0; i < 2; i++) {
    s->ports[i].ctrl = 0x0080;
    if (s->ports[i].dev) {
      usb_device_t* usbdev = s->ports[i].dev;
      uhci_detach(s, i);
      uhci_attach(s, usbdev, i);
      s->ports[i].dev->device_reset(s->ports[i].dev->priv);
    }
  }
  uhci_update_irq(s);
}

uint16_t uhci_reg_readw(uint16_t addr, void *priv) {
  const usb_t *dev = (usb_t *)priv;
  uint16_t val;
  const UHCIState *s = &dev->uhci_state;

  addr &= 0x0000001f;

  switch (addr) {
  case 0x00:
    val = s->cmd;
    break;
  case 0x02:
    val = s->status;
    break;
  case 0x04:
    val = s->intr;
    break;
  case 0x06:
    val = s->frnum;
    break;
  case 0x08:
    val = s->fl_base_addr & 0xffff;
    break;
  case 0x0a:
    val = (s->fl_base_addr >> 16) & 0xffff;
    break;
  case 0x0c:
    val = s->sof_timing;
    break;
  case 0x10:
  case 0x12:
  case 0x14:
  case 0x16:
  case 0x18:
  case 0x1a:
  case 0x1c:
  case 0x1e: {
    int n;
    n = (addr >> 1) & 7;
    if (n >= 2)
      goto read_default;
    val = s->ports[n].ctrl;
  } break;
  default:
  read_default:
    val = 0xff7f; /* disabled port */
    break;
  }

  return val;
}

uint8_t uhci_reg_read(uint16_t addr, void *priv) {
  const usb_t *dev = (usb_t *)priv;
  uint8_t ret;
  const UHCIState *s = &dev->uhci_state;

  addr &= 0x0000001f;

  return (uhci_reg_readw(addr & ~1, priv) >> (8 * (addr & 1))) & 0xFF;
}

void uhci_reg_write(uint16_t addr, uint8_t val, void *priv) {
  usb_t *dev = (usb_t *)priv;
  UHCIState *s = &dev->uhci_state;

  addr &= 0x0000001f;

  switch (addr) {
  case 0x00:
    if ((val & UHCI_CMD_RS) && !(s->cmd & UHCI_CMD_RS)) {
      timer_on_auto(&s->frame_timer, (11936 + s->sof_timing) / 12000.);
      s->status &= ~UHCI_STS_HCHALTED;
    } else if (!(val & UHCI_CMD_RS)) {
      timer_disable(&s->frame_timer);
      s->status |= UHCI_STS_HCHALTED;
    }
    if (val & UHCI_CMD_GRESET) {
      UHCIPort *port;
      int i;

      /* send reset on the USB bus */
      for (i = 0; i < 2; i++) {
        port = &s->ports[i];
        if (port->dev)
        port->dev->device_reset(port->dev->priv);
      }
      uhci_reset(dev);
      return;
    }
    if (val & UHCI_CMD_HCRESET) {
      uhci_reset(dev);
      return;
    }
    s->cmd = val | (s->cmd & 0xFF00);
    break;
  case 0x02:
    s->status &= ~val | 0xFF00;
    if (val & UHCI_STS_HCHALTED)
      s->status2 = 0;
    uhci_update_irq(s);
    break;
  case 0x04:
    s->intr = val | (s->intr & 0xFF00);
    uhci_update_irq(s);
    break;
  case 0x09:
    s->fl_base_addr &= 0xFFFF0000;
    s->fl_base_addr |= (val & 0xF0) << 8;
    break;
  case 0x0a:
  case 0x0b:
    s->fl_base_addr &= (addr & 1) ? 0x00FFFFFF : 0xFF00FFFF;
    s->fl_base_addr |= (val) << ((addr & 1) ? 24 : 16);
    break;
  case 0x0c:
    s->sof_timing = val & 0xFF;
    break;

  default:
    break;
  }
}

void uhci_reg_writew(uint16_t addr, uint16_t val, void *priv) {
  usb_t *dev = (usb_t *)priv;
  UHCIState *s = &dev->uhci_state;

  addr &= 0x1F;

  switch (addr) {
  case 0x00:
    if ((val & UHCI_CMD_RS) && !(s->cmd & UHCI_CMD_RS)) {
      timer_on_auto(&s->frame_timer, (11936 + s->sof_timing) / 12000.);
    } else if (!(val & UHCI_CMD_RS)) {
      timer_disable(&s->frame_timer);
      s->status |= UHCI_STS_HCHALTED;
    }
    if (val & UHCI_CMD_GRESET) {
      UHCIPort *port;
      int i;

      /* send reset on the USB bus */
      for (i = 0; i < 2; i++) {
        port = &s->ports[i];
        if (port->dev)
        port->dev->device_reset(port->dev->priv);
      }
      uhci_reset(dev);
      return;
    }
    if (val & UHCI_CMD_HCRESET) {
      uhci_reset(dev);
      return;
    }
    s->cmd = val;
    break;

  case 0x02:
    s->status &= ~val;
    /* XXX: the chip spec is not coherent, so we add a hidden
       register to distinguish between IOC and SPD */
    if (val & UHCI_STS_USBINT)
      s->status2 = 0;
    uhci_update_irq(s);
    break;
  case 0x04:
    s->intr = val;
    uhci_update_irq(s);
    break;
  case 0x06:
    if (s->status & UHCI_STS_HCHALTED)
      s->frnum = val & 0x7ff;
    break;
  case 0x08:
    s->fl_base_addr &= 0xffff0000;
    s->fl_base_addr |= val & ~0xfff;
    break;
  case 0x0a:
    s->fl_base_addr &= 0x0000ffff;
    s->fl_base_addr |= (val << 16);
    break;
  case 0x0c:
    s->sof_timing = val & 0xff;
    break;

  case 0x10:
  case 0x12:
  case 0x14:
  case 0x16:
  case 0x18:
  case 0x1a:
  case 0x1c:
  case 0x1e: {
    UHCIPort *port;
    usb_device_t *dev;
    int n;

    n = (addr >> 1) & 7;
    if (n >= 2)
      return;
    port = &s->ports[n];
    dev = port->dev;
    if (dev) {
      /* port reset */
      if ((val & UHCI_PORT_RESET) && !(port->ctrl & UHCI_PORT_RESET)) {
        dev->device_reset(dev->priv);
      }
    }
    port->ctrl &= UHCI_PORT_READ_ONLY;
    /* enabled may only be set if a device is connected */
    if (!(port->ctrl & UHCI_PORT_CCS)) {
      val &= ~UHCI_PORT_EN;
    }
    port->ctrl |= (val & ~UHCI_PORT_READ_ONLY);
    /* some bits are reset when a '1' is written to them */
    port->ctrl &= ~(val & UHCI_PORT_WRITE_CLEAR);
  } break;
  }
}

static void uhci_read_td(UHCIState *s, UHCI_TD *td, uint32_t link) {
  dma_bm_read(link & ~0xf, (uint8_t *)td, sizeof(UHCI_TD), 4);
}

static inline int32_t uhci_queue_token(UHCI_TD *td) {
  if ((td->token & (0xf << 15)) == 0) {
    /* ctrl ep, cover ep and dev, not pid! */
    return td->token & 0x7ff00;
  } else {
    /* covers ep, dev, pid -> identifies the endpoint */
    return td->token & 0x7ffff;
  }
}

enum {
  TD_RESULT_STOP_FRAME = 10,
  TD_RESULT_COMPLETE,
  TD_RESULT_NEXT_QH,
  TD_RESULT_ASYNC_START,
  TD_RESULT_ASYNC_CONT,
};

static int uhci_handle_td_error(UHCIState *s, UHCI_TD *td, uint32_t td_addr,
                                int status, uint32_t *int_mask) {
  uint32_t queue_token = uhci_queue_token(td);
  int ret;

  //pclog("UHCI ERROR %d\n", status);
  switch (status) {
  case USB_ERROR_NAK:
    td->ctrl |= TD_CTRL_NAK;
    return TD_RESULT_NEXT_QH;

  case USB_ERROR_STALL:
    td->ctrl |= TD_CTRL_STALL;
    ret = TD_RESULT_NEXT_QH;
    break;

  case USB_ERROR_OVERRUN:
    td->ctrl |= TD_CTRL_BABBLE | TD_CTRL_STALL;
    /* frame interrupted */
    ret = TD_RESULT_STOP_FRAME;
    break;

  default:
    td->ctrl |= TD_CTRL_TIMEOUT;
    td->ctrl &= ~(3 << TD_CTRL_ERROR_SHIFT);
    ret = TD_RESULT_NEXT_QH;
    break;
  }

  td->ctrl &= ~TD_CTRL_ACTIVE;
  s->status |= UHCI_STS_USBERR;
  if (td->ctrl & TD_CTRL_IOC) {
    *int_mask |= 0x01;
  }
  uhci_update_irq(s);
  return ret;
}

static int uhci_complete_td(UHCIState *s, UHCI_TD *td, int status,
                            uint8_t *data, uint32_t pktlen, uint32_t td_addr,
                            uint32_t *int_mask) {
  int len = 0, max_len;
  uint8_t pid;

  max_len = ((td->token >> 21) + 1) & 0x7ff;
  pid = td->token & 0xff;

  if (td->ctrl & TD_CTRL_IOS)
    td->ctrl &= ~TD_CTRL_ACTIVE;

  if (status != USB_ERROR_NO_ERROR)
    return uhci_handle_td_error(s, td, td_addr, status, int_mask);

  len = pktlen;
  td->ctrl = (td->ctrl & ~0x7ff) | ((len - 1) & 0x7ff);

  /* The NAK bit may have been set by a previous frame, so clear it
     here.  The docs are somewhat unclear, but win2k relies on this
     behavior.  */
  td->ctrl &= ~(TD_CTRL_ACTIVE | TD_CTRL_NAK);
  if (td->ctrl & TD_CTRL_IOC)
    *int_mask |= 0x01;

  if (pid == USB_PID_IN) {
    dma_bm_write(td->buffer, (const uint8_t *)data, len, 1);
    if ((td->ctrl & TD_CTRL_SPD) && len < max_len) {
      *int_mask |= 0x02;
      return TD_RESULT_NEXT_QH;
    }
  }
  return TD_RESULT_COMPLETE;
}

static usb_device_t *uhci_find_device(UHCIState *s, uint8_t addr) {
  int i = 0;
  //pclog("Looking for device with addr %d\n", addr);
  for (i = 0; i < 2; i++) {
    if (!(s->ports[i].ctrl & UHCI_PORT_EN))
      continue;
    if (!s->ports[i].dev)
      continue;
    if (s->ports[i].dev->address == addr)
      return s->ports[i].dev;
  }
  //pclog("None found.\n");
  return NULL;
}

static int uhci_handle_td(UHCIState *s, uint32_t qh_addr, UHCI_TD *td,
                          uint32_t td_addr, uint32_t *int_mask) {
  int ret = 0;
  uint8_t result = USB_ERROR_NO_ERROR;
  uint32_t max_len = 0;
  bool spd = false;
  uint8_t pid = td->token & 0xff;
  usb_device_t *dev = NULL;
  uint8_t *buf = NULL;

  /* Is active ? */
  if (!(td->ctrl & TD_CTRL_ACTIVE)) {
    if (td->ctrl & TD_CTRL_IOC) {
      *int_mask |= 0x01;
    }
    return TD_RESULT_NEXT_QH;
  }
  switch (pid) {
  case USB_PID_OUT:
  case USB_PID_SETUP:
  case USB_PID_IN:
    break;
  default:
    /* invalid pid : frame interrupted */
    s->status |= UHCI_STS_HCPERR;
    s->cmd &= ~UHCI_CMD_RS;
    uhci_update_irq(s);
    return TD_RESULT_STOP_FRAME;
  }

  dev = uhci_find_device(s, (td->token >> 8) & 0x7f);
  if (dev == NULL) {
    return uhci_handle_td_error(s, td, td_addr, -1, int_mask);
  }
  max_len = ((td->token >> 21) + 1) & 0x7ff;
  spd = (pid == USB_PID_IN && (td->ctrl & TD_CTRL_SPD) != 0);
  buf = calloc(1, max_len);

  switch (pid) {
  case USB_PID_OUT:
  case USB_PID_SETUP: {
      dma_bm_read(td->buffer, (uint8_t *)buf, max_len, 4);
      result = dev->device_process(dev->priv, buf, &max_len, pid,
                                   (td->token >> 15) & 0xf, spd);
      break;
    }
  case USB_PID_IN:
    result = dev->device_process(dev->priv, buf, &max_len, pid,
                                 (td->token >> 15) & 0xf, spd);
    break;
  }
  ret = uhci_complete_td(s, td, result, buf, max_len, td_addr, int_mask);
  return ret;
}

static void uhci_process_frame(UHCIState *s) {
  uint32_t frame_addr, link, old_td_ctrl, int_mask;
  uint32_t curr_qh, td_count = 0;
  int cnt, ret;
  UHCI_TD td;
  UHCI_QH qh;
  QhDb qhdb = { { 0 }, 0};

  frame_addr = (s->fl_base_addr & ~0xfff) | ((s->frnum & 0x3ff) << 2);
  dma_bm_read(frame_addr, (uint8_t *)&link, 4, 4);
  int_mask = 0;
  curr_qh = 0;

  for (cnt = FRAME_MAX_LOOPS; is_valid(link) && cnt; cnt--) {
    if (s->frame_bytes >= 1280) {
      break;
    }
    if (is_qh(link)) {
      /* QH */
      if (qhdb_insert(&qhdb, link)) {
        if (td_count == 0) {
          break;
        } else {
          td_count = 0;
          qhdb_reset(&qhdb);
          qhdb_insert(&qhdb, link);
        }
      }
      dma_bm_read(link & ~0xf, (uint8_t *)&qh, sizeof(qh), 4);
      if (!is_valid(qh.el_link)) {
        /* QH w/o elements */
        curr_qh = 0;
        link = qh.link;
      } else {
        /* QH with elements */
        curr_qh = link;
        link = qh.el_link;
      }
      continue;
    }
    /* TD */
    uhci_read_td(s, &td, link);
    old_td_ctrl = td.ctrl;
    ret = uhci_handle_td(s, curr_qh, &td, link, &int_mask);

    if (old_td_ctrl != td.ctrl) {
      dma_bm_write((link & ~0xf) + 4, (uint8_t *)&td.ctrl, sizeof(td.ctrl), 4);
    }

    switch (ret) {
    case TD_RESULT_STOP_FRAME: /* interrupted frame */
      goto out;

    case TD_RESULT_NEXT_QH:
    case TD_RESULT_ASYNC_CONT:
      link = curr_qh ? qh.link : td.link;
      continue;

    case TD_RESULT_ASYNC_START:
      link = curr_qh ? qh.link : td.link;
      continue;

    case TD_RESULT_COMPLETE:
        link = td.link;
        td_count++;
        s->frame_bytes += (td.ctrl & 0x7ff) + 1;
        if (curr_qh) {
            qh.el_link = link;
            dma_bm_write((curr_qh & ~0xf) + 4, (uint8_t*)&qh.el_link, sizeof(qh.el_link), 4);
            if (!depth_first(link)) {
                /* done with this QH */
                curr_qh = 0;
                link    = qh.link;
            }
        }
        break;
    }
  }

out:
  s->pending_int_mask |= int_mask;
}

void uhci_frame_timer(void *opaque) {
  UHCIState *s = opaque;
  int i;

  if (!(s->cmd & UHCI_CMD_RS)) {
    timer_disable(&s->frame_timer);
    s->status |= UHCI_STS_HCHALTED;
    return;
  }

  s->frame_bytes = 0;
  uhci_process_frame(s);
  s->frnum = (s->frnum + 1) & 0x7ff;

  if (s->pending_int_mask) {
    s->status2 |= s->pending_int_mask;
    s->status |= UHCI_STS_USBINT;
    uhci_update_irq(s);
  }
  s->pending_int_mask = 0;

  timer_on_auto(&s->frame_timer, (11936 + s->sof_timing) / 12000.);
}

void uhci_attach(UHCIState *s, usb_device_t* device, int index)
{
    UHCIPort *port = &s->ports[index];

    /* set connect status */
    port->ctrl |= UHCI_PORT_CCS | UHCI_PORT_CSC;

    /* update speed */
    {
        //port->ctrl |= UHCI_PORT_LSDA;
    }
    port->dev = device;
    uhci_resume(s);
}

void uhci_detach(UHCIState *s, int index)
{
    UHCIPort *port = &s->ports[index];

    /* set connect status */
    if (port->ctrl & UHCI_PORT_CCS) {
        port->ctrl &= ~UHCI_PORT_CCS;
        port->ctrl |= UHCI_PORT_CSC;
    }
    /* disable port */
    if (port->ctrl & UHCI_PORT_EN) {
        port->ctrl &= ~UHCI_PORT_EN;
        port->ctrl |= UHCI_PORT_ENC;
    }
    port->dev = NULL;
    uhci_resume(s);
}
