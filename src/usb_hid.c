/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          USB HID mouse, keyboard and tablet, written from the Device
 *          Class Definition for Human Interface Devices (HID) Version
 *          1.11 (2001), the HID Usage Tables Version 1.12 (October 28,
 *          2004), the Universal Serial Bus Specification Revision 1.1
 *          (September 23, 1998) and Microsoft's USB HID to PS/2 Scan
 *          Code Translation Table (revised April 2, 2004): a low-speed
 *          boot-interface mouse with three buttons and a wheel, a
 *          low-speed boot-interface keyboard, a full-speed absolute
 *          pointer with three buttons and a wheel that follows the host
 *          pointer, and the Creative GamePad Cobra 2 USB.
 *
 * Authors: skiretic, <skiretic@proton.me>
 *
 *          Copyright 2026 skiretic.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/mouse.h>
#include <86box/keyboard.h>
#include <86box/thread.h>
#include <86box/video.h>
#include <86box/usb_dev.h>
#include <86box/gameport.h>
#include <86box/plat_unused.h>

#define HID_GET_REPORT   0x01
#define HID_GET_IDLE     0x02
#define HID_GET_PROTOCOL 0x03
#define HID_SET_REPORT   0x09
#define HID_SET_IDLE     0x0a
#define HID_SET_PROTOCOL 0x0b

/* Idle rate state for SET_IDLE and GET_IDLE (HID 1.11 s7.2.4, p. 53 / PDF 63). */
typedef struct hid_idle_t {
    uint8_t  rate;    /* in 4 ms units; 0 = report only on a change */
    uint8_t  next;    /* a rate that takes effect after the current period's report */
    uint8_t  pending; /* next is waiting */
    uint64_t last_us; /* time of the last report on the interrupt pipe */
} hid_idle_t;

/* Emulated time in microseconds. TIMER_USEC is the number of tsc ticks per
   microsecond, scaled by 2^32. */
static uint64_t
hid_now_us(void)
{
    return (uint64_t) ((double) tsc * 4294967296.0 / (double) TIMER_USEC);
}

/* A new idle rate acts as if it had been set right after the last report,
   unless it arrives within 4 ms of the end of the running period; then it
   waits until that period's report has gone out (HID 1.11 s7.2.4, Latency,
   p. 53 / PDF 63). With rate 0 no period is running. If the time already
   past the last report exceeds a new, shorter rate, hid_idle_due() is true
   at the next poll, which gives the immediate report the spec asks for. */
static void
hid_idle_set(hid_idle_t *idle, uint8_t rate)
{
    if ((idle->rate != 0) && ((hid_now_us() + 4000ULL) > (idle->last_us + (uint64_t) idle->rate * 4000ULL))) {
        idle->next    = rate;
        idle->pending = 1;
    } else {
        idle->rate    = rate;
        idle->pending = 0;
    }
}

static void
hid_idle_reported(hid_idle_t *idle)
{
    idle->last_us = hid_now_us();
    if (idle->pending) {
        idle->rate    = idle->next;
        idle->pending = 0;
    }
}

/* True when a full idle period has passed since the last report: the device
   sends its current report again even though nothing changed (HID 1.11
   s7.2.4, p. 53 / PDF 63). */
static int
hid_idle_due(const hid_idle_t *idle)
{
    return (idle->rate != 0) && ((hid_now_us() - idle->last_us) >= ((uint64_t) idle->rate * 4000ULL));
}

/* The boot mouse report descriptor (HID 1.11 Appendix B.2, p. 61 / PDF 71)
   with Report Count 3 instead of 2 on the axes, which adds a Wheel byte
   after X and Y. The first three bytes keep the boot layout, which is all
   a BIOS reads (HID 1.11 Appendix B, p. 59 / PDF 69). */
static const uint8_t hid_mouse_report_desc[] = {
    0x05, 0x01, /* Usage Page (Generic Desktop) */
    0x09, 0x02, /* Usage (Mouse) */
    0xa1, 0x01, /* Collection (Application) */
    0x09, 0x01, /*   Usage (Pointer) */
    0xa1, 0x00, /*   Collection (Physical) */
    0x95, 0x03, /*     Report Count (3) */
    0x75, 0x01, /*     Report Size (1) */
    0x05, 0x09, /*     Usage Page (Button) */
    0x19, 0x01, /*     Usage Minimum (1) */
    0x29, 0x03, /*     Usage Maximum (3) */
    0x15, 0x00, /*     Logical Minimum (0) */
    0x25, 0x01, /*     Logical Maximum (1) */
    0x81, 0x02, /*     Input (Data, Variable, Absolute) */
    0x95, 0x01, /*     Report Count (1) */
    0x75, 0x05, /*     Report Size (5) */
    0x81, 0x01, /*     Input (Constant) */
    0x75, 0x08, /*     Report Size (8) */
    0x95, 0x03, /*     Report Count (3) */
    0x05, 0x01, /*     Usage Page (Generic Desktop) */
    0x09, 0x30, /*     Usage (X) */
    0x09, 0x31, /*     Usage (Y) */
    0x09, 0x38, /*     Usage (Wheel) */
    0x15, 0x81, /*     Logical Minimum (-127) */
    0x25, 0x7f, /*     Logical Maximum (127) */
    0x81, 0x06, /*     Input (Data, Variable, Relative) */
    0xc0,       /*   End Collection */
    0xc0        /* End Collection */
};

static const uint8_t hid_mouse_dev_desc[18] = {
    18, 0x01,   /* bLength, DEVICE */
    0x10, 0x01, /* bcdUSB 1.10 */
    0x00, 0x00, 0x00,
    8,          /* bMaxPacketSize0: the only size allowed at low speed (USB 1.1 s5.5.3, p. 37 / PDF 53) */
    0x00, 0x00, /* idVendor */
    0x01, 0x00, /* idProduct */
    0x00, 0x01, /* bcdDevice */
    1, 2, 0,    /* iManufacturer, iProduct, iSerialNumber */
    1           /* bNumConfigurations */
};

#define HID_MOUSE_CFG_LEN (9 + 9 + 9 + 7)

static const uint8_t hid_mouse_cfg_desc[HID_MOUSE_CFG_LEN] = {
    /* Configuration */
    9, 0x02, HID_MOUSE_CFG_LEN, 0x00, 1, 1, 0,
    0x80, /* bus powered, no remote wakeup */
    50,   /* 100 mA, in 2 mA units */
    /* Interface 0: HID, boot subclass, mouse protocol (HID 1.11 s4.2-4.3, p. 8-9 / PDF 18-19) */
    9, 0x04, 0, 0, 1, 0x03, 0x01, 0x02, 0,
    /* HID descriptor, between interface and endpoint (HID 1.11 s7.1, p. 48 / PDF 58) */
    9, 0x21, 0x11, 0x01, 0, 1, 0x22, sizeof(hid_mouse_report_desc), 0x00,
    /* Endpoint 1 IN, interrupt, 4 bytes, 10 ms: the shortest period a
       low-speed endpoint may ask for (USB 1.1 s5.7.4, p. 45 / PDF 61) */
    7, 0x05, 0x81, 0x03, 4, 0x00, 10
};

static const char *const hid_mouse_strings[] = {
    "86Box",
    "USB Mouse"
};

typedef struct hid_mouse_t {
    usb_dev_t  usb;
    uint8_t    protocol; /* 0 boot, 1 report */
    hid_idle_t idle;
    uint8_t    buttons;  /* the buttons in the last report */
} hid_mouse_t;

static int
hid_mouse_report(hid_mouse_t *dev, uint8_t *buf)
{
    int dx;
    int dy;
    int dz;
    int ox;
    int oy;

    /* HID Y grows as the mouse moves toward the user, which is down the
       screen, as the host's Y does. Wheel values grow as the wheel rolls away
       from the user (HID Usage Tables 1.12 s4.2-4.3, p. 29 / PDF 29), as
       86Box's wheel count does. In tablet mode the host wheel goes to the
       tablet, so the mouse reports 0. */
    mouse_subtract_coords(&dx, &dy, &ox, &oy, -127, 127, 0, 0);
    dz = 0;
    if (mouse_input_mode == 0)
        mouse_subtract_z(&dz, -127, 127, 0);
    dev->buttons = mouse_get_buttons_ex() & 0x07;

    buf[0] = dev->buttons;
    buf[1] = (uint8_t) dx;
    buf[2] = (uint8_t) dy;
    buf[3] = (uint8_t) dz;

    hid_idle_reported(&dev->idle);

    /* Boot protocol sends only the three bytes of the boot layout (HID 1.11
       Appendix B, p. 59 / PDF 69); report protocol adds the wheel byte. */
    return dev->protocol ? 4 : 3;
}

static int
hid_mouse_control(usb_dev_t *usb, const uint8_t *setup, uint8_t *data)
{
    hid_mouse_t *dev    = (hid_mouse_t *) usb->priv;
    uint16_t     wvalue = setup[2] | (setup[3] << 8);
    uint16_t     windex = setup[4] | (setup[5] << 8);

    /* Every request here is addressed to interface 0, the only one. */
    if (windex != 0)
        return USB_DATA_STALL;

    /* GET_DESCRIPTOR for a HID class descriptor, addressed to the interface
       (HID 1.11 s7.1.1, p. 49 / PDF 59). There is one HID and one Report
       descriptor, so any index but 0 STALLs. The HID descriptor is the copy
       inside the configuration descriptor, at offset 18. */
    if ((setup[0] == 0x81) && (setup[1] == 0x06)) {
        if ((wvalue & 0xff) != 0)
            return USB_DATA_STALL;
        switch (wvalue >> 8) {
            case 0x21:
                memcpy(data, &hid_mouse_cfg_desc[18], 9);
                return 9;
            case 0x22:
                memcpy(data, hid_mouse_report_desc, sizeof(hid_mouse_report_desc));
                return sizeof(hid_mouse_report_desc);
            default:
                return USB_DATA_STALL;
        }
    }

    if (setup[0] == 0xa1) {
        switch (setup[1]) {
            case HID_GET_REPORT:
                /* wValue is report type (high) and report ID (low). The
                   only report is Input with no report ID: type 1, ID 0
                   (HID 1.11 s7.2.1, p. 51 / PDF 61). */
                if (wvalue != 0x0100)
                    return USB_DATA_STALL;
                return hid_mouse_report(dev, data);
            case HID_GET_IDLE:
                /* The device uses no report IDs, so a nonzero ID names no
                   report (HID 1.11 s7.2.3, p. 52 / PDF 62). */
                if ((wvalue & 0xff) != 0)
                    return USB_DATA_STALL;
                data[0] = dev->idle.rate;
                return 1;
            case HID_GET_PROTOCOL:
                data[0] = dev->protocol;
                return 1;
            default:
                return USB_DATA_STALL;
        }
    }

    if (setup[0] == 0x21) {
        switch (setup[1]) {
            case HID_SET_IDLE:
                /* No report IDs, so a nonzero ID names no report; the high
                   byte is the duration (HID 1.11 s7.2.4, p. 52-53 / PDF 62-63). */
                if ((wvalue & 0xff) != 0)
                    return USB_DATA_STALL;
                hid_idle_set(&dev->idle, wvalue >> 8);
                return 0;
            case HID_SET_PROTOCOL:
                if (wvalue > 1)
                    return USB_DATA_STALL;
                dev->protocol = wvalue;
                return 0;
            default:
                /* SET_REPORT included: the mouse has no Output or Feature report. */
                return USB_DATA_STALL;
        }
    }

    return USB_DATA_STALL;
}

/* Interrupt IN: a report when motion, wheel or buttons changed or when the
   idle period has run out, otherwise NAK (HID 1.11 s7.2.4, p. 53 / PDF 63). */
static int
hid_mouse_data_in(usb_dev_t *usb, UNUSED(int ep), uint8_t *buf, UNUSED(int maxlen))
{
    hid_mouse_t *dev = (hid_mouse_t *) usb->priv;

    if (mouse_state_changed() || ((mouse_get_buttons_ex() & 0x07) != dev->buttons))
        return hid_mouse_report(dev, buf);

    if (hid_idle_due(&dev->idle))
        return hid_mouse_report(dev, buf);

    return USB_DATA_NAK;
}

static void
hid_mouse_reset(usb_dev_t *usb)
{
    hid_mouse_t *dev = (hid_mouse_t *) usb->priv;

    /* Devices start in report protocol (HID 1.11 s7.2.6, p. 54 / PDF 64).
       The recommended default idle rate for mice is infinite, rate 0
       (s7.2.4, p. 53 / PDF 63). */
    dev->protocol     = 1;
    dev->idle.rate    = 0;
    dev->idle.pending = 0;

    /* Host motion keeps adding up while nothing reads it, as when the
       mouse is unplugged. A mouse that was not on the bus saw none of it,
       so it is dropped rather than sent after the next enumeration. */
    mouse_clear_coords();
}

static const usb_dev_ops_t hid_mouse_ops = {
    .control    = hid_mouse_control,
    .data_in    = hid_mouse_data_in,
    .data_out   = NULL,
    .reset      = hid_mouse_reset,
    .configured = NULL
};

static void *
hid_mouse_init(const device_t *info)
{
    hid_mouse_t *dev = (hid_mouse_t *) calloc(1, sizeof(hid_mouse_t));

    dev->usb.name      = "USB Mouse";
    dev->usb.low_speed = 1;
    dev->usb.dev_desc  = hid_mouse_dev_desc;
    dev->usb.cfg_desc  = hid_mouse_cfg_desc;
    dev->usb.cfg_len   = HID_MOUSE_CFG_LEN;
    dev->usb.strings   = hid_mouse_strings;
    dev->usb.n_strings = 2;
    dev->usb.ops       = &hid_mouse_ops;
    dev->usb.priv      = dev;
    usb_dev_init(&dev->usb);
    hid_mouse_reset(&dev->usb);

    /* Three buttons, but mouse_state_changed() counts wheel motion only when
       4 or more are declared. */
    mouse_set_buttons(4);

    usb_dev_add(&dev->usb, info);

    return dev;
}

static void
hid_mouse_close(void *priv)
{
    hid_mouse_t *dev = (hid_mouse_t *) priv;

    usb_dev_remove(&dev->usb);
    free(dev);
}

const device_t mouse_usb_device = {
    .name          = "USB Mouse",
    .internal_name = "usb_mouse",
    .flags         = DEVICE_USB,
    .local         = 0,
    .init          = hid_mouse_init,
    .close         = hid_mouse_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

/* The boot keyboard report descriptor (HID 1.11 Appendix B.1, p. 59-60 /
   PDF 69-70). Item data is two's complement by its size, as in the
   one-byte Logical Minimum 81h = -127 of the spec's own mouse example
   (Appendix E.10, p. 71 / PDF 81), so a one-byte Logical Maximum FFh would
   mean -1; the array's 255 is written in two bytes. */
static const uint8_t hid_kbd_report_desc[] = {
    0x05, 0x01,       /* Usage Page (Generic Desktop) */
    0x09, 0x06,       /* Usage (Keyboard) */
    0xa1, 0x01,       /* Collection (Application) */
    0x75, 0x01,       /*   Report Size (1) */
    0x95, 0x08,       /*   Report Count (8) */
    0x05, 0x07,       /*   Usage Page (Key Codes) */
    0x19, 0xe0,       /*   Usage Minimum (224) */
    0x29, 0xe7,       /*   Usage Maximum (231) */
    0x15, 0x00,       /*   Logical Minimum (0) */
    0x25, 0x01,       /*   Logical Maximum (1) */
    0x81, 0x02,       /*   Input (Data, Variable, Absolute): modifier byte */
    0x95, 0x01,       /*   Report Count (1) */
    0x75, 0x08,       /*   Report Size (8) */
    0x81, 0x01,       /*   Input (Constant): reserved byte */
    0x95, 0x05,       /*   Report Count (5) */
    0x75, 0x01,       /*   Report Size (1) */
    0x05, 0x08,       /*   Usage Page (LEDs) */
    0x19, 0x01,       /*   Usage Minimum (1) */
    0x29, 0x05,       /*   Usage Maximum (5) */
    0x91, 0x02,       /*   Output (Data, Variable, Absolute): LED report */
    0x95, 0x01,       /*   Report Count (1) */
    0x75, 0x03,       /*   Report Size (3) */
    0x91, 0x01,       /*   Output (Constant): padding */
    0x95, 0x06,       /*   Report Count (6) */
    0x75, 0x08,       /*   Report Size (8) */
    0x15, 0x00,       /*   Logical Minimum (0) */
    0x26, 0xff, 0x00, /*   Logical Maximum (255) */
    0x05, 0x07,       /*   Usage Page (Key Codes) */
    0x19, 0x00,       /*   Usage Minimum (0) */
    0x2a, 0xff, 0x00, /*   Usage Maximum (255) */
    0x81, 0x00,       /*   Input (Data, Array) */
    0xc0              /* End Collection */
};

static const uint8_t hid_kbd_dev_desc[18] = {
    18, 0x01,   /* bLength, DEVICE */
    0x10, 0x01, /* bcdUSB 1.10 */
    0x00, 0x00, 0x00,
    8,          /* bMaxPacketSize0: the only size allowed at low speed (USB 1.1 s5.5.3, p. 37 / PDF 53) */
    0x00, 0x00, /* idVendor */
    0x02, 0x00, /* idProduct */
    0x00, 0x01, /* bcdDevice */
    1, 2, 0,    /* iManufacturer, iProduct, iSerialNumber */
    1           /* bNumConfigurations */
};

#define HID_KBD_CFG_LEN (9 + 9 + 9 + 7)

static const uint8_t hid_kbd_cfg_desc[HID_KBD_CFG_LEN] = {
    /* Configuration */
    9, 0x02, HID_KBD_CFG_LEN, 0x00, 1, 1, 0,
    0x80, /* bus powered, no remote wakeup */
    50,   /* 100 mA, in 2 mA units */
    /* Interface 0: HID, boot subclass, keyboard protocol (HID 1.11 s4.2-4.3, p. 8-9 / PDF 18-19) */
    9, 0x04, 0, 0, 1, 0x03, 0x01, 0x01, 0,
    /* HID descriptor, between interface and endpoint (HID 1.11 s7.1, p. 48 / PDF 58) */
    9, 0x21, 0x11, 0x01, 0, 1, 0x22, sizeof(hid_kbd_report_desc), 0x00,
    /* Endpoint 1 IN, interrupt, 8 bytes, 10 ms: the shortest period a
       low-speed endpoint may ask for (USB 1.1 s5.7.4, p. 45 / PDF 61) */
    7, 0x05, 0x81, 0x03, 8, 0x00, 10
};

static const char *const hid_kbd_strings[] = {
    "86Box",
    "USB Keyboard"
};

/* 86Box key codes (scan code set 1 make codes, E0 xx as 1xxh, the E1 1D
   that starts Pause as 100h) to Keyboard/Keypad page usages (HID Usage Tables 1.12 s10, p. 53 /
   PDF 53), taken from Microsoft's USB HID to PS/2 Scan Code Translation
   Table (2004). 0 = no Keyboard/Keypad page usage. Two host keys can share
   a usage: SysRq and Print Screen, Pause and Break. */
static const uint8_t hid_kbd_usage[0x200] = {
    [0x001] = 0x29, /* Escape */
    [0x002] = 0x1e, /* 1 */
    [0x003] = 0x1f, /* 2 */
    [0x004] = 0x20, /* 3 */
    [0x005] = 0x21, /* 4 */
    [0x006] = 0x22, /* 5 */
    [0x007] = 0x23, /* 6 */
    [0x008] = 0x24, /* 7 */
    [0x009] = 0x25, /* 8 */
    [0x00a] = 0x26, /* 9 */
    [0x00b] = 0x27, /* 0 */
    [0x00c] = 0x2d, /* - */
    [0x00d] = 0x2e, /* = */
    [0x00e] = 0x2a, /* Backspace */
    [0x00f] = 0x2b, /* Tab */
    [0x010] = 0x14, /* Q */
    [0x011] = 0x1a, /* W */
    [0x012] = 0x08, /* E */
    [0x013] = 0x15, /* R */
    [0x014] = 0x17, /* T */
    [0x015] = 0x1c, /* Y */
    [0x016] = 0x18, /* U */
    [0x017] = 0x0c, /* I */
    [0x018] = 0x12, /* O */
    [0x019] = 0x13, /* P */
    [0x01a] = 0x2f, /* [ */
    [0x01b] = 0x30, /* ] */
    [0x01c] = 0x28, /* Return */
    [0x01d] = 0xe0, /* Left Control */
    [0x01e] = 0x04, /* A */
    [0x01f] = 0x16, /* S */
    [0x020] = 0x07, /* D */
    [0x021] = 0x09, /* F */
    [0x022] = 0x0a, /* G */
    [0x023] = 0x0b, /* H */
    [0x024] = 0x0d, /* J */
    [0x025] = 0x0e, /* K */
    [0x026] = 0x0f, /* L */
    [0x027] = 0x33, /* ; */
    [0x028] = 0x34, /* ' */
    [0x029] = 0x35, /* ` */
    [0x02a] = 0xe1, /* Left Shift */
    [0x02b] = 0x31, /* \ (also Europe 1, usage 32h) */
    [0x02c] = 0x1d, /* Z */
    [0x02d] = 0x1b, /* X */
    [0x02e] = 0x06, /* C */
    [0x02f] = 0x19, /* V */
    [0x030] = 0x05, /* B */
    [0x031] = 0x11, /* N */
    [0x032] = 0x10, /* M */
    [0x033] = 0x36, /* , */
    [0x034] = 0x37, /* . */
    [0x035] = 0x38, /* / */
    [0x036] = 0xe5, /* Right Shift */
    [0x037] = 0x55, /* Keypad * */
    [0x038] = 0xe2, /* Left Alt */
    [0x039] = 0x2c, /* Space */
    [0x03a] = 0x39, /* Caps Lock */
    [0x03b] = 0x3a, /* F1 */
    [0x03c] = 0x3b, /* F2 */
    [0x03d] = 0x3c, /* F3 */
    [0x03e] = 0x3d, /* F4 */
    [0x03f] = 0x3e, /* F5 */
    [0x040] = 0x3f, /* F6 */
    [0x041] = 0x40, /* F7 */
    [0x042] = 0x41, /* F8 */
    [0x043] = 0x42, /* F9 */
    [0x044] = 0x43, /* F10 */
    [0x045] = 0x53, /* Num Lock */
    [0x046] = 0x47, /* Scroll Lock */
    [0x047] = 0x5f, /* Keypad 7 */
    [0x048] = 0x60, /* Keypad 8 */
    [0x049] = 0x61, /* Keypad 9 */
    [0x04a] = 0x56, /* Keypad - */
    [0x04b] = 0x5c, /* Keypad 4 */
    [0x04c] = 0x5d, /* Keypad 5 */
    [0x04d] = 0x5e, /* Keypad 6 */
    [0x04e] = 0x57, /* Keypad + */
    [0x04f] = 0x59, /* Keypad 1 */
    [0x050] = 0x5a, /* Keypad 2 */
    [0x051] = 0x5b, /* Keypad 3 */
    [0x052] = 0x62, /* Keypad 0 */
    [0x053] = 0x63, /* Keypad . */
    [0x054] = 0x46, /* SysRq: Alt + Print Screen */
    [0x056] = 0x64, /* Europe 2 */
    [0x057] = 0x44, /* F11 */
    [0x058] = 0x45, /* F12 */
    [0x059] = 0x67, /* Keypad = */
    [0x05c] = 0x8c, /* International 6 */
    [0x064] = 0x68, /* F13 */
    [0x065] = 0x69, /* F14 */
    [0x066] = 0x6a, /* F15 */
    [0x067] = 0x6b, /* F16 */
    [0x068] = 0x6c, /* F17 */
    [0x069] = 0x6d, /* F18 */
    [0x06a] = 0x6e, /* F19 */
    [0x06b] = 0x6f, /* F20 */
    [0x06c] = 0x70, /* F21 */
    [0x06d] = 0x71, /* F22 */
    [0x06e] = 0x72, /* F23 */
    [0x070] = 0x88, /* International 2 (Katakana/Hiragana) */
    [0x073] = 0x87, /* International 1 (Ro) */
    [0x076] = 0x73, /* F24 (also LANG5, usage 94h) */
    [0x077] = 0x93, /* LANG4 (Hiragana) */
    [0x078] = 0x92, /* LANG3 (Katakana) */
    [0x079] = 0x8a, /* International 4 (Henkan) */
    [0x07b] = 0x8b, /* International 5 (Muhenkan) */
    [0x07d] = 0x89, /* International 3 (Yen) */
    [0x07e] = 0x85, /* Keypad , (Brazil) */
    [0x0f1] = 0x91, /* LANG2 (Hanja) */
    [0x0f2] = 0x90, /* LANG1 (Hangul/English) */
    [0x100] = 0x48, /* Pause (E1 1D 45) */
    [0x11c] = 0x58, /* Keypad Enter */
    [0x11d] = 0xe4, /* Right Control */
    [0x135] = 0x54, /* Keypad / */
    [0x137] = 0x46, /* Print Screen */
    [0x138] = 0xe6, /* Right Alt */
    [0x146] = 0x48, /* Break: Ctrl + Pause */
    [0x147] = 0x4a, /* Home */
    [0x148] = 0x52, /* Up */
    [0x149] = 0x4b, /* Page Up */
    [0x14b] = 0x50, /* Left */
    [0x14d] = 0x4f, /* Right */
    [0x14f] = 0x4d, /* End */
    [0x150] = 0x51, /* Down */
    [0x151] = 0x4e, /* Page Down */
    [0x152] = 0x49, /* Insert */
    [0x153] = 0x4c, /* Delete */
    [0x15b] = 0xe3, /* Left GUI */
    [0x15c] = 0xe7, /* Right GUI */
    [0x15d] = 0x65, /* Application */
    [0x15e] = 0x66  /* Power */
};

#define HID_KBD_QUEUE 32

typedef struct hid_kbd_t {
    usb_dev_t  usb;
    uint8_t    protocol; /* 0 boot, 1 report; both send the Appendix B.1 report */
    hid_idle_t idle;
    uint8_t    leds;       /* the last LED Output report */
    uint8_t    held[256];  /* per usage, the number of host keys down that map to it */
    uint8_t    order[256]; /* non-modifier usages held, in press order */
    int        n_order;
    uint8_t    sent[8];    /* the last report on the interrupt pipe */
    uint8_t    queue[HID_KBD_QUEUE][8]; /* reports not yet sent, oldest at q_head */
    int        q_head;
    int        q_len;
    int        configured;
    uint8_t    down[0x200]; /* host keys down, so a host typematic repeat is no change */
    uint8_t    pause_tail;  /* the held 045h is the end of Pause, not Num Lock */
    /* Host keys arrive on the GUI thread, polls on the emulation thread. */
    mutex_t   *lock;
} hid_kbd_t;

/* The 8-byte boot report: modifier byte, reserved byte, six array slots
   (HID 1.11 Appendix B.1, p. 60 / PDF 70). More than six non-modifier keys
   is the phantom state: ErrorRollOver (usage 01h) in every slot, with the
   modifiers still reported (HID 1.11 Appendix C, p. 62-63 / PDF 72-73). */
static void
hid_kbd_state(const hid_kbd_t *dev, uint8_t *rep)
{
    memset(rep, 0x00, 8);
    for (int i = 0; i < 8; i++) {
        if (dev->held[0xe0 + i])
            rep[0] |= (1 << i);
    }

    if (dev->n_order > 6)
        memset(rep + 2, 0x01, 6);
    else
        memcpy(rep + 2, dev->order, dev->n_order);
}

/* Every change queues a report, so a press and a release between two polls
   both reach the host. A state equal to the newest queued report (or, with
   an empty queue, the last one sent) queues nothing. A full queue
   overwrites its newest entry. */
static void
hid_kbd_changed(hid_kbd_t *dev)
{
    uint8_t  rep[8];
    uint8_t *last = (dev->q_len > 0) ? dev->queue[(dev->q_head + dev->q_len - 1) % HID_KBD_QUEUE] : dev->sent;

    hid_kbd_state(dev, rep);
    if (!memcmp(rep, last, 8))
        return;

    if (dev->q_len == HID_KBD_QUEUE)
        memcpy(last, rep, 8);
    else {
        memcpy(dev->queue[(dev->q_head + dev->q_len) % HID_KBD_QUEUE], rep, 8);
        dev->q_len++;
    }
}

static void
hid_kbd_press(hid_kbd_t *dev, uint8_t usage, int down)
{
    /* Usages E0h-E7h are the modifiers, sent as bits in byte 0 of the boot
       report; only the others go in the key array (HID 1.11 Appendix B.1,
       p. 60 / PDF 70). */
    const int in_array = usage < 0xe0;

    if (down) {
        if ((dev->held[usage]++ == 0) && in_array)
            dev->order[dev->n_order++] = usage;
    } else if ((dev->held[usage] != 0) && (--dev->held[usage] == 0) && in_array) {
        for (int i = 0; i < dev->n_order; i++) {
            if (dev->order[i] == usage) {
                memmove(&dev->order[i], &dev->order[i + 1], dev->n_order - i - 1);
                dev->n_order--;
                break;
            }
        }
    }

    hid_kbd_changed(dev);
}

/* Input handler: while installed, key_process() in keyboard.c passes every
   host key here instead of to the keyboard controller. */
static void
hid_kbd_key(uint16_t scan, int down, void *priv)
{
    hid_kbd_t *dev   = (hid_kbd_t *) priv;
    uint8_t    usage = hid_kbd_usage[scan & 0x1ff];

    scan &= 0x1ff;
    down = !!down;

    thread_wait_mutex(dev->lock);
    /* The host sends Pause as two keys, 100h and then 045h, because its
       set 1 make code is E1 1D 45 and 45 on its own is Num Lock. A USB
       keyboard reports Pause as the single usage 48h, so a 045h that
       arrives while 100h is down is the rest of Pause: it and its release
       are dropped, and the guest sees no Num Lock. */
    if (scan == 0x045) {
        if (down && dev->down[0x100])
            dev->pause_tail = 1;
        if (dev->pause_tail) {
            if (!down)
                dev->pause_tail = 0;
            usage = 0;
        }
    }
    if ((usage != 0) && (dev->down[scan] != down)) {
        dev->down[scan] = down;
        hid_kbd_press(dev, usage, down);
    }
    thread_release_mutex(dev->lock);
}

static int
hid_kbd_control_locked(hid_kbd_t *dev, const uint8_t *setup, uint8_t *data)
{
    uint16_t   wvalue  = setup[2] | (setup[3] << 8);
    uint16_t   windex  = setup[4] | (setup[5] << 8);
    uint16_t   wlength = setup[6] | (setup[7] << 8);

    /* Every request here is addressed to interface 0, the only one. */
    if (windex != 0)
        return USB_DATA_STALL;

    /* GET_DESCRIPTOR for a HID class descriptor, addressed to the interface
       (HID 1.11 s7.1.1, p. 49 / PDF 59). There is one HID and one Report
       descriptor, so any index but 0 STALLs. The HID descriptor is the copy
       inside the configuration descriptor, at offset 18. */
    if ((setup[0] == 0x81) && (setup[1] == 0x06)) {
        if ((wvalue & 0xff) != 0)
            return USB_DATA_STALL;
        switch (wvalue >> 8) {
            case 0x21:
                memcpy(data, &hid_kbd_cfg_desc[18], 9);
                return 9;
            case 0x22:
                memcpy(data, hid_kbd_report_desc, sizeof(hid_kbd_report_desc));
                return sizeof(hid_kbd_report_desc);
            default:
                return USB_DATA_STALL;
        }
    }

    if (setup[0] == 0xa1) {
        switch (setup[1]) {
            case HID_GET_REPORT:
                /* No report IDs. Input (type 1) is the current key state,
                   Output (type 2) the LED byte last set (HID 1.11 s7.2.1,
                   p. 51 / PDF 61). */
                if ((wvalue & 0xff) != 0)
                    return USB_DATA_STALL;
                switch (wvalue >> 8) {
                    case 0x01:
                        hid_kbd_state(dev, data);
                        return 8;
                    case 0x02:
                        data[0] = dev->leds;
                        return 1;
                    default:
                        return USB_DATA_STALL;
                }
            case HID_GET_IDLE:
                /* The device uses no report IDs, so a nonzero ID names no
                   report (HID 1.11 s7.2.3, p. 52 / PDF 62). */
                if ((wvalue & 0xff) != 0)
                    return USB_DATA_STALL;
                data[0] = dev->idle.rate;
                return 1;
            case HID_GET_PROTOCOL:
                data[0] = dev->protocol;
                return 1;
            default:
                return USB_DATA_STALL;
        }
    }

    if (setup[0] == 0x21) {
        switch (setup[1]) {
            case HID_SET_REPORT:
                /* The LED Output report, bits 0-4: Num Lock, Caps Lock,
                   Scroll Lock, Compose, Kana (HID 1.11 Appendix B.1, p. 60
                   / PDF 70). The guest owns the lock state (Appendix C,
                   p. 62 / PDF 72); 86Box's lock flags copy it, as they do
                   for the LED command of an AT keyboard. */
                if ((wvalue != 0x0200) || (wlength < 1))
                    return USB_DATA_STALL;
                dev->leds = data[0] & 0x1f;
                keyboard_update_states(!!(dev->leds & 0x02), !!(dev->leds & 0x01), !!(dev->leds & 0x04), !!(dev->leds & 0x10));
                return 0;
            case HID_SET_IDLE:
                /* No report IDs, so a nonzero ID names no report; the high
                   byte is the duration (HID 1.11 s7.2.4, p. 52-53 / PDF 62-63). */
                if ((wvalue & 0xff) != 0)
                    return USB_DATA_STALL;
                hid_idle_set(&dev->idle, wvalue >> 8);
                return 0;
            case HID_SET_PROTOCOL:
                if (wvalue > 1)
                    return USB_DATA_STALL;
                dev->protocol = wvalue;
                return 0;
            default:
                return USB_DATA_STALL;
        }
    }

    return USB_DATA_STALL;
}

static int
hid_kbd_control(usb_dev_t *usb, const uint8_t *setup, uint8_t *data)
{
    hid_kbd_t *dev = (hid_kbd_t *) usb->priv;
    int        ret;

    thread_wait_mutex(dev->lock);
    ret = hid_kbd_control_locked(dev, setup, data);
    thread_release_mutex(dev->lock);

    return ret;
}

/* Interrupt IN: the oldest queued report; with none queued, the last report
   again once the idle period has run out (HID 1.11 s7.2.4, p. 53 / PDF 63,
   and Appendix C, p. 62 / PDF 72); otherwise NAK. */
static int
hid_kbd_data_in_locked(hid_kbd_t *dev, uint8_t *buf)
{
    if (dev->q_len > 0) {
        memcpy(dev->sent, dev->queue[dev->q_head], 8);
        dev->q_head = (dev->q_head + 1) % HID_KBD_QUEUE;
        dev->q_len--;
    } else if (!hid_idle_due(&dev->idle))
        return USB_DATA_NAK;

    memcpy(buf, dev->sent, 8);
    hid_idle_reported(&dev->idle);
    return 8;
}

static int
hid_kbd_data_in(usb_dev_t *usb, UNUSED(int ep), uint8_t *buf, UNUSED(int maxlen))
{
    hid_kbd_t *dev = (hid_kbd_t *) usb->priv;
    int        ret;

    thread_wait_mutex(dev->lock);
    ret = hid_kbd_data_in_locked(dev, buf);
    thread_release_mutex(dev->lock);

    return ret;
}

static void
hid_kbd_configured(usb_dev_t *usb, int config)
{
    hid_kbd_t *dev = (hid_kbd_t *) usb->priv;

    /* Reports queued under the old configuration are dropped. Keys still
       held show in the first report of the new one, which is compared
       against an all-zero report. */
    thread_wait_mutex(dev->lock);
    dev->configured = (config != 0);
    dev->q_head     = 0;
    dev->q_len      = 0;
    memset(dev->sent, 0x00, sizeof(dev->sent));
    hid_kbd_changed(dev);
    thread_release_mutex(dev->lock);
}

static void
hid_kbd_reset(usb_dev_t *usb)
{
    hid_kbd_t *dev = (hid_kbd_t *) usb->priv;

    thread_wait_mutex(dev->lock);

    /* Devices start in report protocol (HID 1.11 s7.2.6, p. 54 / PDF 64).
       The recommended default idle rate for keyboards is 500 ms, 125 in
       4 ms units (s7.2.4, p. 53 / PDF 63). */
    dev->protocol     = 1;
    dev->idle.rate    = 125;
    dev->idle.pending = 0;
    dev->idle.last_us = hid_now_us();
    dev->leds         = 0x00;
    dev->configured   = 0;
    dev->q_head       = 0;
    dev->q_len        = 0;
    memset(dev->sent, 0x00, sizeof(dev->sent));

    thread_release_mutex(dev->lock);
}

static const usb_dev_ops_t hid_kbd_ops = {
    .control    = hid_kbd_control,
    .data_in    = hid_kbd_data_in,
    .data_out   = NULL,
    .reset      = hid_kbd_reset,
    .configured = hid_kbd_configured
};

static void *
hid_kbd_init(const device_t *info)
{
    hid_kbd_t *dev = (hid_kbd_t *) calloc(1, sizeof(hid_kbd_t));

    dev->usb.name      = "USB Keyboard";
    dev->usb.low_speed = 1;
    dev->usb.dev_desc  = hid_kbd_dev_desc;
    dev->usb.cfg_desc  = hid_kbd_cfg_desc;
    dev->usb.cfg_len   = HID_KBD_CFG_LEN;
    dev->usb.strings   = hid_kbd_strings;
    dev->usb.n_strings = 2;
    dev->usb.ops       = &hid_kbd_ops;
    dev->usb.priv      = dev;
    dev->lock          = thread_create_mutex();
    usb_dev_init(&dev->usb);
    hid_kbd_reset(&dev->usb);

    usb_dev_add(&dev->usb, info);

    /* This is the machine's keyboard: every host key comes here and none
       goes to the 8042. key_process() drops all keys while keyboard_scan
       is 0. */
    keyboard_scan = 1;
    keyboard_set_input_handler(hid_kbd_key, dev);

    return dev;
}

static void
hid_kbd_close(void *priv)
{
    hid_kbd_t *dev = (hid_kbd_t *) priv;

    keyboard_set_input_handler(NULL, NULL);
    usb_dev_remove(&dev->usb);
    thread_close_mutex(dev->lock);
    free(dev);
}

const device_t keyboard_usb_device = {
    .name          = "USB Keyboard",
    .internal_name = "usb_keyboard",
    .flags         = DEVICE_USB,
    .local         = 0,
    .init          = hid_kbd_init,
    .close         = hid_kbd_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

/* Absolute pointer: a Generic Desktop Mouse collection with absolute X and Y
   (HID Usage Tables 1.12 s4, p. 26 / PDF 26) and Buttons 1-3 (s12, p. 67 /
   PDF 69). The Windows mouse drivers scale X and Y by the Physical extent,
   not the Logical one: Win98 mouhid.vxd posts (v - PhysMin) << 16 /
   (PhysMax - PhysMin), Windows 2000/XP mouhid.sys v * 0FFFFh / PhysMax.
   HID reads Physical 0/0 as equal to Logical (HID 1.11 s6.2.2.7, p. 37-38 /
   PDF 47-48), but with 0/0 Win98 posts v unscaled and 2000/XP divide by
   (1 << Report Size) - 1, so 0..32767 would cover half the screen.
   Physical is therefore declared equal to Logical. Values stay under 2^15
   because the Windows parsers sign-extend a field by its size whatever the
   Logical Minimum. */
#define HID_TABLET_MAX 32767

static const uint8_t hid_tablet_report_desc[] = {
    0x05, 0x01,       /* Usage Page (Generic Desktop) */
    0x09, 0x02,       /* Usage (Mouse) */
    0xa1, 0x01,       /* Collection (Application) */
    0x09, 0x01,       /*   Usage (Pointer) */
    0xa1, 0x00,       /*   Collection (Physical) */
    0x05, 0x09,       /*     Usage Page (Button) */
    0x19, 0x01,       /*     Usage Minimum (1) */
    0x29, 0x03,       /*     Usage Maximum (3) */
    0x15, 0x00,       /*     Logical Minimum (0) */
    0x25, 0x01,       /*     Logical Maximum (1) */
    0x95, 0x03,       /*     Report Count (3) */
    0x75, 0x01,       /*     Report Size (1) */
    0x81, 0x02,       /*     Input (Data, Variable, Absolute) */
    0x95, 0x01,       /*     Report Count (1) */
    0x75, 0x05,       /*     Report Size (5) */
    0x81, 0x01,       /*     Input (Constant) */
    0x05, 0x01,       /*     Usage Page (Generic Desktop) */
    0x09, 0x30,       /*     Usage (X) */
    0x09, 0x31,       /*     Usage (Y) */
    0x15, 0x00,       /*     Logical Minimum (0) */
    0x26, 0xff, 0x7f, /*     Logical Maximum (32767) */
    0x35, 0x00,       /*     Physical Minimum (0) */
    0x46, 0xff, 0x7f, /*     Physical Maximum (32767) */
    0x75, 0x10,       /*     Report Size (16) */
    0x95, 0x02,       /*     Report Count (2) */
    0x81, 0x02,       /*     Input (Data, Variable, Absolute) */
    /* A relative wheel, as on the mouse (HID Usage Tables 1.12 s4.3, p. 29
       / PDF 29). Physical Minimum and Maximum are global items and would
       carry X and Y's 0..32767 over to it; 0/0 restores "same as Logical"
       (HID 1.11 s6.2.2.7, p. 38 / PDF 48). */
    0x09, 0x38,       /*     Usage (Wheel) */
    0x15, 0x81,       /*     Logical Minimum (-127) */
    0x25, 0x7f,       /*     Logical Maximum (127) */
    0x35, 0x00,       /*     Physical Minimum (0) */
    0x45, 0x00,       /*     Physical Maximum (0) */
    0x75, 0x08,       /*     Report Size (8) */
    0x95, 0x01,       /*     Report Count (1) */
    0x81, 0x06,       /*     Input (Data, Variable, Relative) */
    0xc0,             /*   End Collection */
    0xc0              /* End Collection */
};

#define HID_TABLET_REPORT 6

static const uint8_t hid_tablet_dev_desc[18] = {
    18, 0x01,   /* bLength, DEVICE */
    0x10, 0x01, /* bcdUSB 1.10 */
    0x00, 0x00, 0x00,
    64,         /* bMaxPacketSize0 */
    0x00, 0x00, /* idVendor */
    0x06, 0x00, /* idProduct */
    0x00, 0x01, /* bcdDevice */
    1, 2, 0,    /* iManufacturer, iProduct, iSerialNumber */
    1           /* bNumConfigurations */
};

#define HID_TABLET_CFG_LEN (9 + 9 + 9 + 7)

static const uint8_t hid_tablet_cfg_desc[HID_TABLET_CFG_LEN] = {
    /* Configuration */
    9, 0x02, HID_TABLET_CFG_LEN, 0x00, 1, 1, 0,
    0x80, /* bus powered, no remote wakeup */
    50,   /* 100 mA, in 2 mA units */
    /* Interface 0: HID, no boot interface, so subclass and protocol 0
       (HID 1.11 s4.2-4.3, p. 8-9 / PDF 18-19) */
    9, 0x04, 0, 0, 1, 0x03, 0x00, 0x00, 0,
    /* HID descriptor, between interface and endpoint (HID 1.11 s7.1, p. 48 / PDF 58) */
    9, 0x21, 0x11, 0x01, 0, 1, 0x22, sizeof(hid_tablet_report_desc), 0x00,
    /* Endpoint 1 IN, interrupt, 8 bytes, bInterval 10 ms (USB 1.1 s9.6.4, p. 204 / PDF 220) */
    7, 0x05, 0x81, 0x03, 8, 0x00, 10
};

static const char *const hid_tablet_strings[] = {
    "86Box",
    "USB Tablet"
};

typedef struct hid_tablet_t {
    usb_dev_t  usb;
    hid_idle_t idle;
    uint8_t    cur[HID_TABLET_REPORT];  /* the latest state */
    uint8_t    sent[HID_TABLET_REPORT]; /* the last report on the interrupt pipe */
    int        sent_valid;              /* 0 = no report sent yet in this configuration */
} hid_tablet_t;

/* Converts a host position (0..1 across the guest screen) to a logical
   value at the center of the guest pixel under it. size is the screen's
   width or height in pixels; 0 means unknown and skips the snap. A value on
   a pixel edge can round into the pixel before it in a driver that scales
   to 0..65535. The center also keeps the last pixel below 32767: Win98
   scales 32767 to 65536, and its 16-bit path wraps that to the left or top
   edge. */
static uint16_t
hid_tablet_axis(double a, int size)
{
    if (!(a > 0.0))
        a = 0.0;
    if (a > 1.0)
        a = 1.0;
    if (size > 0) {
        /* The host position is a whole pixel divided by the width, and
           (q / w) * w can come out slightly below q; the 1e-6 keeps it in
           pixel q. */
        int p = (int) (a * size + 1e-6);

        if (p >= size)
            p = size - 1;
        a = (p + 0.5) / size;
    }
    if (a >= 1.0)
        return HID_TABLET_MAX;
    return (uint16_t) (a * (HID_TABLET_MAX + 1));
}

/* Samples the host pointer into cur. Host input counts only in tablet mode;
   in mouse mode cur keeps its last value. With take set, a button pressed
   and released since the last take still shows as pressed, and the wheel
   motion is consumed. */
static void
hid_tablet_sample(hid_tablet_t *dev, int take)
{
    const monitor_t *m;
    double           x;
    double           y;
    uint16_t         vx;
    uint16_t         vy;
    int              b;
    int              dz;

    if (mouse_input_mode < 1)
        return;

    mouse_get_abs_coords(&x, &y);

    /* The monitor the host pointer is over; the Qt UI stores its index + 1,
       and 0 when the pointer is over none. Fall back to the first. */
    if ((mouse_tablet_in_proximity > 0) && (mouse_tablet_in_proximity <= MONITORS_NUM))
        m = &monitors[mouse_tablet_in_proximity - 1];
    else
        m = &monitors[0];

    /* With overscan shown, the image adds a border of half the overscan on
       each side of the active display: the guest's screen is the inner
       xsize by ysize. */
    if (enable_overscan && (m->mon_xsize > 0) && (m->mon_ysize > 0)) {
        x = (x * (m->mon_xsize + m->mon_overscan_x) - (m->mon_overscan_x / 2.0)) / m->mon_xsize;
        y = (y * (m->mon_ysize + m->mon_overscan_y) - (m->mon_overscan_y / 2.0)) / m->mon_ysize;
    }

    vx = hid_tablet_axis(x, m->mon_xsize);
    vy = hid_tablet_axis(y, m->mon_ysize);
    b  = tablet_get_buttons_ex();
    dz = 0;
    if (take) {
        b |= tablet_take_pressed();
        /* Positive is away from the user, as on the mouse. */
        mouse_subtract_z(&dz, -127, 127, 0);
    }

    /* Host bits 0-2 are left, right, middle: Buttons 1-3, primary,
       secondary, tertiary (HID Usage Tables 1.12 s12, p. 67 / PDF 69). */
    dev->cur[0] = b & 0x07;
    dev->cur[1] = vx & 0xff;
    dev->cur[2] = vx >> 8;
    dev->cur[3] = vy & 0xff;
    dev->cur[4] = vy >> 8;
    dev->cur[5] = (uint8_t) dz;
}

static int
hid_tablet_control(usb_dev_t *usb, const uint8_t *setup, uint8_t *data)
{
    hid_tablet_t *dev    = (hid_tablet_t *) usb->priv;
    uint16_t      wvalue = setup[2] | (setup[3] << 8);
    uint16_t      windex = setup[4] | (setup[5] << 8);

    /* Every request here is addressed to interface 0, the only one. */
    if (windex != 0)
        return USB_DATA_STALL;

    /* GET_DESCRIPTOR for a HID class descriptor, addressed to the interface
       (HID 1.11 s7.1.1, p. 49 / PDF 59). There is one HID and one Report
       descriptor, so any index but 0 STALLs. The HID descriptor is the copy
       inside the configuration descriptor, at offset 18. */
    if ((setup[0] == 0x81) && (setup[1] == 0x06)) {
        if ((wvalue & 0xff) != 0)
            return USB_DATA_STALL;
        switch (wvalue >> 8) {
            case 0x21:
                memcpy(data, &hid_tablet_cfg_desc[18], 9);
                return 9;
            case 0x22:
                memcpy(data, hid_tablet_report_desc, sizeof(hid_tablet_report_desc));
                return sizeof(hid_tablet_report_desc);
            default:
                return USB_DATA_STALL;
        }
    }

    /* GET_PROTOCOL and SET_PROTOCOL are for boot-subclass devices only (HID
       1.11 s7.2.5-7.2.6, p. 54 / PDF 64), so they STALL here. */
    if (setup[0] == 0xa1) {
        switch (setup[1]) {
            case HID_GET_REPORT:
                /* wValue is report type (high) and report ID (low). The
                   only report is Input with no report ID: type 1, ID 0
                   (HID 1.11 s7.2.1, p. 51 / PDF 61). */
                if (wvalue != 0x0100)
                    return USB_DATA_STALL;
                hid_tablet_sample(dev, 0);
                memcpy(data, dev->cur, HID_TABLET_REPORT);
                return HID_TABLET_REPORT;
            case HID_GET_IDLE:
                /* The device uses no report IDs, so a nonzero ID names no
                   report (HID 1.11 s7.2.3, p. 52 / PDF 62). */
                if ((wvalue & 0xff) != 0)
                    return USB_DATA_STALL;
                data[0] = dev->idle.rate;
                return 1;
            default:
                return USB_DATA_STALL;
        }
    }

    if (setup[0] == 0x21) {
        switch (setup[1]) {
            case HID_SET_IDLE:
                /* No report IDs, so a nonzero ID names no report; the high
                   byte is the duration (HID 1.11 s7.2.4, p. 52-53 / PDF 62-63). */
                if ((wvalue & 0xff) != 0)
                    return USB_DATA_STALL;
                hid_idle_set(&dev->idle, wvalue >> 8);
                return 0;
            default:
                /* SET_REPORT included: the tablet has no Output or Feature report. */
                return USB_DATA_STALL;
        }
    }

    return USB_DATA_STALL;
}

/* Interrupt IN. In tablet mode: a report when the state changed or when none
   was sent yet in this configuration. In either mode: the current state
   again once the idle period has run out (HID 1.11 s7.2.4, p. 53 / PDF 63).
   Otherwise NAK. */
static int
hid_tablet_data_in(usb_dev_t *usb, UNUSED(int ep), uint8_t *buf, UNUSED(int maxlen))
{
    hid_tablet_t *dev = (hid_tablet_t *) usb->priv;

    hid_tablet_sample(dev, 1);

    if (((mouse_input_mode >= 1) && (!dev->sent_valid || memcmp(dev->cur, dev->sent, HID_TABLET_REPORT))) ||
        hid_idle_due(&dev->idle)) {
        memcpy(dev->sent, dev->cur, HID_TABLET_REPORT);
        dev->sent_valid = 1;
        memcpy(buf, dev->cur, HID_TABLET_REPORT);
        /* The wheel is relative: a delta goes out once and an idle repeat
           carries 0. Clearing it in sent too makes any later nonzero delta
           count as a change. */
        dev->cur[5]  = 0;
        dev->sent[5] = 0;
        hid_idle_reported(&dev->idle);
        return HID_TABLET_REPORT;
    }

    return USB_DATA_NAK;
}

static void
hid_tablet_configured(usb_dev_t *usb, UNUSED(int config))
{
    hid_tablet_t *dev = (hid_tablet_t *) usb->priv;

    dev->sent_valid = 0;
}

static void
hid_tablet_reset(usb_dev_t *usb)
{
    hid_tablet_t *dev = (hid_tablet_t *) usb->priv;

    /* The recommended default idle rate for mice is infinite, rate 0 (HID
       1.11 s7.2.4, p. 53 / PDF 63). */
    dev->idle.rate    = 0;
    dev->idle.pending = 0;
    dev->sent_valid   = 0;
}

static const usb_dev_ops_t hid_tablet_ops = {
    .control    = hid_tablet_control,
    .data_in    = hid_tablet_data_in,
    .data_out   = NULL,
    .reset      = hid_tablet_reset,
    .configured = hid_tablet_configured
};

static void *
hid_tablet_init(const device_t *info)
{
    hid_tablet_t *dev = (hid_tablet_t *) calloc(1, sizeof(hid_tablet_t));

    dev->usb.name      = "USB Tablet";
    dev->usb.low_speed = 0;
    dev->usb.dev_desc  = hid_tablet_dev_desc;
    dev->usb.cfg_desc  = hid_tablet_cfg_desc;
    dev->usb.cfg_len   = HID_TABLET_CFG_LEN;
    dev->usb.strings   = hid_tablet_strings;
    dev->usb.n_strings = 2;
    dev->usb.ops       = &hid_tablet_ops;
    dev->usb.priv      = dev;
    usb_dev_init(&dev->usb);
    hid_tablet_reset(&dev->usb);

    mouse_set_buttons(3);

    usb_dev_add(&dev->usb, info);

    return dev;
}

static void
hid_tablet_close(void *priv)
{
    hid_tablet_t *dev = (hid_tablet_t *) priv;

    usb_dev_remove(&dev->usb);
    free(dev);
}

const device_t tablet_usb_device = {
    .name          = "USB Tablet",
    .internal_name = "usb_tablet",
    .flags         = DEVICE_USB,
    .local         = 0,
    .init          = hid_tablet_init,
    .close         = hid_tablet_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

/* USB game pads. Each model is a plain HID device, built at init from one row
   of the table below, so its descriptors and reports always agree. A report is
   the axes in table order, an optional hat, the buttons, then padding to a
   byte.

   The Cobra 2 and the Windows 98 SE entries use their real names and IDs, from
   Creative's and Microsoft's INFs, but no real device was available to dump.
   Their descriptors, speed and report layouts are this driver's choice. The
   SideWinder controls come from Linux's sidewinder driver and Microsoft's INF
   data, and the others are estimates from reviews, or from similar products
   where nothing could be found.

   The two 86Box pads copy no product: one is the usual low-speed class pad of
   its time, the other has the controls of a twin-stick pad like the Xbox 360
   controller. That controller is a vendor-class device with its own driver,
   and this is a HID class pad. Its report does not fit in a low-speed packet,
   so it is full speed. */
#define HID_USAGE_POINTER  0x01
#define HID_USAGE_JOYSTICK 0x04
#define HID_USAGE_GAME_PAD 0x05
#define HID_USAGE_X        0x30
#define HID_USAGE_Y        0x31
#define HID_USAGE_Z        0x32
#define HID_USAGE_RX       0x33
#define HID_USAGE_RY       0x34
#define HID_USAGE_RZ       0x35
#define HID_USAGE_SLIDER   0x36
#define HID_USAGE_HAT      0x39

#define HID_PAD_MAX_AXES   6
#define HID_PAD_REPORT_MAX 16
#define HID_PAD_DESC_MAX   192
#define HID_PAD_CFG_LEN    (9 + 9 + 9 + 7)

enum {
    HID_AXIS_ANALOG,  /* unsigned */
    HID_AXIS_DIGITAL, /* unsigned, three positions */
    HID_AXIS_SIGNED   /* 16 bits, signed */
};

typedef struct hid_pad_axis_t {
    uint8_t usage;
    uint8_t bits;
    uint8_t kind;
} hid_pad_axis_t;

typedef struct hid_pad_model_t {
    uint16_t       vid;
    uint16_t       pid;
    const char    *manufacturer;
    const char    *product;
    uint8_t        usage;
    uint8_t        low_speed;
    uint8_t        interval;     /* ms; a low-speed report must fit in 8 bytes */
    uint8_t        pointer_axes; /* the first axes, in a Physical collection */
    uint8_t        num_axes;
    hid_pad_axis_t axes[HID_PAD_MAX_AXES];
    uint8_t        hat;          /* 0, or 4 or 8 directions */
    uint8_t        buttons;
} hid_pad_model_t;

enum {
    HID_PAD_COBRA2,
    HID_PAD_GAMEPAD,
    HID_PAD_6AXIS,
    HID_PAD_SIDEWINDER_PAD,
    HID_PAD_SIDEWINDER_PRECISION_PRO,
    HID_PAD_SIDEWINDER_FREESTYLE_PRO,
    HID_PAD_TM_RAGE3D,
    HID_PAD_TM_TOP_GUN,
    HID_PAD_TM_TOP_GUN_REV2,
    HID_PAD_NEC_PK_GP201,
    HID_PAD_NEC_PK_GP301,
    HID_PAD_LOGITECH_WINGMAN_EXTREME,
    HID_PAD_LOGITECH_WINGMAN_EXTREME_THROTTLE,
    HID_PAD_GENIUS_JOYSTICK,
    HID_PAD_GENIUS_GAMEPAD,
    HID_PAD_GUILLEMOT_JET_LEADER,
};

static const hid_pad_model_t hid_pad_models[] = {
    [HID_PAD_COBRA2] = {
        .vid          = 0x041e,
        .pid          = 0x1003,
        .manufacturer = "Creative Labs",
        .product      = "Creative USB Game Pad II",
        .usage        = HID_USAGE_GAME_PAD,
        .low_speed    = 1,
        .interval     = 10,
        .pointer_axes = 2,
        .num_axes     = 3,
        .axes         = { { HID_USAGE_X, 8, HID_AXIS_DIGITAL }, { HID_USAGE_Y, 8, HID_AXIS_DIGITAL }, { HID_USAGE_SLIDER, 8, HID_AXIS_ANALOG } },
        .hat          = 0,
        .buttons      = 12
    },
    [HID_PAD_GAMEPAD] = {
        .vid          = 0x0000,
        .pid          = 0x0007,
        .manufacturer = "86Box",
        .product      = "USB Gamepad",
        .usage        = HID_USAGE_GAME_PAD,
        .low_speed    = 1,
        .interval     = 10,
        .pointer_axes = 2,
        .num_axes     = 2,
        .axes         = { { HID_USAGE_X, 8, HID_AXIS_DIGITAL }, { HID_USAGE_Y, 8, HID_AXIS_DIGITAL } },
        .hat          = 0,
        .buttons      = 10
    },
    [HID_PAD_6AXIS] = {
        .vid          = 0x0000,
        .pid          = 0x0008,
        .manufacturer = "86Box",
        .product      = "USB Gamepad (6-axis)",
        .usage        = HID_USAGE_GAME_PAD,
        .low_speed    = 0,
        .interval     = 4,
        .pointer_axes = 4,
        .num_axes     = 6,
        .axes         = { { HID_USAGE_X, 16, HID_AXIS_SIGNED }, { HID_USAGE_Y, 16, HID_AXIS_SIGNED }, { HID_USAGE_RX, 16, HID_AXIS_SIGNED }, { HID_USAGE_RY, 16, HID_AXIS_SIGNED }, { HID_USAGE_Z, 8, HID_AXIS_ANALOG }, { HID_USAGE_RZ, 8, HID_AXIS_ANALOG } },
        .hat          = 8,
        .buttons      = 11
    },
    [HID_PAD_SIDEWINDER_PAD] = {
        .vid          = 0x045e,
        .pid          = 0x0007,
        .manufacturer = "Microsoft",
        .product      = "SideWinder Game Pad USB version 1.0",
        .usage        = HID_USAGE_GAME_PAD,
        .low_speed    = 1,
        .interval     = 10,
        .pointer_axes = 2,
        .num_axes     = 2,
        .axes         = { { HID_USAGE_X, 8, HID_AXIS_DIGITAL }, { HID_USAGE_Y, 8, HID_AXIS_DIGITAL } },
        .hat          = 0,
        .buttons      = 10
    },
    [HID_PAD_SIDEWINDER_PRECISION_PRO] = {
        .vid          = 0x045e,
        .pid          = 0x0008,
        .manufacturer = "Microsoft",
        .product      = "Microsoft SideWinder Precision Pro (USB)",
        .usage        = HID_USAGE_JOYSTICK,
        .low_speed    = 1,
        .interval     = 10,
        .pointer_axes = 2,
        .num_axes     = 4,
        .axes         = { { HID_USAGE_X, 10, HID_AXIS_ANALOG }, { HID_USAGE_Y, 10, HID_AXIS_ANALOG }, { HID_USAGE_RZ, 6, HID_AXIS_ANALOG }, { HID_USAGE_SLIDER, 7, HID_AXIS_ANALOG } },
        .hat          = 8,
        .buttons      = 9
    },
    [HID_PAD_SIDEWINDER_FREESTYLE_PRO] = {
        .vid          = 0x045e,
        .pid          = 0x000e,
        .manufacturer = "Microsoft",
        .product      = "Microsoft SideWinder Freestyle Pro (USB)",
        .usage        = HID_USAGE_GAME_PAD,
        .low_speed    = 1,
        .interval     = 10,
        .pointer_axes = 2,
        .num_axes     = 3,
        .axes         = { { HID_USAGE_X, 10, HID_AXIS_ANALOG }, { HID_USAGE_Y, 10, HID_AXIS_ANALOG }, { HID_USAGE_SLIDER, 6, HID_AXIS_ANALOG } },
        .hat          = 8,
        .buttons      = 10
    },
    [HID_PAD_TM_RAGE3D] = {
        .vid          = 0x044f,
        .pid          = 0xa003,
        .manufacturer = "ThrustMaster",
        .product      = "ThrustMaster Rage 3D USB Game Pad",
        .usage        = HID_USAGE_GAME_PAD,
        .low_speed    = 1,
        .interval     = 10,
        .pointer_axes = 2,
        .num_axes     = 2,
        .axes         = { { HID_USAGE_X, 8, HID_AXIS_ANALOG }, { HID_USAGE_Y, 8, HID_AXIS_ANALOG } },
        .hat          = 0,
        .buttons      = 8
    },
    [HID_PAD_TM_TOP_GUN] = {
        .vid          = 0x044f,
        .pid          = 0xa0a0,
        .manufacturer = "ThrustMaster",
        .product      = "ThrustMaster Top Gun USB Joystick",
        .usage        = HID_USAGE_JOYSTICK,
        .low_speed    = 1,
        .interval     = 10,
        .pointer_axes = 2,
        .num_axes     = 2,
        .axes         = { { HID_USAGE_X, 8, HID_AXIS_ANALOG }, { HID_USAGE_Y, 8, HID_AXIS_ANALOG } },
        .hat          = 4,
        .buttons      = 4
    },
    [HID_PAD_TM_TOP_GUN_REV2] = {
        .vid          = 0x044f,
        .pid          = 0xa0a1,
        .manufacturer = "ThrustMaster",
        .product      = "ThrustMaster Top Gun USB Joystick (rev2)",
        .usage        = HID_USAGE_JOYSTICK,
        .low_speed    = 1,
        .interval     = 10,
        .pointer_axes = 2,
        .num_axes     = 2,
        .axes         = { { HID_USAGE_X, 8, HID_AXIS_ANALOG }, { HID_USAGE_Y, 8, HID_AXIS_ANALOG } },
        .hat          = 4,
        .buttons      = 4
    },
    [HID_PAD_NEC_PK_GP201] = {
        .vid          = 0x044f,
        .pid          = 0xa201,
        .manufacturer = "NEC",
        .product      = "NEC PK-GP201 PlayStick",
        .usage        = HID_USAGE_JOYSTICK,
        .low_speed    = 1,
        .interval     = 10,
        .pointer_axes = 2,
        .num_axes     = 2,
        .axes         = { { HID_USAGE_X, 8, HID_AXIS_ANALOG }, { HID_USAGE_Y, 8, HID_AXIS_ANALOG } },
        .hat          = 4,
        .buttons      = 4
    },
    [HID_PAD_NEC_PK_GP301] = {
        .vid          = 0x044f,
        .pid          = 0xa01b,
        .manufacturer = "NEC",
        .product      = "NEC PK-GP301 Driving Wheel",
        .usage        = HID_USAGE_JOYSTICK,
        .low_speed    = 1,
        .interval     = 10,
        .pointer_axes = 1,
        .num_axes     = 3,
        .axes         = { { HID_USAGE_X, 8, HID_AXIS_ANALOG }, { HID_USAGE_Y, 8, HID_AXIS_ANALOG }, { HID_USAGE_Z, 8, HID_AXIS_ANALOG } },
        .hat          = 0,
        .buttons      = 4
    },
    [HID_PAD_LOGITECH_WINGMAN_EXTREME] = {
        .vid          = 0x046d,
        .pid          = 0x0200,
        .manufacturer = "Logitech",
        .product      = "Logitech USB WingMan Extreme Joystick",
        .usage        = HID_USAGE_JOYSTICK,
        .low_speed    = 1,
        .interval     = 10,
        .pointer_axes = 2,
        .num_axes     = 2,
        .axes         = { { HID_USAGE_X, 8, HID_AXIS_ANALOG }, { HID_USAGE_Y, 8, HID_AXIS_ANALOG } },
        .hat          = 8,
        .buttons      = 6
    },
    [HID_PAD_LOGITECH_WINGMAN_EXTREME_THROTTLE] = {
        .vid          = 0x046d,
        .pid          = 0xc201,
        .manufacturer = "Logitech",
        .product      = "Logitech USB WingMan Extreme Joystick with Throttle",
        .usage        = HID_USAGE_JOYSTICK,
        .low_speed    = 1,
        .interval     = 10,
        .pointer_axes = 2,
        .num_axes     = 4,
        .axes         = { { HID_USAGE_X, 8, HID_AXIS_ANALOG }, { HID_USAGE_Y, 8, HID_AXIS_ANALOG }, { HID_USAGE_RZ, 8, HID_AXIS_ANALOG }, { HID_USAGE_SLIDER, 8, HID_AXIS_ANALOG } },
        .hat          = 8,
        .buttons      = 7
    },
    [HID_PAD_GENIUS_JOYSTICK] = {
        .vid          = 0x0458,
        .pid          = 0x1001,
        .manufacturer = "Genius",
        .product      = "Genius USB Joystick",
        .usage        = HID_USAGE_JOYSTICK,
        .low_speed    = 1,
        .interval     = 10,
        .pointer_axes = 2,
        .num_axes     = 3,
        .axes         = { { HID_USAGE_X, 8, HID_AXIS_ANALOG }, { HID_USAGE_Y, 8, HID_AXIS_ANALOG }, { HID_USAGE_SLIDER, 8, HID_AXIS_ANALOG } },
        .hat          = 8,
        .buttons      = 6
    },
    [HID_PAD_GENIUS_GAMEPAD] = {
        .vid          = 0x0458,
        .pid          = 0x1002,
        .manufacturer = "Genius",
        .product      = "Genius USB Game Pad",
        .usage        = HID_USAGE_GAME_PAD,
        .low_speed    = 1,
        .interval     = 10,
        .pointer_axes = 2,
        .num_axes     = 2,
        .axes         = { { HID_USAGE_X, 8, HID_AXIS_DIGITAL }, { HID_USAGE_Y, 8, HID_AXIS_DIGITAL } },
        .hat          = 0,
        .buttons      = 8
    },
    [HID_PAD_GUILLEMOT_JET_LEADER] = {
        .vid          = 0x06f8,
        .pid          = 0x0002,
        .manufacturer = "Guillemot",
        .product      = "Guillemot Jet Leader USB Joystick",
        .usage        = HID_USAGE_JOYSTICK,
        .low_speed    = 1,
        .interval     = 10,
        .pointer_axes = 2,
        .num_axes     = 4,
        .axes         = { { HID_USAGE_X, 8, HID_AXIS_ANALOG }, { HID_USAGE_Y, 8, HID_AXIS_ANALOG }, { HID_USAGE_RZ, 8, HID_AXIS_ANALOG }, { HID_USAGE_SLIDER, 8, HID_AXIS_ANALOG } },
        .hat          = 8,
        .buttons      = 6
    }
};

/* device_t.local: the model in the high byte, the pad in the low byte. */
#define HID_PAD_LOCAL(model, pad) (((model) << 8) | (pad))

typedef struct hid_pad_t {
    usb_dev_t              usb;
    const hid_pad_model_t *model;
    int                    slot; /* which pad in joystick_state */
    int                    report_len;
    int                    report_desc_len;
    const char            *strings[2];
    uint8_t                dev_desc[18];
    uint8_t                cfg_desc[HID_PAD_CFG_LEN];
    uint8_t                report_desc[HID_PAD_DESC_MAX];
    hid_idle_t             idle;
    uint8_t                sent[HID_PAD_REPORT_MAX];
    uint8_t                sent_valid;
} hid_pad_t;

static int
hid_pad_report_bits(const hid_pad_model_t *model)
{
    int bits = model->buttons + (model->hat ? 4 : 0);

    for (int i = 0; i < model->num_axes; i++)
        bits += model->axes[i].bits;

    return bits;
}

/* Logical range, Report Size, Report Count 1, and an Input item. */
static int
hid_pad_desc_field(uint8_t *d, const hid_pad_axis_t *axis)
{
    int len = 0;

    if (axis->kind == HID_AXIS_SIGNED) {
        d[len++] = 0x16; /* Logical Minimum (-32768) */
        d[len++] = 0x00;
        d[len++] = 0x80;
        d[len++] = 0x26; /* Logical Maximum (32767) */
        d[len++] = 0xff;
        d[len++] = 0x7f;
    } else {
        int max = (1 << axis->bits) - 1;

        d[len++] = 0x15; /* Logical Minimum (0) */
        d[len++] = 0x00;
        d[len++] = 0x26; /* Logical Maximum */
        d[len++] = max & 0xff;
        d[len++] = max >> 8;
    }

    d[len++] = 0x75; /* Report Size */
    d[len++] = axis->bits;
    d[len++] = 0x95; /* Report Count (1) */
    d[len++] = 0x01;
    d[len++] = 0x81; /* Input (Data, Variable, Absolute) */
    d[len++] = 0x02;

    return len;
}

static int
hid_pad_build_report_desc(const hid_pad_model_t *model, uint8_t *d)
{
    int len     = 0;
    int padding = -hid_pad_report_bits(model) & 7;

    d[len++] = 0x05; /* Usage Page (Generic Desktop) */
    d[len++] = 0x01;
    d[len++] = 0x09; /* Usage */
    d[len++] = model->usage;
    d[len++] = 0xa1; /* Collection (Application) */
    d[len++] = 0x01;

    for (int i = 0; i < model->num_axes; i++) {
        if (i == 0) {
            d[len++] = 0x09; /* Usage (Pointer) */
            d[len++] = HID_USAGE_POINTER;
            d[len++] = 0xa1; /* Collection (Physical) */
            d[len++] = 0x00;
        }

        d[len++] = 0x09; /* Usage */
        d[len++] = model->axes[i].usage;
        len += hid_pad_desc_field(&d[len], &model->axes[i]);

        if (i == (model->pointer_axes - 1))
            d[len++] = 0xc0; /* End Collection */
    }

    if (model->hat) {
        int degrees = 360 - (360 / model->hat);

        d[len++] = 0x09; /* Usage (Hat Switch) */
        d[len++] = HID_USAGE_HAT;
        d[len++] = 0x15; /* Logical Minimum (0) */
        d[len++] = 0x00;
        d[len++] = 0x25; /* Logical Maximum */
        d[len++] = model->hat - 1;
        d[len++] = 0x35; /* Physical Minimum (0) */
        d[len++] = 0x00;
        d[len++] = 0x46; /* Physical Maximum */
        d[len++] = degrees & 0xff;
        d[len++] = degrees >> 8;
        d[len++] = 0x65; /* Unit (Degrees) */
        d[len++] = 0x14;
        d[len++] = 0x75; /* Report Size (4) */
        d[len++] = 0x04;
        d[len++] = 0x95; /* Report Count (1) */
        d[len++] = 0x01;
        d[len++] = 0x81; /* Input (Data, Variable, Absolute, Null State) */
        d[len++] = 0x42;
        d[len++] = 0x45; /* Physical Maximum (0) */
        d[len++] = 0x00;
        d[len++] = 0x65; /* Unit (None) */
        d[len++] = 0x00;
    }

    d[len++] = 0x05; /* Usage Page (Button) */
    d[len++] = 0x09;
    d[len++] = 0x19; /* Usage Minimum (1) */
    d[len++] = 0x01;
    d[len++] = 0x29; /* Usage Maximum */
    d[len++] = model->buttons;
    d[len++] = 0x15; /* Logical Minimum (0) */
    d[len++] = 0x00;
    d[len++] = 0x25; /* Logical Maximum (1) */
    d[len++] = 0x01;
    d[len++] = 0x75; /* Report Size (1) */
    d[len++] = 0x01;
    d[len++] = 0x95; /* Report Count */
    d[len++] = model->buttons;
    d[len++] = 0x81; /* Input (Data, Variable, Absolute) */
    d[len++] = 0x02;

    if (padding) {
        d[len++] = 0x75; /* Report Size */
        d[len++] = padding;
        d[len++] = 0x95; /* Report Count (1) */
        d[len++] = 0x01;
        d[len++] = 0x81; /* Input (Constant) */
        d[len++] = 0x01;
    }

    d[len++] = 0xc0; /* End Collection */

    return len;
}

static void
hid_pad_build_descs(hid_pad_t *dev)
{
    const hid_pad_model_t *model = dev->model;

    static const uint8_t dev_template[18] = {
        18, 0x01,   /* bLength, DEVICE */
        0x10, 0x01, /* bcdUSB 1.10 */
        0x00, 0x00, 0x00,
        0,          /* bMaxPacketSize0 */
        0x00, 0x00, /* idVendor */
        0x00, 0x00, /* idProduct */
        0x00, 0x01, /* bcdDevice */
        1, 2, 0,    /* iManufacturer, iProduct, iSerialNumber */
        1           /* bNumConfigurations */
    };
    static const uint8_t cfg_template[HID_PAD_CFG_LEN] = {
        /* Configuration */
        9, 0x02, HID_PAD_CFG_LEN, 0x00, 1, 1, 0,
        0x80, /* bus powered, no remote wakeup */
        50,   /* 100 mA, in 2 mA units */
        /* Interface 0: HID, no subclass, no protocol (HID 1.11 s4.2-4.3, p. 8-9 / PDF 18-19) */
        9, 0x04, 0, 0, 1, 0x03, 0x00, 0x00, 0,
        /* HID descriptor, between interface and endpoint (HID 1.11 s7.1, p. 48 / PDF 58) */
        9, 0x21, 0x11, 0x01, 0, 1, 0x22, 0x00, 0x00,
        /* Endpoint 1 IN, interrupt. 10 ms is the shortest period a low-speed
           endpoint may ask for (USB 1.1 s5.7.4, p. 45 / PDF 61). */
        7, 0x05, 0x81, 0x03, 0x00, 0x00, 0
    };

    dev->report_len      = (hid_pad_report_bits(model) + 7) / 8;
    dev->report_desc_len = hid_pad_build_report_desc(model, dev->report_desc);

    memcpy(dev->dev_desc, dev_template, sizeof(dev_template));
    /* 8 is the only size allowed at low speed (USB 1.1 s5.5.3, p. 37 / PDF 53). */
    dev->dev_desc[7]  = model->low_speed ? 8 : 64;
    dev->dev_desc[8]  = model->vid & 0xff;
    dev->dev_desc[9]  = model->vid >> 8;
    dev->dev_desc[10] = model->pid & 0xff;
    dev->dev_desc[11] = model->pid >> 8;

    memcpy(dev->cfg_desc, cfg_template, sizeof(cfg_template));
    dev->cfg_desc[25] = dev->report_desc_len;
    dev->cfg_desc[31] = dev->report_len;
    dev->cfg_desc[33] = model->interval;

    dev->strings[0] = model->manufacturer;
    dev->strings[1] = model->product;
}

/* An axis as an unsigned field of the given width. */
static int
hid_pad_axis_unsigned(int value, int bits)
{
    int ret = (value + 32768) >> (16 - bits);
    int max = (1 << bits) - 1;

    return (ret < 0) ? 0 : ((ret > max) ? max : ret);
}

/* A digital axis has only its ends and a centre. */
static int
hid_pad_axis_digital(int value, int bits)
{
    if (value < -16384)
        return 0;

    return (value > 16384) ? ((1 << bits) - 1) : (1 << (bits - 1));
}

/* Host axes are not always in range, so this keeps a stick in its field. */
static int
hid_pad_axis_signed(int value)
{
    return ((value < -32768) ? -32768 : ((value > 32767) ? 32767 : value)) & 0xffff;
}

/* The hat counts in steps from up, and out of range is released. */
static int
hid_pad_hat(int angle, int directions)
{
    int step = 360 / directions;

    if (angle < 0)
        return 8;

    return ((angle + (step / 2)) % 360) / step;
}

static void
hid_pad_put_bits(uint8_t *rep, int *pos, int value, int bits)
{
    for (int i = 0; i < bits; i++, (*pos)++)
        if (value & (1 << i))
            rep[*pos >> 3] |= 1 << (*pos & 7);
}

static int
hid_pad_report(const hid_pad_t *dev, uint8_t *rep)
{
    const hid_pad_model_t  *model = dev->model;
    const joystick_state_t *state = &joystick_state[0][dev->slot];
    int                     pos   = 0;

    memset(rep, 0x00, dev->report_len);

    for (int i = 0; i < model->num_axes; i++) {
        const hid_pad_axis_t *axis  = &model->axes[i];
        int                   value = state->axis[i];

        switch (axis->kind) {
            case HID_AXIS_DIGITAL:
                value = hid_pad_axis_digital(value, axis->bits);
                break;
            case HID_AXIS_SIGNED:
                value = hid_pad_axis_signed(value);
                break;
            default:
                value = hid_pad_axis_unsigned(value, axis->bits);
                break;
        }

        hid_pad_put_bits(rep, &pos, value, axis->bits);
    }

    if (model->hat)
        hid_pad_put_bits(rep, &pos, hid_pad_hat(state->pov[0], model->hat), 4);

    for (int i = 0; i < model->buttons; i++)
        hid_pad_put_bits(rep, &pos, !!state->button[i], 1);

    return dev->report_len;
}

static int
hid_pad_control(usb_dev_t *usb, const uint8_t *setup, uint8_t *data)
{
    hid_pad_t *dev    = (hid_pad_t *) usb->priv;
    uint16_t   wvalue = setup[2] | (setup[3] << 8);
    uint16_t   windex = setup[4] | (setup[5] << 8);

    /* Every request here is addressed to interface 0, the only one. */
    if (windex != 0)
        return USB_DATA_STALL;

    /* GET_DESCRIPTOR for a HID class descriptor, addressed to the interface
       (HID 1.11 s7.1.1, p. 49 / PDF 59). There is one HID and one Report
       descriptor, so any index but 0 STALLs. The HID descriptor is the copy
       inside the configuration descriptor, at offset 18. */
    if ((setup[0] == 0x81) && (setup[1] == 0x06)) {
        if ((wvalue & 0xff) != 0)
            return USB_DATA_STALL;
        switch (wvalue >> 8) {
            case 0x21:
                memcpy(data, &dev->cfg_desc[18], 9);
                return 9;
            case 0x22:
                memcpy(data, dev->report_desc, dev->report_desc_len);
                return dev->report_desc_len;
            default:
                return USB_DATA_STALL;
        }
    }

    /* GET_PROTOCOL and SET_PROTOCOL are for boot-subclass devices only (HID
       1.11 s7.2.5-7.2.6, p. 54 / PDF 64), so they STALL here. */
    if (setup[0] == 0xa1) {
        switch (setup[1]) {
            case HID_GET_REPORT:
                /* wValue is report type (high) and report ID (low). The
                   only report is Input with no report ID: type 1, ID 0
                   (HID 1.11 s7.2.1, p. 51 / PDF 61). */
                if (wvalue != 0x0100)
                    return USB_DATA_STALL;
                return hid_pad_report(dev, data);
            case HID_GET_IDLE:
                /* The device uses no report IDs, so a nonzero ID names no
                   report (HID 1.11 s7.2.3, p. 52 / PDF 62). */
                if ((wvalue & 0xff) != 0)
                    return USB_DATA_STALL;
                data[0] = dev->idle.rate;
                return 1;
            default:
                return USB_DATA_STALL;
        }
    }

    if (setup[0] == 0x21) {
        switch (setup[1]) {
            case HID_SET_IDLE:
                /* No report IDs, so a nonzero ID names no report; the high
                   byte is the duration (HID 1.11 s7.2.4, p. 52-53 / PDF 62-63). */
                if ((wvalue & 0xff) != 0)
                    return USB_DATA_STALL;
                hid_idle_set(&dev->idle, wvalue >> 8);
                return 0;
            default:
                /* SET_REPORT included: the pad has no Output or Feature report. */
                return USB_DATA_STALL;
        }
    }

    return USB_DATA_STALL;
}

/* Interrupt IN: a report when the state changed, when none was sent yet in
   this configuration, or when the idle period has run out, otherwise NAK
   (HID 1.11 s7.2.4, p. 53 / PDF 63). */
static int
hid_pad_data_in(usb_dev_t *usb, UNUSED(int ep), uint8_t *buf, UNUSED(int maxlen))
{
    hid_pad_t *dev = (hid_pad_t *) usb->priv;
    uint8_t    rep[HID_PAD_REPORT_MAX];
    int        len = hid_pad_report(dev, rep);

    if (!dev->sent_valid || memcmp(rep, dev->sent, len) || hid_idle_due(&dev->idle)) {
        memcpy(dev->sent, rep, len);
        dev->sent_valid = 1;
        memcpy(buf, rep, len);
        hid_idle_reported(&dev->idle);
        return len;
    }

    return USB_DATA_NAK;
}

static void
hid_pad_configured(usb_dev_t *usb, UNUSED(int config))
{
    hid_pad_t *dev = (hid_pad_t *) usb->priv;

    dev->sent_valid = 0;
}

static void
hid_pad_reset(usb_dev_t *usb)
{
    hid_pad_t *dev = (hid_pad_t *) usb->priv;

    /* The recommended default idle rate for joysticks is infinite, rate 0
       (HID 1.11 s7.2.4, p. 53 / PDF 63). */
    dev->idle.rate    = 0;
    dev->idle.pending = 0;
    dev->sent_valid   = 0;
}

static const usb_dev_ops_t hid_pad_ops = {
    .control    = hid_pad_control,
    .data_in    = hid_pad_data_in,
    .data_out   = NULL,
    .reset      = hid_pad_reset,
    .configured = hid_pad_configured
};

static void *
hid_pad_init(const device_t *info)
{
    hid_pad_t *dev = (hid_pad_t *) calloc(1, sizeof(hid_pad_t));

    dev->model = &hid_pad_models[info->local >> 8];
    dev->slot  = info->local & 0xff;
    hid_pad_build_descs(dev);

    dev->usb.name      = info->name;
    dev->usb.low_speed = dev->model->low_speed;
    dev->usb.dev_desc  = dev->dev_desc;
    dev->usb.cfg_desc  = dev->cfg_desc;
    dev->usb.cfg_len   = HID_PAD_CFG_LEN;
    dev->usb.strings   = dev->strings;
    dev->usb.n_strings = 2;
    dev->usb.ops       = &hid_pad_ops;
    dev->usb.priv      = dev;
    usb_dev_init(&dev->usb);
    hid_pad_reset(&dev->usb);

    usb_dev_add(&dev->usb, info);

    return dev;
}

static void
hid_pad_close(void *priv)
{
    hid_pad_t *dev = (hid_pad_t *) priv;

    usb_dev_remove(&dev->usb);
    free(dev);
}

/* Each pad has two devices, so that a second pad gets its own hot-plug slot.
   Only one joystick type is in use at a time, so all models share the slots. */
#define HID_PAD_DEVICES(var, model, pad_name)                  \
    const device_t var##_device = {                            \
        .name          = pad_name,                             \
        .internal_name = "usb_gamepad",                        \
        .flags         = DEVICE_USB,                           \
        .local         = HID_PAD_LOCAL(model, 0),              \
        .init          = hid_pad_init,                         \
        .close         = hid_pad_close,                        \
        .reset         = NULL,                                 \
        .available     = NULL,                                 \
        .speed_changed = NULL,                                 \
        .force_redraw  = NULL,                                 \
        .config        = NULL                                  \
    };                                                         \
    const device_t var##_2_device = {                          \
        .name          = pad_name " (second pad)",             \
        .internal_name = "usb_gamepad_2",                      \
        .flags         = DEVICE_USB,                           \
        .local         = HID_PAD_LOCAL(model, 1),              \
        .init          = hid_pad_init,                         \
        .close         = hid_pad_close,                        \
        .reset         = NULL,                                 \
        .available     = NULL,                                 \
        .speed_changed = NULL,                                 \
        .force_redraw  = NULL,                                 \
        .config        = NULL                                  \
    }

HID_PAD_DEVICES(gamepad_cobra2_usb, HID_PAD_COBRA2, "Creative USB Game Pad II");
HID_PAD_DEVICES(gamepad_usb, HID_PAD_GAMEPAD, "USB Gamepad");
HID_PAD_DEVICES(gamepad_usb_6axis, HID_PAD_6AXIS, "USB Gamepad (6-axis)");
HID_PAD_DEVICES(gamepad_sidewinder_pad, HID_PAD_SIDEWINDER_PAD, "SideWinder Game Pad USB version 1.0");
HID_PAD_DEVICES(gamepad_sidewinder_precision_pro, HID_PAD_SIDEWINDER_PRECISION_PRO, "Microsoft SideWinder Precision Pro (USB)");
HID_PAD_DEVICES(gamepad_sidewinder_freestyle_pro, HID_PAD_SIDEWINDER_FREESTYLE_PRO, "Microsoft SideWinder Freestyle Pro (USB)");
HID_PAD_DEVICES(gamepad_tm_rage3d, HID_PAD_TM_RAGE3D, "ThrustMaster Rage 3D USB Game Pad");
HID_PAD_DEVICES(gamepad_tm_top_gun, HID_PAD_TM_TOP_GUN, "ThrustMaster Top Gun USB Joystick");
HID_PAD_DEVICES(gamepad_tm_top_gun_rev2, HID_PAD_TM_TOP_GUN_REV2, "ThrustMaster Top Gun USB Joystick (rev2)");
HID_PAD_DEVICES(gamepad_nec_pk_gp201, HID_PAD_NEC_PK_GP201, "NEC PK-GP201 PlayStick");
HID_PAD_DEVICES(gamepad_nec_pk_gp301, HID_PAD_NEC_PK_GP301, "NEC PK-GP301 Driving Wheel");
HID_PAD_DEVICES(gamepad_logitech_wingman_extreme, HID_PAD_LOGITECH_WINGMAN_EXTREME, "Logitech USB WingMan Extreme Joystick");
HID_PAD_DEVICES(gamepad_logitech_wingman_extreme_throttle, HID_PAD_LOGITECH_WINGMAN_EXTREME_THROTTLE, "Logitech USB WingMan Extreme Joystick with Throttle");
HID_PAD_DEVICES(gamepad_genius_joystick, HID_PAD_GENIUS_JOYSTICK, "Genius USB Joystick");
HID_PAD_DEVICES(gamepad_genius_gamepad, HID_PAD_GENIUS_GAMEPAD, "Genius USB Game Pad");
HID_PAD_DEVICES(gamepad_guillemot_jet_leader, HID_PAD_GUILLEMOT_JET_LEADER, "Guillemot Jet Leader USB Joystick");
