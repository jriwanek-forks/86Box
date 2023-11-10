/*
 * 86Box	A hypervisor and IBM PC system emulator that specializes in
 *		running old operating systems and software designed for IBM
 *		PC systems and compatibles from 1981 through fairly recent
 *		system designs based on the PCI bus.
 *
 *		This file is part of the 86Box distribution.
 *
 *		Definitions for the Distributed DMA emulation.
 *
 *
 *
 * Authors:	Miran Grca, <mgrca8@gmail.com>
 *
 *		Copyright 2020 Miran Grca.
 */

#ifndef USB_H
#define USB_H

#ifdef __cplusplus
extern "C" {
#endif

enum usb_pid
{
    USB_PID_OUT   = 0xE1,
    USB_PID_IN    = 0x69,
    USB_PID_SETUP = 0x2D
};

enum usb_errors
{
    USB_ERROR_NO_ERROR = 0,
    USB_ERROR_NAK      = 1,
    USB_ERROR_OVERRUN  = 2,
    USB_ERROR_UNDERRUN = 3,
    USB_ERROR_STALL    = 4
};

#pragma pack(push, 1)

/* Base USB descriptor struct. */
typedef struct usb_desc_base_t {
    uint8_t bLength;
    uint8_t bDescriptorType;
} usb_desc_base_t;

enum usb_desc_setup_req_types {
    USB_SETUP_TYPE_DEVICE    = 0x0,
    USB_SETUP_TYPE_INTERFACE = 0x1,
    USB_SETUP_TYPE_ENDPOINT  = 0x2,
    USB_SETUP_TYPE_OTHER     = 0x3,
};

enum usb_desc_setup_reqs
{
    USB_SETUP_GET_STATUS = 0x00,
    USB_SETUP_CLEAR_FEATURE = 0x01,
    USB_SETUP_SET_FEATURE = 0x03,
    USB_SETUP_SET_ADDRESS = 0x05,
    USB_SETUP_GET_DESCRIPTOR = 0x06,
    USB_SETUP_SET_DESCRIPTOR = 0x07,
    USB_SETUP_GET_CONFIGURATION = 0x08,
    USB_SETUP_SET_CONFIGURATION = 0x09,
    USB_SETUP_GET_INTERFACE = 0x0A,
    USB_SETUP_SET_INTERFACE = 0x0B,
    USB_SETUP_SYNCH_FRAME = 0x0C
};

enum usb_desc_setup_reqs_hid
{
    USB_SETUP_HID_GET_REPORT = 0x01,
    USB_SETUP_HID_SET_REPORT = 0x09,
    USB_SETUP_HID_GET_IDLE = 0x02,
    USB_SETUP_HID_SET_IDLE = 0x0a,
    USB_SETUP_HID_GET_PROTOCOL = 0x03,
    USB_SETUP_HID_SET_PROTOCOL = 0x0b,
};

#define USB_SETUP_TYPE_MAX 0x1F

#define USB_SETUP_DEV_TO_HOST 0x80

typedef struct usb_desc_setup_t {
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} usb_desc_setup_t;

typedef struct usb_desc_endpoint_t {
    usb_desc_base_t base;
    uint8_t         bEndpointAddress;
    uint8_t         bmAttributes;
    uint16_t        wMaxPacketSize;
    uint8_t         bInterval;
} usb_desc_endpoint_t;

typedef struct usb_desc_hid_t {
    usb_desc_base_t base;

    uint16_t bcdHID;
    uint8_t  bCountryCode;
    uint8_t  bNumDescriptors;
    uint8_t  bDescriptorType;
    uint16_t wDescriptorLength;
} usb_desc_hid_t;

typedef struct usb_desc_interface_t {
    usb_desc_base_t base;

    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
} usb_desc_interface_t;

typedef struct usb_desc_string_t {
    usb_desc_base_t base;
    uint16_t        bString[];
} usb_desc_string_t;

typedef struct usb_desc_conf_t {
    usb_desc_base_t base;

    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  bMaxPower;
} usb_desc_conf_t;

typedef struct usb_desc_device_t {
    usb_desc_base_t base;

    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
} usb_desc_device_t;

/* Fake descriptor with type of 0xFF, used as pointers to real USB descriptors that doesn't follow the length/type pair.
   Length describes length in bytes of data pointed to by "ptr" variable. */
typedef struct
{
    usb_desc_base_t base;

    uint8_t* ptr;
} usb_desc_ptr_t;

#pragma pack(pop)

/* USB endpoint device struct. Incomplete and unused. */
typedef struct usb_device_t {
    usb_desc_device_t device_desc;
    struct {
        usb_desc_conf_t        conf_desc;
        const usb_desc_base_t *other_descs[16];
    } conf_desc_items;

    const usb_desc_string_t* string_desc[256];
    uint16_t interface_altsetting[0x10000];

    /* General-purpose function for I/O. Non-zero value indicates error. */
    uint8_t (*device_process)(void* priv, uint8_t* data, uint32_t *len, uint8_t pid_token, uint8_t endpoint, uint8_t underrun_not_allowed);
    /* Device reset. */
    void (*device_reset)(void* priv);
    /* Current address of device */
    uint8_t address;
    /* Buffer for endpoint 0 setups/ins/outs */
    Fifo8 fifo;
    uint16_t status_bits;
    uint8_t control_endpoint_pid;
    uint8_t current_configuration;
    usb_desc_setup_t setup_desc;
    
    void* priv;
} usb_device_t;

typedef struct UHCIPort
{
    uint16_t ctrl;
    usb_device_t* dev;
} UHCIPort;

struct usb_t;
typedef struct usb_t usb_t;

typedef struct UHCIState {
    pc_timer_t frame_timer;
    uint32_t pending_int_mask;
    uint16_t cmd; /* cmd register */
    uint16_t status;
    uint16_t intr; /* interrupt enable register */
    uint16_t frnum; /* frame number */
    uint32_t fl_base_addr; /* frame list base address */
    uint32_t frame_bytes;
    uint8_t sof_timing;
    uint8_t status2; /* bit 0 and 1 are used to generate UHCI_STS_USBINT */
    uint8_t irq_state;

    UHCIPort ports[2];
    usb_t* dev;
} UHCIState;

typedef struct usb_params_t
{
    int pci_slot;
    uint8_t* pci_regs;
} usb_params_t;

typedef struct usb_t {
    UHCIState     uhci_state;
    uint8_t       ohci_mmio[4096];
    uint16_t      uhci_io_base;
    int           uhci_enable;
    int           ohci_enable;
    uint32_t      ohci_mem_base;
    mem_mapping_t ohci_mmio_mapping;
    usb_params_t params;
} usb_t;

/* Global variables. */
extern const device_t usb_device;

/* Functions. */
extern void uhci_update_io_mapping(usb_t *dev, uint8_t base_l, uint8_t base_h, int enable);
extern void ohci_update_mem_mapping(usb_t *dev, uint8_t base1, uint8_t base2, uint8_t base3, int enable);

#ifdef __cplusplus
}
#endif

#endif /*USB_H*/
