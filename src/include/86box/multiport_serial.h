#ifndef EMU_MULTIPORT_SERIAL_H
#define EMU_MULTIPORT_SERIAL_H

/* Registry of ISA multiport serial adapters offered in the peripheral UI. */
extern int                multiport_serial_count(void);
#ifdef EMU_DEVICE_H
extern const device_t    *multiport_serial_get_device(int id);
extern const device_t     stallion_easyio_device;
extern const device_t     ast_fourport_device;
extern const device_t     accent_async_device;
extern const device_t     boca_serial_device;
extern const device_t     hub6_serial_device;
extern const device_t     exar_st16c554_device;
#endif
extern const char         *multiport_serial_get_name(int id);
extern const char         *multiport_serial_get_internal_name(int id);
extern int                 multiport_serial_get_from_internal_name(const char *name);

#endif
