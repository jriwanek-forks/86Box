#include <stdint.h>
#include <string.h>
#include <86box/device.h>
#include <86box/multiport_serial.h>

typedef struct multiport_serial_entry_s {
    const device_t *device;
    uint8_t channels;
} multiport_serial_entry_t;

/* Append supported boards here; IDs are persisted in configs by name. */
static const multiport_serial_entry_t devices[] = {
    { NULL, 0 },
    { &stallion_easyio_device, 8 },
    { &ast_fourport_device, 4 },
    { &accent_async_device, 2 },
    { &boca_serial_device, 16 },
    { &boca_bb1004_device, 4 },
    { &boca_bb1008_device, 8 },
    { &hub6_serial_device, 12 },
    { &exar_st16c554_device, 4 },
    { &cyclades_cyclomy_device, 32 },
    { &moxa_c168_isa_device, 8 },
    { &moxa_c104_isa_device, 4 },
    { &moxa_ci104j_isa_device, 4 },
    { &moxa_c102_isa_device, 4 },
    { &moxa_ci132_isa_device, 4 },
    { &moxa_ci134_isa_device, 4 },
    { &stallion_ech_device, 32 }
};

int
multiport_serial_count(void)
{
    return (int) (sizeof(devices) / sizeof(devices[0])) - 1;
}

int
multiport_serial_get_channel_count(int id)
{
    return (id > 0 && id <= multiport_serial_count()) ? devices[id].channels : 0;
}

const device_t *
multiport_serial_get_device(int id)
{
    return (id > 0 && id <= multiport_serial_count()) ? devices[id].device : NULL;
}

const char *
multiport_serial_get_name(int id)
{
    const device_t *device = multiport_serial_get_device(id);
    return device ? device->name : NULL;
}

const char *
multiport_serial_get_internal_name(int id)
{
    const device_t *device = multiport_serial_get_device(id);
    return device ? device->internal_name : NULL;
}

int
multiport_serial_get_from_internal_name(const char *name)
{
    if (!name)
        return 0;
    for (int id = 1; id <= multiport_serial_count(); id++)
        if (!strcmp(name, devices[id].device->internal_name))
            return id;
    return 0;
}
