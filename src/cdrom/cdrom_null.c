/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Null CD-ROM.
 */

#include <stdint.h>
#include <string.h>
//#include <86box/86box.h>
#include <86box/scsi_device.h>
#include <86box/cdrom_ioctl.h>

int cdrom_drive;

static cdrom_ops_t null_atapi;

void
cdrom_null_audio_callback(int16_t *output, int len)
{
    memset(output, 0, len * 2);
}

void
cdrom_null_audio_stop(void)
{
}

static void
null_playaudio(uint32_t pos, uint32_t len, int ismsf)
{
}

static void
null_pause(void)
{
}

static void
null_resume(void)
{
}

static void
null_stop(void)
{
}

static void
null_seek(uint32_t pos)
{
}

static int
null_ready(void)
{
    return 0;
}

/* Always return 0, the contents of a null CD-ROM drive never change. */
static int
null_medium_changed(void)
{
    return 0;
}

static uint8_t
null_getcurrentsubchannel(uint8_t *b, int msf)
{
    return 0x13;
}

static void
null_eject(void)
{
}

static void
null_load(void)
{
}

static int
null_readsector(uint8_t *b, int sector, int count)
{
    return 0;
}

static void
null_readsector_raw(uint8_t *b, int sector)
{
}

static int
null_readtoc(unsigned char *b, unsigned char starttrack, int msf, int maxlen, int single)
{
    return 0;
}

static int
null_readtoc_session(unsigned char *b, int msf, int maxlen)
{
    return 0;
}

static int
null_readtoc_raw(unsigned char *b, int maxlen)
{
    return 0;
}

static uint32_t
null_size(void)
{
    return 0;
}

static int
null_status(void)
{
    return CD_STATUS_EMPTY;
}

void
cdrom_null_reset(void)
{
}

int
cdrom_null_open(char d)
{
    atapi = &null_atapi;
    return 0;
}

void
null_close(void)
{
}

static void
null_exit(void)
{
}

static int
null_is_track_audio(uint32_t pos, int ismsf)
{
    return 0;
}

static cdrom_ops_t null_atapi = {
    null_ready,
    null_medium_changed,
    null_readtoc,
    null_readtoc_session,
    null_readtoc_raw,
    null_getcurrentsubchannel,
    null_readsector,
    null_readsector_raw,
    null_playaudio,
    null_seek,
    null_load,
    null_eject,
    null_pause,
    null_resume,
    null_size,
    null_status,
    null_is_track_audio,
    null_stop,
    null_exit
};
