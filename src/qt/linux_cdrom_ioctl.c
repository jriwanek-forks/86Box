/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Linux CD-ROM support via IOCTL.
 *
 * Authors: GitHub Copilot Assistant
 *
 *          Copyright 2025 86Box contributors.
 */
#define HAVE_STDARG_H
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <scsi/sg.h>

#ifdef ENABLE_IOCTL_LOG
#    include <stdarg.h>
#endif

#include <stdio.h>
#include <stdint.h>
#include <limits.h>

#include <86box/86box.h>
#include <86box/cdrom.h>
#include <86box/log.h>
#include <86box/plat_cdrom_ioctl.h>
#include <86box/scsi_device.h>

#include <linux/cdrom.h>

typedef struct linux_ioctl_t {
    cdrom_t          *dev;
    void             *log;
    int               is_dvd;
    int               has_audio;
    int               blocks_num; /* Number of TOC entries like Windows */
    raw_track_info_t *tracks;
    int               fd;
    char              path[256];
} linux_ioctl_t;

// caching data for performance
int      prev_track_idx;
uint32_t prev_sector;

// SCSI command for READ TOC/PMA/ATIP
typedef struct {
    uint8_t  opcode;    // 0x43
    uint8_t  reserved1; // MSF bit in bit 1
    uint8_t  format;    // Format field
    uint8_t  reserved2[3];
    uint8_t  start_track;
    uint16_t alloc_length; // Big endian
    uint8_t  control;
} __attribute__((packed)) scsi_read_toc_cmd_t;

// SCSI command for READ CD
typedef struct {
    uint8_t  opcode;     // 0xBE
    uint8_t  misc;       // Expected sector type, etc.
    uint32_t lba;        // Big endian
    uint8_t  len[3];     // Big endian, transfer length in sectors
    uint8_t  flags;      // Main channel selection
    uint8_t  subchannel; // Sub-channel selection
    uint8_t  control;
} __attribute__((packed)) scsi_read_cd_cmd_t;

// TOC response header
typedef struct {
    uint16_t data_length; // Big endian
    uint8_t  first_session;
    uint8_t  last_session;
} __attribute__((packed)) toc_header_t;

#ifdef ENABLE_IOCTL_LOG
int ioctl_do_log = ENABLE_IOCTL_LOG;

static void
linux_ioctl_log(void *priv, const char *fmt, ...)
{
    va_list ap;

    if (ioctl_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define linux_ioctl_log(priv, fmt, ...)
#endif

/* Internal functions. */
uint16_t
cpu_to_be16(uint16_t val)
{ // Convert host byte order to big endian 16-bit
    return ((val & 0xFF) << 8) | ((val >> 8) & 0xFF);
}

uint32_t
cpu_to_be32(uint32_t val)
{ // Convert host byte order to big endian 32-bit
    return ((val & 0xFF) << 24) | ((val & 0xFF00) << 8) | ((val >> 8) & 0xFF00) | ((val >> 24) & 0xFF);
}

uint16_t
be16_to_cpu(uint16_t val)
{ // Convert big endian 16-bit to host byte order
    return ((val & 0xFF) << 8) | ((val >> 8) & 0xFF);
}

static void
linux_ioctl_close_handle(const linux_ioctl_t *dev_ioctl)
{
    if (dev_ioctl->fd >= 0) {
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: Closing device fd=%d\n", dev_ioctl->fd);
        close(dev_ioctl->fd);
    }
}

static void
linux_ioctl_set_max_speed(linux_ioctl_t *dev_ioctl)
{
    // Method 1: Set to maximum speed (-1)
    if (ioctl(dev_ioctl->fd, CDROM_SELECT_SPEED, -1) == 0) {
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: Set to maximum speed\n");
        return;
    }

    // Method 2: Try common high speeds in descending order
    int speeds[] = { 52, 48, 40, 32, 24, 16, 12, 8, 4, 2, 1, 0 };
    for (int i = 0; speeds[i] > 0; i++) {
        if (ioctl(dev_ioctl->fd, CDROM_SELECT_SPEED, speeds[i]) == 0) {
            linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: Set speed to %dx\n", speeds[i]);
            return;
        }
    }

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: Failed to set any speed\n");
}

static int
linux_ioctl_open_handle(linux_ioctl_t *dev_ioctl)
{
    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: Opening device: %s\n", dev_ioctl->path);

    // Use regular blocking open since we'll use async I/O for reads
    dev_ioctl->fd = open(dev_ioctl->path, O_RDONLY);

    if (dev_ioctl->fd < 0) {
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: Failed to open %s: %s\n",
                        dev_ioctl->path, strerror(errno));
        return 0;
    }

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: Successfully opened device fd=%d\n", dev_ioctl->fd);
    return 1;
}

static int
linux_ioctl_read_raw_toc(linux_ioctl_t *dev_ioctl)
{
    scsi_read_toc_cmd_t cdb;
    sg_io_hdr_t         sg_hdr;
    uint8_t             sense_buffer[32];
    uint8_t             data_buffer[2048];
    toc_header_t       *header;

    dev_ioctl->is_dvd     = 0; // TODO: Support dvd
    dev_ioctl->has_audio  = 0;
    dev_ioctl->blocks_num = 0;  /* This will track TOC entries */
    prev_sector           = -1; // Invalidate cache
    prev_track_idx        = -1;

    // Prepare SCSI command - READ TOC with format 2 (raw TOC)
    memset(&cdb, 0, sizeof(cdb));
    cdb.opcode       = 0x43; // READ TOC/PMA/ATIP
    cdb.reserved1    = 0x02; // MSF bit set (bit 1)
    cdb.format       = 0x02; // Format 2: Raw TOC
    cdb.start_track  = 0x01; // Starting session
    cdb.alloc_length = cpu_to_be16(sizeof(data_buffer));
    cdb.control      = 0;

    // Prepare SG_IO header
    memset(&sg_hdr, 0, sizeof(sg_hdr));
    sg_hdr.interface_id    = 'S';
    sg_hdr.cmd_len         = sizeof(cdb);
    sg_hdr.mx_sb_len       = sizeof(sense_buffer);
    sg_hdr.dxfer_direction = SG_DXFER_FROM_DEV;
    sg_hdr.dxfer_len       = sizeof(data_buffer);
    sg_hdr.dxferp          = data_buffer;
    sg_hdr.cmdp            = (unsigned char *) &cdb;
    sg_hdr.sbp             = sense_buffer;
    sg_hdr.timeout         = 10000; // 10 seconds

    // Execute SCSI command
    if (ioctl(dev_ioctl->fd, SG_IO, &sg_hdr) < 0) {
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: SG_IO ioctl failed: %s\n", strerror(errno));
        close(dev_ioctl->fd);
        return -1;
    }

    // Check for SCSI errors
    if (sg_hdr.status != 0 || sg_hdr.host_status != 0 || sg_hdr.driver_status != 0) {
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: SCSI command failed: status=0x%02x, host_status=0x%04x, driver_status=0x%04x\n",
                        sg_hdr.status, sg_hdr.host_status, sg_hdr.driver_status);

        if (sg_hdr.sb_len_wr > 0) {
            linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: Sense data: ");
            for (int i = 0; i < sg_hdr.sb_len_wr && i < 16; i++)
                linux_ioctl_log(dev_ioctl->log, "%02x ", sense_buffer[i]);

            linux_ioctl_log(dev_ioctl->log, "\n");
        }
        close(dev_ioctl->fd);
        return -1;
    }

    // Parse response
    header               = (toc_header_t *) data_buffer;
    uint16_t data_length = be16_to_cpu(header->data_length);

    linux_ioctl_log("TOC Data Length: %d bytes\n", data_length);
    linux_ioctl_log("Sessions: %d to %d\n", header->first_session, header->last_session);

    // Calculate number of descriptors
    if (data_length < sizeof(toc_header_t)) {
        linux_ioctl_log(dev_ioctl->log, "Invalid TOC data length %d\n", data_length);
        close(dev_ioctl->fd);
        return -1;
    }

    dev_ioctl->blocks_num = (data_length - sizeof(toc_header_t) + 2) / sizeof(raw_track_info_t);

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: Retrieved %d TOC entries\n", dev_ioctl->blocks_num);

    if (dev_ioctl->blocks_num <= 0) {
        linux_ioctl_log(dev_ioctl->log, "No TOC descriptors found\n");
        close(dev_ioctl->fd);
        return -1;
    }

    dev_ioctl->tracks = malloc(dev_ioctl->blocks_num * sizeof(raw_track_info_t));
    if (!dev_ioctl->tracks) {
        linux_ioctl_log(dev_ioctl->log, "Memory allocation failure for TOC entries\n");
        close(dev_ioctl->fd);
        return -1;
    }

    // Cast the data buffer directly to our tracks array structure
    raw_track_info_t *toc_data = (raw_track_info_t *) (data_buffer + sizeof(toc_header_t));
    memcpy(dev_ioctl->tracks, toc_data, dev_ioctl->blocks_num * sizeof(raw_track_info_t));

    // Determine first and last track numbers
    for (int i = 0; i < dev_ioctl->blocks_num; i++) {
        if (dev_ioctl->tracks[i].adr_ctl & 0xF & 4) {
            dev_ioctl->has_audio = 1;
            break;
        }
    }

#ifdef ENABLE_IOCTL_LOG
    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: TOC Summary - %d entries found:\n", dev_ioctl->blocks_num);
    for (int i = 0; i < dev_ioctl->blocks_num; i++) {
        const raw_track_info_t *track = &dev_ioctl->tracks[i]; // Now you can use array indexing
        uint8_t                 adr   = (track->adr_ctl >> 4) & 0xF;
        uint8_t                 ctl   = track->adr_ctl & 0xF;

        if (adr == 1) { // Position information
            if (track->point >= 1 && track->point <= 99) {
                linux_ioctl_log(dev_ioctl->log, "Track %02d: Start at %02d:%02d:%02d, %s, %s (index %d)\n",
                                track->point, track->pm, track->ps, track->pf,
                                (ctl & 4) ? "DATA" : "AUDIO",
                                (ctl & 2) ? "DIGITAL_COPY_PERMITTED" : "", i);
            } else if (track->point == 0xA0)
                linux_ioctl_log(dev_ioctl->log, "First track: %02d, Disc type: %02X (index %d)\n", track->pm, track->ps, i);
            else if (track->point == 0xA1)
                linux_ioctl_log(dev_ioctl->log, "Last track: %02d (index %d)\n", track->pm, i);
            else if (track->point == 0xA2)
                linux_ioctl_log(dev_ioctl->log, "Lead-out start: %02d:%02d:%02d (index %d)\n", track->pm, track->ps, track->pf, i);
        }
    }
#endif

    return dev_ioctl->blocks_num;
}

static int
linux_ioctl_get_track_index(const linux_ioctl_t *dev_ioctl, const uint32_t sector)
{
    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: ioctl_get_track_index() searching for sector %u\n", sector);

    if (sector == prev_sector) {
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: ioctl_get_track_index() cache hit for sector %u, returning index %d\n",
                        sector, prev_track_idx);
        return prev_track_idx;
    }

    /* Sanity check for ridiculous sector numbers */
    if (sector > 90 * 60 * 75) { /* 90 minute CD max */
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: ioctl_get_track_index() sector %u too large, returning -1\n", sector);
        return -1;
    }

    if (!dev_ioctl->tracks || dev_ioctl->blocks_num <= 0) {
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: ioctl_get_track_index() no tracks data\n");
        return -1;
    }

    /* Optimized search: find the track with highest start <= sector */
    int      best_track = -1;
    uint32_t best_start = 0;

    for (int i = 0; i < dev_ioctl->blocks_num; i++) {
        const raw_track_info_t *ct = &(dev_ioctl->tracks[i]);

        /* Only consider actual track entries (point 1-99), skip TOC control entries */
        if (ct->point >= 1 && ct->point <= 99) {
            const uint32_t start = MSFtoLBA(ct->pm, ct->ps, ct->pf) - 150;

            /* If track start is beyond our sector, we can't use this track */
            if (start > sector)
                continue;

            /* If this start is better than our current best, use it */
            if (start >= best_start) {
                best_track = i;
                best_start = start;

                /* Perfect match - this track starts exactly at our sector */
                if (start == sector) {
                    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: track %d: perfect match at start=%u\n", i, start);
                    break;
                }
            }
        }
    }

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: ioctl_get_track_index(%u) found index %d (start=%u)\n",
                    sector, best_track, best_start);

    prev_sector    = sector;
    prev_track_idx = best_track;
    return best_track;
}

static int
linux_ioctl_is_track_audio(const linux_ioctl_t *dev_ioctl, const uint32_t pos)
{
    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: ioctl_is_track_audio() called for sector %u\n", pos);
    int ret = 0;

    /* Sanity check for ridiculous sector numbers */
    if (pos > 90 * 60 * 75) { /* 90 minute CD max */
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: ioctl_is_track_audio() sector %u too large, returning 0\n", pos);
        return 0;
    }

    /* Additional check for clearly corrupted values */
    if (pos > 500000) { /* Even larger than any reasonable CD */
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: ioctl_is_track_audio() sector %u appears corrupted, returning 0\n", pos);
        return 0;
    }

    const int idx = linux_ioctl_get_track_index(dev_ioctl, pos);
    if (idx != -1) {
        const raw_track_info_t *track   = &dev_ioctl->tracks[idx];
        const int               control = track->adr_ctl & 0xF; // Extract control bits
        ret                             = !(control & 4);

        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: ioctl_is_track_audio(%08X): track_idx=%d, control=0x%02X, ret=%i\n",
                        pos, idx, control, ret);
        return ret;
    }

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: ioctl_is_track_audio(%08X): no track found, ret=%i\n", pos, ret);
    return ret;
}

/* Shared functions. */
static int
linux_ioctl_get_track_info(const void *local, const uint32_t track,
                           int end, track_info_t *ti)
{
    const linux_ioctl_t *dev_ioctl = (const linux_ioctl_t *) local;
    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: get_track_info() called for track %u\n", track);

    for (int i = 0; i < dev_ioctl->blocks_num; i++) {
        const raw_track_info_t *ct = &(dev_ioctl->tracks[i]);
        if (ct->point == track) {
            ti->number = track;
            ti->attr   = ct->adr_ctl;
            ti->m      = ct->pm;
            ti->s      = ct->ps;
            ti->f      = ct->pf;
            return 1;
        }
    }

    return -1;
}

static void
linux_ioctl_get_raw_track_info(const void *local, int *num, uint8_t *rti)
{
    const linux_ioctl_t *dev_ioctl = (const linux_ioctl_t *) local;

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: get_raw_track_info() called\n");

    *num = dev_ioctl->blocks_num; /* Number of TOC entries */
    memcpy(rti, dev_ioctl->tracks, dev_ioctl->blocks_num * sizeof(raw_track_info_t));

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: Returning %d raw track info blocks\n", *num);
    return;
}

static int
linux_ioctl_is_track_pre(const void *local, const uint32_t sector)
{
    const linux_ioctl_t *dev_ioctl = (const linux_ioctl_t *) local;

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: is_track_pre() called for sector %u\n", sector);

    /* Linux doesn't easily provide pre-emphasis info, return 0 */
    return 0;
}

static uint32_t
linux_ioctl_get_last_block(const void *local)
{
    const linux_ioctl_t *dev_ioctl = (const linux_ioctl_t *) local;

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: get_last_block() called\n");

    for (int i = 0; i < dev_ioctl->blocks_num; i++) {
        const raw_track_info_t *ct = &(dev_ioctl->tracks[i]);
        if (ct->point == 0xA2) { // Lead-out track
            uint32_t lb = MSFtoLBA(ct->pm, ct->ps, ct->pf) - 150;
            linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: Found A2 entry at index %d, MSF=%02d:%02d:%02d, LB=%d\n",
                            i, ct->pm, ct->ps, ct->pf, lb);
            linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: get_last_block() returning %d\n", lb);
            return lb;
        }
    }

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: No A2 entry found, returning 0\n");
    return -1;
}

static int
linux_ioctl_read_sector(const void *local, uint8_t *buffer, const uint32_t sector)
{
    const linux_ioctl_t *dev_ioctl         = (const linux_ioctl_t *) local;
    const int            sc_offs           = (sector == 0xffffffff) ? 0 : 2352;
    int                  len               = (sector == 0xffffffff) ? 16 : 2368;
    uint32_t             lba               = sector;
    int                  track             = -1;
    int                  toc_index         = -1;
    int                  ret               = 0;
    int                  bytes_transferred = 0;

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: read_sector() called for sector %u\n", sector);

    if (!dev_ioctl || dev_ioctl->fd < 0 || !buffer) {
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: read_sector() invalid parameters: dev_ioctl=%p, fd=%d, buffer=%p\n",
                        (void *) dev_ioctl, dev_ioctl ? dev_ioctl->fd : -1, (void *) buffer);
        return 0;
    }

    if (lba == 0xffffffff) {
        lba   = dev_ioctl->dev->seek_pos;
        track = linux_ioctl_get_track_index(dev_ioctl, lba);
        if (track == -1)
            return 0;

        toc_index         = track;
        len               = 16;
        ret               = 1;
        bytes_transferred = len;
    } else {
        /* Bounds check */
        uint32_t disc_capacity = linux_ioctl_get_last_block(local);
        if (disc_capacity > 0 && sector >= disc_capacity) {
            linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: Sector %u beyond disc end (%u blocks)\n",
                            sector, disc_capacity);
            return 0;
        }

        track = linux_ioctl_get_track_index(dev_ioctl, lba);
        if (track == -1)
            return 0;

        toc_index = track;

        /* Use SCSI READ CD command like Windows implementation */
        scsi_read_cd_cmd_t cdb;
        sg_io_hdr_t        sg_hdr;
        uint8_t            sense_buffer[32];

        memset(&cdb, 0, sizeof(cdb));
        cdb.opcode = 0xBE; /* READ CD */
        cdb.misc   = 0x00;
        cdb.lba    = cpu_to_be32(sector); /* Starting LBA in big endian */
        cdb.len[0] = 0x00;
        cdb.len[1] = 0x00;
        cdb.len[2] = 0x01; /* Transfer length: 1 sector */
        /* If sector is FFFFFFFF, only return the subchannel, otherwise return full sector */
        cdb.flags      = (sector == 0xffffffff) ? 0x00 : 0xf8; /* Main channel selection */
        cdb.subchannel = 0x02;                                 /* Raw subchannel data */
        cdb.control    = 0x00;

        /* Prepare SG_IO header */
        memset(&sg_hdr, 0, sizeof(sg_hdr));
        sg_hdr.interface_id    = 'S';
        sg_hdr.cmd_len         = sizeof(cdb);
        sg_hdr.mx_sb_len       = sizeof(sense_buffer);
        sg_hdr.dxfer_direction = SG_DXFER_FROM_DEV;
        sg_hdr.dxfer_len       = len;
        sg_hdr.dxferp          = buffer;
        sg_hdr.cmdp            = (unsigned char *) &cdb;
        sg_hdr.sbp             = sense_buffer;
        sg_hdr.timeout         = 6000; /* 6 seconds like Windows */

        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: SCSI READ CD: LBA=%u, len=%d, flags=0x%02X\n",
                        sector, len, cdb.flags);

        /* Execute SCSI command */
        if (ioctl(dev_ioctl->fd, SG_IO, &sg_hdr) < 0) {
            linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: SG_IO ioctl failed: %s\n", strerror(errno));
            return 0;
        }

        /* Check for SCSI errors like Windows implementation */
        if (sg_hdr.status != 0 || sg_hdr.host_status != 0 || sg_hdr.driver_status != 0) {
            linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: SCSI READ CD failed: status=0x%02x, host_status=0x%04x, driver_status=0x%04x\n",
                            sg_hdr.status, sg_hdr.host_status, sg_hdr.driver_status);

            if (sg_hdr.sb_len_wr >= 16) {
                /* Check for CIRC error like Windows - treat as error to indicate CIRC error to guest */
                if ((sense_buffer[2] == 0x03) && (sense_buffer[12] == 0x11)) {
                    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: CIRC error detected\n");
                    return 0;
                }

                linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: Sense data: ");
                for (int i = 0; i < sg_hdr.sb_len_wr && i < 16; i++) {
                    linux_ioctl_log(dev_ioctl->log, "%02x ", sense_buffer[i]);
                }
                linux_ioctl_log(dev_ioctl->log, "\n");
            }
            return 0;
        }

        bytes_transferred = len - sg_hdr.resid;
        ret               = (sg_hdr.resid == 0 || bytes_transferred >= len) ? 1 : 0;

        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: SCSI READ CD success: transferred %d bytes, resid=%d\n",
                        bytes_transferred, sg_hdr.resid);

        /* Debug: Show the actual data we read for sector 16 */
        if (sector == 16 && ret) {
            linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: Sector 16 raw data: ");
            for (int i = 16; i < 32; i++)
                linux_ioctl_log(dev_ioctl->log, "%02X ", buffer[i]);

            linux_ioctl_log(dev_ioctl->log, " (ASCII: ");
            for (int i = 16; i < 32; i++)
                linux_ioctl_log(dev_ioctl->log, "%c", (buffer[i] >= 32 && buffer[i] < 127) ? buffer[i] : '.');

            linux_ioctl_log(dev_ioctl->log, ")\n");
        }
    }

    /* Construct raw subchannel data from Q subchannel if we have track info (like Windows) */
    if (ret && toc_index != -1 && bytes_transferred >= len) {

        /* Windows-style subchannel construction - only if not already present */
        if (sector != 0xffffffff) {
            for (int i = 11; i >= 0; i--) {
                for (int j = 7; j >= 0; j--)
                    buffer[2352 + (i * 8) + j] = ((buffer[sc_offs + i] >> (7 - j)) & 0x01) << 6;
            }
        }
    }

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: read_sector final result: ret=%d, len=%d\n", ret, len);

    return ret ? 1 : -1;
}

static uint8_t
linux_ioctl_get_track_type(const void *local, const uint32_t sector)
{
    const linux_ioctl_t *dev_ioctl = (const linux_ioctl_t *) local;

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: get_track_type() called for sector %u\n", sector);

    /* Sanity check for ridiculous sector numbers */
    if (sector > 90 * 60 * 75) { /* 90 minute CD max */
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: get_track_type() sector %u too large, returning 0x00\n", sector);
        return 0x00;
    }

    /* Additional check for clearly corrupted values */
    if (sector > 500000) { /* Even larger than any reasonable CD */
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: get_track_type() sector %u appears corrupted, returning 0x00\n", sector);
        return 0x00;
    }

    int track_idx = linux_ioctl_get_track_index(dev_ioctl, sector);
    if (track_idx == -1) {
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: get_track_type() no track found, returning 0x00\n");
        return 0x00;
    }

    uint8_t ret;
    if (linux_ioctl_is_track_audio(dev_ioctl, sector)) {
        ret = CD_TRACK_AUDIO;
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: get_track_type() audio track, returning 0x%02X\n", ret);
    } else {
        // For data tracks, return CD_TRACK_NORMAL (Mode 1)
        ret = CD_TRACK_NORMAL;
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: get_track_type() data track, returning 0x%02X\n", ret);
    }

    return ret;
}

static int
linux_ioctl_read_dvd_structure(const void *local, const uint8_t layer, const uint8_t format,
                               uint8_t *buffer, uint32_t *info)
{
    const linux_ioctl_t *dev_ioctl = (const linux_ioctl_t *) local;

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: read_dvd_structure() called for layer %u, format %u\n",
                    layer, format);
    /* This is not a DVD, return 0. */
    return 0;
}

static int
linux_ioctl_is_dvd(const void *local)
{
    const linux_ioctl_t *dev_ioctl = (const linux_ioctl_t *) local;

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: is_dvd() called\n");

    return dev_ioctl->is_dvd;
}

static int
linux_ioctl_has_audio(const void *local)
{
    const linux_ioctl_t *dev_ioctl = (const linux_ioctl_t *) local;

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: has_audio() called\n");

    return dev_ioctl->has_audio;
}

static int
linux_ioctl_is_empty(const void *local)
{
    const linux_ioctl_t *dev_ioctl = (const linux_ioctl_t *) local;

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: is_empty() called\n");

    if (dev_ioctl->fd < 0)
        return 1;

    if (ioctl(dev_ioctl->fd, CDROM_DRIVE_STATUS, CDSL_CURRENT) != CDS_DISC_OK) {
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: No disc in drive\n");
        return 1;
    }

    return 0;
}

static void
linux_ioctl_close(void *local)
{
    linux_ioctl_t *dev_ioctl = (linux_ioctl_t *) local;

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: close() called\n");

    if (dev_ioctl) {
        linux_ioctl_close_handle(dev_ioctl);
        free(dev_ioctl);
    }
}

static void
linux_ioctl_load(const void *local)
{
    linux_ioctl_t *dev_ioctl = (linux_ioctl_t *) local;

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: load() called\n");

    if (!dev_ioctl || dev_ioctl->fd < 0)
        return;

    linux_ioctl_set_max_speed(dev_ioctl);

    linux_ioctl_read_raw_toc(dev_ioctl);
}

static const cdrom_ops_t linux_ioctl_ops = {
    linux_ioctl_get_track_info,
    linux_ioctl_get_raw_track_info,
    linux_ioctl_is_track_pre,
    linux_ioctl_read_sector,
    linux_ioctl_get_track_type,
    linux_ioctl_get_last_block,
    linux_ioctl_read_dvd_structure,
    linux_ioctl_is_dvd,
    linux_ioctl_has_audio,
    linux_ioctl_is_empty,
    linux_ioctl_close,
    linux_ioctl_load
};

/* Public functions. */
void *
ioctl_open(cdrom_t *dev, const char *drv)
{
    linux_ioctl_t *dev_ioctl;

    dev_ioctl = (linux_ioctl_t *) calloc(1, sizeof(linux_ioctl_t));
    if (!dev_ioctl)
        return NULL;

    dev_ioctl->dev = dev;
    dev_ioctl->fd  = -1;

    /* Copy device path */
    strncpy(dev_ioctl->path, drv, sizeof(dev_ioctl->path) - 1);
    dev_ioctl->path[sizeof(dev_ioctl->path) - 1] = '\0';

    if (!linux_ioctl_open_handle(dev_ioctl)) {
        free(dev_ioctl);
        return NULL;
    }

    /* Set up the device operations */
    dev->ops   = &linux_ioctl_ops;
    dev->local = dev_ioctl;

    linux_ioctl_load(dev_ioctl);

    /* Validate device state before completing initialization */
    if (dev_ioctl->blocks_num <= 0) {
        linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: No TOC entries, aborting\n");
        linux_ioctl_close_handle(dev_ioctl);
        free(dev_ioctl);
        return NULL;
    }

    linux_ioctl_log(dev_ioctl->log, "Linux IOCTL: CD-ROM opened successfully on %s\n",
                    dev_ioctl->path);

    return dev_ioctl;
}
