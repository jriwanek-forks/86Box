/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of removable disk drives with SCSI(-like)
 *          commands, for both ATAPI and SCSI usage.
 *
 * Authors: Miran Grca, <mgrca8@gmail.com>
 *          Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2018-2025 Miran Grca.
 *          Copyright 2022-2026 Jasmine Iwanek.
 */
#ifndef EMU_RDISK_H
#define EMU_RDISK_H

#define RDISK_NUM                  4

#define BUF_SIZE               32768

#define RDISK_TIME              10.0

#define ZIP_SECTORS      (96 * 2048)

#define ZIP_250_SECTORS     (489532)

/* Estimated.  A real Zip 750 (Aaru device report 53, only tested with Zip 100 media)
   reports a flexible disk page geometry of 718 cylinders x 64 heads x 32 sectors, i.e. 1,470,464
   sectors.  The Zip 250's block count is slightly higher than its page geometry (489,532 vs.
   489,472), so the real value is probably at or a little above 1,470,464, not below it.  Confirm
   with a real READ CAPACITY before relying on this for image sizes. */
#define ZIP_750_SECTORS     (1468596)

/* 393,380 blocks per Aaru device report 118, a real Sony HiFD SFD200 with a HiFD medium
   (13 heads x 890 cylinders x 34 sectors). */
#define HIFD_SECTORS            (393380)

/* 40,662 blocks per Aaru device report 258, a real INSITE I325VM Floptical with a 20 MB medium. */
#define FLOPTICAL_SECTORS       (40662)

/* 175,856 blocks per Aaru device reports 186 and 202 (8 heads x 758 cylinders x 29 sectors). */
#define BERNOULLI_90_SECTORS    (175856)

/* ESTIMATED.  No report captured the block count of a 150 MB or 230 MB Bernoulli cartridge, so these are
   simply the marketing capacities (150,000,000 / 512 and 230,000,000 / 512).  The 90 MB cartridge's real
   size is 0.04% above its marketing figure, so the real values are probably slightly higher.  Replace
   them with the real READ CAPACITY values before relying on them for image sizes. */
#define BERNOULLI_150_SECTORS   (292969)
#define BERNOULLI_230_SECTORS   (449219)

/* 4,307,184 blocks per Aaru device report 200, a real Castlewood ORB 2.2 GB medium. */
#define ORB_2_2GB_SECTORS       (4307184)

/* SyQuest cartridges, from Aaru device reports 195, 196 and 197. */
#define SQ400_SECTORS           (86700)      /* 5.25", 44 MB */
#define SQ800_SECTORS           (173456)     /* 5.25", 88 MB */
#define SQ2000_SECTORS          (390696)     /* 5.25", 200 MB */
#define SQ310_SECTORS           (215440)     /* 3.5", 105 MB */
/* The EZ135 medium is 262,144 blocks (128 MiB, CHS 512 x 16 x 32), per Aaru report 195 and the
   "hdd: SyQuest EZ135A ... CHS=512/16/32" line of a 1995 comp.os.linux.hardware post. */
#define EZ135_SECTORS           (262144)
#define EZFLYER_230_SECTORS     (450560)

/* Floppy media read by the SuperDisk, HiFD and Floptical drives, from Aaru device reports 66 and 67.
   The NEC 2HD medium has 1232 sectors of 1024 bytes (1,261,568 bytes); the rest use 512-byte sectors. */
#define FD_2HD_NEC_SECTORS      (1232)       /* 1.2 MB, 1024 bytes per sector */
#define FD_720K_SECTORS         (1440)
#define FD_1440K_SECTORS        (2880)
#define FD32MB_SECTORS          (65536)      /* Imation FD32MB, medium type 0x28 */

#define JAZ_1GB_SECTORS     (2091050)

#define JAZ_2GB_SECTORS     (3915600)

#define SUPERDISK_SECTORS      (963 * 256)

#define SUPERDISK_240_SECTORS     (469504)

/* 78882 blocks of 512 bytes (40,387,584 bytes) per Aaru device report 50, a real
   Iomega Clik! 40 CZ with a Clik! medium. */
#define CLIK_SECTORS              (78882)

/* Last LBA of 2,929,799 (i.e. 2,929,800 sectors) per the SyQuest SyJet SCSI
   Technical Reference (1997, P/N 112857-001A), section "Read Capacity (25H)". */
#define SYJET_SECTORS       (2929800)

/* Total addressable LBA count of 1,961,069 sectors per the SyQuest SparQ
   Internal EIDE Technical Reference (1997, P/N 113277-001A), section
   "Identify Device (ECh)", words 60-61. The SparQ only ever shipped as an
   IDE/EIDE (and external parallel-port) drive - it has no real SCSI variant. */
#define SPARQ_SECTORS       (1961069)

#define RDISK_IMAGE_HISTORY       10

enum {
    RDISK_TYPE_GENERIC = 0,
    RDISK_TYPE_ZIP_100,
    RDISK_TYPE_ZIP_250,
    RDISK_TYPE_JAZ_1GB,
    RDISK_TYPE_JAZ_2GB,
    RDISK_TYPE_SYJET_1_5GB,
    RDISK_TYPE_SPARQ_1GB,
    RDISK_TYPE_SUPERDISK_120,
    RDISK_TYPE_SUPERDISK_240,
    /* Appended, in the same order as the media in rdisk_types[] below, so that the
       values of the types above do not change. */
    RDISK_TYPE_CLIK_40,
    RDISK_TYPE_ZIP_750,
    RDISK_TYPE_HIFD_200,
    RDISK_TYPE_FLOPTICAL_20,
    RDISK_TYPE_BERNOULLI_90,
    RDISK_TYPE_BERNOULLI_150,
    RDISK_TYPE_BERNOULLI_230,
    RDISK_TYPE_ORB_2_2GB,
    RDISK_TYPE_SQ5200C,
    RDISK_TYPE_SQ3105S,
    RDISK_TYPE_EZ230S
};

typedef struct rdisk_type_t {
    const char *name;
    uint32_t    sectors;
    uint16_t    bytes_per_sector;
} rdisk_type_t;

#define KNOWN_RDISK_TYPES 26
static const rdisk_type_t rdisk_types[KNOWN_RDISK_TYPES] = {
    { "ZIP 100",       ZIP_SECTORS,           512 },
    { "ZIP 250",       ZIP_250_SECTORS,       512 },
    { "Jaz 1 GB",      JAZ_1GB_SECTORS,       512 },
    { "Jaz 2 GB",      JAZ_2GB_SECTORS,       512 },
    { "Syjet",         SYJET_SECTORS,         512 },
    { "Sparq",         SPARQ_SECTORS,         512 },
    { "LS-120",        SUPERDISK_SECTORS,     512 },
    { "LS-240",        SUPERDISK_240_SECTORS, 512 },
    { "Clik!",         CLIK_SECTORS,          512 },
    { "ZIP 750",       ZIP_750_SECTORS,       512 },
    { "HiFD",          HIFD_SECTORS,          512 },
    { "Floptical",     FLOPTICAL_SECTORS,     512 },
    { "Bernoulli 90",  BERNOULLI_90_SECTORS,  512 },
    { "Bernoulli 150", BERNOULLI_150_SECTORS, 512 },
    { "Bernoulli 230", BERNOULLI_230_SECTORS, 512 },
    /* The media up to here are the own media of the drive types, in drive type order (media index =
       drive type - 1); default_media in rdisk_drive_types[] points at them. */
    { "ORB 2.2 GB",    ORB_2_2GB_SECTORS,     512 },
    { "SQ2000",        SQ2000_SECTORS,        512 },
    { "SQ310",         SQ310_SECTORS,         512 },
    { "EZFlyer 230",   EZFLYER_230_SECTORS,   512 },
    /* Media that are only ever read by other drives. */
    { "SQ400",         SQ400_SECTORS,         512 },
    { "SQ800",         SQ800_SECTORS,         512 },
    { "EZ135",         EZ135_SECTORS,         512 },
    { "2HD (1.2 MB)",  FD_2HD_NEC_SECTORS,   1024 },
    { "720 KB",        FD_720K_SECTORS,       512 },
    { "1.44 MB",       FD_1440K_SECTORS,      512 },
    { "FD32MB",        FD32MB_SECTORS,        512 }
};

enum {
    RDISK_BUS_DISABLED =  0,
    RDISK_BUS_LPT      =  6,  /* coincides with HDD_BUS_LPT/CDROM_BUS_LPT for config strings */
    RDISK_BUS_ATAPI    =  8,  /* coincides with HDD_BUS_ATAPI */
    RDISK_BUS_SCSI     =  9,  /* coincides with HDD_BUS_SCSI */
    RDISK_BUS_USB      = 10,
};

/* Which bus(es) a drive type belongs on, modelled on tape_drive_type_t's drive_type in scsi_tape.h.
   A single bus is that bus's RDISK_BUS_* value; the negative values are combinations. */
#define RDISK_DRIVE_TYPE_LPT            RDISK_BUS_LPT
#define RDISK_DRIVE_TYPE_ATAPI          RDISK_BUS_ATAPI
#define RDISK_DRIVE_TYPE_SCSI           RDISK_BUS_SCSI
#define RDISK_DRIVE_TYPE_SCSI_ATAPI     (-2)
#define RDISK_DRIVE_TYPE_NONE           (-1)
#define RDISK_DRIVE_TYPE_ATAPI_LPT      (-3)
#define RDISK_DRIVE_TYPE_SCSI_ATAPI_LPT (-4)

typedef struct rdisk_drive_type_t {
    const char *vendor;
    const char *model;
    const char *revision;
    uint8_t     default_media;
    int8_t      drive_type;
    int8_t      supported_media[KNOWN_RDISK_TYPES];
} rdisk_drive_type_t;

#define KNOWN_RDISK_DRIVE_TYPES 20
/* drive_type says which bus(es) each type belongs on (see RDISK_DRIVE_TYPE_* above).  These describe
   the interfaces the real drive shipped with where that is known (from the Aaru device reports, the
   model strings below and the notes in this file); the generic drive and the Zip 100/250, which
   rdisk.c emulates on SCSI, ATAPI and through the parallel-port bridge, take all three.  rdisk.c
   itself does not refuse a type on a bus: use rdisk_drive_type_supports_bus() below to filter.
   supported_media[] is indexed like rdisk_types[] (all 26 media above).  rdisk.c takes the media a
   drive accepts, its largest medium and whether it is a one-medium drive from this table. */
static const rdisk_drive_type_t rdisk_drive_types[KNOWN_RDISK_DRIVE_TYPES] = {
    { "86BOX",    "REMOVABLE DISK",             "5.00", 0, RDISK_DRIVE_TYPE_SCSI_ATAPI_LPT, { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
    { "IOMEGA",   "ZIP 100",                    "E.08", 0, RDISK_DRIVE_TYPE_SCSI_ATAPI_LPT, { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
    { "IOMEGA",   "ZIP 250",                    "42.S", 1, RDISK_DRIVE_TYPE_SCSI_ATAPI_LPT, { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
    /* The Jaz was a SCSI drive; rdisk.c also has ATAPI identify data for it, so both are allowed. */
    { "IOMEGA",   "JAZ 1GB",                    "H.72", 2, RDISK_DRIVE_TYPE_SCSI_ATAPI, { 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
    { "IOMEGA",   "JAZ 2GB",                    "E.17", 3, RDISK_DRIVE_TYPE_SCSI_ATAPI, { 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
    /* The SCSI SyJet-S was seen with firmware 0111, 0095 and 0117 (Aaru reports 327, 328, 329); see
       SYJET_S_REV in rdisk.c.  The ATAPI strings in rdisk.c are not confirmed real. */
    { "SYQUEST",  "SyJet 1.5GB",                "0117", 4, RDISK_DRIVE_TYPE_SCSI_ATAPI, { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
    /* Firmware "SA_0043" is from Aaru device report 175, a real SparQ.  That drive was native ATA (not
       ATAPI), as the SparQ only ever shipped as an IDE/EIDE (and external parallel-port) drive, hence
       ATAPI_LPT; rdisk.c still has SCSI mode pages for it. */
    { "SYQUEST",  "SparQ 1.0GB",                "SA_0043", 5, RDISK_DRIVE_TYPE_ATAPI_LPT, { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
    /* The SuperDisk drives also read 720 KB, 1.2 MB (NEC 2HD) and 1.44 MB floppies, and the LS-240
       also the 32 MB (FD32MB) format (Aaru reports 66 and 67). */
    { "IMATION",  "SUPERDISK 120 ATAPI",        "04",   6, RDISK_DRIVE_TYPE_ATAPI, { 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0 } },
    { "IMATION",  "SUPERDISK  240       ATAPI", "04",   7, RDISK_DRIVE_TYPE_ATAPI, { 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1 } },
    /* From Aaru device report 50, a real ATAPI Clik! 40 CZ. */
    { "IOMEGA",   "Clik! 40 CZ",               "30cJ",  8, RDISK_DRIVE_TYPE_ATAPI, { 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
    /* Firmware "78.a" is from a real USB Zip 750 (ZuluIDE firmware issue 39); "72.u" was also seen
       (Aaru device report 53).  Compatibility: reads and writes 750 and 250 MB media, reads
       (read-only) 100 MB media. */
    { "IOMEGA",   "ZIP 750",                    "78.a",  9, RDISK_DRIVE_TYPE_ATAPI, { 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
    /* From Aaru device report 118, a real ATAPI Sony HiFD SFD200.  Sony documents the HiFD as reading
       1.44 MB floppies, but that report's test did not recognise the floppies it tried (MFD-2DD and
       MFD-2HD), so the 1.44 MB medium is listed on the strength of Sony's documentation only. */
    { "SONY",     "HiFD",                       "1264", 10, RDISK_DRIVE_TYPE_ATAPI, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 } },
    /* From Aaru device report 258, a real SCSI-1 INSITE Floptical (the INQUIRY product field is
       "I325VM" + 8 spaces + "*F"); the report says it also reads 720 KB and 1.44 MB floppies. */
    { "INSITE",   "I325VM *F",                  "0387", 11, RDISK_DRIVE_TYPE_SCSI, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0 } },
    /* From Aaru device reports 186 (BETA90), 202 (BETA150) and 311 (BETA230), all SCSI-only drives.
       Compatibility of the larger drives with the smaller cartridges is an assumption (the 150 is
       documented as read/write compatible with 90 MB cartridges; the 230 is only described as
       backwards compatible with most earlier capacities). */
    { "IOMEGA",   "BETA90",                     "1.4",  12, RDISK_DRIVE_TYPE_SCSI, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
    { "IOMEGA",   "BETA150",                    "T.47", 13, RDISK_DRIVE_TYPE_SCSI, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
    { "IOMEGA",   "BETA230",                    "2.02", 14, RDISK_DRIVE_TYPE_SCSI, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
    /* Aaru device report 200, a real SCSI Castlewood ORB 2.2 GB (INQUIRY vendor "CWS ORB2", product
       "-SE U ID 0").  The ORB also shipped as an IDE/ATAPI drive, hence SCSI_ATAPI. */
    { "CASTLEWOOD", "ORB 2.2GB",                "D01K", 15, RDISK_DRIVE_TYPE_SCSI_ATAPI, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
    /* Aaru device report 197, a real SCSI SyQuest SQ5200C, which reads the SQ400, SQ800 and SQ2000. */
    { "SYQUEST",  "SQ5200C",                    "3CE4", 16, RDISK_DRIVE_TYPE_SCSI, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0 } },
    /* Aaru device report 196, a real SCSI SyQuest SQ3105S (SQ310 cartridge). */
    { "SYQUEST",  "SQ3105S",                    "1_21", 17, RDISK_DRIVE_TYPE_SCSI, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 } },
    /* Aaru device report 195, a real SCSI SyQuest EZ230S, which reads EZ135 and EZFlyer 230 media. */
    { "SYQUEST",  "EZ230S",                     "7220", 18, RDISK_DRIVE_TYPE_SCSI, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0 } }
};

/* Does this drive type belong on this RDISK_BUS_* bus, according to its drive_type? */
static inline int
rdisk_drive_type_supports_bus(const int drive_type, const uint8_t bus_type)
{
    switch (drive_type) {
        case RDISK_DRIVE_TYPE_NONE:
            return 0;
        case RDISK_DRIVE_TYPE_SCSI_ATAPI:
            return (bus_type == RDISK_BUS_SCSI) || (bus_type == RDISK_BUS_ATAPI);
        case RDISK_DRIVE_TYPE_ATAPI_LPT:
            return (bus_type == RDISK_BUS_ATAPI) || (bus_type == RDISK_BUS_LPT);
        case RDISK_DRIVE_TYPE_SCSI_ATAPI_LPT:
            return (bus_type == RDISK_BUS_SCSI) || (bus_type == RDISK_BUS_ATAPI) ||
                   (bus_type == RDISK_BUS_LPT);
        default:
            return drive_type == (int) bus_type;
    }
}

typedef struct rdisk_drive_t {
    uint8_t            id;

    union {
        uint8_t            res;
        /* Reserved for other ID's. */
        uint8_t            res0;
        uint8_t            res1;
        uint8_t            ide_channel;
        uint8_t            scsi_device_id;
    };

    uint8_t            bus_type;  /* 0 = ATAPI, 1 = SCSI */
    uint8_t            bus_mode;  /* Bit 0 = PIO suported;
                                     Bit 1 = DMA supportd. */
    uint8_t            read_only; /* Struct variable reserved for
                                     media status. */
    uint8_t            pad;
    uint8_t            pad0;

    FILE              *fp;
    void              *priv;

    char               image_path[MAX_IMAGE_PATH_LEN];
    char               prev_image_path[MAX_IMAGE_PATH_LEN + 256];

    char              *image_history[RDISK_IMAGE_HISTORY];

    uint32_t           type;
    uint32_t           medium_size;
    uint32_t           base;

    uint32_t           medium_type;
} rdisk_drive_t;

typedef struct rdisk_t {
    mode_sense_pages_t ms_pages_saved;

    rdisk_drive_t       *drv;
#ifdef EMU_IDE_H
    ide_tf_t          *tf;
#else
    void              *tf;
#endif

    void              *log;

    uint8_t           *buffer;
    size_t             buffer_sz;
    uint8_t            atapi_cdb[16];
    uint8_t            current_cdb[16];
    uint8_t            sense[256];

    uint8_t            id;
    uint8_t            cur_lun;
    uint8_t            pad0;
    uint8_t            pad1;

    uint16_t           max_transfer_len;
    uint16_t           pad2;

    int                requested_blocks;
    int                packet_status;
    int                total_length;
    int                do_page_save;
    int                unit_attention;
    int                request_pos;
    int                old_len;
    int                transition;

    uint32_t           sector_pos;
    uint32_t           sector_len;
    uint32_t           packet_len;
    uint32_t           block_len;

    double             callback;

    uint8_t            (*ven_cmd)(void *sc, uint8_t *cdb, int32_t *BufLen);
} rdisk_t;

extern rdisk_t      *rdisk[RDISK_NUM];
extern rdisk_drive_t rdisk_drives[RDISK_NUM];
extern uint8_t     atapi_rdisk_drives[8];
extern uint8_t     scsi_rdisk_drives[16];

#define rdisk_sense_error dev->sense[0]
#define rdisk_sense_key   dev->sense[2]
#define rdisk_info        *(uint32_t *) &(dev->sense[3])
#define rdisk_asc         dev->sense[12]
#define rdisk_ascq        dev->sense[13]

#ifdef __cplusplus
extern "C" {
#endif

extern void rdisk_disk_close(const rdisk_t *dev);
extern void rdisk_disk_reload(const rdisk_t *dev);
extern void rdisk_insert(rdisk_t *dev);

extern void rdisk_global_init(void);
extern void rdisk_hard_reset(void);

extern void rdisk_reset(scsi_common_t *sc);
extern int  rdisk_is_empty(const uint8_t id);
extern void rdisk_load(const rdisk_t *dev, const char *fn, const int skip_insert);
extern void rdisk_close(void);
#ifdef SCSI_DEVICE_H
extern scsi_device_t *rdisk_get_lpt_device(const uint8_t port);
#endif

#ifdef __cplusplus
}
#endif

#endif /*EMU_RDISK_H*/
