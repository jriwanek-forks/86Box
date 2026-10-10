/*
 * 86Box    A hypervisor and IBM PC system emulator.
 *
 *          Shared modem definitions used by the character and network
 *          modem front ends.
 */
#ifndef EMU_MODEM_COMMON_H
#define EMU_MODEM_COMMON_H

#include <86box/modem/modem_voice.h>

typedef enum ResTypes {
    ResNONE,
    ResOK,
    ResERROR,
    ResCONNECT,
    ResRING,
    ResBUSY,
    ResNODIALTONE,
    ResNOCARRIER,
    ResNOANSWER
} ResTypes;

enum modem_types {
    MODEM_TYPE_NONE  = 0,
    MODEM_TYPE_SLIP  = 1,
    MODEM_TYPE_PPP   = 2,
    MODEM_TYPE_TCPIP = 3,
    MODEM_TYPE_CSLIP = 4,
    MODEM_TYPE_RAS   = 5
};

typedef enum modem_mode_t {
    MODEM_MODE_COMMAND = 0,
    MODEM_MODE_DATA    = 1,
    MODEM_MODE_FAX_TX  = 2,
    MODEM_MODE_FAX_WAIT = 3,
    MODEM_MODE_VOICE_TX = 4,
    MODEM_MODE_VOICE_RX = 5,
    MODEM_MODE_VOICE_TR = 6
} modem_mode_t;

typedef enum modem_fax_support_t {
    MODEM_FAX_SUPPORT_DISABLED = 0,
    MODEM_FAX_SUPPORT_CLASS_0  = 1 << 0,
    MODEM_FAX_SUPPORT_CLASS_1  = 1 << 1,
    MODEM_FAX_SUPPORT_CLASS_8  = 1 << 2
} modem_fax_support_t;

typedef enum modem_fax_transfer_status_t {
    MODEM_FAX_TRANSFER_ACTIVE,
    MODEM_FAX_TRANSFER_COMPLETE,
    MODEM_FAX_TRANSFER_ABORTED
} modem_fax_transfer_status_t;

typedef enum modem_slip_stage_t {
    MODEM_SLIP_STAGE_USERNAME,
    MODEM_SLIP_STAGE_PASSWORD
} modem_slip_stage_t;

enum modem_identity_t {
    MODEM_IDENTITY_GENERIC = 0,
    MODEM_IDENTITY_SUPRAEXPRESS = 1
};

enum modem_register_index_t {
    MREG_AUTOANSWER_COUNT = 0,
    MREG_RING_COUNT = 1,
    MREG_ESCAPE_CHAR = 2,
    MREG_CR_CHAR = 3,
    MREG_LF_CHAR = 4,
    MREG_BACKSPACE_CHAR = 5,
    MREG_GUARD_TIME = 12,
    MREG_DTR_DELAY = 25
};

enum modem_telnet_side_t {
    TEL_CLIENT = 0,
    TEL_SERVER = 1
};

#define MODEM_COMMAND_BUFFER_SIZE 512
#define MODEM_NUMBER_BUFFER_SIZE  128
#define MODEM_LOCAL_DIAL_SOUND_NUMBER "+18007160023"
#define MODEM_PHONEBOOK_SIZE 256
#define MODEM_REGS 100
#define MODEM_DATA_FIFO_SIZE 0x40000
#define MODEM_VOICE_PLAYBACK_BUFFER_SIZE (VOICE_LINE_RATE * 2)

/* Legacy internal spellings retained while existing command code is migrated. */
#define COMMAND_BUFFER_SIZE MODEM_COMMAND_BUFFER_SIZE
#define NUMBER_BUFFER_SIZE  MODEM_NUMBER_BUFFER_SIZE
#define PHONEBOOK_SIZE      MODEM_PHONEBOOK_SIZE

typedef struct modem_phonebook_entry_t {
    char phone[MODEM_NUMBER_BUFFER_SIZE];
    char address[MODEM_NUMBER_BUFFER_SIZE];
} modem_phonebook_entry_t;

#endif