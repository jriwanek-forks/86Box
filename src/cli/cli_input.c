/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          ANSI input module for the command line interface.
 *
 *          Escape code parsing state machine based on:
 *          Williams, Paul Flo. "A parser for DEC's ANSI-compatible video
 *          terminals." VT100.net. <https://vt100.net/emu/dec_ansi_parser>
 *
 * Authors: RichardG, <richardg867@gmail.com>
 *
 *          Copyright 2021-2025 RichardG.
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#    include <fcntl.h>
#    include <windows.h>
#else
#    include <errno.h>
#    include <termios.h>
#    include <unistd.h>
#endif
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/cli.h>
#include <86box/device.h>
#include <86box/keyboard.h>
#include <86box/mouse.h>
#include <86box/plat.h>
#include <86box/plat_fallthrough.h>
#include <86box/thread.h>
#include <86box/video.h>

#ifdef USE_CLI
/* Escape sequence parser states. */
enum {
    VT_GROUND = 0,
    VT_ESCAPE,
    VT_ESCAPE_INTERMEDIATE,
    VT_CSI_ENTRY,
    VT_CSI_IGNORE,
    VT_CSI_PARAM,
    VT_CSI_INTERMEDIATE,
    VT_DCS_ENTRY,
    VT_DCS_INTERMEDIATE,
    VT_DCS_IGNORE,
    VT_DCS_PARAM,
    VT_DCS_PASSTHROUGH,
    VT_SOS_PM_APC_STRING,
    VT_OSC_STRING,
    VT_MOUSE_BTN,
    VT_MOUSE_X,
    VT_MOUSE_Y
};
#endif

/* Lookup tables for converting keys and escape sequences to keyboard scan codes. */
const uint16_t ascii_seqs[128] = {
    ['\b'] = 0x000e, /* terminals prefer 7F/del for backspace */
    ['\t'] = 0x000f,
    ['\n'] = 0x001c,
    ['\r'] = 0x001c,
    [' ']  = 0x0039,
    ['!']  = 0x2a02,
    ['"']  = 0x2a28,
    ['#']  = 0x2a04,
    ['$']  = 0x2a05,
    ['%']  = 0x2a06,
    ['&']  = 0x2a08,
    ['\''] = 0x0028,
    ['(']  = 0x2a0a,
    [')']  = 0x2a0b,
    ['*']  = 0x2a09,
    ['+']  = 0x2a0d,
    [',']  = 0x0033,
    ['-']  = 0x000c,
    ['.']  = 0x0034,
    ['/']  = 0x0035,
    ['0']  = 0x000b,
    ['1']  = 0x0002,
    ['2']  = 0x0003,
    ['3']  = 0x0004,
    ['4']  = 0x0005,
    ['5']  = 0x0006,
    ['6']  = 0x0007,
    ['7']  = 0x0008,
    ['8']  = 0x0009,
    ['9']  = 0x000a,
    [':']  = 0x2a27,
    [';']  = 0x0027,
    ['<']  = 0x2a33,
    ['=']  = 0x000d,
    ['>']  = 0x2a34,
    ['?']  = 0x2a35,
    ['@']  = 0x2a03,
    ['A']  = 0x2a1e,
    ['B']  = 0x2a30,
    ['C']  = 0x2a2e,
    ['D']  = 0x2a20,
    ['E']  = 0x2a12,
    ['F']  = 0x2a21,
    ['G']  = 0x2a22,
    ['H']  = 0x2a23,
    ['I']  = 0x2a17,
    ['J']  = 0x2a24,
    ['K']  = 0x2a25,
    ['L']  = 0x2a26,
    ['M']  = 0x2a32,
    ['N']  = 0x2a31,
    ['O']  = 0x2a18,
    ['P']  = 0x2a19,
    ['Q']  = 0x2a10,
    ['R']  = 0x2a13,
    ['S']  = 0x2a1f,
    ['T']  = 0x2a14,
    ['U']  = 0x2a16,
    ['V']  = 0x2a2f,
    ['W']  = 0x2a11,
    ['X']  = 0x2a2d,
    ['Y']  = 0x2a15,
    ['Z']  = 0x2a2c,
    ['[']  = 0x001a,
    ['\\'] = 0x002b,
    [']']  = 0x001b,
    ['^']  = 0x2a07,
    ['_']  = 0x2a0c,
    ['`']  = 0x0029,
    ['a']  = 0x001e,
    ['b']  = 0x0030,
    ['c']  = 0x002e,
    ['d']  = 0x0020,
    ['e']  = 0x0012,
    ['f']  = 0x0021,
    ['g']  = 0x0022,
    ['h']  = 0x0023,
    ['i']  = 0x0017,
    ['j']  = 0x0024,
    ['k']  = 0x0025,
    ['l']  = 0x0026,
    ['m']  = 0x0032,
    ['n']  = 0x0031,
    ['o']  = 0x0018,
    ['p']  = 0x0019,
    ['q']  = 0x0010,
    ['r']  = 0x0013,
    ['s']  = 0x001f,
    ['t']  = 0x0014,
    ['u']  = 0x0016,
    ['v']  = 0x002f,
    ['w']  = 0x0011,
    ['x']  = 0x002d,
    ['y']  = 0x0015,
    ['z']  = 0x002c,
    ['{']  = 0x2a1a,
    ['|']  = 0x2a2b,
    ['}']  = 0x2a1b,
    ['~']  = 0x2a29,
    [0x7f] = 0x000e
};
#ifdef USE_CLI
static const uint16_t csi_num_seqs[] = {
    [1]  = 0xe047, /* Home */
    [2]  = 0xe052, /* Insert */
    [3]  = 0xe053, /* Delete */
    [4]  = 0xe04f, /* End */
    [5]  = 0xe049, /* Page Up */
    [6]  = 0xe051, /* Page Down */
    [11] = 0x003b, /* F1 */
    [12] = 0x003c, /* F2 */
    [13] = 0x003d, /* F3 */
    [14] = 0x003e, /* F4 */
    [15] = 0x003f, /* F5 */
    [17] = 0x0040, /* F6 */
    [18] = 0x0041, /* F7 */
    [19] = 0x0042, /* F8 */
    [20] = 0x0043, /* F9 */
    [21] = 0x0044, /* F10 */
    [23] = 0x0057, /* F11 */
    [24] = 0x0058, /* F12 */
    [25] = 0xe037, /* F13 => SysRq (for Apple keyboards) */
    [26] = 0x0046, /* F14 => Scroll Lock (for Apple keyboards) */
    [28] = 0xe11d, /* F15 => Pause (for Apple keyboards) */
    [29] = 0xe05d  /* Menu */
};
static const uint16_t csi_letter_seqs[] = {
    [' '] = 0x0039, /* Space */
    ['j'] = 0x0037, /* Num* */
    ['k'] = 0x004e, /* Num+ */
    ['l'] = 0x0053, /* Num, => NumDel */
    ['m'] = 0x004a, /* Num- */
    ['n'] = 0x0053, /* Num. => NumDel */
    ['o'] = 0xe035, /* Num/ */
    ['p'] = 0x0052, /* Num0 */
    ['q'] = 0x004f, /* Num1 */
    ['r'] = 0x0050, /* Num2 */
    ['s'] = 0x0051, /* Num3 */
    ['t'] = 0x004b, /* Num4 */
    ['u'] = 0x004c, /* Num5 */
    ['v'] = 0x004d, /* Num6 */
    ['w'] = 0x0047, /* Num7 */
    ['x'] = 0x0048, /* Num8 */
    ['y'] = 0x0049, /* Num9 */
    ['A'] = 0xe048, /* Up */
    ['B'] = 0xe050, /* Down */
    ['C'] = 0xe04d, /* Right */
    ['D'] = 0xe04b, /* Left */
    ['E'] = 0xe047, /* Begin => Home */
    ['F'] = 0xe04f, /* End */
    ['H'] = 0xe047, /* Home */
    ['I'] = 0x000f, /* Tab */
    ['M'] = 0xe01c, /* NumEnter */
    ['P'] = 0x003b, /* F1 */
    ['Q'] = 0x003c, /* F2 */
    ['R'] = 0x003d, /* F3 */
    ['S'] = 0x003e, /* F4 */
    ['X'] = 0x0059, /* Num= (multimedia) */
    ['Z'] = 0x2a0f, /* Shift+Tab */
};
static const uint16_t csi_pua_seqs[] = {
    [0x0e] = 0x003a, /* CAPS_LOCK */
    [0x0f] = 0x0046, /* SCROLL_LOCK */
    [0x10] = 0x0045, /* NUM_LOCK */
    [0x11] = 0xe037, /* PRINT_SCREEN */
    [0x12] = 0xe11d, /* PAUSE */
    [0x13] = 0xe05d, /* MENU */
    [0x20] = 0x005d, /* F13 */
    [0x21] = 0x005e, /* F14 */
    [0x22] = 0x005f, /* F15 */
    [0x23] = 0x0067, /* F16 */
    [0x24] = 0x0068, /* F17 */
    [0x25] = 0x0069, /* F18 */
    [0x26] = 0x006a, /* F19 */
    [0x27] = 0x006b, /* F20 */
    [0x28] = 0x006c, /* F21 */
    [0x29] = 0x006d, /* F22 */
    [0x2a] = 0x006e, /* F23 */
    [0x2b] = 0x0076, /* F24 */
    [0x37] = 0x0052, /* KP_0 */
    [0x38] = 0x004f, /* KP_1 */
    [0x39] = 0x0050, /* KP_2 */
    [0x3a] = 0x0051, /* KP_3 */
    [0x3b] = 0x004b, /* KP_4 */
    [0x3c] = 0x004c, /* KP_5 */
    [0x3d] = 0x004d, /* KP_6 */
    [0x3e] = 0x0047, /* KP_7 */
    [0x3f] = 0x0048, /* KP_8 */
    [0x40] = 0x0049, /* KP_9 */
    [0x41] = 0x0053, /* KP_DECIMAL => NumDel */
    [0x42] = 0xe035, /* KP_DIVIDE */
    [0x43] = 0x0037, /* KP_MULTIPLY */
    [0x44] = 0x004a, /* KP_SUBTRACT */
    [0x45] = 0x004e, /* KP_ADD */
    [0x46] = 0xe01c, /* KP_ENTER */
    [0x47] = 0x0059, /* KP_EQUAL (multimedia) */
    [0x48] = 0x0053, /* KP_SEPARATOR => NumDel */
    [0x49] = 0x004b, /* KP_LEFT => Num4 */
    [0x4a] = 0x004d, /* KP_RIGHT => Num6 */
    [0x4b] = 0x0048, /* KP_UP => Num8 */
    [0x4c] = 0x0050, /* KP_DOWN => Num2 */
    [0x4d] = 0x0049, /* KP_PAGE_UP => Num9 */
    [0x4e] = 0x0051, /* KP_PAGE_DOWN => Num3 */
    [0x4f] = 0x0047, /* KP_HOME => Num7 */
    [0x50] = 0x004f, /* KP_END => Num1 */
    [0x51] = 0x0052, /* KP_INSERT => Num0 */
    [0x52] = 0x0053, /* KP_DELETE */
    [0x53] = 0x0047, /* KP_BEGIN => Num7 */
    [0x54] = 0xe052, /* MEDIA_PLAY => Play/Pause */
    [0x55] = 0xe052, /* MEDIA_PAUSE => Play/Pause */
    [0x56] = 0xe052, /* MEDIA_PLAY_PAUSE */
    [0x57] = 0xe06a, /* MEDIA_REVERSE => Back */
    [0x58] = 0xe068, /* MEDIA_STOP */
    [0x59] = 0xe069, /* MEDIA_FAST_FORWARD => Forward */
    [0x5a] = 0xe010, /* MEDIA_REWIND => Previous */
    [0x5b] = 0xe019, /* MEDIA_TRACK_NEXT */
    [0x5c] = 0xe010, /* MEDIA_TRACK_PREVIOUS */
    [0x5d] = 0xe078, /* MEDIA_RECORD (Logitech) */
    [0x5e] = 0xe02e, /* LOWER_VOLUME */
    [0x5f] = 0xe030, /* RAISE_VOLUME */
    [0x60] = 0xe020, /* MUTE_VOLUME */
    [0x61] = 0x002a, /* LEFT_SHIFT */
    [0x62] = 0x001d, /* LEFT_CONTROL */
    [0x63] = 0x0038, /* LEFT_ALT */
    [0x64] = 0xe05b, /* LEFT_SUPER => Left Win */
    [0x65] = 0xe05b, /* LEFT_HYPER => Left Win */
    [0x66] = 0xe05b, /* LEFT_META => Left Win */
    [0x67] = 0x0036, /* RIGHT_SHIFT */
    [0x68] = 0xe01d, /* RIGHT_CONTROL */
    [0x69] = 0xe038, /* RIGHT_ALT */
    [0x6a] = 0xe05c, /* RIGHT_SUPER => Right Win */
    [0x6b] = 0xe05c, /* RIGHT_HYPER => Right Win */
    [0x6c] = 0xe05c  /* RIGHT_META => Right Win */
};
static const uint8_t mouse_button_values[] = {
    [0] = 1, /* left */
    [1] = 4, /* middle */
    [2] = 2, /* right */
    [3] = 0, /* none */
    [8] = 8, /* 4th */
    [9] = 16 /* 5th */
};

static int  in_raw = 0, param_buf_pos = 0, collect_buf_pos = 0, dcs_buf_pos = 0, osc_buf_pos = 0;
static char param_buf[32], collect_buf[32], dcs_buf[32], osc_buf[32];
#    ifdef _WIN32
static DWORD saved_console_mode = 0;
#    else
static tcflag_t saved_lflag = 0, saved_iflag = 0;
#    endif
#endif

#define ENABLE_CLI_INPUT_LOG 1
#ifdef ENABLE_CLI_INPUT_LOG
int cli_input_do_log = ENABLE_CLI_INPUT_LOG;

static void
cli_input_log(const char *fmt, ...)
{
    va_list ap;

    if (cli_input_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}

#    ifdef USE_CLI
static void
cli_input_log_key(const char *func, int c)
{
    if ((c >= 0x20) && (c <= 0x7e))
        cli_input_log("CLI Input: %s(%c)\n", func, c);
    else
        cli_input_log("CLI Input: %s(%02X)\n", func, c);
}
#    endif
#else
#    define cli_input_log(fmt, ...)
#    define cli_input_log_key(func, c)
#endif

void
cli_input_send(uint16_t code, uint16_t modifier)
{
    cli_input_log("CLI Input: send(%04X, %03X)", code, modifier);

    /* Add modifiers set by the keycode definition. */
    switch (code >> 8) {
        case 0x1d:
            modifier |= VT_CTRL;
            break;

        case 0x2a:
            modifier |= VT_SHIFT;
            break;

        case 0x38:
            modifier |= VT_ALT;
            break;

        case 0x5b:
            modifier |= VT_SUPER;
            break;

        default:
            break;
    }

    /* Handle special cases. */
    switch (code) {
        case 0xe037: /* SysRq */
            if (modifier & (VT_SHIFT | VT_CTRL)) {
                modifier &= ~(VT_SHIFT | VT_CTRL);
            } else if (modifier & VT_ALT) {
                modifier &= ~VT_ALT;
                code = 0x0054;
            } else {
                modifier |= VT_SHIFT_FAKE;
            }
            break;

        case 0xe11d: /* Pause */
            if (modifier & VT_CTRL) {
                modifier &= ~VT_CTRL;
                code = 0xe046;
            }
            break;

        default:
            break;
    }

    /* Press key with modifiers unless an explicit key up with no explicit key down is requested. */
    if ((modifier & (VT_KEY_UP | VT_KEY_DOWN)) != VT_KEY_UP) {
        cli_input_log(" press");
        if (modifier & (VT_SUPER | VT_HYPER | VT_META))
            keyboard_input(1, 0xe05b);
        if (modifier & VT_CTRL)
            keyboard_input(1, 0x001d);
        if (modifier & VT_ALT)
            keyboard_input(1, 0x0038);
        if (modifier & VT_SHIFT)
            keyboard_input(1, 0x002a);
        if (modifier & VT_SHIFT_FAKE)
            keyboard_input(1, 0xe02a);
        if (code)
            keyboard_input(1, code);
    }

    /* Release key with modifiers if kitty event types are disabled or an explicit key up is requested. */
    if (!(cli_term.kitty_input & 2) || (modifier & VT_KEY_UP)) {
        cli_input_log(" release");
        if (code)
            keyboard_input(0, code);
        if (modifier & VT_SHIFT_FAKE)
            keyboard_input(0, 0xe02a);
        if (modifier & VT_SHIFT)
            keyboard_input(0, 0x002a);
        if (modifier & VT_ALT)
            keyboard_input(0, 0x0038);
        if (modifier & VT_CTRL)
            keyboard_input(0, 0x001d);
        if (modifier & (VT_SUPER | VT_HYPER | VT_META))
            keyboard_input(0, 0xe05b);
    }

    cli_input_log("\n");
}

#ifdef USE_CLI
static void
cli_input_raw(void)
{
    /* Don't do anything if raw input is already enabled. */
    if (in_raw)
        return;
    in_raw = 1;

#    ifdef _WIN32
    /* Enable window events and disable quickedit mode.
       Note that we use ReadConsoleInput instead of ANSI mode. */
    HANDLE h = GetStdHandle(STD_INPUT_HANDLE);
    if (h) {
        /* Save existing mode for restoration purposes. */
        if (GetConsoleMode(h, &saved_console_mode))
            in_raw = 2;
        else
            cli_input_log("CLI Input: GetConsoleMode failed (%08X)\n", GetLastError());

        /* Set new mode. */
        if (!SetConsoleMode(h, ENABLE_WINDOW_INPUT | ENABLE_EXTENDED_FLAGS)) /* ENABLE_EXTENDED_FLAGS disables quickedit */
            cli_input_log("CLI Input: SetConsoleMode failed (%08X)\n", GetLastError());
    } else {
        cli_input_log("CLI Input: GetStdHandle failed (%08X)\n", GetLastError());
    }
#    else
    /* Enable raw input. */
    struct termios ios;
    if (tcgetattr(STDIN_FILENO, &ios)) {
        cli_input_log("CLI Input: tcgetattr failed (%d)\n", errno);
    } else {
        /* Save existing flags for restoration purposes. */
        in_raw      = 2;
        saved_lflag = ios.c_lflag;
        saved_iflag = ios.c_iflag;

        /* Set new flags. */
        ios.c_lflag &= ~(ECHO | ICANON | ISIG);
        ios.c_iflag &= ~IXON;
        if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &ios))
            cli_input_log("CLI Input: tcsetattr failed (%d)\n", errno);
    }
#    endif
}

static void
cli_input_unraw(void)
{
    /* Don't do anything if raw input is not enabled. */
    if (!in_raw)
        return;

    /* Restore saved terminal state. */
    if (in_raw == 2) {
#    ifdef _WIN32
        HANDLE h = GetStdHandle(STD_INPUT_HANDLE);
        if (h) {
            if (!SetConsoleMode(h, saved_console_mode))
                cli_input_log("CLI Input: SetConsoleMode failed (%08X)\n", GetLastError());
        } else {
            cli_input_log("CLI Input: GetStdHandle failed (%08X)\n", GetLastError());
        }
#    else
        struct termios ios;
        if (tcgetattr(STDIN_FILENO, &ios)) {
            cli_input_log("CLI Input: tcgetattr failed (%d)\n", errno);
        } else {
            ios.c_lflag = saved_lflag;
            ios.c_iflag = saved_iflag;
            if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &ios))
                cli_input_log("CLI Input: tcsetattr failed (%d)\n", errno);
        }
#    endif
    }

    in_raw = 0;
}

static int
cli_input_response_strstr(char *response, const char *cmp)
{
    /* Allocate a local buffer. */
    int len = strlen(response);
    if (!len)
        return -1;
    char *response_cleaned = malloc(len + 2),
         *p                = response_cleaned, ch;

    /* Copy response while removing double colons. */
    for (int i = 0; i < len; i++) {
        ch = response[i];
        if ((ch >= ':') && (ch <= '?')) {
            if ((p != response_cleaned) && (*(p - 1) != ':'))
                *p++ = ':';
        } else {
            *p++ = ch;
        }
    }

    /* Replace non-numeric first character with a colon. */
    ch = response_cleaned[0];
    if ((ch < '0') || (ch > '9'))
        response_cleaned[0] = ':';

    /* Add or replace last character with a colon. */
    ch = *(p - 1);
    if ((ch >= '0') && (ch <= '9'))
        *p++ = ':';
    else
        *(p - 1) = ':';
    *p = '\0';

    /* Perform comparison. */
    p = strstr(response_cleaned, cmp);
    free(response_cleaned);
    return !!p;
}

static void
cli_input_clear(int c)
{
    cli_input_log_key("clear", c);

    collect_buf_pos = param_buf_pos = 0;
    collect_buf[0] = param_buf[0] = '\0';
}

static void
cli_input_collect(int c)
{
    cli_input_log_key("collect", c);

    if (collect_buf_pos < (sizeof(collect_buf) - 1)) {
        collect_buf[collect_buf_pos++] = c;
        collect_buf[collect_buf_pos]   = '\0';
    }
}

static void
cli_input_csi_dispatch(int c)
{
    cli_input_log_key("csi_dispatch", c);

    /* Discard an invalid sequence with no letter or numeric code. */
    if ((c == '~') && (param_buf_pos < 1))
        return;

    /* Read numeric code and modifier parameters if applicable. */
    unsigned int code, modifier, third;
    char         delimiter1, delimiter2;
    switch (sscanf(param_buf, "%u%c%u%c%u", &code, &delimiter1, &modifier, &delimiter2, &third)) {
        case EOF:
        case 0:
            code = 0;
            fallthrough;

        case 1:
            delimiter1 = 0;
            fallthrough;

        case 2:
            modifier = 0;
            fallthrough;

        case 3:
            delimiter2 = 0;
            fallthrough;

        case 4:
            third = 0;
            fallthrough;

        default:
            break;
    }

    /* Determine if this is a terminal size query response. */
    if (cli_term.cpr && (c == 'R') && (modifier > 1)) {
        if (code == 1) {
            cli_term.cpr &= ~2;

            /* If we're exactly one character in, we can assume the
               terminal has interpreted our UTF-8 sequence as UTF-8. */
            cli_term.can_utf8 = modifier == 2;
            cli_input_log("CLI Input: CPR probe reports %sUTF-8\n", cli_term.can_utf8 ? "" : "no ");
        } else {
            cli_term.cpr &= ~1;

            /* Set 0-based terminal size to the current 1-based cursor position. */
            cli_term_setsize(modifier, code, "CPR");
        }
        return;
    }

    /* Determine if this is a device attribute query response. */
    if ((c == 'c') && (collect_buf[0] == '?')) {
        cli_input_log("CLI Input: Primary attributes report: ");

        /* Enable sixel graphics if supported. */
        modifier = cli_input_response_strstr(param_buf, ":4:");
        cli_input_log("%ssixel, ", modifier ? "" : "no ");
        if (modifier)
            cli_term.gfx_level |= TERM_GFX_SIXEL;
        else
            cli_term.gfx_level &= ~TERM_GFX_SIXEL;

        /* Enable 4-bit color if supported. */
        modifier = cli_input_response_strstr(param_buf, ":22:");
        cli_input_log("%scolor\n", modifier ? "" : "no ");
        if ((cli_term.color_level < TERM_COLOR_4BIT) && modifier)
            cli_term_setcolor(TERM_COLOR_4BIT, "attributes");

        return;
    }

    /* Determine if this is a graphics attribute query response. */
    if ((c == 'S') && (collect_buf[0] == '?')) {
        cli_input_log("CLI Input: Graphics attribute %d reports: response %d, ", code, modifier);
        if ((code == 1) && (modifier == 0) && (third > 0)) {
            /* Set sixel color register count. */
            cli_input_log("%d sixel color registers\n", third);
            cli_term.sixel_color_regs = third;

            /* Update libsixel dithering level. */
            cli_render_setcolorlevel();
        } else {
            cli_input_log("nothing we care about\n");
        }

        return;
    }

    /* Determine if this is a kitty keyboard protocol query response. */
    if ((c == 'u') && (collect_buf[0] == '?')) {
        cli_input_log("CLI Input: kitty keyboard protocol reports flags %d\n", code);
        cli_term.kitty_input = code;

        return;
    }

    /* Account for the +1 offset on received modifiers. */
    if (modifier)
        modifier = (modifier - 1) & VT_MODS_ONLY;

    /* Determine keycode. */
#define SAFE_INDEX(a, i) ((((i) >= 0) && ((i) < (sizeof((a)) / sizeof((a)[0])))) ? (a)[(i)] : 0)
    switch (c) {
        case '~':
            if (code == 27) /* CSI 27 ; modifier ; ascii ~ (xterm modifyOtherKeys=2) */
                code = SAFE_INDEX(ascii_seqs, third);
            if ((code & ~0x1fff) == 0xe000) /* Unicode PUA (kitty) - only documented for KP_BEGIN/e053, mistake? */
                code = SAFE_INDEX(csi_pua_seqs, code & 0x1fff);
            else /* CSI code [; modifier] ~ */
                code = SAFE_INDEX(csi_num_seqs, code);
            break;

        case 'u': /* CSI ascii ; modifier [: kittyevent] u (xterm modifyOtherKeys>0 && formatOtherKeys=1 or kitty) */
            if ((code & ~0x1fff) == 0xe000) /* Unicode PUA (kitty) */
                code = SAFE_INDEX(csi_pua_seqs, code & 0x1fff);
            else
                code = SAFE_INDEX(ascii_seqs, code);
            if (delimiter1 == ':') { /* just in case we get kitty alternate codes without asking for it */
                cli_input_log("CLI Input: Ignoring unsupported kitty keypress\n");
                return;
            }
kitty_event:
            if (delimiter2 == ':') { /* kitty event types */
                if (third == 3) /* release */
                    modifier |= VT_KEY_UP;
                else if ((third != 1) && (third != 2)) /* other events outside of press and repeat */
                    return;
            }
            break;

        default: /* CSI [[1 ;] modifier] letter */
            if ((code > 1) && !modifier)
                modifier = (code - 1) & VT_MODS_ONLY; /* if 1 ; is missing (xterm modify*Keys=1), first parameter is the modifier */
            code = SAFE_INDEX(csi_letter_seqs, c);
            goto kitty_event;
    }

    /* Press key with any modifiers. */
    cli_input_send(code, modifier);

    /* Update lock states based on kitty modifiers. */
    if (cli_term.kitty_input & 1) {
        uint8_t cl, nl, sl, kl;
        keyboard_get_states(&cl, &nl, &sl, &kl);
        keyboard_update_states(modifier & VT_CAPSLOCK, modifier & VT_NUMLOCK, sl, kl);
    }
}

static void
cli_input_esc_dispatch(int c)
{
    cli_input_log_key("esc_dispatch", c);

    switch (collect_buf[0]) {
        case '\0': /* no parameter */
            switch (c) {
                case 0x20 ... 0x7f: /* Alt+Space to Alt+Backspace */
                    cli_input_send(ascii_seqs[c], VT_ALT);
                    break;

                default:
                    break;
            }
            break;

        case 'O': /* SS3 (VT220 Application Keypad) */
        case '?': /* (VT52 Application Keypad) */
            cli_input_csi_dispatch(c); /* route numpad keys */
            break;

        default:
            break;
    }
}

static void
cli_input_execute(int c)
{
    cli_input_log_key("execute", c);

    switch (c) {
        case 0x01 ... 0x08: /* Ctrl+A to Ctrl+H */
        case 0x0b ... 0x0c: /* Ctrl+K to Ctrl+L */
        case 0x0e ... 0x1a: /* Ctrl+N to Ctrl+Z */
            cli_input_send(ascii_seqs['`' + c], VT_CTRL);
            break;

        case 0x09: /* Ctrl+I (Tab) */
        case 0x0a: /* Ctrl+J (Enter) */
        case 0x0d: /* Ctrl+M (Enter) */
            cli_input_send(ascii_seqs[c], 0);
            break;

        case 0x1b ... 0x1f: /* Ctrl+[ to Ctrl+_ */
            cli_input_send(ascii_seqs['@' + c], VT_CTRL);
            break;

        default:
            break;
    }
}

static void
cli_input_hook(int c)
{
    cli_input_log_key("hook", c);

    /* Initialize DCS buffer. */
    dcs_buf_pos            = 0;
    dcs_buf[dcs_buf_pos++] = c;
    dcs_buf[dcs_buf_pos]   = '\0';
}

static void
cli_input_put(int c)
{
    cli_input_log_key("put", c);

    /* Append character to DCS buffer. */
    if (dcs_buf_pos < (sizeof(dcs_buf) - 1)) {
        dcs_buf[dcs_buf_pos++] = c;
        dcs_buf[dcs_buf_pos]   = '\0';
    }
}

static void
cli_input_unhook(int c)
{
    cli_input_log_key("unhook", c);

    /* Process DECRQSS. */
    if ((collect_buf[0] == '$') && (dcs_buf[0] == 'r')) {
        cli_input_log("CLI Input: DECRQSS response: %s\n", dcs_buf);

        /* Interpret color-related responses. */
        char last_char = dcs_buf[dcs_buf_pos - 1];
        switch (last_char) {
            case 'm':
                /* Interpret response according to the color level currently being queried. */
                switch (cli_term.decrqss_color) {
                    case TERM_COLOR_24BIT:
                        if (cli_input_response_strstr(dcs_buf, ":2:255:255:255:")) {
                            /* 24-bit color supported. */
                            cli_term_setcolor(TERM_COLOR_24BIT, "DECRQSS");
                        } else if (cli_term.color_level < TERM_COLOR_8BIT) {
                            /* Try 8-bit color if we don't explicitly know it's supported. */
                            cli_term.decrqss_color = TERM_COLOR_8BIT;
                            cli_render_write(
                                "\033[38;5;255m" /* set 8-bit color to the last gray */
                                "\033P$qm\033\\\033[0m" /* query SGR */
                            );
                            break;
                        }
                        break;

                    case TERM_COLOR_8BIT:
                        if (cli_input_response_strstr(dcs_buf, ":5:255:")) {
                            /* 8-bit color supported. */
                            cli_term_setcolor(TERM_COLOR_8BIT, "DECRQSS");
                        } else if (cli_term.color_level < TERM_COLOR_4BIT) {
                            /* Try 4-bit color if we don't explicitly know it's supported. */
                            cli_term.decrqss_color = TERM_COLOR_4BIT;
                            cli_render_write(
                                "\033[97m" /* set foreground to bright white */
                                "\033P$qm\033\\\033[0m" /* query SGR */
                            );
                            break;
                        }
                        break;

                    case TERM_COLOR_4BIT:
                        if (cli_input_response_strstr(dcs_buf, ":97:")) {
                            /* 4-bit color supported. */
                            cli_term_setcolor(TERM_COLOR_4BIT, "DECRQSS");
                        }
                        break;

                    default:
                        /* Spurious response. */
                        return;
                }
                cli_term.decrqss_color = TERM_COLOR_NONE;
                break;

            case 'q':
                /* Save current cursor style. */
                if (sscanf(&dcs_buf[1], "%u", &cli_term.decrqss_cursor) != 1)
                    cli_term.decrqss_cursor = 0;
                cli_input_log("CLI Input: DECRQSS reports a cursor style of %d\n", cli_term.decrqss_cursor);
                break;

            default:
                break;
        }
    }
}

static void
cli_input_osc_start(int c)
{
    cli_input_log_key("osc_start", c);

    /* Initialize OSC buffer. */
    osc_buf_pos = 0;
    osc_buf[0]  = '\0';
}

static void
cli_input_osc_put(int c)
{
    cli_input_log_key("osc_put", c);

    /* Append character to OSC buffer. */
    if (osc_buf_pos < (sizeof(osc_buf) - 1)) {
        osc_buf[osc_buf_pos++] = c;
        osc_buf[osc_buf_pos]   = '\0';
    }
}

static void
cli_input_osc_end(int c)
{
    cli_input_log_key("osc_end", c);
}

static void
cli_input_param(int c)
{
    cli_input_log_key("param", c);

    if (param_buf_pos < (sizeof(param_buf) - 1)) {
        param_buf[param_buf_pos++] = c;
        param_buf[param_buf_pos]   = '\0';
    }
}

void
cli_input_process(void *priv)
{
    int c = 0, state = VT_GROUND, prev_state = VT_GROUND;
    int mouse_x_prev = 0;
    int mouse_y_prev = 0;
#ifdef _WIN32
    HANDLE h = GetStdHandle(STD_INPUT_HANDLE);
    INPUT_RECORD ir;
    int prev_key = 0, prev_ctrl_state = 0;
#endif

    /* Run state machine loop. */
    while (1) {
        /* Handle state exits. */
        if ((prev_state == VT_DCS_PASSTHROUGH) && (state != VT_DCS_PASSTHROUGH))
            cli_input_unhook(c);
        else if ((prev_state == VT_OSC_STRING) && (state != VT_OSC_STRING))
            cli_input_osc_end(c);
        prev_state = state;

        /* Read character. */
#ifdef _WIN32
        if (!ReadConsoleInput(h, &ir, 1, (LPDWORD) &c)) {
            cli_input_log("CLI Input: stdin read error (%08X)\n", GetLastError());
            return;
        }
        if (c < 1) {
            continue;
        } else if (ir.EventType == KEY_EVENT) { /* keyboard events */
            if (ir.Event.KeyEvent.wVirtualScanCode == 0) {
                /* A null scancode indicates a pseudo-terminal, which may or
                   may not be inputting ANSI, so we parse as that instead. */
                if (ir.Event.KeyEvent.bKeyDown) /* only on press or one-shot */
                    c = ir.Event.KeyEvent.uChar.AsciiChar;
                else
                    continue;
            } else {
                cli_input_log("CLI Input: Win32 process(%d, %04X, %04X)\n", ir.Event.KeyEvent.bKeyDown,
                              ir.Event.KeyEvent.wVirtualScanCode, ir.Event.KeyEvent.dwControlKeyState);

                /* Check for Esc Enter monitor sequence. */
                c = (ir.Event.KeyEvent.dwControlKeyState & ENHANCED_KEY) | /* conveniently sets 0x100 for E0 keys */
                    ir.Event.KeyEvent.wVirtualScanCode;
                if (ir.Event.KeyEvent.bKeyDown) {
                    if ((prev_key == 0x0001) && (c == 0x001c) && (ir.Event.KeyEvent.dwControlKeyState == prev_ctrl_state)) {
                        prev_key = c;
                        prev_ctrl_state = ir.Event.KeyEvent.dwControlKeyState;
                        goto monitor;
                    }
                    prev_key = c;
                    prev_ctrl_state = ir.Event.KeyEvent.dwControlKeyState;
                }

                /* Send modifier keys. */
                if (ir.Event.KeyEvent.dwControlKeyState & LEFT_ALT_PRESSED)
                    keyboard_input(ir.Event.KeyEvent.bKeyDown, 0x0038);
                if (ir.Event.KeyEvent.dwControlKeyState & LEFT_CTRL_PRESSED)
                    keyboard_input(ir.Event.KeyEvent.bKeyDown, 0x001d);
                if (ir.Event.KeyEvent.dwControlKeyState & RIGHT_ALT_PRESSED)
                    keyboard_input(ir.Event.KeyEvent.bKeyDown, 0xe038);
                if (ir.Event.KeyEvent.dwControlKeyState & RIGHT_CTRL_PRESSED)
                    keyboard_input(ir.Event.KeyEvent.bKeyDown, 0xe01d);
                if (ir.Event.KeyEvent.dwControlKeyState & SHIFT_PRESSED)
                    keyboard_input(ir.Event.KeyEvent.bKeyDown, 0x002a);

                /* Send key. */
                keyboard_input(ir.Event.KeyEvent.bKeyDown, c);

                /* Update lock states. */
                uint8_t cl, nl, sl, kl;
                keyboard_get_states(&cl, &nl, &sl, &kl);
                keyboard_update_states(!!(ir.Event.KeyEvent.dwControlKeyState & CAPSLOCK_ON),
                                       !!(ir.Event.KeyEvent.dwControlKeyState & NUMLOCK_ON),
                                       !!(ir.Event.KeyEvent.dwControlKeyState & SCROLLLOCK_ON),
                                       kl);

                /* Don't process as ANSI. */
                continue;
            }
        } else if (ir.EventType == WINDOW_BUFFER_SIZE_EVENT) { /* window events */
            /* Update terminal size. */
            cli_term_updatesize(1);
            continue;
        } else {
            continue;
        }
#else
        c = getchar();
#endif
        cli_input_log_key("process", c);

        /* Interpret conditions for any state. */
        switch (c) {
            case 0x1b:
                /* Interpret Esc Esc as escaped Esc. Note that some terminals
                   may emit extended codes prefixed with Esc Esc, but there's
                   not much we can do to parse those. */
                if (state == VT_ESCAPE) {
                    cli_input_send(0x0001, 0);
                    state = VT_GROUND;
                } else {
                    state = VT_ESCAPE;
                }
                continue;

            case 0x7f:
                /* Ignore, unless this is an user-initiated Backspace. */
                if ((state == VT_GROUND) || (state == VT_ESCAPE))
                    break;
                else
                    continue;

            case EOF:
                /* Something went wrong. */
                cli_input_log("CLI Input: stdin read error\n");
                return;

            default:
                break;
        }

        /* Interpret conditions for specific states. */
        switch (state) {
            case VT_GROUND:
                switch (c) {
                    case 0x00 ... 0x1f:
                        cli_input_execute(c);
                        continue;

                    case 0x20 ... 0x7e: /* ASCII */
                        cli_input_send(ascii_seqs[c], 0);
                        break;

                    case 0x7f: /* Backspace */
                        cli_input_send(ascii_seqs['\b'], 0);
                        break;

                    default:
                        break;
                }
                break;

            case VT_ESCAPE:
                switch (c) {
                    case 0x00 ... 0x09:
                    case 0x0b ... 0x1f:
                        cli_input_execute(c);
                        break;

                    case 0x0a: /* Esc Enter */
#ifdef _WIN32
monitor:
#endif
                        /* Block render thread. */
                        cli_render_monitorenter();

                        /* Disable raw input. */
                        cli_input_unraw();

                        /* Enter monitor loop. */
                        cli_monitor_thread(NULL);

                        /* Don't resume render thread if we're exiting. */
                        if (is_quit)
                            return;

                        /* Re-enable raw input. */
                        cli_input_raw();

                        /* Resume render thread. */
                        cli_render_monitorexit();

                        state = VT_GROUND;
                        break;

                    case 0x21 ... 0x2f:
                    case 0x4f:
                        cli_input_clear(c);
                        cli_input_collect(c);
                        state = VT_ESCAPE_INTERMEDIATE;
                        break;

                    case 0x20:
                    case 0x30 ... 0x4e:
                    case 0x51 ... 0x57:
                    case 0x59:
                    case 0x5a:
                    case 0x5c:
                    case 0x60 ... 0x7f:
                        cli_input_clear(c);
                        cli_input_esc_dispatch(c);
                        state = VT_GROUND;
                        break;

                    case 0x50:
                        state = VT_DCS_ENTRY;
                        cli_input_clear(c);
                        break;

                    case 0x58:
                    case 0x5e:
                    case 0x5f:
                        state = VT_SOS_PM_APC_STRING;
                        break;

                    case 0x5b:
                        state = VT_CSI_ENTRY;
                        cli_input_clear(c);
                        break;

                    case 0x5d:
                        state = VT_OSC_STRING;
                        cli_input_osc_start(c);
                        break;

                    default:
                        break;
                }
                break;

            case VT_ESCAPE_INTERMEDIATE:
                switch (c) {
                    case 0x00 ... 0x1a:
                    case 0x1c ... 0x1f:
                        cli_input_execute(c);
                        continue;

                    case 0x20 ... 0x2f:
                        cli_input_collect(c);
                        break;

                    case 0x30 ... 0x7e:
                        cli_input_esc_dispatch(c);
                        state = VT_GROUND;
                        break;

                    default:
                        break;
                }
                break;

            case VT_CSI_ENTRY:
                switch (c) {
                    case 0x00 ... 0x1a:
                    case 0x1c ... 0x1f:
                        cli_input_execute(c);
                        continue;

                    case 0x20 ... 0x2f:
                        cli_input_collect(c);
                        state = VT_ESCAPE_INTERMEDIATE;
                        break;

                    case 0x30 ... 0x39:
                    case 0x3b:
                        cli_input_param(c);
                        state = VT_CSI_PARAM;
                        break;

                    case 0x3a:
                        state = VT_CSI_IGNORE;
                        break;

                    case 0x3c ... 0x3f:
                        cli_input_collect(c);
                        state = VT_CSI_PARAM;
                        break;

                    case 0x4d:
                        /* Potential mouse tracking event. */
                        if (param_buf_pos == 0) {
                            state = VT_MOUSE_BTN;
                            break;
                        }
                        fallthrough;

                    case 0x40 ... 0x4c:
                    case 0x4e ... 0x7e:
                        cli_input_csi_dispatch(c);
                        state = VT_GROUND;
                        break;

                    default:
                        break;
                }
                break;

            case VT_CSI_IGNORE:
                switch (c) {
                    case 0x00 ... 0x1a:
                    case 0x1c ... 0x1f:
                        cli_input_execute(c);
                        continue;

                    case 0x40 ... 0x7e:
                        state = VT_GROUND;
                        break;

                    default:
                        break;
                }
                break;

            case VT_CSI_PARAM:
                switch (c) {
                    case 0x00 ... 0x1a:
                    case 0x1c ... 0x1f:
                        cli_input_execute(c);
                        continue;

                    case 0x20 ... 0x2f:
                        cli_input_collect(c);
                        state = VT_CSI_INTERMEDIATE;
                        break;

                    case 0x30 ... 0x3b:
                        cli_input_param(c);
                        break;

                    case 0x3c ... 0x3f:
                        state = VT_CSI_IGNORE;
                        break;

                    case 0x40 ... 0x7e:
                        cli_input_csi_dispatch(c);
                        state = VT_GROUND;
                        break;

                    default:
                        break;
                }
                break;

            case VT_CSI_INTERMEDIATE:
                switch (c) {
                    case 0x00 ... 0x1a:
                    case 0x1c ... 0x1f:
                        cli_input_execute(c);
                        continue;

                    case 0x20 ... 0x2f:
                        cli_input_collect(c);
                        break;

                    case 0x30 ... 0x3f:
                        state = VT_CSI_IGNORE;
                        break;

                    case 0x40 ... 0x7e:
                        cli_input_csi_dispatch(c);
                        state = VT_GROUND;
                        break;

                    default:
                        break;
                }
                break;

            case VT_DCS_ENTRY:
                if (prev_state != VT_DCS_ENTRY)
                    cli_input_clear(c);
                switch (c) {
                    case 0x20 ... 0x2f:
                        cli_input_collect(c);
                        state = VT_DCS_INTERMEDIATE;
                        break;

                    case 0x30 ... 0x39:
                    case 0x3b:
                        cli_input_param(c);
                        state = VT_DCS_PARAM;
                        break;

                    case 0x3a:
                        state = VT_DCS_IGNORE;
                        break;

                    case 0x3c ... 0x3f:
                        cli_input_collect(c);
                        state = VT_DCS_PARAM;
                        break;

                    case 0x40 ... 0x7e:
                        state = VT_DCS_PASSTHROUGH;
                        cli_input_hook(c);
                        break;

                    default:
                        break;
                }
                break;

            case VT_DCS_INTERMEDIATE:
                switch (c) {
                    case 0x20 ... 0x2f:
                        cli_input_collect(c);
                        break;

                    case 0x30 ... 0x3f:
                        state = VT_DCS_IGNORE;
                        break;

                    case 0x40 ... 0x7e:
                        state = VT_DCS_PASSTHROUGH;
                        cli_input_hook(c);
                        break;

                    default:
                        break;
                }
                break;

            case VT_DCS_PARAM:
                switch (c) {
                    case 0x20 ... 0x2f:
                        cli_input_collect(c);
                        state = VT_DCS_INTERMEDIATE;
                        break;

                    case 0x30 ... 0x39:
                    case 0x3b:
                        cli_input_param(c);
                        break;

                    case 0x3a:
                    case 0x3c ... 0x3f:
                        state = VT_DCS_IGNORE;
                        break;

                    case 0x40 ... 0x7e:
                        state = VT_DCS_PASSTHROUGH;
                        cli_input_hook(c);
                        break;

                    default:
                        break;
                }
                break;

            case VT_DCS_PASSTHROUGH:
                switch (c) {
                    case 0x00 ... 0x7e:
                        cli_input_put(c);
                        break;

                    default:
                        break;
                }
                break;

            case VT_OSC_STRING:
                switch (c) {
                    case 0x20 ... 0x7e:
                        cli_input_osc_put(c);
                        break;

                    default:
                        break;
                }
                break;

            case VT_MOUSE_BTN:
                state = VT_MOUSE_X;
                cli_input_param(c);
                break;

            case VT_MOUSE_X:
                state = VT_MOUSE_Y;
                cli_input_param(c);
                break;

            case VT_MOUSE_Y:
                state = VT_GROUND;
                cli_input_param(c);

                /* Check for mouse parameter validity. */
                if (param_buf_pos < 3)
                    return;

                /* Interpret mouse tracking data. */
                int btn = param_buf[0] - ' ';
                int mod = (btn >> 2) & 0x07; /* modifiers [4:2] */
                btn = (btn & 0x03) | ((btn & 0xc0) >> 4); /* buttons [7:6,1:0] */
                int x = param_buf[1] - ' ' - 1;
                int y = param_buf[2] - ' ' - 1;
                cli_input_log("CLI Input: Mouse buttons %d modifiers %02X at %d,%d\n", btn, mod, x, y);

                /* Convert and send coordinates. */
                int mouse_x_abs = x * ((double) get_actual_size_x() / (cli_term.size_x - 1));
                cli_input_log("X %d * (%d / %d) = %d\n", x, get_actual_size_x(), cli_term.size_x - 1, mouse_x_abs);
                int mouse_y_abs = y * ((double) get_actual_size_y() / (cli_term.size_y - 1));
                cli_input_log("Y %d * (%d / %d) = %d\n", y, get_actual_size_y(), cli_term.size_y - 1, mouse_y_abs);
                mouse_scale(mouse_x_abs - mouse_x_prev, mouse_y_abs - mouse_y_prev);
                mouse_x_prev = mouse_x_abs;
                mouse_y_prev = mouse_y_abs;
                cli_input_log("afterwards %d %d\n", mouse_x_abs, mouse_y_abs);

                /* Send buttons. */
                if (btn == 4) /* wheel back */
                    mouse_set_z(-1);
                else if (btn == 5) /* wheel forward */
                    mouse_set_z(1);
                else if (btn < sizeof(mouse_button_values))
                    mouse_set_buttons_ex(mouse_button_values[btn]);
                break;

            default:
                break;
        }
    }
}

void
cli_input_init(void)
{
    /* Don't initialize input altogether if stdin is not a tty. */
    if (!isatty(fileno(stdin))) {
        cli_input_log("CLI Input: stdin is not a tty\n");
        return;
    }
    cli_term.can_input = 1;

    /* Enable raw input. */
    cli_input_raw();

    /* Start input processing thread. */
    thread_create(cli_input_process, NULL);
}

void
cli_input_close(void)
{
    /* Restore terminal state. */
    cli_input_unraw();
}
#endif
