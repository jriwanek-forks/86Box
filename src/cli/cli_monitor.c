/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Monitor console module for the command line interface.
 *
 * Authors: Cacodemon345
 *          RichardG, <richardg867@gmail.com>
 *
 *          Copyright 2021-2023 Cacodemon345.
 *          Copyright 2021-2025 RichardG.
 */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#ifdef _WIN32
#    include <fcntl.h>
#else
#    include <dlfcn.h>
#    include <unistd.h>
#endif
#include <86box/86box.h>
#include <86box/cartridge.h>
#include <86box/scsi_device.h>
#include <86box/cdrom.h>
#include <86box/cli.h>
#include <86box/config.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/fdd.h>
#include <86box/mo.h>
#include <86box/plat.h>
#include <86box/plat_dir.h>
#include <86box/plat_dynld.h>
#include <86box/thread.h>
#include <86box/version.h>
#include <86box/video.h>
#include <86box/zip.h>

#define MONITOR_CMD_EXIT      0x01
#define MONITOR_CMD_UNBOUNDED 0x02
#define MONITOR_CMD_NOQUOTE   0x04

#ifndef _WIN32
#    ifdef __APPLE__
#        define PATH_LIBEDIT_DLL     "libedit.2.dylib"
#        define PATH_LIBEDIT_DLL_ALT "libedit.dylib"
#    else
#        define PATH_LIBEDIT_DLL     "libedit.so.2"
#        define PATH_LIBEDIT_DLL_ALT "libedit.so"
#    endif
#endif

/* Lookup tables for converting key names to keyboard scan codes. */
static const struct {
    const char *name;
    uint16_t    code;
} named_seqs[] = {
  // clang-format off
    {"tab",        0x000f },
    { "enter",           0x001c },
    { "ret",0x001c },
    { "return",       0x001c },
    { "spc",           0x0039 },
    { "space",0x0039 },
    { "bksp",       0x000e },
    { "bkspc",           0x000e },
    { "backsp",       0x000e },
    { "backspc",      0x000e },
    { "backspace",           0x000e },
    { "menu",0xe05d },

    { "esc",    0x0001 },
    { "escape",           0x0001 },
    { "f1", 0x003b },
    { "f2", 0x003c },
    { "f3",           0x003d },
    { "f4",           0x003e },
    { "f5",       0x003f },
    { "f6",           0x0040 },
    { "f7",0x0041 },
    { "f8",        0x0042 },
    { "f9",           0x0043 },
    { "f10",      0x0044 },
    { "f11",        0x0057 },
    { "f12",           0x0058 },
    { "prtsc",0xe037 },
    { "prtscreen",        0xe037 },
    { "printsc",           0xe037 },
    { "printscreen",0xe037 },
    { "sysrq",        0xe037 },
    { "pause",           0xe11d },
    { "brk",0xe11d },
    { "break",        0xe11d },
    { "pausebrk",           0xe11d },
    { "pausebreak", 0xe11d },

    { "home",       0xe047 },
    { "ins",           0xe052 },
    { "insert",0xe052 },
    { "del",     0xe053 },
    { "delete",           0xe053 },
    { "end",0xe04f },
    { "pgup",   0xe049 },
    { "pageup",           0xe049 },
    { "pgdn",           0xe051 },
    { "pgdown",     0xe051 },
    { "pagedn",           0xe051 },
    { "pagedown",0xe051 },

    { "up",       0xe048 },
    { "down",           0xe050 },
    { "right",0xe04d },
    { "left",  0xe04b },

    { "numlk",           0x0045 },
    { "numlock",      0x0045 },
    { "capslk",      0x003a },
    { "capslock",           0x003a },
    { "scrlk",0x0046 },
    { "scrlock",    0x0046 },
    { "scrolllk",           0x0046 },
    { "scrolllock",  0x0046 },
    { 0   }
  // clang-format on
};

static int      first_run = 1;
static event_t *screenshot_event;

#ifndef _WIN32
/* libedit dynamic loading imports. */
static char *(*readline)(const char *)          = NULL;
static int (*add_history)(const char *)         = NULL;
static void (*rl_callback_handler_remove)(void) = NULL;
static FILE **rl_outstream                      = NULL;

static dllimp_t libedit_imports[] = {
  // clang-format off
    { "readline",                   &readline                   },
    { "add_history",                &add_history                },
    { "rl_callback_handler_remove", &rl_callback_handler_remove },
    { "rl_outstream",               &rl_outstream               },
    { NULL,                         NULL                        }
  // clang-format on
};
static void *libedit_handle = NULL;
#endif

typedef struct {
    void       *func;
    int         ndrives;
    uint8_t     allow_dir;
    const char *drive;
} media_cmd_t;

static int
cli_monitor_parsebool(char *arg)
{
    char ch = arg[0];
    if ((ch == 'o') || (ch == 'O')) {
        ch = arg[1];
        return (ch == 'n') || (ch == 'N');
    }
    return (ch == '1') || (ch == 'y') || (ch == 'Y') || (ch == 't') || (ch == 'T') || (ch == 'e') || (ch == 'E');
}

static int
cli_monitor_parsefile(char *path, uint8_t allow_dir, uint8_t wp)
{
    if (strlen(path) >= PATH_MAX) {
        fprintf(CLI_RENDER_OUTPUT, "File path too long.\n");
        return -4;
    }

    FILE *f = fopen(path, "rb");
    DIR *dir;
    if (f) {
        fclose(f);
        if (!wp) {
            f = fopen(path, "ab");
            if (f) {
                fclose(f);
            } else {
                if (errno == EPERM)
                    fprintf(CLI_RENDER_OUTPUT, "No permission to write file, enabling write protection.\n");
                else
                    fprintf(CLI_RENDER_OUTPUT, "File is read-only, enabling write protection.\n");
                wp = 1;
            }
        }
        return wp;
    } else if ((dir = opendir(path))) {
        closedir(dir);
        if (allow_dir) {
            return wp;
        } else {
            fprintf(CLI_RENDER_OUTPUT, "Not a file: %s\n", path);
            return -3;
        }
    } else if (errno == EPERM) {
        fprintf(CLI_RENDER_OUTPUT, "No permission to read file: %s\n", path);
        return -2;
    } else {
        fprintf(CLI_RENDER_OUTPUT, "File not found %d: %s\n", errno, path);
        return -1;
    }
}

static int
cli_monitor_parsemediaid(media_cmd_t *cmd, char *id_s)
{
    int id;
    if ((sscanf(id_s, "%d", &id) != 1) || (id < 0) || (id >= cmd->ndrives)) {
        fprintf(CLI_RENDER_OUTPUT, "Invalid %s ID, expected 0-%d.\n", cmd->drive, cmd->ndrives);
        return -1;
    }
    return id;
}

static void
cli_monitor_mediaload(int argc, char **argv, const void *priv)
{
    /* Read media command information. */
    media_cmd_t *cmd                                     = (media_cmd_t *) priv;
    void (*mount_func)(uint8_t id, char *fn, uint8_t wp) = cmd->func;

    /* Read and validate ID. */
    int id = cli_monitor_parsemediaid(cmd, argv[1]);
    if (id < 0)
        return;

    /* Read write protect flag. */
    int wp = (argc >= 3) && cli_monitor_parsebool(argv[3]);

    /* Validate file path. */
    wp = cli_monitor_parsefile(argv[2], cmd->allow_dir, wp);
    if (wp < 0)
        return;

    /* Provide feedback. */
    fprintf(CLI_RENDER_OUTPUT, "Inserting %simage into %s %d: %s\n",
            wp ? "write-protected " : "",
            cmd->drive, id,
            argv[2]);

    /* Call mount function. */
    mount_func(id, argv[2], wp);
}

static void
cli_monitor_mediaload_nowp(int argc, char **argv, const void *priv)
{
    /* Read media command information. */
    media_cmd_t *cmd                         = (media_cmd_t *) priv;
    void (*mount_func)(uint8_t id, char *fn) = cmd->func;

    /* Read and validate ID. */
    int id = cli_monitor_parsemediaid(cmd, argv[1]);
    if (id < 0)
        return;

    /* Validate file path. */
    if (cli_monitor_parsefile(argv[2], cmd->allow_dir, 1) < 0)
        return;

    /* Provide feedback. */
    fprintf(CLI_RENDER_OUTPUT, "Inserting image into %s %d: %s\n",
            cmd->drive, id, argv[2]);

    /* Call mount function. */
    mount_func(id, argv[2]);
}

static void
cli_monitor_mediaeject(int argc, char **argv, const void *priv)
{
    /* Read media command information. */
    media_cmd_t *cmd               = (media_cmd_t *) priv;
    void (*eject_func)(uint8_t id) = cmd->func;

    /* Read and validate ID. */
    int id = cli_monitor_parsemediaid(cmd, argv[1]);
    if (id < 0)
        return;

    /* Provide feedback. */
    fprintf(CLI_RENDER_OUTPUT, "Ejecting image from %s %d.\n",
            cmd->drive, id);

    /* Call eject function. */
    eject_func(id);
}

static void
cli_monitor_mediaeject_mountblank_nowp(int argc, char **argv, const void *priv)
{
    /* Read media command information. */
    media_cmd_t *cmd                         = (media_cmd_t *) priv;
    void (*mount_func)(uint8_t id, char *fn) = cmd->func;

    /* Read and validate ID. */
    int id = cli_monitor_parsemediaid(cmd, argv[1]);
    if (id < 0)
        return;

    /* Provide feedback. */
    fprintf(CLI_RENDER_OUTPUT, "Ejecting image from %s %d.\n",
            cmd->drive, id);

    /* Call eject function. */
    mount_func(id, "");
}

static void
cli_monitor_sendkey(int argc, char **argv, const void *priv)
{
    /* Parse key combination. */
    char     ch, sch, *p = argv[1];
    uint8_t  modifier = 0;
    uint16_t code     = 0;
    int      j;
    for (int i = 0;; i++) {
        /* We don't care about non-separator characters. */
        ch = argv[1][i];
        if ((ch != '\0') && (ch != ' ') && (ch != '-') && (ch != '+') && (ch != ',') && (ch != ';') && (ch != '_') && (ch != ':'))
            continue;

        /* Terminate this key name here. */
        argv[1][i] = '\0';

        /* Parse this key name. */
        if (!p[0]) { /* blank */
            /* Treat the separator as a character. (ctrl-- = ctrl + -) */
            argv[1][i] = sch = ch;
            goto separator;
        } else if (!p[1]) { /* single character */
            sch = p[0];

            /* Lowercase letters. */
            if ((sch >= 'A') && (sch <= 'Z'))
                sch += 32;

separator: /* Search lookup table. */
            if ((sch < (sizeof(ascii_seqs) / sizeof(ascii_seqs[0]))) && ascii_seqs[(uint8_t) sch]) {
                /* Set keycode. */
                code = ascii_seqs[(uint8_t) sch];
            } else {
                /* No match. */
                fprintf(CLI_RENDER_OUTPUT, "Unknown key: %c\n", sch);
                return;
            }
        } else if (!stricmp(p, "ctrl") || !stricmp(p, "control")) { /* modifiers */
            modifier |= VT_CTRL;
        } else if (!stricmp(p, "shift")) {
            modifier |= VT_SHIFT;
        } else if (!stricmp(p, "alt")) {
            modifier |= VT_ALT;
        } else if (!stricmp(p, "win") || !stricmp(p, "windows") || !stricmp(p, "meta")) {
            modifier |= VT_META;
        } else { /* other */
            /* Search named sequences. */
            for (j = 0; named_seqs[j].name; j++) {
                /* Move on if this sequence doesn't match. */
                if (stricmp(p, named_seqs[j].name))
                    continue;

                /* Set keycode and stop search. */
                code = named_seqs[j].code;
                break;
            }

            if (!code) {
                /* No match. */
                fprintf(CLI_RENDER_OUTPUT, "Unknown key: %s\n", p);
                return;
            }
        }

        /* Convert key name to sentence case for display purposes. */
        if (p[0]) {
            if ((p[0] >= 'a') && (p[0] <= 'z'))
                p[0] -= 32;
            for (j = 1; p[j]; j++) {
                sch = p[j];
                if ((sch >= 'A') && (sch <= 'Z'))
                    p[j] += 32;
            }
        }

        /* Stop if we've reached a termination key or the end. */
        if (code || (ch == '\0')) {
            argv[1][i] = '\0';
            break;
        }

        /* Restore separator and start the next key name. */
        argv[1][i] = '+';
        p          = &argv[1][i + 1];
    }

    /* Send key combination. */
    cli_input_send(code, modifier | VT_KEY_DOWN | VT_KEY_UP);
    fprintf(CLI_RENDER_OUTPUT, "Key combination sent: %s\n", argv[1]);
}

static void
cli_monitor_type(int argc, char **argv, const void *priv)
{
    /* Send individual keys. */
    char     ch;
    uint16_t code;
    int      utf8_warned = 0;
    for (int i = 0; argv[1][i]; i++) {
        /* Ignore and warn about UTF-8 sequences. */
        ch = argv[1][i];
        if (ch & 0x80) {
            if (!utf8_warned) {
                utf8_warned = 1;
                fprintf(CLI_RENDER_OUTPUT, "Ignoring UTF-8 characters.\n");
            }
            continue;
        }

        /* Convert character to keycode. */
        if (ch < (sizeof(ascii_seqs) / sizeof(ascii_seqs[0])))
            code = ascii_seqs[(uint8_t) ch];
        else
            code = 0;

        /* Convert Ctrl+letter codes. */
        if (!code && (ch <= 0x1a))
            code = ascii_seqs['`' + ch];

        /* Send key if a table match was found, otherwise warn about unknown keys. */
        if (code)
            cli_input_send(code, VT_KEY_DOWN | VT_KEY_UP);
        else
            fprintf(CLI_RENDER_OUTPUT, "Ignoring unknown key: %c\n", ch);
    }
}

static void
cli_monitor_hardreset(int argc, char **argv, const void *priv)
{
    fprintf(CLI_RENDER_OUTPUT, "Hard resetting emulated machine.\n");
    pc_reset_hard();
}

static void
cli_monitor_pause(int argc, char **argv, const void *priv)
{
    plat_pause(dopause ^ 1);
    fprintf(CLI_RENDER_OUTPUT, "Emulated machine %spaused.\n", dopause ? "" : "un");
}

static void
cli_monitor_fullscreen(int argc, char **argv, const void *priv)
{
    video_fullscreen ^= 1;
    fprintf(CLI_RENDER_OUTPUT, "Fullscreen mode %s.\n", video_fullscreen ? "entered" : "exited");
}

static void
cli_monitor_screenshot_hook(char *path, uint32_t *buf, int start_x, int start_y, int w, int h, int row_len)
{
    /* This hook should only be called once. */
    screenshot_hook = NULL;

    /* Print screenshot path. */
    fprintf(CLI_RENDER_OUTPUT, "Saved screenshot to: %s\n", path);

#ifdef USE_CLI
    /* Render screenshot if supported by the terminal. */
    cli_render_process_screenshot(path, buf, start_x, start_y, w, h, row_len);
#endif

    /* Allow monitor thread to proceed. */
    thread_set_event(screenshot_event);
}

static void
cli_monitor_screenshot(int argc, char **argv, const void *priv)
{
    /* Set up screenshot hook. */
    screenshot_event = thread_create_event();
    screenshot_hook  = cli_monitor_screenshot_hook;

    /* Take screenshot. */
    startblit();
    for (int i = 0; i < MONITORS_NUM; i++)
        monitors[i].mon_screenshots++;
    endblit();
    device_force_redraw();

    /* Wait for the hook. */
    thread_wait_event(screenshot_event, -1);
    thread_destroy_event(screenshot_event);
}

static void
cli_monitor_exit(int argc, char **argv, const void *priv)
{
    fprintf(CLI_RENDER_OUTPUT, "Exiting.\n");
    do_stop();
}

static void cli_monitor_help(int argc, char **argv, const void *priv);

enum {
    MONITOR_CATEGORY_MEDIALOAD = 0,
    MONITOR_CATEGORY_MEDIAEJECT,
    MONITOR_CATEGORY_INPUT,
    MONITOR_CATEGORY_EMULATOR,
    MONITOR_CATEGORY_HIDDEN
};

static const struct {
    const char *name, *helptext, **args;
    uint8_t     args_min, args_max, flags, category;
    void (*handler)(int argc, char **argv, const void *priv);
    const void *priv;
} commands[] = {
    {.name     = "fddload",
     .helptext = "Load floppy disk image <filename> into drive <id>.\n[wp] enables write protection when set to 1.",
     .args     = (const char *[]) { "id", "filename", "wp" },
     .args_min = 2,
     .args_max = 3,
     .category = MONITOR_CATEGORY_MEDIALOAD,
     .handler  = cli_monitor_mediaload,
     .priv     = &(const media_cmd_t) { floppy_mount, FDD_NUM, 0, "floppy drive" } },
    { .name     = "cdload",
     .helptext = "Load CD-ROM image <filename> into drive <id>.",
     .args     = (const char *[]) { "id", "filename" },
     .args_min = 2,
     .args_max = 2,
     .category = MONITOR_CATEGORY_MEDIALOAD,
     .handler  = cli_monitor_mediaload,
     .priv     = &(const media_cmd_t) { cdrom_mount, CDROM_NUM, 1, "CD-ROM drive" } },
    { .name     = "zipload",
     .helptext = "Load ZIP disk image <filename> into drive <id>.\n[wp] enables write protection when set to 1.",
     .args     = (const char *[]) { "id", "filename", "wp" },
     .args_min = 2,
     .args_max = 3,
     .category = MONITOR_CATEGORY_MEDIALOAD,
     .handler  = cli_monitor_mediaload,
     .priv     = &(const media_cmd_t) { zip_load, ZIP_NUM, 0, "ZIP drive" } },
    { .name     = "moload",
     .helptext = "Load MO disk image <filename> into drive <id>.\n[wp] enables write protection when set to 1.",
     .args     = (const char *[]) { "id", "filename", "wp" },
     .args_min = 2,
     .args_max = 3,
     .category = MONITOR_CATEGORY_MEDIALOAD,
     .handler  = cli_monitor_mediaload,
     .priv     = &(const media_cmd_t) { mo_mount, MO_NUM, 0, "MO drive" } },
    { .name     = "cartload",
     .helptext = "Load cartridge <filename> image into slot <id>.\n[wp] enables write protection when set to 1.",
     .args     = (const char *[]) { "id", "filename", "wp" },
     .args_min = 2,
     .args_max = 3,
     .category = MONITOR_CATEGORY_MEDIALOAD,
     .handler  = cli_monitor_mediaload_nowp,
     .priv     = &(const media_cmd_t) { cart_load, sizeof(cart_fns) / sizeof(cart_fns[0]), 0, "cartridge slot" } },

    { .name     = "fddeject",
     .helptext = "Eject disk from floppy drive <id>.",
     .args     = (const char *[]) { "id" },
     .args_min = 1,
     .args_max = 1,
     .category = MONITOR_CATEGORY_MEDIAEJECT,
     .handler  = cli_monitor_mediaeject,
     .priv     = &(const media_cmd_t) { floppy_eject, FDD_NUM, 0, "floppy drive" } },
    { .name     = "cdeject",
     .helptext = "Eject disc from CD-ROM drive <id>.",
     .args     = (const char *[]) { "id" },
     .args_min = 1,
     .args_max = 1,
     .category = MONITOR_CATEGORY_MEDIAEJECT,
     .handler  = cli_monitor_mediaeject_mountblank_nowp,
     .priv     = &(const media_cmd_t) { cdrom_mount, CDROM_NUM, 1, "CD-ROM drive" } },
    { .name     = "zipeject",
     .helptext = "Eject disk from ZIP drive <id>.",
     .args     = (const char *[]) { "id" },
     .args_min = 1,
     .args_max = 1,
     .category = MONITOR_CATEGORY_MEDIAEJECT,
     .handler  = cli_monitor_mediaeject,
     .priv     = &(const media_cmd_t) { zip_eject, ZIP_NUM, 0, "ZIP drive" } },
    { .name     = "moeject",
     .helptext = "Eject disk from MO drive <id>.",
     .args     = (const char *[]) { "id" },
     .args_min = 1,
     .args_max = 1,
     .category = MONITOR_CATEGORY_MEDIAEJECT,
     .handler  = cli_monitor_mediaeject,
     .priv     = &(const media_cmd_t) { mo_eject, MO_NUM, 0, "MO drive" } },
    { .name     = "carteject",
     .helptext = "Eject cartridge from slot <id>.",
     .args     = (const char *[]) { "id" },
     .args_min = 1,
     .args_max = 1,
     .category = MONITOR_CATEGORY_MEDIAEJECT,
     .handler  = cli_monitor_mediaeject,
     .priv     = &(const media_cmd_t) { cartridge_eject, sizeof(cart_fns) / sizeof(cart_fns[0]), 0, "cartridge slot" } },
    {
     .name     = "sendkey",
     .helptext = "Send key combination <combo>.",
     .args     = (const char *[]) { "combo" },
     .args_min = 1,
     .flags    = MONITOR_CMD_UNBOUNDED,
     .category = MONITOR_CATEGORY_INPUT,
     .handler  = cli_monitor_sendkey,
     },
    {
     .name     = "type",
     .helptext = "Type <text> on the keyboard.",
     .args     = (const char *[]) { "text" },
     .args_min = 1,
     .flags    = MONITOR_CMD_UNBOUNDED | MONITOR_CMD_NOQUOTE,
     .category = MONITOR_CATEGORY_INPUT,
     .handler  = cli_monitor_type,
     },

    { .name     = "hardreset",
     .helptext = "Hard reset the emulated machine.",
     .category = MONITOR_CATEGORY_EMULATOR,
     .handler  = cli_monitor_hardreset },
    { .name     = "pause",
     .helptext = "Pause or unpause the emulated machine.",
     .category = MONITOR_CATEGORY_EMULATOR,
     .handler  = cli_monitor_pause },
    { .name     = "fullscreen",
     .helptext = "Enter or exit fullscreen mode.",
     .category = MONITOR_CATEGORY_EMULATOR,
     .handler  = cli_monitor_fullscreen },
    { .name     = "screenshot",
     .helptext = "Take a screenshot.",
     .category = MONITOR_CATEGORY_EMULATOR,
     .handler  = cli_monitor_screenshot },
    { .name     = "exit",
     .helptext = "Exit " EMU_NAME ".",
     .flags    = MONITOR_CMD_EXIT,
     .category = MONITOR_CATEGORY_EMULATOR,
     .handler  = cli_monitor_exit
#ifdef USE_CLI
    },
    { .name     = "back",
     .helptext = "Return to the screen.",
     .flags    = MONITOR_CMD_EXIT,
     .category = MONITOR_CATEGORY_EMULATOR
#endif
    },

    { .name     = "help",
     .helptext = "List all commands, or show detailed usage for <command>.",
     .args     = (const char *[]) { "command" },
     .args_max = 1,
     .category = MONITOR_CATEGORY_HIDDEN,
     .handler  = cli_monitor_help },
    { 0                                                   }
};

static void
cli_monitor_printargs(int cmd)
{
    /* Make sure this is a valid command. */
    if ((cmd >= (sizeof(commands) / sizeof(commands[0]))) || !commands[cmd].name)
        return;

    /* Output command name. */
    fputs(commands[cmd].name, CLI_RENDER_OUTPUT);

    /* Determine argument count. */
    if (!commands[cmd].args)
        return;
    int max_args = (commands[cmd].flags & MONITOR_CMD_UNBOUNDED) ? 1 : commands[cmd].args_max;

    /* Output argument names. */
    for (int arg = 0; (arg < max_args) && commands[cmd].args[arg]; arg++) {
        if (arg < commands[cmd].args_min)
            fprintf(CLI_RENDER_OUTPUT, " <%s>", commands[cmd].args[arg]);
        else
            fprintf(CLI_RENDER_OUTPUT, " [%s]", commands[cmd].args[arg]);
    }
}

static void
cli_monitor_helptext(int cmd, int limit)
{
    /* Output nothing if the command has no helptext. */
    if (!commands[cmd].helptext)
        return;

    /* Copy the command's helptext. */
    char *buf = strdup(commands[cmd].helptext), *p = buf, ch;
    if (!buf)
        return;

    /* Print each helptext line. */
    for (int i = 0;; i++) {
        /* We don't care about non-termination characters. */
        ch = buf[i];
        if ((ch != '\n') && (ch != '\0'))
            continue;

        /* Remove trailing period if this is a single-line helptext. */
        if ((limit == 1) && (i > 0) && (buf[i - 1] == '.'))
            buf[i - 1] = '\0';

        /* Terminate this line here and print it. */
        buf[i] = '\0';
        fprintf(CLI_RENDER_OUTPUT, "%c %s\n", (p == buf) ? '-' : ' ', p);

        /* Stop if we've reached the line limit or the helptext's end. */
        if ((--limit == 0) || (ch == '\0'))
            break;

        /* Start the next line. */
        p = &buf[i + 1];
    }

    /* Clean up. */
    free(buf);
}

static void
cli_monitor_usage(int cmd)
{
    cli_monitor_printargs(cmd);
    fprintf(CLI_RENDER_OUTPUT, "\n");
    cli_monitor_helptext(cmd, 0);
}

static int
cli_monitor_getcmd(const char *name)
{
    /* Go through the command list. */
    size_t cmd_len = strlen(name), cmd_count = 0;
    int    cmd, first_cmd                    = -1;
    for (cmd = 0; commands[cmd].name; cmd++) {
        /* Check if the command name matches. */
        if (!strnicmp(name, commands[cmd].name, cmd_len)) {
            /* Store the first found command. */
            if (first_cmd < 0)
                first_cmd = cmd;

            /* Stop if more than one matching command was found. */
            if (cmd_count++ >= 1)
                break;
        }
    }

    /* Handle ambiguous commands. */
    if (cmd_count > 1) {
        /* Output list of ambiguous commands. */
        fprintf(CLI_RENDER_OUTPUT, "Ambiguous command: ");
        for (cmd = 0; commands[cmd].name; cmd++) {
            if (!strnicmp(name, commands[cmd].name, cmd_len)) {
                if (cmd_count) {
                    cmd_count = 0;
                    fputs(commands[cmd].name, CLI_RENDER_OUTPUT);
                } else {
                    fprintf(CLI_RENDER_OUTPUT, ", %s", commands[cmd].name);
                }
            }
        }
        fputc('\n', CLI_RENDER_OUTPUT);

        /* Inform callers that this was an ambiguous command. */
        return -1;
    } else if (first_cmd > -1) {
        /* Take the first/only found command. */
        return first_cmd;
    } else {
        /* Take the final dummy command. */
        return cmd;
    }
}

static void
cli_monitor_help(int argc, char **argv, const void *priv)
{
    /* Print help for a specific command if one was provided. */
    int cmd;
    if (argv[1] && argv[1][0]) {
        cmd = cli_monitor_getcmd(argv[1]);
        if (cmd < 0) {
            return;
        } else if (commands[cmd].name) {
            cli_monitor_usage(cmd);
            return;
        }
        fprintf(CLI_RENDER_OUTPUT, "Unknown command: %s\n", argv[1]);
        return;
    }

    /* List all commands. */
    int category = 0;
    for (cmd = 0; commands[cmd].name; cmd++) {
        /* Don't list commands with no helptext or hidden commands. */
        if (!commands[cmd].helptext || (commands[cmd].category == MONITOR_CATEGORY_HIDDEN))
            continue;

        /* Print blank line if this is a new category. */
        if (commands[cmd].category != category) {
            category = commands[cmd].category;
            fprintf(CLI_RENDER_OUTPUT, "\n");
        }

        /* Print arguments and single-line helptext. */
        cli_monitor_printargs(cmd);
        fputc(' ', CLI_RENDER_OUTPUT);
        cli_monitor_helptext(cmd, 1);
    }
}

void
cli_monitor_thread(void *priv)
{
    /* The monitor should only be available if both stdin and output are not redirected. */
    if (!isatty(fileno(stdin)) || !isatty(fileno(CLI_RENDER_OUTPUT)))
        return;

    if (first_run) {
        first_run = 0;
        fprintf(CLI_RENDER_OUTPUT, EMU_NAME " monitor console.\n");
    }

    char buf[4096], *line = NULL, *argv[8],
                    ch, in_quote;
    int argc, end, i, j, cmd, arg_start;

    /* Read and process commands. */
    while (!feof(stdin)) {
        /* Read line. */
#ifndef _WIN32
        if (readline) {
            line = readline("(" EMU_NAME ") ");
        } else
#endif
        {
            fprintf(CLI_RENDER_OUTPUT, "(" EMU_NAME ") ");
            line = fgets(buf, sizeof(buf), stdin);
        }
        if (!line)
            continue;

        /* Remove trailing newline characters. */
        end = strlen(line) - 1;
        while (end && (((line[end] == '\r') || (line[end] == '\n'))))
            line[end--] = '\0';

#ifndef _WIN32
        /* Add line to libedit history. */
        if (add_history)
            add_history(line);
#endif

        /* Prepare line parsing. */
        memset(argv, 0, sizeof(argv));
        argc = arg_start = 0;
        in_quote         = 0;

        /* Remove leading spaces from the line. */
        i = j = 0;
        while (line[i] == ' ')
            i++;

        /* Remove trailing spaces and newlines from the line. */
        while ((line[end] == ' ') || (line[end] == '\r') || (line[end] == '\n'))
            line[end--] = '\0';

        /* Ignore blank lines. */
        cmd = (sizeof(commands) / sizeof(commands[0])) - 1;
        if (!line[0])
            goto have_cmd;

        /* Parse line. */
        for (; i <= end; i++) {
            ch = line[i];
            if (ch == '\\') {
#ifdef _WIN32 /* On Windows, treat \ as a path separator if the \
                 next character is a valid filename character. */
                ch = line[i + 1];
                if ((ch != '\\') && (in_quote || (ch != ' ')) && (ch != '/') && (ch != ':') && (ch != '*') && (ch != '?') && (ch != '"') && (ch != '<') && (ch != '>') && (ch != '|')) {
                    line[j++] = '\\';
                    continue;
                }
#endif
                /* Add escaped character. */
                line[j++] = line[++i];
            } else if ((ch == '"') || (ch == '\'')) {
                /* Enter or exit quote mode. */
                if (!in_quote && !(commands[cmd].flags & MONITOR_CMD_NOQUOTE) && (i != end))
                    in_quote = ch;
                else if (in_quote == ch)
                    in_quote = 0;
                else
                    line[j++] = ch;
            } else if (!in_quote && (line[i] == ' ')) {
                /* Terminate and save this argument. */
                line[j++]    = '\0';
                argv[argc++] = &line[arg_start];
                arg_start    = j;

                /* Identify command now to disable quote mode for commands which request that. */
                if (argc == 1) {
identify_cmd:
                    /* Find one or more matching commands, and bypass
                       handling if this is an ambiguous command. */
                    cmd = cli_monitor_getcmd(argv[0]);
                    if (cmd < 0)
                        goto have_cmd;

                    /* Return to main flow if this is the second check further down. */
                    if (!ch)
                        goto have_args;
                }

                /* Stop if we have too many arguments. */
                if (argc >= (sizeof(argv) / sizeof(argv[0])))
                    goto have_args;
            } else {
                /* Add character. */
                line[j++] = ch;
            }
        }

        /* Add final argument. */
        line[j++]    = '\0';
        argv[argc++] = &line[arg_start];

        /* Identify command now if it wasn't identified earlier. */
        if (argc == 1) {
            ch = 0;
            goto identify_cmd;
        }

have_args:
        argc--;

        /* Process command if any match was found. */
        if (commands[cmd].name) {
            /* Flatten arguments for unbounded commands. */
            if (commands[cmd].flags & MONITOR_CMD_UNBOUNDED) {
                for (i = 2; i <= argc; i++)
                    *(&argv[i][0] - 1) = ' ';
            }

            /* Check number of arguments. */
            if ((argc < commands[cmd].args_min) || (!(commands[cmd].flags & MONITOR_CMD_UNBOUNDED) && (argc > commands[cmd].args_max))) {
                /* Print usage and don't process this command. */
                cli_monitor_usage(cmd);
            } else {
                /* Call command handler. */
                if (commands[cmd].handler)
                    commands[cmd].handler(argc, argv, commands[cmd].priv);
            }
        } else {
            /* No matching command was found. */
            fprintf(CLI_RENDER_OUTPUT, "Unknown command: %s\n", argv[0]);
        }

have_cmd:
        /* Clean up, while saving a "line not blank" flag. */
        ch = line[0];
#ifndef _WIN32
        if (readline)
            free(line);
#endif

        /* Stop thread if the line has a valid exiting command. */
        if (ch && (commands[cmd].flags & MONITOR_CMD_EXIT))
            break;
    }
}

void
cli_monitor_init(int independent)
{
    /* The monitor should only be available if both stdin and output are not redirected. */
    if (!isatty(fileno(stdin)) || !isatty(fileno(CLI_RENDER_OUTPUT)))
        return;

#ifndef _WIN32
    /* Try loading libedit. We don't need libedit on Windows since cmd has
       its own line editing, which is activated when raw input is disabled. */
    libedit_handle = dynld_module(PATH_LIBEDIT_DLL, libedit_imports);
    if (!libedit_handle)
        libedit_handle = dynld_module(PATH_LIBEDIT_DLL_ALT, libedit_imports);
    if (libedit_handle && readline) {
        if (rl_outstream)
            *rl_outstream = CLI_RENDER_OUTPUT;
    } else {
        fprintf(CLI_RENDER_OUTPUT, "libedit not loaded, monitor line editing will be limited.\n");
    }
#endif

    if (independent) {
        /* Start monitor processing thread. */
        thread_create(cli_monitor_thread, NULL);
    }
}

void
cli_monitor_close(void)
{
#ifndef _WIN32
    /* Stop and close libedit. */
    if (rl_callback_handler_remove)
        rl_callback_handler_remove();
    if (libedit_handle)
        dynld_close(libedit_handle);
#endif
}
