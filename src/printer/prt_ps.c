/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of a generic PostScript printer and a
 *          generic PCL 5e printer.
 *
 * Authors: David Hrdlička, <hrdlickadavid@outlook.com>
 *          Cacodemon345
 *
 *          Copyright 2019-2026 David Hrdlička.
 *          Copyright 2024-2026 Cacodemon345.
 */
/*
 * Compatibility work copy based on prt_ps.c. The original baseline is left
 * unchanged. This version adds generic PJL/UEL handling, binary-safe output, and separate
 * Ghostscript/GhostPCL API lifetimes while retaining the two existing devices.
 */
#include <inttypes.h>
#include <memory.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/lpt.h>
#include <86box/pit.h>
#include <86box/path.h>
#include <86box/plat.h>
#include <86box/plat_dynld.h>
#include <86box/ui.h>
#include "cpu.h"
#include <86box/prt_papersizes.h>

#ifdef _WIN32
#    define GSDLLAPI __stdcall
#else
#    define GSDLLAPI
#endif

#define GS_ARG_ENCODING_UTF8 1
#define gs_error_Quit        (-101)

#define POSTSCRIPT_BUFFER_LENGTH 65536

enum {
    LANG_RAW    = 0,
    LANG_PS,
    LANG_PCL_5E,
    LANG_PCL_5C,
    LANG_HP_RTL,
    LANG_PCL_6
};

typedef struct ps_t {
    void *      lpt;

    pc_timer_t  pulse_timer;
    pc_timer_t  timeout_timer;

    bool        ack;
    bool        select;
    bool        autofeed;
    bool        pcl;
    bool        pending;
    bool        pjl;
    bool        pjl_mismatch;
    bool        pjl_line_overflow;
    bool        interpreter_ref;

    uint8_t     data;

    char        printer_path[260];
    char        filename[260];

    uint8_t     buffer[POSTSCRIPT_BUFFER_LENGTH];

    uint8_t     ctrl;

    char        pjl_line[256];
    size_t      pjl_line_pos;
    uint8_t     uel_candidate[9];
    size_t      uel_pos;

    int         lang;
    int         paper_size;
    int         resolution;

    size_t      buffer_pos;
} ps_t;

typedef struct gsapi_revision_s {
    const char *product;
    const char *copyright;

    long        revision;
    long        revisiondate;
} gsapi_revision_t;

typedef struct gsapi_api_s {
    int(GSDLLAPI *revision)(gsapi_revision_t *pr, int len);
    int(GSDLLAPI *new_instance)(void **pinstance, void *caller_handle);
    void(GSDLLAPI *delete_instance)(void *instance);
    int(GSDLLAPI *set_arg_encoding)(void *instance, int encoding);
    int(GSDLLAPI *init_with_args)(void *instance, int argc, char **argv);
    int(GSDLLAPI *exit)(void *instance);
} gsapi_api_t;

/* Ghostscript and GhostPCL have similarly named APIs but are separate modules.
   Keep their function tables and module lifetimes independent. */
static gsapi_api_t ghostscript_api;
static gsapi_api_t ghostpcl_api;

static dllimp_t ghostscript_imports[] = {
  // clang-format off
    { "gsapi_revision",         &ghostscript_api.revision         },
    { "gsapi_new_instance",     &ghostscript_api.new_instance     },
    { "gsapi_delete_instance",  &ghostscript_api.delete_instance  },
    { "gsapi_set_arg_encoding", &ghostscript_api.set_arg_encoding },
    { "gsapi_init_with_args",   &ghostscript_api.init_with_args   },
    { "gsapi_exit",             &ghostscript_api.exit             },
    { NULL,                      NULL                             }
  // clang-format on
};

static dllimp_t ghostpcl_imports[] = {
  // clang-format off
    { "gsapi_revision",         &ghostpcl_api.revision         },
    { "gsapi_new_instance",     &ghostpcl_api.new_instance     },
    { "gsapi_delete_instance",  &ghostpcl_api.delete_instance  },
    { "gsapi_set_arg_encoding", &ghostpcl_api.set_arg_encoding },
    { "gsapi_init_with_args",   &ghostpcl_api.init_with_args   },
    { "gsapi_exit",             &ghostpcl_api.exit             },
    { NULL,                      NULL                          }
  // clang-format on
};

static void *ghostscript_handle = NULL;
static void *ghostpcl_handle    = NULL;
static unsigned ghostscript_refs = 0;
static unsigned ghostpcl_refs    = 0;

static void
pulse_timer(void *priv)
{
    ps_t *dev = (ps_t *) priv;

    if (dev->ack) {
        dev->ack = 0;
        lpt_irq(dev->lpt, 1);
    }

    timer_disable(&dev->pulse_timer);
}

static int
convert_to_pdf(ps_t *dev)
{
    volatile int code, arg = 0;
    void        *instance = NULL;
    char         input_fn[1024];
    char         output_fn[1024];
    char         resolution_arg[32];
    char         resolution_prolog[192];
    char         width_arg[48];
    char         height_arg[48];
    char        *gsargv[20];
    gsapi_api_t *api = dev->pcl ? &ghostpcl_api : &ghostscript_api;
    double       paper_width = 0.0;
    double       paper_height = 0.0;

    strcpy(input_fn, dev->printer_path);
    path_slash(input_fn);
    strcat(input_fn, dev->filename);

    strcpy(output_fn, input_fn);
    strcpy(output_fn + strlen(output_fn) - (dev->pcl ? 4 : 3), ".pdf");

    gsargv[arg++] = "";
    gsargv[arg++] = "-dNOPAUSE";
    gsargv[arg++] = "-dBATCH";
    gsargv[arg++] = "-dSAFER";
    gsargv[arg++] = "-sDEVICE=pdfwrite";

    /* An automatic setting leaves the incoming job's page size/resolution
       alone. Explicit paper settings model the physical media available in
       the generic printer and therefore constrain the interpreter's media. */
    switch (dev->paper_size) {
        case 1: paper_width = LETTER_PAGE_WIDTH;  paper_height = LETTER_PAGE_HEIGHT;  break;
        case 2: paper_width = LEGAL_PAGE_WIDTH;   paper_height = LEGAL_PAGE_HEIGHT;   break;
        case 3: paper_width = LEDGER_PAGE_WIDTH;  paper_height = LEDGER_PAGE_HEIGHT;  break;
        case 4: paper_width = A0_PAGE_WIDTH;      paper_height = A0_PAGE_HEIGHT;      break;
        case 5: paper_width = A1_PAGE_WIDTH;      paper_height = A1_PAGE_HEIGHT;      break;
        case 6: paper_width = A2_PAGE_WIDTH;      paper_height = A2_PAGE_HEIGHT;      break;
        case 7: paper_width = A3_PAGE_WIDTH;      paper_height = A3_PAGE_HEIGHT;      break;
        case 8: paper_width = A4_PAGE_WIDTH;      paper_height = A4_PAGE_HEIGHT;      break;
        case 9:  paper_width = B4_PAGE_WIDTH;  paper_height = B4_PAGE_HEIGHT;  break;
        case 10: paper_width = A5_PAGE_WIDTH;  paper_height = A5_PAGE_HEIGHT;  break;
        case 11: paper_width = A6_PAGE_WIDTH;  paper_height = A6_PAGE_HEIGHT;  break;
        case 12: paper_width = A7_PAGE_WIDTH;  paper_height = A7_PAGE_HEIGHT;  break;
        case 13: paper_width = A8_PAGE_WIDTH;  paper_height = A8_PAGE_HEIGHT;  break;
        case 14: paper_width = A9_PAGE_WIDTH;  paper_height = A9_PAGE_HEIGHT;  break;
        case 15: paper_width = A10_PAGE_WIDTH; paper_height = A10_PAGE_HEIGHT; break;
        case 16: paper_width = B0_PAGE_WIDTH;  paper_height = B0_PAGE_HEIGHT;  break;
        case 17: paper_width = B1_PAGE_WIDTH;  paper_height = B1_PAGE_HEIGHT;  break;
        case 18: paper_width = B2_PAGE_WIDTH;  paper_height = B2_PAGE_HEIGHT;  break;
        case 19: paper_width = B3_PAGE_WIDTH;  paper_height = B3_PAGE_HEIGHT;  break;
        case 20: paper_width = B5_PAGE_WIDTH;  paper_height = B5_PAGE_HEIGHT;  break;
        case 21: paper_width = B6_PAGE_WIDTH;  paper_height = B6_PAGE_HEIGHT;  break;
        case 22: paper_width = B7_PAGE_WIDTH;  paper_height = B7_PAGE_HEIGHT;  break;
        case 23: paper_width = B8_PAGE_WIDTH;  paper_height = B8_PAGE_HEIGHT;  break;
        case 24: paper_width = B9_PAGE_WIDTH;  paper_height = B9_PAGE_HEIGHT;  break;
        case 25: paper_width = B10_PAGE_WIDTH; paper_height = B10_PAGE_HEIGHT; break;
        case 26: paper_width = EXECUTIVE_PAGE_WIDTH; paper_height = EXECUTIVE_PAGE_HEIGHT; break;
        case 27: paper_width = STATEMENT_PAGE_WIDTH; paper_height = STATEMENT_PAGE_HEIGHT; break;
        case 28: paper_width = FOLIO_PAGE_WIDTH; paper_height = FOLIO_PAGE_HEIGHT; break;
        case 29: paper_width = TEN_BY_FOURTEEN_PAGE_WIDTH; paper_height = TEN_BY_FOURTEEN_PAGE_HEIGHT; break;
        case 30: paper_width = ANSI_C_PAGE_WIDTH; paper_height = ANSI_C_PAGE_HEIGHT; break;
        case 31: paper_width = ANSI_D_PAGE_WIDTH; paper_height = ANSI_D_PAGE_HEIGHT; break;
        case 32: paper_width = ANSI_E_PAGE_WIDTH; paper_height = ANSI_E_PAGE_HEIGHT; break;
        case 33: paper_width = ENVELOPE_DL_PAGE_WIDTH; paper_height = ENVELOPE_DL_PAGE_HEIGHT; break;
        case 34: paper_width = ENVELOPE_C5_PAGE_WIDTH; paper_height = ENVELOPE_C5_PAGE_HEIGHT; break;
        case 35: paper_width = ENVELOPE_NO10_PAGE_WIDTH; paper_height = ENVELOPE_NO10_PAGE_HEIGHT; break;
        case 36: paper_width = ENVELOPE_MONARCH_PAGE_WIDTH; paper_height = ENVELOPE_MONARCH_PAGE_HEIGHT; break;
        default: break;
    }

    if (dev->resolution > 0) {
        snprintf(resolution_arg, sizeof(resolution_arg), "-r%d", dev->resolution);
        gsargv[arg++] = resolution_arg;
    }

    if (!dev->pcl) {
        /* Ghostscript does not provide statusdict/setresolution by default,
           while this Windows driver conditionally calls it. Supply a generic
           implementation: preserve the job's requested DPI in automatic mode,
           or force the configured printer DPI when the user selected one. */
        if (dev->resolution > 0) {
            snprintf(resolution_prolog, sizeof(resolution_prolog),
                     "statusdict begin /setresolution { pop 1 dict dup /HWResolution [%d %d] put setpagedevice } bind def end",
                     dev->resolution, dev->resolution);
        } else {
            snprintf(resolution_prolog, sizeof(resolution_prolog),
                     "statusdict begin /setresolution { /requested_dpi exch def 1 dict dup /HWResolution [ requested_dpi requested_dpi ] put setpagedevice } bind def end");
        }
    }

    if ((paper_width > 0.0) && (paper_height > 0.0)) {
        snprintf(width_arg, sizeof(width_arg), "-dDEVICEWIDTHPOINTS=%.4f", paper_width * 72.0);
        snprintf(height_arg, sizeof(height_arg), "-dDEVICEHEIGHTPOINTS=%.4f", paper_height * 72.0);
        gsargv[arg++] = width_arg;
        gsargv[arg++] = height_arg;
        gsargv[arg++] = "-dFIXEDMEDIA";
    }

    if (dev->pcl) {
        if (dev->lang == LANG_PCL_6)
            gsargv[arg++] = "-LPCLXL";
        else {
            gsargv[arg++] = "-LPCL";
            switch (dev->lang) {
                default:
                case LANG_PCL_5E:
                    gsargv[arg++] = "-lPCL5E";
                    break;
                case LANG_PCL_5C:
                    gsargv[arg++] = "-lPCL5C";
                    break;
                case LANG_HP_RTL:
                    gsargv[arg++] = "-lRTL";
                    break;
            }
        }
    }
    gsargv[arg++] = "-q";
    gsargv[arg++] = "-o";
    gsargv[arg++] = output_fn;
    if (!dev->pcl) {
        /* -c executes PostScript immediately, so keep it after interpreter
           and device options and explicitly mark the following input file. */
        gsargv[arg++] = "-c";
        gsargv[arg++] = resolution_prolog;
        gsargv[arg++] = "-f";
    }
    gsargv[arg++] = input_fn;

    code = api->new_instance(&instance, dev);
    if (code < 0)
        return code;

    code = api->set_arg_encoding(instance, GS_ARG_ENCODING_UTF8);

    if (code == 0)
        code = api->init_with_args(instance, arg, gsargv);

    if (code == 0 || code == gs_error_Quit)
        code = api->exit(instance);
    else
        api->exit(instance);

    api->delete_instance(instance);

    if (code == 0)
        plat_remove(input_fn);
    else
        plat_remove(output_fn);

    return code;
}

static bool
interpreter_available(const ps_t *dev)
{
    return dev->pcl ? (ghostpcl_handle != NULL) : (ghostscript_handle != NULL);
}

static bool
load_interpreter(ps_t *dev)
{
    static const char *ghostscript_libs[] = {
#ifdef _WIN32
        "gsdll64.dll", "libgs-10.dll", "libgs-9.dll"
#elif defined(__APPLE__)
        "libgs.dylib"
#else
        "libgs.so.10", "libgs.so.9", "libgs.so"
#endif
    };
    static const char *ghostpcl_libs[] = {
#ifdef _WIN32
        "gpcl6dll64.dll", "libgpcl6-10.dll", "libgpcl6-9.dll"
#elif defined(__APPLE__)
        "libgpcl6.dylib", "libgpcl6.9.dylib", "libgpcl6.9.54.dylib"
#else
        "libgpcl6.so.10", "libgpcl6.so.9", "libgpcl6.so"
#endif
    };
    const char *const *libs = dev->pcl ? ghostpcl_libs : ghostscript_libs;
    size_t             lib_count = dev->pcl ? (sizeof(ghostpcl_libs) / sizeof(ghostpcl_libs[0]))
                                             : (sizeof(ghostscript_libs) / sizeof(ghostscript_libs[0]));
    dllimp_t          *imports = dev->pcl ? ghostpcl_imports : ghostscript_imports;
    gsapi_api_t       *api = dev->pcl ? &ghostpcl_api : &ghostscript_api;
    void             **handle = dev->pcl ? &ghostpcl_handle : &ghostscript_handle;
    unsigned          *refs = dev->pcl ? &ghostpcl_refs : &ghostscript_refs;
    gsapi_revision_t   rev;

    if (*handle == NULL) {
        for (size_t i = 0; i < lib_count; i++) {
            *handle = dynld_module(libs[i], imports);
            if (*handle != NULL)
                break;
        }
    }

    if (*handle == NULL) {
        ui_msgbox(MBX_WARNING, plat_get_string(dev->pcl ? STRING_GHOSTPCL_ERROR : STRING_GHOSTSCRIPT_ERROR));
        return false;
    }

    if ((api->revision == NULL) || (api->revision(&rev, sizeof(rev)) != 0)) {
        if (*refs == 0) {
            dynld_close(*handle);
            *handle = NULL;
            memset(api, 0, sizeof(*api));
        }
        return false;
    }

    (*refs)++;
    dev->interpreter_ref = true;
    pclog("Loaded %s, rev %ld (%ld)\n", rev.product, rev.revision, rev.revisiondate);
    return true;
}

static void
release_interpreter(ps_t *dev)
{
    void     **handle;
    unsigned  *refs;
    gsapi_api_t *api;

    if (!dev->interpreter_ref)
        return;

    if (dev->pcl) {
        handle = &ghostpcl_handle;
        refs   = &ghostpcl_refs;
        api    = &ghostpcl_api;
    } else {
        handle = &ghostscript_handle;
        refs   = &ghostscript_refs;
        api    = &ghostscript_api;
    }

    if (*refs > 0)
        (*refs)--;

    if ((*refs == 0) && (*handle != NULL)) {
        dynld_close(*handle);
        *handle = NULL;
        memset(api, 0, sizeof(*api));
    }

    dev->interpreter_ref = false;
}

static void
reset_ps(ps_t *dev)
{
    dev->ack = false;

    if (dev->pending) {
        if ((dev->lang != LANG_RAW) && interpreter_available(dev))
            convert_to_pdf(dev);

        dev->filename[0] = 0;

        dev->pending     = false;
    }

    dev->buffer[0]  = 0;
    dev->buffer_pos = 0;
    dev->pjl        = false;
    dev->pjl_mismatch = false;
    dev->uel_pos = 0;
    dev->pjl_line_pos = 0;
    dev->pjl_line_overflow = false;
    dev->pjl_line[0] = 0;

    timer_disable(&dev->pulse_timer);
    timer_stop(&dev->timeout_timer);
}

static void
write_buffer(ps_t *dev, bool finish)
{
    char  path[1024];
    FILE *fp;

    if (dev->buffer_pos == 0)
        return;

    if (dev->filename[0] == 0) {
        if (dev->lang == LANG_RAW)
            plat_tempfile(dev->filename, NULL, ".raw");
        else if (dev->pcl)
            plat_tempfile(dev->filename, NULL, (dev->lang == LANG_PCL_6) ? ".pxl" : ".pcl");
        else
            plat_tempfile(dev->filename, NULL, ".ps");
    }

    strcpy(path, dev->printer_path);
    path_slash(path);
    strcat(path, dev->filename);

    /* PostScript streams can contain binary data and must not undergo text-mode
       newline translation. buffer_pos, not a C-string terminator, is authoritative. */
    fp = plat_fopen(path, "ab");
    if (fp == NULL)
        return;

    if (fwrite(dev->buffer, 1, dev->buffer_pos, fp) != dev->buffer_pos) {
        fclose(fp);
        return;
    }

    fclose(fp);

    dev->buffer[0]  = 0;
    dev->buffer_pos = 0;

    if (finish) {
        if ((dev->lang != LANG_RAW) && interpreter_available(dev))
            convert_to_pdf(dev);

        dev->filename[0] = 0;

        dev->pending     = false;
    } else
        dev->pending     = true;
}

static void
timeout_timer(void *priv)
{
    ps_t *dev = (ps_t *) priv;

    if (dev->buffer_pos != 0)
        write_buffer(dev, true);
    else if (dev->pending) {
        if ((dev->lang != LANG_RAW) && interpreter_available(dev))
            convert_to_pdf(dev);

        dev->filename[0] = 0;

        dev->pending     = false;
    }

    timer_stop(&dev->timeout_timer);
}

static void
ps_write_data(uint8_t val, void *priv)
{
    ps_t *dev = (ps_t *) priv;

    if (dev == NULL)
        return;

    dev->data = val;
}

static void
append_buffer(ps_t *dev, uint8_t val)
{
    if (dev->buffer_pos >= POSTSCRIPT_BUFFER_LENGTH - 1) {
        write_buffer(dev, false);
        if (dev->buffer_pos >= POSTSCRIPT_BUFFER_LENGTH - 1)
            return;
    }

    dev->buffer[dev->buffer_pos++] = val;
    dev->buffer[dev->buffer_pos]   = 0;
}

static int
ascii_upper(int c)
{
    return (c >= 'a' && c <= 'z') ? c - ('a' - 'A') : c;
}

static bool
pjl_equal(const char *a, const char *b, size_t n)
{
    while (n-- > 0) {
        if ((*a == 0) || (*b == 0) ||
            (ascii_upper((unsigned char) *a++) != ascii_upper((unsigned char) *b++)))
            return false;
    }

    return true;
}

static const char *
pjl_skip_space(const char *p)
{
    while ((*p == ' ') || (*p == '\t'))
        p++;

    return p;
}

static bool
pjl_enter_language(const char *line, const char *language)
{
    const char *p = pjl_skip_space(line);
    size_t      n = strlen(language);

    if (!pjl_equal(p, "@PJL", 4))
        return false;
    p = pjl_skip_space(p + 4);

    if (!pjl_equal(p, "ENTER", 5) || ((p[5] != ' ') && (p[5] != '\t')))
        return false;
    p = pjl_skip_space(p + 5);

    if (!pjl_equal(p, "LANGUAGE", 8))
        return false;
    p = pjl_skip_space(p + 8);

    if (*p != '=')
        return false;
    p++;
    p = pjl_skip_space(p);

    if (!pjl_equal(p, language, n))
        return false;

    p += n;
    return (*p == 0) || (*p == ' ') || (*p == '\t');
}

static bool
pjl_is_command(const char *line)
{
    const char *p = pjl_skip_space(line);

    return pjl_equal(p, "@PJL", 4) &&
           ((p[4] == 0) || (p[4] == ' ') || (p[4] == '\t'));
}

static void
process_pjl_data(ps_t *dev, uint8_t val)
{
    static const uint8_t uel[] = { 0x1b, '%', '-', '1', '2', '3', '4', '5', 'X' };

    /* UEL can also terminate a PJL job after @PJL EOJ and its newline. */
    if (dev->uel_pos != 0) {
        if (val == uel[dev->uel_pos]) {
            dev->uel_candidate[dev->uel_pos++] = val;
            if (dev->uel_pos == sizeof(uel)) {
                dev->uel_pos = 0;
                dev->pjl_line_pos = 0;
                dev->pjl_line_overflow = false;
                dev->pjl_line[0] = 0;
            }
            return;
        }

        for (size_t i = 0; i < dev->uel_pos; i++) {
            if (dev->pjl_line_pos < sizeof(dev->pjl_line) - 1)
                dev->pjl_line[dev->pjl_line_pos++] = (char) dev->uel_candidate[i];
            else
                dev->pjl_line_overflow = true;
        }
        dev->pjl_line[dev->pjl_line_pos] = 0;
        dev->uel_pos = 0;
    }

    if (val == uel[0]) {
        dev->uel_candidate[0] = val;
        dev->uel_pos = 1;
        return;
    }

    if (val != '\n') {
        if (dev->pjl_line_pos < sizeof(dev->pjl_line) - 1) {
            dev->pjl_line[dev->pjl_line_pos++] = (char) val;
            dev->pjl_line[dev->pjl_line_pos] = 0;

            /* A second UEL can follow @PJL EOJ while already in PJL mode. */
            if ((dev->pjl_line_pos >= sizeof(uel) - 1) &&
                !memcmp(&dev->pjl_line[dev->pjl_line_pos - (sizeof(uel) - 1)],
                        uel, sizeof(uel) - 1)) {
                dev->pjl_line_pos -= sizeof(uel) - 1;
                dev->pjl_line[dev->pjl_line_pos] = 0;
            }
        } else {
            /* Ignore overlong PJL lines instead of allowing the print buffer to overflow. */
            dev->pjl_line_overflow = true;
        }
        return;
    }

    dev->pjl_line[dev->pjl_line_pos] = 0;
    bool had_cr = (dev->pjl_line_pos > 0) && (dev->pjl_line[dev->pjl_line_pos - 1] == '\r');
    if (had_cr)
        dev->pjl_line[--dev->pjl_line_pos] = 0;

    if (!dev->pjl_line_overflow && pjl_is_command(dev->pjl_line)) {
        bool enter_ps  = pjl_enter_language(dev->pjl_line, "POSTSCRIPT");
        bool enter_pcl = pjl_enter_language(dev->pjl_line, "PCL");

        /* PJL is transport/job control, not PostScript or PCL input. */
        if (enter_ps || enter_pcl) {
            if ((enter_ps && !dev->pcl) || (enter_pcl && dev->pcl)) {
                dev->pjl_mismatch = false;
                dev->pjl = false;
            } else {
                /* Do not feed a mismatched language to the wrong interpreter. */
                dev->pjl_mismatch = true;
            }
        }
    } else if (!dev->pjl_line_overflow && (dev->pjl_line_pos != 0) && !dev->pjl_mismatch) {
        /* A non-PJL line means language data has started without an ENTER command. */
        dev->pjl = false;
        for (size_t i = 0; i < dev->pjl_line_pos; i++)
            append_buffer(dev, (uint8_t) dev->pjl_line[i]);
        if (had_cr)
            append_buffer(dev, '\r');
        append_buffer(dev, '\n');
    }

    dev->pjl_line_pos      = 0;
    dev->pjl_line_overflow = false;
    dev->pjl_line[0]       = 0;
}

static int
process_escape(ps_t *dev, int do_pjl)
{
    static const uint8_t uel[] = { 0x1b, '%', '-', '1', '2', '3', '4', '5', 'X' };

    /* Return 1 when consumed as a complete UEL, 2 while holding a possible
       UEL prefix, and 0 when the caller should process the current byte. */
    if (dev->uel_pos == 0) {
        if (dev->data != uel[0])
            return 0;

        dev->uel_candidate[0] = dev->data;
        dev->uel_pos = 1;
        return 2;
    }

    if (dev->data == uel[dev->uel_pos]) {
        dev->uel_candidate[dev->uel_pos++] = dev->data;
        if (dev->uel_pos < sizeof(uel))
            return 2;

        dev->uel_pos = 0;
        if (do_pjl) {
            dev->pjl_line_pos = 0;
            dev->pjl_line_overflow = false;
            dev->pjl_line[0] = 0;
            dev->pjl = true;
            dev->pending = false;
            return 1;
        }

        for (size_t i = 0; i < sizeof(uel); i++)
            append_buffer(dev, uel[i]);
        return 1;
    }

    /* This was an ordinary ESC sequence. Restore every held byte before
       processing the byte that disproved the UEL candidate. */
    for (size_t i = 0; i < dev->uel_pos; i++)
        append_buffer(dev, dev->uel_candidate[i]);
    dev->uel_pos = 0;

    if (dev->data == uel[0]) {
        dev->uel_candidate[0] = dev->data;
        dev->uel_pos = 1;
        return 2;
    }

    return 0;
}

static void
process_data(ps_t *dev)
{
    /* Raw mode is an exact byte capture: do not parse or strip PJL/UEL. */
    if (dev->lang == LANG_RAW) {
        append_buffer(dev, dev->data);
        return;
    }

    /* PJL is recognized automatically for either generic printer backend. */
    if (dev->pjl) {
        process_pjl_data(dev, dev->data);
        return;
    }

    /* The Universal Exit Language sequence is recognized in both PS and PCL. */
    if ((dev->data == 0x1b) || (dev->uel_pos > 0)) {
        if (process_escape(dev, 1) != 0)
            return;
    }

    if (!dev->pcl && ((dev->data < 0x20) || (dev->data == 0x7f))) {
        /* On PostScript, ignore unsupported controls but preserve legal whitespace. */
        switch (dev->data) {
            case '\t':
            case '\n':
            case '\f':
            case '\r':
                break;

            case '\0':
                dev->data = ' ';
                break;

            /* Ctrl+D is a common PostScript job terminator. */
            case '\4':
                write_buffer(dev, true);
                return;

            default:
                return;
        }
    }

    append_buffer(dev, dev->data);
}

static void
ps_strobe(uint8_t old, uint8_t val, void *priv)
{
    ps_t *dev = (ps_t *) priv;

    if (dev == NULL)
        return;

    if (!(val & 0x01) && (old & 0x01)) {
        process_data(dev);

        if (timer_is_on(&dev->timeout_timer)) {
            timer_stop(&dev->timeout_timer);
#ifdef USE_DYNAREC
            if (cpu_use_dynarec)
                update_tsc();
#endif
        }

        dev->ack = true;

        timer_set_delay_u64(&dev->pulse_timer, ISACONST);
        timer_on_auto(&dev->timeout_timer, 5000000.0);
    }
}

static void
ps_write_ctrl(uint8_t val, void *priv)
{
    ps_t *dev = (ps_t *) priv;

    if (dev == NULL)
        return;

    dev->autofeed = val & 0x02 ? true : false;

    if (val & 0x08)
        dev->select = true;

    if ((val & 0x04) && !(dev->ctrl & 0x04)) {
        /* Reset printer */
        dev->select = false;

        reset_ps(dev);
    }

    if (!(val & 0x01) && (dev->ctrl & 0x01)) {
        process_data(dev);

        if (timer_is_on(&dev->timeout_timer)) {
            timer_stop(&dev->timeout_timer);
#ifdef USE_DYNAREC
            if (cpu_use_dynarec)
                update_tsc();
#endif
        }

        dev->ack = true;

        timer_set_delay_u64(&dev->pulse_timer, ISACONST);
        timer_on_auto(&dev->timeout_timer, 5000000.0);
    }

    dev->ctrl = val;
}

static uint8_t
ps_read_status(void *priv)
{
    const ps_t   *dev = (ps_t *) priv;
    uint8_t       ret = 0x9f;

    if (!dev->ack)
        ret |= 0x40;

    return ret;
}

static void *
ps_init(const device_t *info)
{
    ps_t *dev = (ps_t *) calloc(1, sizeof(ps_t));

    if (dev == NULL)
        return NULL;

    dev->ctrl = 0x04;
    dev->pcl  = false;

    dev->lpt  = lpt_attach(ps_write_data, ps_write_ctrl, ps_strobe, ps_read_status, NULL, NULL, NULL, dev);
    dev->lang       = device_get_config_int("language");
    dev->paper_size = device_get_config_int("paper_size");
    dev->resolution = device_get_config_int("resolution");

    if (dev->lang != LANG_RAW)
        load_interpreter(dev);

    /* Cache print folder path. */
    memset(dev->printer_path, 0x00, sizeof(dev->printer_path));
    path_append_filename(dev->printer_path, usr_path, "printer");
    if (!plat_dir_check(dev->printer_path))
        plat_dir_create(dev->printer_path);
    path_slash(dev->printer_path);

    timer_add(&dev->pulse_timer, pulse_timer, dev, 0);
    timer_add(&dev->timeout_timer, timeout_timer, dev, 0);

    reset_ps(dev);

    return dev;
}

static void *
pcl_init(const device_t *info)
{
    ps_t *dev = (ps_t *) calloc(1, sizeof(ps_t));

    if (dev == NULL)
        return NULL;

    dev->ctrl = 0x04;
    dev->pcl  = true;

    dev->lpt  = lpt_attach(ps_write_data, ps_write_ctrl, ps_strobe, ps_read_status, NULL, NULL, NULL, dev);
    dev->lang       = device_get_config_int("language");
    dev->paper_size = device_get_config_int("paper_size");
    dev->resolution = device_get_config_int("resolution");

    if (dev->lang != LANG_RAW)
        load_interpreter(dev);

    /* Cache print folder path. */
    memset(dev->printer_path, 0x00, sizeof(dev->printer_path));
    path_append_filename(dev->printer_path, usr_path, "printer");
    if (!plat_dir_check(dev->printer_path))
        plat_dir_create(dev->printer_path);
    path_slash(dev->printer_path);

    timer_add(&dev->pulse_timer, pulse_timer, dev, 0);
    timer_add(&dev->timeout_timer, timeout_timer, dev, 0);

    reset_ps(dev);

    return dev;
}

static void
ps_close(void *priv)
{
    ps_t *dev = (ps_t *) priv;

    if (dev == NULL)
        return;

    if (dev->buffer[0] != 0)
        write_buffer(dev, true);

    release_interpreter(dev);

    timer_disable(&dev->pulse_timer);
    timer_disable(&dev->timeout_timer);

    free(dev);
}

// clang-format off
static const device_config_t lpt_prt_ps_config[] = {
    {
        .name           = "language",
        .description    = "Language",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = LANG_PS,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Raw",              .value = LANG_RAW },
            { .description = "PDF (PostScript)", .value = LANG_PS  },
            { .description = ""                                    }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "paper_size",
        .description    = "Paper Size",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Use job setting", .value = 0 },
            { .description = "Letter",          .value = 1 },
            { .description = "Legal",           .value = 2 },
            { .description = "Ledger / Tabloid",.value = 3 },
            { .description = "A0",              .value = 4 },
            { .description = "A1",              .value = 5 },
            { .description = "A2",              .value = 6 },
            { .description = "A3",              .value = 7 },
            { .description = "A4",              .value = 8 },
            { .description = "B4",              .value = 9 },
            { .description = "A5",              .value = 10 },
            { .description = "A6",              .value = 11 },
            { .description = "A7",              .value = 12 },
            { .description = "A8",              .value = 13 },
            { .description = "A9",              .value = 14 },
            { .description = "A10",             .value = 15 },
            { .description = "B0",              .value = 16 },
            { .description = "B1",              .value = 17 },
            { .description = "B2",              .value = 18 },
            { .description = "B3",              .value = 19 },
            { .description = "B5",              .value = 20 },
            { .description = "B6",              .value = 21 },
            { .description = "B7",              .value = 22 },
            { .description = "B8",              .value = 23 },
            { .description = "B9",              .value = 24 },
            { .description = "B10",             .value = 25 },
            { .description = "Executive",        .value = 26 },
            { .description = "Statement",        .value = 27 },
            { .description = "Folio",            .value = 28 },
            { .description = "10 x 14",          .value = 29 },
            { .description = "ANSI C",           .value = 30 },
            { .description = "ANSI D",           .value = 31 },
            { .description = "ANSI E",           .value = 32 },
            { .description = "Envelope DL",      .value = 33 },
            { .description = "Envelope C5",      .value = 34 },
            { .description = "Envelope #10",     .value = 35 },
            { .description = "Envelope Monarch", .value = 36 },
            { .description = "" }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "resolution",
        .description    = "Resolution (DPI)",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Use job setting", .value = 0 },
            { .description = "150 DPI",         .value = 150 },
            { .description = "300 DPI",         .value = 300 },
            { .description = "600 DPI",         .value = 600 },
            { .description = "1200 DPI",        .value = 1200 },
            { .description = "" }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
};
// clang-format on

// clang-format off
static const device_config_t lpt_prt_pcl_config[] = {
    {
        .name           = "language",
        .description    = "Language",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = LANG_PCL_5E,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Raw",          .value = LANG_RAW    },
            { .description = "PDF (PCL 5e)", .value = LANG_PCL_5E },
            { .description = "PDF (PCL 5c)", .value = LANG_PCL_5C },
            { .description = "PDF (HP-RTL)", .value = LANG_HP_RTL },
            { .description = "PDF (PCL 6)",  .value = LANG_PCL_6  },
            { .description = ""                                   }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "paper_size",
        .description    = "Paper Size",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Use job setting", .value = 0 },
            { .description = "Letter",          .value = 1 },
            { .description = "Legal",           .value = 2 },
            { .description = "Ledger / Tabloid",.value = 3 },
            { .description = "A0",              .value = 4 },
            { .description = "A1",              .value = 5 },
            { .description = "A2",              .value = 6 },
            { .description = "A3",              .value = 7 },
            { .description = "A4",              .value = 8 },
            { .description = "B4",              .value = 9 },
            { .description = "A5",              .value = 10 },
            { .description = "A6",              .value = 11 },
            { .description = "A7",              .value = 12 },
            { .description = "A8",              .value = 13 },
            { .description = "A9",              .value = 14 },
            { .description = "A10",             .value = 15 },
            { .description = "B0",              .value = 16 },
            { .description = "B1",              .value = 17 },
            { .description = "B2",              .value = 18 },
            { .description = "B3",              .value = 19 },
            { .description = "B5",              .value = 20 },
            { .description = "B6",              .value = 21 },
            { .description = "B7",              .value = 22 },
            { .description = "B8",              .value = 23 },
            { .description = "B9",              .value = 24 },
            { .description = "B10",             .value = 25 },
            { .description = "Executive",        .value = 26 },
            { .description = "Statement",        .value = 27 },
            { .description = "Folio",            .value = 28 },
            { .description = "10 x 14",          .value = 29 },
            { .description = "ANSI C",           .value = 30 },
            { .description = "ANSI D",           .value = 31 },
            { .description = "ANSI E",           .value = 32 },
            { .description = "Envelope DL",      .value = 33 },
            { .description = "Envelope C5",      .value = 34 },
            { .description = "Envelope #10",     .value = 35 },
            { .description = "Envelope Monarch", .value = 36 },
            { .description = "" }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "resolution",
        .description    = "Resolution (DPI)",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Use job setting", .value = 0 },
            { .description = "150 DPI",         .value = 150 },
            { .description = "300 DPI",         .value = 300 },
            { .description = "600 DPI",         .value = 600 },
            { .description = "1200 DPI",        .value = 1200 },
            { .description = "" }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
};
// clang-format on

const device_t lpt_prt_ps_device = {
    .name          = "Generic PostScript Printer",
    .internal_name = "postscript",
    .flags         = DEVICE_LPT | DEVICE_HOTPLUG,
    .local         = 0,
    .init          = ps_init,
    .close         = ps_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = lpt_prt_ps_config
};

const device_t lpt_prt_pcl_device = {
    .name          = "Generic PCL Printer",
    .internal_name = "pcl",
    .flags         = DEVICE_LPT | DEVICE_HOTPLUG,
    .local         = 0,
    .init          = pcl_init,
    .close         = ps_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = lpt_prt_pcl_config
};
