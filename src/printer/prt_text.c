/*
 * VARCem   Virtual ARchaeological Computer EMulator.
 *          An emulator of (mostly) x86-based PC systems and devices,
 *          using the ISA,EISA,VLB,MCA  and PCI system buses, roughly
 *          spanning the era between 1981 and 1995.
 *
 *          Implementation of a generic text printer.
 *
 *          Simple old text printers were unable to do any formatting
 *          of the text. They were just sheets of paper with a fixed
 *          size (in the U.S., this would be Letter, 8.5"x11") with a
 *          set of fixed margins to allow for proper operation of the
 *          printer mechanics. This would lead to a page being 66 lines
 *          of 80 characters each.
 *
 * Authors: Fred N. van Kempen, <decwiz@yahoo.com>
 *
 *          Copyright 2018-2019 Fred N. van Kempen.
 *
 *          Redistribution and  use  in source  and binary forms, with
 *          or  without modification, are permitted  provided that the
 *          following conditions are met:
 *
 *          1. Redistributions of  source  code must retain the entire
 *             above notice, this list of conditions and the following
 *             disclaimer.
 *
 *          2. Redistributions in binary form must reproduce the above
 *             copyright  notice,  this list  of  conditions  and  the
 *             following disclaimer in  the documentation and/or other
 *             materials provided with the distribution.
 *
 *          3. Neither the  name of the copyright holder nor the names
 *             of  its  contributors may be used to endorse or promote
 *             products  derived from  this  software without specific
 *             prior written permission.
 *
 * THIS SOFTWARE  IS  PROVIDED BY THE  COPYRIGHT  HOLDERS AND CONTRIBUTORS
 * "AS IS" AND  ANY EXPRESS  OR  IMPLIED  WARRANTIES,  INCLUDING, BUT  NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE  ARE  DISCLAIMED. IN  NO  EVENT  SHALL THE COPYRIGHT
 * HOLDER OR  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL,  EXEMPLARY,  OR  CONSEQUENTIAL  DAMAGES  (INCLUDING,  BUT  NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES;  LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED  AND ON  ANY
 * THEORY OF  LIABILITY, WHETHER IN  CONTRACT, STRICT  LIABILITY, OR  TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING  IN ANY  WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <wchar.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/pit.h>
#include <86box/path.h>
#include <86box/plat.h>
#include <86box/lpt.h>
#include <86box/printer.h>
#include <86box/prt_devs.h>
#include "cpu.h"
#include <86box/prt_papersizes.h>

#define FULL_PAGE 1 /* set if no top/bot margins */

/* Default page values (for now.) */
#define PAGE_LMARGIN 0.25 /* 0.25" left and right */
#define PAGE_RMARGIN 0.25
#if FULL_PAGE
#    define PAGE_TMARGIN 0
#    define PAGE_BMARGIN 0
#else
#    define PAGE_TMARGIN 0.25
#    define PAGE_BMARGIN 0.25
#endif
#define PAGE_CPI           10.0  /* standard 10 cpi */
#define PAGE_LPI           6.0   /* standard 6 lpi */
#define PAGE_CONDENSED_CPI 17.14 /* the usual pitch of SI */
#define PAGE_MIN_WIDTH     8.0   /* 80 columns fit on all the paper sizes */

/* Enough for the longest paper at the highest pitch. */
#define MAX_COLUMNS 256
#define MAX_LINES   128

enum {
    PAPER_LETTER = 0,
    PAPER_A4,
    PAPER_LEGAL
};

/* Printer languages this printer does not speak are dropped, not printed. */
enum {
    ESC_NONE = 0,
    ESC_START,        /* ESC was seen */
    ESC_PCL_VALUE,    /* in a parameterized PCL sequence */
    ESC_LENGTH_LOW,   /* ESC ( x nL nH, the length of what follows */
    ESC_LENGTH_HIGH,
    ESC_COUNT_LOW,    /* ESC K nL nH, the number of columns */
    ESC_COUNT_HIGH,
    ESC_IMAGE_MODE,   /* ESC * m nL nH */
    ESC_SKIP
};

typedef struct psurface_t {
    int8_t dirty; /* has the page been printed on? */
    char   pad;

    uint16_t w; /* size //INFO */
    uint16_t h;

    uint8_t *chars; /* character data, 0x00 where nothing was printed */
} psurface_t;

typedef struct prnt_t {
    const char *name;

    void *lpt;

    /* Output file name. */
    char filename[1024];

    /* Printer timeout. */
    pc_timer_t pulse_timer;
    pc_timer_t timeout_timer;

    /* page data */
    int    paper_size;
    double page_width; /* all in inches */
    double page_height;
    double left_margin;
    double top_margin;
    double right_margin;
    double bot_margin;

    /* internal page data */
    psurface_t *page;
    uint16_t    max_chars;
    uint16_t    max_lines;
    uint16_t    curr_x; /* print head position (x, chars) */
    uint16_t    curr_y; /* print head position (y, chars) */
    bool        form_feed;

    /* font data */
    double cpi; /* defined chars per inch */
    double lpi; /* defined lines per inch */

    /* command skipping */
    uint8_t  esc_state;
    uint8_t  esc_bytes_per_column;
    uint32_t esc_skip;
    uint32_t esc_value;
    bool     esc_binary;

    /* handshake data */
    uint8_t data;
    int8_t  ack;
    int8_t  select;
    int8_t  busy;
    int8_t  int_pending;
    int8_t  error;
    int8_t  autofeed;
    uint8_t ctrl;
} prnt_t;

/* Dump the current page into a formatted file. */
static void
dump_page(prnt_t *dev)
{
    char     path[1024];
    FILE    *fp;
    uint16_t lines = 0;

    /* Create the full path for this file. */
    memset(path, 0x00, sizeof(path));
    path_append_filename(path, usr_path, "printer");
    if (!plat_dir_check(path))
        plat_dir_create(path);
    path_slash(path);
    strcat(path, dev->filename);

    /* Create the file. */
    fp = plat_fopen(path, "a");
    if (fp == NULL) {
        // ERRLOG("PRNT: unable to create print page '%s'\n", path);
        return;
    }
    fseek(fp, 0, SEEK_END);

    /* The previous sheet ended with a form feed: start a new one. */
    if (dev->form_feed && (ftell(fp) != 0))
        fputc('\014', fp);
    dev->form_feed = false;

    /* Blank lines at the end of the sheet are not worth keeping. */
    for (uint16_t y = 0; y < dev->page->h; y++) {
        for (uint16_t x = 0; x < dev->page->w; x++) {
            if (dev->page->chars[(y * dev->page->w) + x] != 0x00)
                lines = y + 1;
        }
    }

    for (uint16_t y = 0; y < lines; y++) {
        const uint8_t *line  = &dev->page->chars[y * dev->page->w];
        uint16_t       width = dev->page->w;

        while ((width > 0) && ((line[width - 1] == 0x00) || (line[width - 1] == ' ')))
            width--;

        /* Head movement leaves gaps, which are spaces on paper. */
        for (uint16_t x = 0; x < width; x++)
            fputc(line[x] ? line[x] : ' ', fp);

        fputc('\n', fp);
    }

    /* All done, close the file. */
    fclose(fp);
}

static void
clear_page(prnt_t *dev)
{
    memset(dev->page->chars, 0x00, (size_t) dev->page->h * dev->page->w);
    dev->curr_x      = 0;
    dev->curr_y      = 0;
    dev->page->dirty = 0;
}

static void
new_page(prnt_t *dev)
{
    /* Dump the current page if needed. */
    if (dev->page->dirty) {
        dump_page(dev);
        dev->form_feed = true;
    }

    clear_page(dev);
}

static void
pulse_timer(void *priv)
{
    prnt_t *dev = (prnt_t *) priv;

    if (dev->ack) {
        dev->ack = 0;
        lpt_irq(dev->lpt, 1);
    }

    timer_disable(&dev->pulse_timer);
}

static void
timeout_timer(void *priv)
{
    prnt_t *dev = (prnt_t *) priv;

    /* A pause is not the end of the sheet, so no form feed is written. */
    if (dev->page->dirty) {
        dump_page(dev);
        clear_page(dev);
    }

    timer_stop(&dev->timeout_timer);
}

static void
update_layout(prnt_t *dev)
{
    double width = dev->page_width - dev->left_margin - dev->right_margin;

    if (width < PAGE_MIN_WIDTH)
        width = PAGE_MIN_WIDTH;

    dev->max_chars = (uint16_t) (width * dev->cpi);
    dev->max_lines = (uint16_t) ((dev->page_height - dev->top_margin - dev->bot_margin) * dev->lpi);

    if (dev->max_chars > MAX_COLUMNS)
        dev->max_chars = MAX_COLUMNS;
    if (dev->max_lines > MAX_LINES)
        dev->max_lines = MAX_LINES;

    if (dev->curr_x >= dev->max_chars)
        dev->curr_x = dev->max_chars - 1;
}

static void
reset_printer(prnt_t *dev)
{
    /* Whatever has been printed so far is on paper already. */
    if ((dev->page != NULL) && dev->page->dirty)
        dump_page(dev);

    switch (dev->paper_size) {
        case PAPER_A4:
            dev->page_width  = A4_PAGE_WIDTH;
            dev->page_height = A4_PAGE_HEIGHT;
            break;
        case PAPER_LEGAL:
            dev->page_width  = LEGAL_PAGE_WIDTH;
            dev->page_height = LEGAL_PAGE_HEIGHT;
            break;
        case PAPER_LETTER:
        default:
            dev->page_width  = LETTER_PAGE_WIDTH;
            dev->page_height = LETTER_PAGE_HEIGHT;
            break;
    }

    dev->left_margin  = PAGE_LMARGIN;
    dev->right_margin = PAGE_RMARGIN;
    dev->top_margin   = PAGE_TMARGIN;
    dev->bot_margin   = PAGE_BMARGIN;
    dev->cpi          = PAGE_CPI;
    dev->lpi          = PAGE_LPI;
    dev->ack          = 0;
    dev->esc_state    = ESC_NONE;
    dev->form_feed    = false;

    /* Default page layout. */
    update_layout(dev);

    if (dev->page != NULL)
        clear_page(dev);

    /* Create a file for this page. */
    plat_tempfile(dev->filename, NULL, ".txt");

    timer_disable(&dev->pulse_timer);
    timer_stop(&dev->timeout_timer);
}

static void
line_feed(prnt_t *dev)
{
    dev->curr_x = 0;
    if (++dev->curr_y >= dev->max_lines)
        new_page(dev);
}

/* Printing on top of a character adds to it on paper. A space adds nothing, and
   the character stays readable in a text file when an underline adds to it. */
static void
put_char(prnt_t *dev, uint16_t x, uint8_t ch)
{
    uint8_t *cell = &dev->page->chars[((size_t) dev->curr_y * dev->page->w) + x];

    if ((*cell != 0x00) && (*cell != ' ') && ((ch == ' ') || (ch == '_')))
        return;

    *cell = ch;
}

/* Text is all that this printer knows. Drop the commands of the common printer
   languages rather than print them, including any graphics data they carry. */
static void
skip_escape(prnt_t *dev, uint8_t ch)
{
    switch (dev->esc_state) {
        case ESC_START:
            dev->esc_state = ESC_NONE;

            if ((ch == '&') || (ch == '%') || (ch == ')')) {
                dev->esc_value = 0;
                dev->esc_state = ESC_PCL_VALUE;
            } else if ((ch == '(') || (ch == '*')) {
                /* ESC/P follows with a command or mode, PCL with a group or value. */
                dev->esc_binary = true;
                dev->esc_value  = ch;
                dev->esc_state  = ESC_IMAGE_MODE;
            } else if ((ch != 0x00) && (strchr(" !-/3ACIJNQRSUWaklprstwx+", ch) != NULL)) {
                dev->esc_skip  = 1;
                dev->esc_state = ESC_SKIP;
            } else if ((ch == '$') || (ch == '\\')) {
                dev->esc_skip  = 2;
                dev->esc_state = ESC_SKIP;
            } else if ((ch != 0x00) && (strchr("KLYZ", ch) != NULL)) {
                dev->esc_bytes_per_column = 1;
                dev->esc_state            = ESC_COUNT_LOW;
            }
            break;

        case ESC_IMAGE_MODE:
            if (((ch >= '0') && (ch <= '9')) || (ch == '-') || (ch == '+') ||
                ((ch >= 'a') && (ch <= 'z') && (dev->esc_value == '*')) ||
                ((ch == 's') && (dev->esc_value == '('))) {
                dev->esc_value = 0;
                dev->esc_state = ESC_PCL_VALUE;
                if ((ch >= '0') && (ch <= '9'))
                    dev->esc_value = ch - '0';
            } else if (dev->esc_value == '(')
                dev->esc_state = ESC_LENGTH_LOW;
            else {
                dev->esc_bytes_per_column = (ch < 32) ? 1 : ((ch < 64) ? 3 : 6);
                dev->esc_state            = ESC_COUNT_LOW;
            }
            break;

        case ESC_PCL_VALUE:
            if ((ch >= '0') && (ch <= '9'))
                dev->esc_value = (dev->esc_value * 10) + (ch - '0');
            else if ((ch >= '@') && (ch <= '^')) {
                dev->esc_state = ESC_NONE;
                if ((ch == 'W') || (ch == 'V')) {
                    dev->esc_skip  = dev->esc_value;
                    dev->esc_state = dev->esc_skip ? ESC_SKIP : ESC_NONE;
                }
            } else if ((ch == 'w') || (ch == 'v')) {
                dev->esc_skip  = dev->esc_value;
                dev->esc_value = 0;
                dev->esc_state = dev->esc_skip ? ESC_SKIP : ESC_PCL_VALUE;
            } else if ((ch >= 'a') && (ch <= 'z'))
                dev->esc_value = 0;
            else if ((ch != '-') && (ch != '+') && (ch != '.'))
                dev->esc_state = ESC_NONE;
            break;

        case ESC_LENGTH_LOW:
        case ESC_COUNT_LOW:
            dev->esc_value = ch;
            dev->esc_state++;
            break;

        case ESC_LENGTH_HIGH:
            dev->esc_skip  = dev->esc_value | ((uint32_t) ch << 8);
            dev->esc_state = dev->esc_skip ? ESC_SKIP : ESC_NONE;
            break;

        case ESC_COUNT_HIGH:
            dev->esc_skip  = (dev->esc_value | ((uint32_t) ch << 8)) * dev->esc_bytes_per_column;
            dev->esc_state = dev->esc_skip ? ESC_SKIP : ESC_NONE;
            break;

        case ESC_SKIP:
            if (--dev->esc_skip == 0)
                dev->esc_state = ESC_NONE;
            break;

        default:
            dev->esc_state = ESC_NONE;
            break;
    }
}

static int
process_char(prnt_t *dev, uint8_t ch)
{
    uint16_t i;

    switch (ch) {
        case 0x08: /* Backspace (BS) */
            if (dev->curr_x > 0)
                dev->curr_x--;
            return 1;

        case 0x09: /* Tab horizontally (HT) */
            /* Find tab right to current pos. */
            i = dev->curr_x;
            do {
                put_char(dev, i++, ' ');
            } while ((i < dev->max_chars) && ((i % 8) != 0));
            dev->curr_x = (i < dev->max_chars) ? i : (dev->max_chars - 1);
            return 1;

        case 0x0a: /* Line feed */
        case 0x0b: /* Tab vertically (VT), with no vertical tabs set */
            line_feed(dev);
            return 1;

        case 0x0c: /* Form feed (FF) */
            new_page(dev);
            return 1;

        case 0x0d: /* Carriage Return (CR) */
            dev->curr_x = 0;
            if (dev->autofeed)
                line_feed(dev);
            return 1;

        case 0x0f: /* select condensed printing (SI) */
            dev->cpi = PAGE_CONDENSED_CPI;
            update_layout(dev);
            return 1;

        case 0x12: /* cancel condensed printing (DC2) */
            dev->cpi = PAGE_CPI;
            update_layout(dev);
            return 1;

        case 0x1b: /* ESC */
            dev->esc_state = ESC_START;
            return 1;

        case 0x7f:
            return 1;

        default:
            break;
    }

    /* The other control codes do not print. */
    return ch < 0x20;
}

static void
handle_char(prnt_t *dev)
{
    const uint8_t ch = dev->data;

    if (dev->page == NULL)
        return;

    if (dev->esc_state != ESC_NONE) {
        skip_escape(dev, ch);
        return;
    }

    if (process_char(dev, ch) == 1) {
        /* Command was processed. */
        return;
    }

    /* Store character in the page buffer. */
    put_char(dev, dev->curr_x, ch);

    if (ch != ' ')
        dev->page->dirty = 1;

    /* Update print head position. */
    if (++dev->curr_x >= dev->max_chars)
        line_feed(dev);
}

static void
write_data(uint8_t val, void *priv)
{
    prnt_t *dev = (prnt_t *) priv;

    if (dev == NULL)
        return;

    dev->data = val;
}

static void
strobe(uint8_t old, uint8_t val, void *priv)
{
    prnt_t *dev = (prnt_t *) priv;

    if (dev == NULL)
        return;

    if (!(val & 0x01) && (old & 0x01)) { /* STROBE */
        /* Process incoming character. */
        handle_char(dev);

        if (timer_is_on(&dev->timeout_timer)) {
            timer_stop(&dev->timeout_timer);
#ifdef USE_DYNAREC
            if (cpu_use_dynarec)
                update_tsc();
#endif
        }

        /* ACK it, will be read on next READ STATUS. */
        dev->ack = 1;

        timer_set_delay_u64(&dev->pulse_timer, ISACONST);
        timer_on_auto(&dev->timeout_timer, 5000000.0);
    }
}

static void
write_ctrl(uint8_t val, void *priv)
{
    prnt_t *dev = (prnt_t *) priv;

    if (dev == NULL)
        return;

    /* set autofeed value */
    dev->autofeed = (val & 0x02) ? 1 : 0;

    if (val & 0x08) { /* SELECT */
        /* select printer */
        dev->select = 1;
    }

    if ((val & 0x04) && !(dev->ctrl & 0x04)) {
        /* reset printer */
        dev->select = 0;

        reset_printer(dev);
    }

    if (!(val & 0x01) && (dev->ctrl & 0x01)) { /* STROBE */
        /* Process incoming character. */
        handle_char(dev);

        /* ACK it, will be read on next READ STATUS. */
        dev->ack = 1;

        if (timer_is_on(&dev->timeout_timer)) {
            timer_stop(&dev->timeout_timer);
#ifdef USE_DYNAREC
            if (cpu_use_dynarec)
                update_tsc();
#endif
        }

        timer_set_delay_u64(&dev->pulse_timer, ISACONST);
        timer_on_auto(&dev->timeout_timer, 5000000.0);
    }

    dev->ctrl = val;
}

static uint8_t
read_status(void *priv)
{
    const prnt_t *dev = (prnt_t *) priv;
    uint8_t       ret = 0x1f;

    ret |= 0x80;

    if (!dev->ack)
        ret |= 0x40;

    return ret;
}

static void *
prnt_init(const device_t *info)
{
    /* Initialize a device instance. */
    prnt_t *dev = (prnt_t *) calloc(1, sizeof(prnt_t));

    dev->ctrl = 0x04;
    dev->lpt  = lpt_attach(write_data, write_ctrl, strobe, read_status, NULL, NULL, NULL, dev);

    dev->paper_size = device_get_config_int("paper_size");

    /* Create a page buffer. */
    dev->page        = (psurface_t *) calloc(1, sizeof(psurface_t));
    dev->page->w     = MAX_COLUMNS;
    dev->page->h     = MAX_LINES;
    dev->page->chars = (uint8_t *) calloc(dev->page->w, dev->page->h);

    /* Initialize parameters. */
    reset_printer(dev);

    timer_add(&dev->pulse_timer, pulse_timer, dev, 0);
    timer_add(&dev->timeout_timer, timeout_timer, dev, 0);

    return dev;
}

static void
prnt_close(void *priv)
{
    prnt_t *dev = (prnt_t *) priv;

    if (dev == NULL)
        return;

    if (dev->page) {
        /* print last page if it contains data */
        if (dev->page->dirty)
            dump_page(dev);

        if (dev->page->chars != NULL)
            free(dev->page->chars);
        free(dev->page);
    }

    timer_disable(&dev->pulse_timer);
    timer_disable(&dev->timeout_timer);

    free(dev);
}

// clang-format off
static const device_config_t lpt_prt_text_config[] = {
    {
        .name           = "paper_size",
        .description    = "Paper Size",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = PAPER_LETTER,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Letter", .value = PAPER_LETTER },
            { .description = "A4",     .value = PAPER_A4     },
            { .description = "Legal",  .value = PAPER_LEGAL  },
            { .description = ""                              }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
};
// clang-format on

const device_t lpt_prt_text_device = {
    .name          = "Generic Text Printer",
    .internal_name = "text_prt",
    .flags         = DEVICE_LPT | DEVICE_HOTPLUG,
    .local         = 0,
    .init          = prnt_init,
    .close         = prnt_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = lpt_prt_text_config
};
