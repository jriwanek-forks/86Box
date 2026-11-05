/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of a PCL 3 page engine for the HP DeskJet 500
 *          series of inkjet printers.
 *
 *          These printers speak an inkjet dialect of PCL which general
 *          PCL 5 interpreters do not cover: KCMY palettes, Configure
 *          Raster Data, and raster compression mode 9. Pages are rendered
 *          here as one ink bit per color, and are saved as PNG images.
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2026 Jasmine Iwanek.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ft2build.h>
#include FT_FREETYPE_H
#include <86box/86box.h>
#include <86box/mem.h>
#include <86box/path.h>
#include <86box/plat.h>
#include <86box/rom.h>
#include <86box/video.h>
#include <86box/plat_fallthrough.h>
#include <86box/png_struct.h>
#include <86box/printer.h>
#include <86box/prt_papersizes.h>

/* The ink bits of a page pixel. */
#define INK_C 0x01
#define INK_M 0x02
#define INK_Y 0x04
#define INK_K 0x08

#define MAX_PLANES   8
#define MAX_ROW      0x8000    /* bytes in one raster plane */
#define MAX_DATA     0x100000  /* longest binary payload kept in memory */
#define MAX_SOFTFONT 16
#define MAX_CART_FONT 64
#define MAX_RAM_PAGE  8         /* two RAM cartridges of 256 KB */
#define RAM_PAGE      0x10000   /* a font pass cannot cross one */
#define RAM_BLOCK     0x1000    /* fonts are given memory in these */
#define MAX_CART_BANK 32
#define CART_BANK    0x10000   /* each 64 KB of a cartridge has its own header */
#define CART_BASELINE 42       /* the descriptor does not give one that is known */
#define MAX_GLYPH    0x10000   /* largest bitmap of one soft font character */
#define PCL_DPI      300.0     /* PCL units, used by soft fonts and *p moves */
#define DJ_BAND      50        /* dots in one pass of a DeskJet font */
#define DJ_STEP      48        /* dots between the passes, when a font does not say */
#define DJ_MAX_BASE  200       /* more than the height of four passes */
#define DJ_BOXES     7         /* boxes per dot column: 6 of 8 rows, 1 of 2 rows */

enum {
    STATE_TEXT = 0,
    STATE_ESC,
    STATE_GROUP,
    STATE_VALUE,
    STATE_DATA,
    STATE_TRANSPARENT,
    STATE_SWALLOW
};

typedef struct pcl3_glyph_t {
    int16_t  left;
    int16_t  top;
    uint16_t width;
    uint16_t height;
    int16_t  delta; /* in quarter dots */
    uint8_t *bits;
    uint32_t bytes;  /* size of bits */
    uint32_t filled; /* how much of bits has arrived */
    uint8_t  passes; /* DeskJet: mask of the passes that have arrived */
    bool     nonspacing;
} pcl3_glyph_t;

typedef struct pcl3_softfont_t {
    int          id; /* -1 = free */
    bool         proportional;
    double       hmi;
    double       res_x;
    double       res_y;
    bool         deskjet;  /* character data is in the DeskJet column format */
    bool         landscape;
    int          format;
    int          passes;   /* print head passes per character */
    int          baseline; /* dots from the top of the bottom pass */
    int          pass_base[4]; /* dots from the top of each pass to the baseline */
    /* Cartridge fonts only. */
    bool         pool;       /* glyphs are by HP's index, not by character code */
    bool         bold;
    bool         italic;
    double       height;     /* proportional fonts, in points */
    double       pitch;
    int          typeface;
    int          symbol_set; /* the set of a recreation; a real cartridge has them all */
    /* Soft fonts only. */
    uint32_t     ram_used[MAX_RAM_PAGE]; /* of each memory page */
    bool         ram_by_glyph;           /* it did not say how much it needs */
    pcl3_glyph_t glyphs[256];
} pcl3_softfont_t;

/* What a PCL font selection asks for. */
typedef struct pcl3_fontsel_t {
    int    symbol_set;
    int    spacing;
    double pitch;
    double height;
    int    style;
    int    weight;
    int    typeface;
    int    soft_id; /* -1 = selected by attributes */
} pcl3_fontsel_t;

typedef struct pcl3_t {
    /* Page. */
    uint8_t *pixels;
    int      w;
    int      h;
    int      dpi;
    double   default_width;
    double   default_height;
    double   page_width;  /* inches */
    double   page_height;
    bool     dirty;
    char     page_fn[260];
    char     page_path[1024];
    char     font_path[1024];
    PALETTE  palette;

    /* Command parser. */
    int      state;
    uint8_t  param_char;
    uint8_t  group_char;
    bool     value_seen;
    bool     value_negative;
    bool     value_signed;
    bool     value_fraction;
    double   value;
    double   value_scale;
    uint8_t  data_term;
    uint32_t data_len;
    uint32_t data_pos;
    uint8_t *data;
    bool     data_continues;
    uint32_t transparent_left;

    /* Cursor and page layout, all in inches. */
    double x;
    double y;
    double left_origin;
    double top_margin;
    double left_margin;
    double right_margin;
    double hmi;
    double vmi;
    bool   hmi_set;
    bool   wrap;
    int    line_termination;
    bool   underline;
    double push_x[20];
    double push_y[20];
    int    push_count;
    bool   display_functions;
    bool   landscape;

    /* Raster graphics. */
    int      res;
    int      palette_planes;
    int      plane_count;
    uint8_t  plane_ink[MAX_PLANES];
    int      plane_hres[MAX_PLANES];
    int      plane_vres[MAX_PLANES];
    uint8_t *seed[MAX_PLANES];
    uint32_t seed_bytes[MAX_PLANES];
    int      curr_plane;
    int      compression;
    bool     raster_active;
    double   raster_x;
    uint32_t raster_width; /* in dots, 0 = as sent */
    int32_t  raster_x_offset;
    double   raster_shift; /* set by driver configuration, in inches */

    /* Text. */
    pcl3_fontsel_t  font[2];
    int             font_active;
    FT_Face         face;
    bool            face_valid;
    uint16_t        symbol_map[256];
    uint8_t         pen_ink;
    pcl3_softfont_t softfonts[MAX_SOFTFONT];
    pcl3_softfont_t *carts[MAX_CART_FONT]; /* fonts in cartridges; a reset keeps them */
    int              cart_fonts;

    /* What the model can do. */
    bool     color;
    bool     landscape_graphics;
    bool     deskjet_fonts;
    bool     ram_limited;
    int      ram_pages;
    uint32_t ram_free[MAX_RAM_PAGE];
    int             font_id;
    int             char_code;
    double          rect_w;
    double          rect_h;
} pcl3_t;

/* HP Roman-8, upper half. */
static const uint16_t roman8_map[128] = {
    0x0080, 0x0081, 0x0082, 0x0083, 0x0084, 0x0085, 0x0086, 0x0087,
    0x0088, 0x0089, 0x008a, 0x008b, 0x008c, 0x008d, 0x008e, 0x008f,
    0x0090, 0x0091, 0x0092, 0x0093, 0x0094, 0x0095, 0x0096, 0x0097,
    0x0098, 0x0099, 0x009a, 0x009b, 0x009c, 0x009d, 0x009e, 0x009f,
    0x00a0, 0x00c0, 0x00c2, 0x00c8, 0x00ca, 0x00cb, 0x00ce, 0x00cf,
    0x00b4, 0x02cb, 0x02c6, 0x00a8, 0x02dc, 0x00d9, 0x00db, 0x20a4,
    0x00af, 0x00dd, 0x00fd, 0x00b0, 0x00c7, 0x00e7, 0x00d1, 0x00f1,
    0x00a1, 0x00bf, 0x00a4, 0x00a3, 0x00a5, 0x00a7, 0x0192, 0x00a2,
    0x00e2, 0x00ea, 0x00f4, 0x00fb, 0x00e1, 0x00e9, 0x00f3, 0x00fa,
    0x00e0, 0x00e8, 0x00f2, 0x00f9, 0x00e4, 0x00eb, 0x00f6, 0x00fc,
    0x00c5, 0x00ee, 0x00d8, 0x00c6, 0x00e5, 0x00ed, 0x00f8, 0x00e6,
    0x00c4, 0x00ec, 0x00d6, 0x00dc, 0x00c9, 0x00ef, 0x00df, 0x00d4,
    0x00c1, 0x00c3, 0x00e3, 0x00d0, 0x00f0, 0x00cd, 0x00cc, 0x00d3,
    0x00d2, 0x00d5, 0x00f5, 0x0160, 0x0161, 0x00da, 0x0178, 0x00ff,
    0x00de, 0x00fe, 0x00b7, 0x00b5, 0x00b6, 0x00be, 0x2014, 0x00bc,
    0x00bd, 0x00aa, 0x00ba, 0x00ab, 0x25a0, 0x00bb, 0x00b1, 0x0000
};

static void
fill_rect(pcl3_t *dev, double x0, double y0, double x1, double y1, uint8_t ink)
{
    int px0 = (int) lround(x0 * dev->dpi);
    int px1 = (int) lround(x1 * dev->dpi);
    int py0 = (int) lround(y0 * dev->dpi);
    int py1 = (int) lround(y1 * dev->dpi);

    if (px1 <= px0)
        px1 = px0 + 1;
    if (py1 <= py0)
        py1 = py0 + 1;

    if (px0 < 0)
        px0 = 0;
    if (py0 < 0)
        py0 = 0;
    if (px1 > dev->w)
        px1 = dev->w;
    if (py1 > dev->h)
        py1 = dev->h;

    for (int y = py0; y < py1; y++) {
        uint8_t *row = dev->pixels + ((size_t) y * dev->w);

        if (ink != 0) {
            for (int x = px0; x < px1; x++)
                row[x] |= ink;
        } else
            memset(row + px0, 0x00, (px0 < px1) ? (px1 - px0) : 0);
    }

    dev->dirty = true;
}

static void
save_page(pcl3_t *dev)
{
    char path[1280];

    if (dev->page_fn[0] == '\0')
        plat_tempfile(dev->page_fn, NULL, ".png");

    snprintf(path, sizeof(path), "%s%s", dev->page_path, dev->page_fn);
    png_write_rgb(path, dev->pixels, dev->w, dev->h, dev->w, dev->palette);
}

static void
set_page_size(pcl3_t *dev, double width, double height)
{
    const int new_w = (int) (width * dev->dpi);
    const int new_h = (int) (height * dev->dpi);

    if ((new_w != dev->w) || (new_h != dev->h)) {
        uint8_t *pixels = (uint8_t *) calloc((size_t) new_w, new_h);

        if (pixels == NULL)
            return;

        free(dev->pixels);
        dev->pixels = pixels;
        dev->w      = new_w;
        dev->h      = new_h;
    }

    dev->page_width  = width;
    dev->page_height = height;
}

static void
home_cursor(pcl3_t *dev)
{
    dev->x = dev->left_margin;
    dev->y = 0.75 * dev->vmi;
}

static void
eject_page(pcl3_t *dev)
{
    if (dev->dirty) {
        save_page(dev);
        memset(dev->pixels, 0x00, (size_t) dev->w * dev->h);
        dev->dirty = false;
    }

    dev->page_fn[0]    = '\0';
    dev->raster_active = false;

    home_cursor(dev);
}

static void
set_palette_planes(pcl3_t *dev, int planes)
{
    static const uint8_t kcmy[4] = { INK_K, INK_C, INK_M, INK_Y };
    static const uint8_t cmy[3]  = { INK_C, INK_M, INK_Y };

    /* A black only model has no other palette. */
    if (!dev->color)
        planes = 1;

    dev->palette_planes = planes;

    /* A positive plane count is additive: the ink goes where the bit is clear. */
    switch (planes) {
        case -4:
            dev->plane_count = 4;
            memcpy(dev->plane_ink, kcmy, 4);
            break;
        case 3:
        case -3:
            dev->plane_count = 3;
            memcpy(dev->plane_ink, cmy, 3);
            break;
        case -1:
        case 1:
        default:
            dev->palette_planes = 1;
            dev->plane_count    = 1;
            dev->plane_ink[0]   = INK_K;
            break;
    }

    for (int i = 0; i < MAX_PLANES; i++) {
        dev->plane_hres[i] = 0;
        dev->plane_vres[i] = 0;
    }
}

/* Configure Raster Data sets the resolution of each ink on its own. Four inks are
   always listed as C, M, Y, K. Format 2 sends them in that order, while format 1
   (DeskJet 520 and 560C) still sends black first. */
static void
configure_raster_data(pcl3_t *dev, const uint8_t *data, uint32_t len)
{
    static const uint8_t cmyk[4]   = { INK_C, INK_M, INK_Y, INK_K };
    static const uint8_t cmy[3]    = { INK_C, INK_M, INK_Y };
    static const uint8_t black[1]  = { INK_K };
    static const uint8_t list[4]   = { 0, 1, 2, 3 };
    static const uint8_t bfirst[4] = { 3, 0, 1, 2 };
    const uint8_t       *inks;
    const uint8_t       *order = list;
    int                  components;
    int                  plane = 0;

    if ((len < 2) || (data[0] < 1) || (data[0] > 2))
        return;

    components = data[1];
    if ((components == 0) || (len < (2 + ((uint32_t) components * 6))))
        return;

    if (components == 1)
        inks = black;
    else if (components == 3)
        inks = cmy;
    else if (components == 4) {
        inks = cmyk;
        if (data[0] == 1)
            order = bfirst;
    } else
        return;

    for (int i = 0; (i < components) && (plane < MAX_PLANES); i++) {
        const int      index  = order[i];
        const uint8_t *comp   = data + 2 + (index * 6);
        const int      levels = (comp[4] << 8) | comp[5];
        int            planes = 1;

        /* Each ink has enough planes for its levels, the lowest bit first. */
        while ((1 << planes) < levels)
            planes++;

        for (int j = 0; (j < planes) && (plane < MAX_PLANES); j++) {
            dev->plane_ink[plane]  = inks[index];
            dev->plane_hres[plane] = (comp[0] << 8) | comp[1];
            dev->plane_vres[plane] = (comp[2] << 8) | comp[3];
            plane++;
        }
    }

    dev->plane_count    = plane;
    dev->palette_planes = -plane;
}

static void
clear_seeds(pcl3_t *dev)
{
    for (int i = 0; i < MAX_PLANES; i++) {
        memset(dev->seed[i], 0x00, MAX_ROW);
        dev->seed_bytes[i] = 0;
    }
}

static void
start_raster(pcl3_t *dev, int mode)
{
    dev->raster_x      = (mode & 1) ? dev->x : 0.0;
    dev->raster_active = true;
    dev->curr_plane    = 0;

    clear_seeds(dev);
}

/* Compression mode 3: changes against the previous row. */
static void
decode_delta_row(uint8_t *row, const uint8_t *data, uint32_t len)
{
    uint32_t in  = 0;
    uint32_t out = 0;

    while (in < len) {
        const uint8_t cmd   = data[in++];
        uint32_t      count = (cmd >> 5) + 1;
        uint32_t      skip  = cmd & 0x1f;

        if (skip == 31) {
            uint8_t extra;

            do {
                if (in >= len)
                    return;
                extra = data[in++];
                skip += extra;
            } while (extra == 255);
        }

        out += skip;
        while ((count-- > 0) && (in < len)) {
            if (out < MAX_ROW)
                row[out] = data[in];
            out++;
            in++;
        }
    }
}

/* The extra bytes of a mode 9 count run until one which is not 255. */
static uint32_t
read_extended(const uint8_t *data, uint32_t len, uint32_t *in)
{
    uint32_t total = 0;
    uint8_t  extra;

    do {
        if (*in >= len)
            return total;
        extra = data[(*in)++];
        total += extra;
    } while (extra == 255);

    return total;
}

/* Compression mode 9: compressed replacement delta row. */
static void
decode_replacement_delta(uint8_t *row, const uint8_t *data, uint32_t len)
{
    uint32_t in  = 0;
    uint32_t out = 0;

    while (in < len) {
        const uint8_t cmd = data[in++];
        uint32_t      offset;
        uint32_t      count;

        if (cmd & 0x80) { /* a run */
            offset = (cmd >> 5) & 0x03;
            count  = (cmd & 0x1f) + 2;
            if (offset == 3)
                offset += read_extended(data, len, &in);
            if ((cmd & 0x1f) == 0x1f)
                count += read_extended(data, len, &in);

            out += offset;
            if (in >= len)
                return;

            const uint8_t value = data[in++];

            while (count-- > 0) {
                if (out < MAX_ROW)
                    row[out] = value;
                out++;
            }
        } else { /* literal bytes */
            offset = (cmd >> 3) & 0x0f;
            count  = (cmd & 0x07) + 1;
            if (offset == 15)
                offset += read_extended(data, len, &in);
            if ((cmd & 0x07) == 0x07)
                count += read_extended(data, len, &in);

            out += offset;
            while ((count-- > 0) && (in < len)) {
                if (out < MAX_ROW)
                    row[out] = data[in];
                out++;
                in++;
            }
        }
    }
}

static uint32_t
decode_runs(uint8_t *row, const uint8_t *data, uint32_t len)
{
    uint32_t out = 0;

    for (uint32_t in = 0; (in + 1) < len; in += 2) {
        uint32_t count = (uint32_t) data[in] + 1;

        while ((count-- > 0) && (out < MAX_ROW))
            row[out++] = data[in + 1];
    }

    return out;
}

static uint32_t
decode_packbits(uint8_t *row, const uint8_t *data, uint32_t len)
{
    uint32_t in  = 0;
    uint32_t out = 0;

    while (in < len) {
        const int8_t cmd = (int8_t) data[in++];

        if (cmd >= 0) {
            uint32_t count = (uint32_t) cmd + 1;

            while ((count-- > 0) && (in < len)) {
                if (out < MAX_ROW)
                    row[out++] = data[in];
                in++;
            }
        } else if (cmd != -128) {
            uint32_t count = 1 - (int32_t) cmd;

            if (in >= len)
                break;
            while ((count-- > 0) && (out < MAX_ROW))
                row[out++] = data[in];
            in++;
        }
    }

    return out;
}

/* A transfer's data always becomes the seed row of its plane. */
static void
decode_plane(pcl3_t *dev, const uint8_t *data, uint32_t len)
{
    uint8_t *row;
    uint32_t bytes;

    if (dev->curr_plane >= MAX_PLANES)
        return;

    row = dev->seed[dev->curr_plane];

    switch (dev->compression) {
        case 3:
            decode_delta_row(row, data, len);
            return;

        case 9:
            decode_replacement_delta(row, data, len);
            return;

        case 1:
            memset(row, 0x00, MAX_ROW);
            decode_runs(row, data, len);
            return;

        case 2:
            memset(row, 0x00, MAX_ROW);
            decode_packbits(row, data, len);
            return;

        default:
            memset(row, 0x00, MAX_ROW);
            bytes = (len > MAX_ROW) ? MAX_ROW : len;
            if (bytes > 0)
                memcpy(row, data, bytes);
            return;
    }
}

static void
print_row(pcl3_t *dev, int planes)
{
    const bool additive = (dev->palette_planes == 3);
    double     row_step = 0.0;

    if (planes > dev->plane_count)
        planes = dev->plane_count;

    for (int p = 0; p < planes; p++) {
        const int      hres    = dev->plane_hres[p] ? dev->plane_hres[p] : dev->res;
        const int      vres    = dev->plane_vres[p] ? dev->plane_vres[p] : dev->res;
        const uint8_t  ink     = dev->plane_ink[p];
        const uint8_t *row     = dev->seed[p];
        const double   step    = 1.0 / vres;
        const double   dot     = (double) dev->dpi / hres;
        const double   origin  = (dev->left_origin + dev->raster_x + dev->raster_shift) * dev->dpi;
        const int      y0      = (int) lround((dev->y + dev->top_margin) * dev->dpi);
        int            y1      = (int) lround((dev->y + dev->top_margin + step) * dev->dpi);
        uint32_t       width   = dev->raster_width ? ((uint64_t) dev->raster_width * hres / dev->res) : (uint32_t) (dev->page_width * hres);

        if (step > row_step)
            row_step = step;

        if (width > (MAX_ROW * 8))
            width = MAX_ROW * 8;
        if (y1 <= y0)
            y1 = y0 + 1;

        for (uint32_t b = 0; b < ((width + 7) / 8); b++) {
            const uint8_t value = additive ? (uint8_t) ~row[b] : row[b];

            if (value == 0x00)
                continue;

            for (int bit = 0; bit < 8; bit++) {
                const int32_t col = (int32_t) (b * 8) + bit + dev->raster_x_offset;

                if (!(value & (0x80 >> bit)) || (((b * 8) + bit) >= width))
                    continue;

                int x0 = (int) lround(origin + (col * dot));
                int x1 = (int) lround(origin + ((col + 1) * dot));

                if (x1 <= x0)
                    x1 = x0 + 1;
                if (x0 < 0)
                    x0 = 0;
                if (x1 > dev->w)
                    x1 = dev->w;

                for (int y = (y0 < 0) ? 0 : y0; (y < y1) && (y < dev->h); y++) {
                    uint8_t *line = dev->pixels + ((size_t) y * dev->w);

                    for (int x = x0; x < x1; x++)
                        line[x] |= ink;
                }

                dev->dirty = true;
            }
        }
    }

    if (row_step == 0.0)
        row_step = 1.0 / dev->res;

    dev->y += row_step;
}

/* The next character is drawn with the face this selection maps to. */
/* Symbol sets that differ from a base set, from Appendix C of the Technical Reference. */
typedef struct pcl3_set_change_t {
    uint8_t  code;
    uint16_t unicode;
} pcl3_set_change_t;

static const pcl3_set_change_t set_turkish[] = {
    { 0x8D, 0x0131 }, { 0x98, 0x0130 }, { 0x9E, 0x015E }, { 0x9F, 0x015F }, { 0xA6, 0x011E },
    { 0xA7, 0x011F },
};

static const pcl3_set_change_t set_danish[] = {
    { 0x8D, 0x00EC }, { 0x9B, 0x00F8 }, { 0x9D, 0x00D8 }, { 0x9E, 0x013F }, { 0x9F, 0x0140 },
    { 0xA6, 0x00F5 }, { 0xA7, 0x00D5 }, { 0xA9, 0x00E3 }, { 0xAA, 0x00C3 }, { 0xAB, 0x2113 },
    { 0xAC, 0x0149 }, { 0xAD, 0x00A1 }, { 0xAE, 0x00B3 }, { 0xAF, 0x00A4 },
};

static const pcl3_set_change_t set_windows[] = {
    { 0x91, 0x2018 }, { 0x92, 0x2019 },
};

static const pcl3_set_change_t set_latin5[] = {
    { 0xD0, 0x011E }, { 0xDD, 0x0130 }, { 0xDE, 0x015E }, { 0xF0, 0x011F }, { 0xFD, 0x0131 },
    { 0xFE, 0x015F },
};

static const uint16_t set_latin2[96] = {
    0x00A0, 0x0104, 0x02D8, 0x0141, 0x00A4, 0x013D, 0x015A, 0x00A7, 0x00A8, 0x0160, 0x015E, 0x0164,
    0x0179, 0x00AD, 0x017D, 0x017B, 0x00B0, 0x0105, 0x02DB, 0x0142, 0x00B4, 0x013E, 0x015B, 0x02C7,
    0x00B8, 0x0161, 0x015F, 0x0165, 0x017A, 0x02DD, 0x017E, 0x017C, 0x0154, 0x00C1, 0x00C2, 0x0102,
    0x00C4, 0x0139, 0x0106, 0x00C7, 0x010C, 0x00C9, 0x0118, 0x00CB, 0x011A, 0x00CD, 0x00CE, 0x010E,
    0x0110, 0x0143, 0x0147, 0x00D3, 0x00D4, 0x0150, 0x00D6, 0x00D7, 0x0158, 0x016E, 0x00DA, 0x0170,
    0x00DC, 0x00DD, 0x0162, 0x00DF, 0x0155, 0x00E1, 0x00E2, 0x0103, 0x00E4, 0x013A, 0x0107, 0x00E7,
    0x010D, 0x00E9, 0x0119, 0x00EB, 0x011B, 0x00ED, 0x00EE, 0x010F, 0x0111, 0x0144, 0x0148, 0x00F3,
    0x00F4, 0x0151, 0x00F6, 0x00F7, 0x0159, 0x016F, 0x00FA, 0x0171, 0x00FC, 0x00FD, 0x0163, 0x02D9,
};

static const uint16_t set_line_draw[224] = {
    0x0020, 0x2520, 0x2528, 0x2530, 0x2538, 0x255F, 0x2562, 0x2564, 0x2567, 0x2551, 0x2542, 0x253F,
    0x2500, 0x2514, 0x2502, 0x253C, 0x254B, 0x2523, 0x252B, 0x2533, 0x253B, 0x251C, 0x2524, 0x252C,
    0x2534, 0x2550, 0x2503, 0x2501, 0x256B, 0x250C, 0x256A, 0x256C, 0x255E, 0x2517, 0x2542, 0x2588,
    0x258C, 0x2503, 0x2514, 0x2518, 0x2510, 0x2561, 0x252C, 0x2565, 0x2566, 0x254B, 0x2548, 0x2563,
    0x2518, 0x250F, 0x250C, 0x251B, 0x2510, 0x2524, 0x2542, 0x2513, 0x25A0, 0x2510, 0x25AC, 0x2561,
    0x254C, 0x2568, 0x25AA, 0x2556, 0x255E, 0x2517, 0x2542, 0x2588, 0x258C, 0x2503, 0x2514, 0x2518,
    0x2510, 0x2561, 0x252C, 0x2565, 0x2566, 0x254B, 0x2548, 0x2563, 0x2518, 0x250F, 0x250C, 0x251B,
    0x2510, 0x2524, 0x2542, 0x2513, 0x25A0, 0x2510, 0x25AC, 0x2561, 0x254C, 0x2568, 0x25AA, 0x2592,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
};

static const uint16_t set_math7[224] = {
    0x0020, 0x221A, 0x007C, 0x00A7, 0x2207, 0x00B1, 0x221D, 0x2320, 0x00F7, 0x2243, 0x03A0, 0x0393,
    0x03A8, 0x2261, 0x03A6, 0x039E, 0x0030, 0x0031, 0x0032, 0x0033, 0x0034, 0x0035, 0x0036, 0x0037,
    0x0038, 0x0039, 0x03A9, 0x039B, 0x221E, 0x2321, 0x2020, 0x03A3, 0x00B6, 0x03B1, 0x03B2, 0x03C8,
    0x03C6, 0x03B5, 0x2202, 0x03BB, 0x03B7, 0x03B9, 0x0398, 0x03BA, 0x03C9, 0x03BC, 0x03BD, 0x03C1,
    0x03C0, 0x03B3, 0x03B8, 0x03C3, 0x03C4, 0x03BE, 0x0394, 0x03B4, 0x03C7, 0x03C5, 0x03B6, 0x2191,
    0x2192, 0x03A4, 0x2190, 0x2193, 0x00B6, 0x03B1, 0x03B2, 0x03C8, 0x03C6, 0x03B5, 0x2202, 0x03BB,
    0x03B7, 0x03B9, 0x0398, 0x03BA, 0x03C9, 0x03BC, 0x03BD, 0x03C1, 0x03C0, 0x03B3, 0x03B8, 0x03C3,
    0x03C4, 0x03BE, 0x0394, 0x03B4, 0x03C7, 0x03C5, 0x03B6, 0x2191, 0x2192, 0x03A4, 0x2190, 0x2592,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
};

static const uint16_t set_math8[224] = {
    0x0020, 0x221A, 0x2033, 0x00B0, 0x221E, 0x00F7, 0x221D, 0x2032, 0x0028, 0x0029, 0x00D7, 0x002B,
    0x002C, 0x2212, 0x002E, 0x002F, 0x0030, 0x0031, 0x0032, 0x0033, 0x0034, 0x0035, 0x0036, 0x0037,
    0x0038, 0x0039, 0x0065, 0x03B5, 0x003C, 0x003D, 0x003E, 0x2248, 0x2234, 0x0391, 0x0392, 0x0393,
    0x0394, 0x0395, 0x0396, 0x0397, 0x0398, 0x0399, 0x039A, 0x039B, 0x039C, 0x039D, 0x039E, 0x039F,
    0x03A0, 0x03A1, 0x03A3, 0x03A4, 0x03D2, 0x03A6, 0x03A7, 0x03A8, 0x03A9, 0x2207, 0x2202, 0x03C2,
    0x2264, 0x2260, 0x2265, 0x005F, 0x2235, 0x03B1, 0x03B2, 0x03B3, 0x03B4, 0x03B5, 0x03B6, 0x03B7,
    0x03B8, 0x03B9, 0x03BA, 0x03BB, 0x03BC, 0x03BD, 0x03BE, 0x03BF, 0x03C0, 0x03C1, 0x03C3, 0x03C4,
    0x03C5, 0x03C6, 0x03C7, 0x03C8, 0x03C9, 0x03D1, 0x03D5, 0x03D6, 0x2243, 0x2261, 0x2262, 0x2592,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x2191, 0x2192, 0x2193,
    0x2190, 0x21D1, 0x21D2, 0x21D3, 0x21D0, 0x2195, 0x2194, 0x21D5, 0x21D4, 0x21C4, 0x21C6, 0x2192,
    0x203E, 0x2200, 0x2203, 0x22A4, 0x22A5, 0x222A, 0x2229, 0x2208, 0x220B, 0x2209, 0x2282, 0x2283,
    0x2284, 0x2285, 0x2286, 0x2287, 0x2295, 0x2299, 0x2297, 0x2296, 0x2298, 0x2227, 0x2228, 0x22BB,
    0x00AC, 0x2218, 0x22C5, 0x2022, 0x25CF, 0x25CB, 0x2020, 0x2021, 0x00C5, 0x22A3, 0x22A2, 0x221F,
    0x220D, 0x222B, 0x222E, 0x2220, 0x2205, 0x2135, 0x2136, 0x2137, 0x212D, 0x2111, 0x211C, 0x2128,
    0x23A1, 0x23A3, 0x23A7, 0x23A8, 0x23A9, 0x2320, 0x233D, 0x2321, 0x2572, 0x2016, 0x2220, 0x2571,
    0x0020, 0x23A4, 0x23A6, 0x23AB, 0x23AC, 0x23AD, 0x23AE, 0x23AE, 0x230B, 0x00AC, 0x27E9, 0x231F,
    0x2572, 0x0020, 0x2213, 0x00B1, 0x0020, 0x0020, 0x0020, 0x0020,
};

static const uint16_t set_pi_font[224] = {
    0x0020, 0x0020, 0x0020, 0x0020, 0x201C, 0x201D, 0x2018, 0x2019, 0x27E8, 0x27E9, 0x2122, 0x2120,
    0x00AE, 0x00A9, 0x24C2, 0x0020, 0x203E, 0x02D8, 0x02C7, 0x02D9, 0x2197, 0x2198, 0x2199, 0x2196,
    0x25B3, 0x25B7, 0x25BD, 0x25C1, 0x226A, 0x00A7, 0x226B, 0x00B6, 0x2237, 0x225C, 0x0020, 0x0020,
    0x0020, 0x0020, 0x1DA0, 0x0020, 0x210F, 0x0020, 0x0020, 0x0020, 0x2112, 0x2113, 0x0020, 0x0020,
    0x2118, 0x2118, 0x211E, 0x2211, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x27E6,
    0x2016, 0x27E7, 0x0020, 0x0020, 0x250C, 0x2514, 0x256D, 0x2570, 0x253C, 0x251C, 0x2500, 0x2551,
    0x222A, 0x2229, 0x27E6, 0x27E6, 0x25A1, 0x25C7, 0x0020, 0x0020, 0x2510, 0x2518, 0x256E, 0x256F,
    0x252C, 0x2524, 0x2534, 0x2502, 0x2294, 0x2293, 0x27E7, 0x27E7, 0x25A0, 0x25C6, 0x0020, 0x2592,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
    0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020, 0x0020,
};

static void
apply_set_changes(pcl3_t *dev, const pcl3_set_change_t *changes, size_t count)
{
    for (size_t i = 0; i < count; i++)
        dev->symbol_map[changes[i].code] = changes[i].unicode;
}

/* Sets of symbols: the printable codes are listed, and the rest are not printed. */
static void
apply_symbol_table(pcl3_t *dev, const uint16_t *table)
{
    memset(dev->symbol_map, 0x00, sizeof(dev->symbol_map));
    for (int i = 0x20; i < 0x100; i++)
        dev->symbol_map[i] = table[i - 0x20];
}

static void
select_symbol_set(pcl3_t *dev, int symbol_set)
{
    switch (symbol_set) {
        case 10 * 32 + 21: /* 10U: PC-8 */
            select_codepage(437, dev->symbol_map);
            break;
        case 12 * 32 + 21: /* 12U: PC-850 */
            select_codepage(850, dev->symbol_map);
            break;
        case 8 * 32 + 21: /* 8U: Roman-8 */
            for (int i = 0; i < 128; i++) {
                dev->symbol_map[i]       = i;
                dev->symbol_map[i + 128] = roman8_map[i];
            }
            break;
        case 11 * 32 + 21: /* 11U: PC-8 D/N */
            select_codepage(437, dev->symbol_map);
            apply_set_changes(dev, set_danish, sizeof(set_danish) / sizeof(set_danish[0]));
            break;
        case 17 * 32 + 21: /* 17U: PC-852 */
            select_codepage(852, dev->symbol_map);
            break;
        case 9 * 32 + 20: /* 9T: PC-8 Turkish */
            select_codepage(437, dev->symbol_map);
            apply_set_changes(dev, set_turkish, sizeof(set_turkish) / sizeof(set_turkish[0]));
            break;
        case 2 * 32 + 14: /* 2N: ISO 8859/2 */
            for (int i = 0; i < 0xA0; i++)
                dev->symbol_map[i] = i;
            for (int i = 0xA0; i < 0x100; i++)
                dev->symbol_map[i] = set_latin2[i - 0xA0];
            break;
        case 0 * 32 + 12: /* 0L: Line Draw */
            apply_symbol_table(dev, set_line_draw);
            break;
        case 0 * 32 + 13: /* 0M: Math-7 */
            apply_symbol_table(dev, set_math7);
            break;
        case 8 * 32 + 13: /* 8M: Math-8 */
            apply_symbol_table(dev, set_math8);
            break;
        case 15 * 32 + 21: /* 15U: Pi Font */
            apply_symbol_table(dev, set_pi_font);
            break;
        default: /* 0N: ISO 8859-1, 9U and 19U: Windows, 5N: ISO 8859/9, 0U: ASCII */
            for (int i = 0; i < 256; i++)
                dev->symbol_map[i] = i;
            if (symbol_set == (5 * 32 + 14))
                apply_set_changes(dev, set_latin5, sizeof(set_latin5) / sizeof(set_latin5[0]));
            else if (symbol_set == (9 * 32 + 21)) {
                for (int i = 0x80; i < 0xA0; i++)
                    dev->symbol_map[i] = 0;
                apply_set_changes(dev, set_windows, sizeof(set_windows) / sizeof(set_windows[0]));
            }
            break;
    }
}

static void
update_face(pcl3_t *dev)
{
    const pcl3_fontsel_t *sel = &dev->font[dev->font_active];
    char                  path[1280];
    const char           *fn;
    double                points;
    FT_Matrix             matrix;

    if (dev->face_valid)
        return;

    dev->face_valid = true;

    select_symbol_set(dev, sel->symbol_set);

    if (dev->face != NULL) {
        FT_Done_Face(dev->face);
        dev->face = NULL;
    }

    if (ft_lib == NULL)
        return;

    if (sel->spacing == 0)
        fn = FONT_FILE_COURIER;
    else if ((sel->typeface == 4) || (sel->typeface == 24) || (sel->typeface == 16602))
        fn = FONT_FILE_SANSSERIF;
    else
        fn = FONT_FILE_ROMAN;

    snprintf(path, sizeof(path), "%s%s", dev->font_path, fn);
    if (FT_New_Face(ft_lib, path, 0, &dev->face)) {
        dev->face = NULL;
        return;
    }

    /* A fixed pitch face is 0.6 em wide. */
    points = (sel->spacing == 0) ? (72.0 / (sel->pitch * 0.6)) : sel->height;
    FT_Set_Char_Size(dev->face, 0, (FT_F26Dot6) (points * 64.0), dev->dpi, dev->dpi);

    if (sel->style & 1) {
        matrix.xx = 0x10000L;
        matrix.xy = (FT_Fixed) (0.20 * 0x10000L);
        matrix.yx = 0;
        matrix.yy = 0x10000L;
        FT_Set_Transform(dev->face, &matrix, 0);
    }
}

static pcl3_softfont_t *
find_softfont(pcl3_t *dev, int id, bool create)
{
    pcl3_softfont_t *spare = NULL;

    for (int i = 0; i < MAX_SOFTFONT; i++) {
        if (dev->softfonts[i].id == id)
            return &dev->softfonts[i];
        if ((dev->softfonts[i].id < 0) && (spare == NULL))
            spare = &dev->softfonts[i];
    }

    if (create && (spare != NULL))
        spare->id = id;

    return create ? spare : NULL;
}

static void
delete_glyph(pcl3_glyph_t *glyph)
{
    free(glyph->bits);
    memset(glyph, 0x00, sizeof(pcl3_glyph_t));
}

static void
delete_softfont(pcl3_softfont_t *font)
{
    for (int i = 0; i < 256; i++)
        delete_glyph(&font->glyphs[i]);

    font->id = -1;
}

/* Memory for soft fonts is in the RAM cartridges, which are divided into 64 KB pages.
   The printer takes the page with the most room. */
static bool
take_font_ram(pcl3_t *dev, pcl3_softfont_t *font, uint32_t size)
{
    int page = 0;

    if (!dev->ram_limited)
        return true;

    if (dev->ram_pages == 0)
        return false;

    for (int i = 1; i < dev->ram_pages; i++) {
        if (dev->ram_free[i] > dev->ram_free[page])
            page = i;
    }

    if (size > dev->ram_free[page])
        return false;

    dev->ram_free[page] -= size;
    font->ram_used[page] += size;

    return true;
}

static void
free_softfont(pcl3_t *dev, pcl3_softfont_t *font)
{
    for (int i = 0; i < MAX_RAM_PAGE; i++) {
        dev->ram_free[i] += font->ram_used[i];
        font->ram_used[i] = 0;
    }

    font->ram_by_glyph = false;
    delete_softfont(font);
}

static void
store_font_descriptor(pcl3_t *dev, const uint8_t *data, uint32_t len)
{
    pcl3_softfont_t *font;
    int              format;
    uint32_t         size = 0;

    if (len < 18)
        return;

    format = data[2];

    /* The DeskJet 540 only takes LaserJet format fonts, and the others need RAM. */
    if (!dev->deskjet_fonts && ((format == 5) || (format == 9) || (format == 12)))
        return;
    if (dev->ram_limited && (dev->ram_pages == 0))
        return;

    /* Creating a font replaces one with the same ID. */
    font = find_softfont(dev, dev->font_id, false);
    if (font != NULL)
        free_softfont(dev, font);

    font = find_softfont(dev, dev->font_id, true);
    if (font == NULL)
        return;

    if (((format == 5) || (format == 9) || (format == 12)) && (len >= (uint32_t) ((format == 12) ? 78 : 76))) {
        const uint32_t at = (format == 12) ? 76 : 74;

        size = (data[at] << 8) | data[at + 1];
    }

    /* The DeskJet formats have a 600 x 300 dpi grid, and format 20 names its own. */
    font->res_x   = 300.0;
    font->res_y   = 300.0;
    font->format  = format;
    /* The DeskJet has no way to rotate a font: a landscape one is drawn on its side. */
    font->landscape = data[12] == 1;
    font->deskjet = (format == 5) || (format == 9) || (format == 12);
    if (font->deskjet) {
        const int cell_height = (data[10] << 8) | data[11];
        const int offsets     = (format == 12) ? 88 : 86;
        int       passes      = 1;

        font->res_x    = 600.0;
        font->baseline = (data[6] << 8) | data[7];

        /* The baseline position is for the bottom cell, and each further cell has its
           own offset to the same baseline: the higher a pass, the larger its offset. */
        font->pass_base[0] = font->baseline;
        for (int i = 0; (i < 3) && (len >= (uint32_t) (offsets + (i * 2) + 2)); i++) {
            const int base = (data[offsets + (i * 2)] << 8) | data[offsets + (i * 2) + 1];

            if ((base > 0) && (base <= DJ_MAX_BASE))
                font->pass_base[passes++] = base;
        }

        /* Without offsets, take the cell height, with cells the usual 48 dots apart. */
        if (passes == 1) {
            passes = (cell_height + DJ_STEP - 1 - (DJ_BAND - DJ_STEP)) / DJ_STEP;
            if (passes < 1)
                passes = 1;
            if (passes > 4)
                passes = 4;
            for (int i = 1; i < passes; i++)
                font->pass_base[i] = font->baseline + (i * DJ_STEP);
        }

        /* Largest first, so that the index is the print head pass, from the top. */
        for (int i = 1; i < passes; i++) {
            const int base = font->pass_base[i];
            int       j    = i;

            for (; (j > 0) && (font->pass_base[j - 1] < base); j--)
                font->pass_base[j] = font->pass_base[j - 1];
            font->pass_base[j] = base;
        }
        font->passes = passes;

        /* The font disappears if a pass of it does not fit. */
        for (int i = 0; size && (i < passes); i++) {
            const uint32_t pass = (((size / passes) + RAM_BLOCK - 1) / RAM_BLOCK) * RAM_BLOCK;

            if ((pass > RAM_PAGE) || !take_font_ram(dev, font, pass)) {
                free_softfont(dev, font);
                return;
            }
        }
    } else if ((format == 20) && (len >= 68)) {
        font->res_x = (data[64] << 8) | data[65];
        font->res_y = (data[66] << 8) | data[67];
        if ((font->res_x <= 0.0) || (font->res_y <= 0.0))
            font->res_x = font->res_y = 300.0;
    }

    font->ram_by_glyph = size == 0;
    font->proportional = data[13] != 0;
    /* The pitch field is in quarter dots, which are 300 dpi dots in all the formats
       except format 20. */
    font->hmi = (double) ((data[16] << 8) | data[17]) / (4.0 * ((format == 20) ? font->res_x : PCL_DPI));
}

/* Class 2 rows are a repeat count, then alternating white and black run lengths. */
static void
expand_compressed_glyph(pcl3_glyph_t *glyph, const uint8_t *data, uint32_t len)
{
    const int stride = (glyph->width + 7) / 8;
    uint32_t  in     = 0;
    int       y      = 0;

    while ((in < len) && (y < glyph->height)) {
        const int repeat = (int) data[in++] + 1;
        const int row    = y;
        int       x      = 0;
        int       black  = 0;

        while ((x < glyph->width) && (in < len)) {
            int run = data[in++];

            for (int i = 0; (i < run) && (x < glyph->width); i++, x++) {
                if (black)
                    glyph->bits[(row * stride) + (x / 8)] |= 0x80 >> (x & 7);
            }

            black = !black;
        }

        y++;
        for (int i = 1; (i < repeat) && (y < glyph->height); i++, y++)
            memcpy(&glyph->bits[y * stride], &glyph->bits[row * stride], stride);
    }
}

/* DeskJet characters are stored by print head column. Each column of 1/600 inch
   has a flag byte, whose bits say which boxes of its 50 rows have data bytes. Tall
   characters are in one block per pass, of which pass 1 is the top, and the bottom
   pass comes first: all the characters of a font are sent a pass at a time. */
static void
store_deskjet_glyph(pcl3_softfont_t *font, pcl3_glyph_t *glyph, const uint8_t *data, uint32_t len)
{
    const uint32_t header = 2 + ((data[2] < 6) ? 6 : data[2]);
    const int      type   = data[3];
    const int      width  = data[4];
    const int      band   = (type < 2) ? 0 : (type - 2);
    uint32_t       in;

    if ((len < header) || (type == 1) || (band >= font->passes))
        return;

    /* A pass already seen, or another width, is a new definition of the character. */
    if ((glyph->bits == NULL) || (glyph->passes & (1 << band)) || (width != glyph->width)) {
        const bool signed_pads = font->format == 12;
        /* A landscape character has its left pad in the compressed width field. */
        const int  left        = font->landscape ? data[5] : (signed_pads ? (int8_t) data[6] : data[6]);
        const int  right       = font->landscape ? 0 : (signed_pads ? (int8_t) data[7] : data[7]);

        delete_glyph(glyph);

        /* 255 is a character which neither prints nor moves. */
        if (width == 255) {
            glyph->nonspacing = true;
            return;
        }

        glyph->nonspacing = false;
        glyph->left       = (font->proportional || font->landscape) ? left : 0;
        glyph->top        = font->pass_base[0];
        glyph->width      = width;
        glyph->height     = font->pass_base[0] - font->pass_base[font->passes - 1] + DJ_BAND;
        glyph->delta      = (width + left + right) * 4;
        glyph->bytes      = (uint32_t) ((glyph->width + 7) / 8) * glyph->height;
        glyph->filled     = glyph->bytes;
        glyph->bits       = (uint8_t *) calloc(glyph->bytes ? glyph->bytes : 1, 1);
        if (glyph->bits == NULL) {
            glyph->bytes = 0;
            return;
        }
    }

    glyph->passes |= 1 << band;

    if ((header + width) > len)
        return;

    in = header + width;

    for (int col = 0; col < width; col++) {
        const uint8_t flags = data[header + col];

        for (int box = 0; box < DJ_BOXES; box++) {
            const int rows = (box == (DJ_BOXES - 1)) ? 2 : 8;

            if (!(flags & (1 << box)))
                continue;
            if (in >= len)
                return;

            const uint8_t value = data[in++];

            for (int row = 0; row < rows; row++) {
                if (value & (0x80 >> row)) {
                    const int y = (font->pass_base[0] - font->pass_base[band]) + (box * 8) + row;

                    glyph->bits[(y * ((glyph->width + 7) / 8)) + (col / 8)] |= 0x80 >> (col & 7);
                }
            }
        }
    }
}

static void
store_glyph(pcl3_t *dev, const uint8_t *data, uint32_t len)
{
    pcl3_softfont_t *font = find_softfont(dev, dev->font_id, false);
    pcl3_glyph_t    *glyph;
    uint32_t         count;

    if ((font == NULL) || (len < 2) || (dev->char_code < 0) || (dev->char_code > 255))
        return;

    glyph = &font->glyphs[dev->char_code];

    if (font->deskjet && ((data[0] == 5) || (data[0] == 9) || (data[0] == 12))) {
        if (len >= 8) {
            store_deskjet_glyph(font, glyph, data, len);
            if (font->ram_by_glyph && !take_font_ram(dev, font, len))
                delete_glyph(glyph);
        }
        return;
    }

    /* Continuation blocks carry more of the bitmap from their third byte. */
    if (data[1] != 0) {
        if (glyph->bits != NULL) {
            count = len - 2;
            if (count > (glyph->bytes - glyph->filled))
                count = glyph->bytes - glyph->filled;
            memcpy(glyph->bits + glyph->filled, data + 2, count);
            glyph->filled += count;
        }
        return;
    }

    delete_glyph(glyph);

    if ((data[0] != 4) || (len < 16))
        return;

    if (font->ram_by_glyph && !take_font_ram(dev, font, len))
        return;

    glyph->left   = (int16_t) ((data[6] << 8) | data[7]);
    glyph->top    = (int16_t) ((data[8] << 8) | data[9]);
    glyph->width  = (data[10] << 8) | data[11];
    glyph->height = (data[12] << 8) | data[13];
    glyph->delta  = (int16_t) ((data[14] << 8) | data[15]);
    if (glyph->delta < 0)
        glyph->delta = 0;

    /* A definition this large cannot be real: keep only its spacing. */
    glyph->bytes = (uint32_t) ((glyph->width + 7) / 8) * glyph->height;
    if (glyph->bytes > MAX_GLYPH) {
        glyph->bytes = 0;
        return;
    }

    glyph->bits = (uint8_t *) calloc(glyph->bytes ? glyph->bytes : 1, 1);
    if (glyph->bits == NULL) {
        glyph->bytes = 0;
        return;
    }

    if (data[3] == 2) {
        expand_compressed_glyph(glyph, data + 16, len - 16);
        glyph->filled = glyph->bytes;
    } else if (data[3] == 1) {
        count = len - 16;
        if (count > glyph->bytes)
            count = glyph->bytes;
        memcpy(glyph->bits, data + 16, count);
        glyph->filled = count;
    } else
        delete_glyph(glyph);
}

static double
char_advance(pcl3_t *dev, double natural)
{
    const pcl3_fontsel_t *sel = &dev->font[dev->font_active];

    if (dev->hmi_set)
        return dev->hmi;

    return (sel->spacing == 0) ? (1.0 / sel->pitch) : natural;
}

/* A landscape page is turned 90 degrees counter-clockwise on the sheet, so text runs
   up the sheet, and later lines sit further right. A character is stored as it lies
   on the sheet, with its top to the left: a left pad, then its columns, across a cell
   as wide as the line spacing. The rows run from its right side to its left, and the
   cell is one character pitch tall. Such fonts are always fixed pitch. */
static void
blit_landscape_glyph(pcl3_t *dev, const pcl3_softfont_t *font, const pcl3_glyph_t *glyph)
{
    const double px     = 1.0 / font->res_x;
    const double py     = 1.0 / font->res_y;
    const double pitch  = dev->hmi_set ? dev->hmi : font->hmi;
    const double base_x = dev->top_margin + dev->y - (font->baseline * py) + (glyph->left * px);
    const double base_y = dev->page_height - dev->left_origin - dev->x - pitch;
    const int    stride = (glyph->width + 7) / 8;

    if (glyph->bits != NULL) {
        for (int y = 0; y < glyph->height; y++) {
            for (int x = 0; x < glyph->width; x++) {
                if (glyph->bits[(y * stride) + (x / 8)] & (0x80 >> (x & 7)))
                    fill_rect(dev, base_x + (x * px), base_y + (y * py),
                              base_x + (x * px) + (1.0 / PCL_DPI), base_y + ((y + 1) * py), dev->pen_ink);
            }
        }
    }
}

static void
draw_landscape_glyph(pcl3_t *dev, const pcl3_softfont_t *font, const pcl3_glyph_t *glyph)
{
    blit_landscape_glyph(dev, font, glyph);
    dev->x += dev->hmi_set ? dev->hmi : font->hmi;
}

/* What the glyphs of a real cartridge are, by index. Zero is not a character. */
static const uint16_t cart_pool[255] = {
    0x0020, 0x0021, 0x0022, 0x0023, 0x0024, 0x0025, 0x0026, 0x0027, 0x0028, 0x0029, 0x002A, 0x002B,
    0x002C, 0x002D, 0x002E, 0x002F, 0x0030, 0x0031, 0x0032, 0x0033, 0x0034, 0x0035, 0x0036, 0x0037,
    0x0038, 0x0039, 0x003A, 0x003B, 0x003C, 0x003D, 0x003E, 0x003F, 0x0040, 0x0041, 0x0042, 0x0043,
    0x0044, 0x0045, 0x0046, 0x0047, 0x0048, 0x0049, 0x004A, 0x004B, 0x004C, 0x004D, 0x004E, 0x004F,
    0x0050, 0x0051, 0x0052, 0x0053, 0x0054, 0x0055, 0x0056, 0x0057, 0x0058, 0x0059, 0x005A, 0x005B,
    0x005C, 0x005D, 0x005E, 0x005F, 0x0060, 0x0061, 0x0062, 0x0063, 0x0064, 0x0065, 0x0066, 0x0067,
    0x0068, 0x0069, 0x006A, 0x006B, 0x006C, 0x006D, 0x006E, 0x006F, 0x0070, 0x0071, 0x0072, 0x0073,
    0x0074, 0x0075, 0x0076, 0x0077, 0x0078, 0x0079, 0x007A, 0x007B, 0x007C, 0x007D, 0x007E, 0x2592,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x00D7, 0x0000, 0x0000, 0x00A1, 0x00BF, 0x00A4, 0x00A3,
    0x00A5, 0x00A7, 0x0192, 0x00A2, 0x00D8, 0x00C6, 0x00E5, 0x00F8, 0x00E6, 0x00DF, 0x00D0, 0x00F0,
    0x0160, 0x0161, 0x00DE, 0x00FE, 0x2022, 0x00B5, 0x00B6, 0x00BE, 0x2014, 0x00BC, 0x00BD, 0x00AA,
    0x00BA, 0x00AB, 0x25A0, 0x00BB, 0x00B1, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x263A, 0x263B, 0x2665, 0x2666, 0x2663, 0x2660, 0x25D8, 0x25CB, 0x25D9, 0x2642, 0x2640, 0x266A,
    0x266B, 0x263C, 0x25BA, 0x25C4, 0x203C, 0x25AC, 0x21A8, 0x2191, 0x2193, 0x2192, 0x2190, 0x221F,
    0x25B2, 0x25BC, 0x00A6, 0x20A7, 0x2310, 0x00AC, 0x2591, 0x2592, 0x2593, 0x2502, 0x2524, 0x2561,
    0x2562, 0x2556, 0x2555, 0x2563, 0x2551, 0x2557, 0x255D, 0x255C, 0x255B, 0x2510, 0x2514, 0x2534,
    0x252C, 0x251C, 0x255E, 0x255F, 0x255A, 0x2554, 0x2569, 0x2566, 0x2560, 0x2550, 0x256C, 0x2567,
    0x2568, 0x2564, 0x2565, 0x2559, 0x2558, 0x2552, 0x2553, 0x2518, 0x250C, 0x2584, 0x258C, 0x2590,
    0x2580, 0x03B1, 0x03B2, 0x0393, 0x03C0, 0x03A3, 0x03C3, 0x03C4, 0x03A6, 0x0398, 0x03A9, 0x03B4,
    0x221E, 0x03C6, 0x03B5, 0x2229, 0x2261, 0x2265, 0x2264, 0x2320, 0x2321, 0x00F7, 0x2248, 0x2219,
    0x221A, 0x207F, 0x00B2, 0x25AA, 0x0000, 0x2113, 0x0149, 0x00B3, 0x00B7, 0x00A9, 0x00AE, 0x00B9,
    0x0000, 0x2020, 0x2122,
};

/* Letters that a real cartridge makes from a base letter and a loose accent. */
static const struct { uint16_t code; uint8_t base; uint8_t accent; } cart_accents[] = {
    { 0x00C0, 0x41, 0x61 }, { 0x00C1, 0x41, 0x60 }, { 0x00C2, 0x41, 0x62 }, { 0x00C3, 0x41, 0x64 },
    { 0x00C4, 0x41, 0x63 }, { 0x00C5, 0x41, 0x67 }, { 0x00C7, 0x43, 0x89 }, { 0x00C8, 0x45, 0x61 },
    { 0x00C9, 0x45, 0x60 }, { 0x00CA, 0x45, 0x62 }, { 0x00CB, 0x45, 0x63 }, { 0x00CC, 0x49, 0x61 },
    { 0x00CD, 0x49, 0x60 }, { 0x00CE, 0x49, 0x62 }, { 0x00CF, 0x49, 0x63 }, { 0x00D1, 0x4E, 0x64 },
    { 0x00D2, 0x4F, 0x61 }, { 0x00D3, 0x4F, 0x60 }, { 0x00D4, 0x4F, 0x62 }, { 0x00D5, 0x4F, 0x64 },
    { 0x00D6, 0x4F, 0x63 }, { 0x00D9, 0x55, 0x61 }, { 0x00DA, 0x55, 0x60 }, { 0x00DB, 0x55, 0x62 },
    { 0x00DC, 0x55, 0x63 }, { 0x00DD, 0x59, 0x60 }, { 0x00E0, 0x61, 0x8B }, { 0x00E1, 0x61, 0x8A },
    { 0x00E2, 0x61, 0x8C }, { 0x00E3, 0x61, 0x8E }, { 0x00E4, 0x61, 0x8D }, { 0x00E7, 0x63, 0x89 },
    { 0x00E8, 0x65, 0x8B }, { 0x00E9, 0x65, 0x8A }, { 0x00EA, 0x65, 0x8C }, { 0x00EB, 0x65, 0x8D },
    { 0x00EC, 0x69, 0x8B }, { 0x00ED, 0x69, 0x8A }, { 0x00EE, 0x69, 0x8C }, { 0x00EF, 0x69, 0x8D },
    { 0x00F1, 0x6E, 0x8E }, { 0x00F2, 0x6F, 0x8B }, { 0x00F3, 0x6F, 0x8A }, { 0x00F4, 0x6F, 0x8C },
    { 0x00F5, 0x6F, 0x8E }, { 0x00F6, 0x6F, 0x8D }, { 0x00F9, 0x75, 0x8B }, { 0x00FA, 0x75, 0x8A },
    { 0x00FB, 0x75, 0x8C }, { 0x00FC, 0x75, 0x8D }, { 0x00FD, 0x79, 0x8A }, { 0x00FF, 0x79, 0x8D },
    { 0x0106, 0x43, 0x60 }, { 0x0107, 0x63, 0x8A }, { 0x0108, 0x43, 0x62 }, { 0x0109, 0x63, 0x8C },
    { 0x011C, 0x47, 0x62 }, { 0x011D, 0x67, 0x8C }, { 0x0122, 0x47, 0x89 }, { 0x0123, 0x67, 0x89 },
    { 0x0124, 0x48, 0x62 }, { 0x0125, 0x68, 0x8C }, { 0x0128, 0x49, 0x64 }, { 0x0129, 0x69, 0x8E },
    { 0x0134, 0x4A, 0x62 }, { 0x0135, 0x6A, 0x8C }, { 0x0136, 0x4B, 0x89 }, { 0x0137, 0x6B, 0x89 },
    { 0x0139, 0x4C, 0x60 }, { 0x013A, 0x6C, 0x8A }, { 0x013B, 0x4C, 0x89 }, { 0x013C, 0x6C, 0x89 },
    { 0x0143, 0x4E, 0x60 }, { 0x0144, 0x6E, 0x8A }, { 0x0145, 0x4E, 0x89 }, { 0x0146, 0x6E, 0x89 },
    { 0x0154, 0x52, 0x60 }, { 0x0155, 0x72, 0x8A }, { 0x0156, 0x52, 0x89 }, { 0x0157, 0x72, 0x89 },
    { 0x015A, 0x53, 0x60 }, { 0x015B, 0x73, 0x8A }, { 0x015C, 0x53, 0x62 }, { 0x015D, 0x73, 0x8C },
    { 0x015E, 0x53, 0x89 }, { 0x015F, 0x73, 0x89 }, { 0x0162, 0x54, 0x89 }, { 0x0163, 0x74, 0x89 },
    { 0x0168, 0x55, 0x64 }, { 0x0169, 0x75, 0x8E }, { 0x016E, 0x55, 0x67 }, { 0x016F, 0x75, 0x67 },
    { 0x0174, 0x57, 0x62 }, { 0x0175, 0x77, 0x8C }, { 0x0176, 0x59, 0x62 }, { 0x0177, 0x79, 0x8C },
    { 0x0178, 0x59, 0x63 }, { 0x0179, 0x5A, 0x60 }, { 0x017A, 0x7A, 0x8A },
};

static int
cart_pool_index(uint16_t code)
{
    for (int i = 0; i < (int) (sizeof(cart_pool) / sizeof(cart_pool[0])); i++) {
        if (cart_pool[i] == code)
            return i;
    }

    return -1;
}

/* Prints a character of the selected symbol set from a cartridge font. */
static void
draw_cart_glyph(pcl3_t *dev, pcl3_softfont_t *font, uint8_t ch)
{
    if (font->pool) {
        const uint16_t code  = dev->symbol_map[ch];
        const int      index = cart_pool_index(code);

        if (index >= 0)
            blit_landscape_glyph(dev, font, &font->glyphs[index]);
        else {
            for (size_t i = 0; i < (sizeof(cart_accents) / sizeof(cart_accents[0])); i++) {
                if (cart_accents[i].code == code) {
                    blit_landscape_glyph(dev, font, &font->glyphs[cart_accents[i].base - 0x20]);
                    blit_landscape_glyph(dev, font, &font->glyphs[cart_accents[i].accent]);
                    break;
                }
            }
        }
    } else
        blit_landscape_glyph(dev, font, &font->glyphs[ch]);

    dev->x += dev->hmi_set ? dev->hmi : font->hmi;
}

/* Fonts are chosen by attributes, as the printer does it. */
static pcl3_softfont_t *
find_cart_font(pcl3_t *dev, const pcl3_fontsel_t *sel)
{
    pcl3_softfont_t *best       = NULL;
    int              best_score = -1;

    for (int i = 0; i < dev->cart_fonts; i++) {
        pcl3_softfont_t *font = dev->carts[i];
        int              score;

        if ((font->landscape != dev->landscape) || (font->proportional != (sel->spacing != 0)))
            continue;
        if (font->typeface != sel->typeface)
            continue;
        if (font->proportional ? (fabs(font->height - sel->height) > 0.5) : (fabs(font->pitch - sel->pitch) > 0.5))
            continue;
        /* An ASCII font answers for any set. */
        if (!font->pool && (font->symbol_set != sel->symbol_set) && (font->symbol_set != 21))
            continue;

        score = ((font->bold == (sel->weight > 0)) ? 2 : 0) + ((font->italic == ((sel->style & 1) != 0)) ? 1 : 0);
        if (score > best_score) {
            best       = font;
            best_score = score;
        }
    }

    return best;
}

static void
draw_soft_glyph(pcl3_t *dev, pcl3_softfont_t *font, uint8_t ch)
{
    const pcl3_glyph_t *glyph = &font->glyphs[ch];
    const double        px    = 1.0 / font->res_x;
    const double        py    = 1.0 / font->res_y;
    /* A DeskJet dot is 1/300 inch wide, on a 1/600 inch grid. */
    const double        dot   = font->deskjet ? (1.0 / PCL_DPI) : px;
    double              base_x;
    double              base_y;
    double              advance;

    /* What was not sent is nothing at all if a control code, and otherwise a space. */
    if (font->deskjet && (glyph->nonspacing || ((glyph->bits == NULL) && (ch < 0x20))))
        return;

    /* A font is only selected for the orientation it was made for. */
    if (font->landscape) {
        if (dev->landscape)
            draw_landscape_glyph(dev, font, glyph);
        else
            dev->x += dev->hmi_set ? dev->hmi : font->hmi;
        return;
    }

    base_x = dev->left_origin + dev->x + (glyph->left * px);
    /* Fixed pitch DeskJet characters are centred in their cell, without their pads. */
    if (font->deskjet && !font->proportional && (((font->hmi / px) - glyph->width) > 0.0))
        base_x += floor(((font->hmi / px) - glyph->width) / 2.0) * px;
    base_y = dev->top_margin + dev->y - (glyph->top * py);

    if (glyph->bits != NULL) {
        const int stride = (glyph->width + 7) / 8;

        for (int y = 0; y < glyph->height; y++) {
            for (int x = 0; x < glyph->width; x++) {
                if (glyph->bits[(y * stride) + (x / 8)] & (0x80 >> (x & 7)))
                    fill_rect(dev, base_x + (x * px), base_y + (y * py),
                              base_x + (x * px) + dot, base_y + ((y + 1) * py), dev->pen_ink);
            }
        }
    }

    advance = font->proportional ? ((glyph->delta / 4.0) * px) : font->hmi;
    if ((advance <= 0.0) && font->deskjet && (glyph->bits == NULL))
        advance = (font->glyphs[0x20].delta / 4.0) * px;
    if (advance <= 0.0)
        advance = font->hmi;

    dev->x += dev->hmi_set ? dev->hmi : advance;
}

static void
draw_face_glyph(pcl3_t *dev, uint8_t ch)
{
    const pcl3_fontsel_t *sel     = &dev->font[dev->font_active];
    double                natural = 0.0;

    update_face(dev);

    if (dev->face != NULL) {
        const FT_UInt index = FT_Get_Char_Index(dev->face, dev->symbol_map[ch]);

        if (!FT_Load_Glyph(dev->face, index, FT_LOAD_DEFAULT | FT_LOAD_TARGET_MONO) &&
            !FT_Render_Glyph(dev->face->glyph, FT_RENDER_MODE_MONO)) {
            const FT_GlyphSlot slot   = dev->face->glyph;
            const FT_Bitmap   *bitmap = &slot->bitmap;
            const int          pen_x  = (int) lround((dev->left_origin + dev->x) * dev->dpi) + slot->bitmap_left;
            const int          pen_y  = (int) lround((dev->top_margin + dev->y) * dev->dpi) - slot->bitmap_top;
            const int          bold   = (sel->weight > 0) ? (dev->dpi / 150) : 0;

            for (unsigned int y = 0; y < bitmap->rows; y++) {
                const uint8_t *src  = bitmap->buffer + ((int) y * bitmap->pitch);
                const int      py   = pen_y + (int) y;
                uint8_t       *line;

                if ((py < 0) || (py >= dev->h))
                    continue;

                line = dev->pixels + ((size_t) py * dev->w);

                for (unsigned int x = 0; x < bitmap->width; x++) {
                    if (!(src[x / 8] & (0x80 >> (x & 7))))
                        continue;

                    for (int b = 0; b <= bold; b++) {
                        const int px = pen_x + (int) x + b;

                        if ((px >= 0) && (px < dev->w))
                            line[px] |= dev->pen_ink;
                    }
                }
            }

            natural = (double) slot->advance.x / (64.0 * dev->dpi);

            if (bitmap->rows > 0)
                dev->dirty = true;
        }
    }

    if (natural <= 0.0)
        natural = 0.5 * sel->height / 72.0;

    dev->x += char_advance(dev, natural);
}

static void
line_feed(pcl3_t *dev)
{
    dev->y += dev->vmi;
    if ((dev->y + dev->top_margin) >= dev->page_height)
        eject_page(dev);
}

static void
carriage_return(pcl3_t *dev)
{
    dev->x = dev->left_margin;
}

static void
print_char(pcl3_t *dev, uint8_t ch)
{
    const pcl3_fontsel_t *sel  = &dev->font[dev->font_active];
    pcl3_softfont_t      *font = (sel->soft_id >= 0) ? find_softfont(dev, sel->soft_id, false) : NULL;
    pcl3_softfont_t      *cart = (sel->soft_id >= 0) ? NULL : find_cart_font(dev, sel);
    const double          from = dev->x;

    if (dev->wrap && (dev->x >= dev->right_margin)) {
        carriage_return(dev);
        line_feed(dev);
    }

    if (cart != NULL) {
        update_face(dev);
        if (cart->landscape)
            draw_cart_glyph(dev, cart, ch);
        else
            draw_soft_glyph(dev, cart, ch);
    } else if ((font != NULL) && (font->deskjet || (font->glyphs[ch].bits != NULL) || (font->glyphs[ch].delta != 0)))
        draw_soft_glyph(dev, font, ch);
    else
        draw_face_glyph(dev, ch);

    if (dev->underline)
        fill_rect(dev, dev->left_origin + from, dev->top_margin + dev->y + (3.0 / PCL_DPI),
                  dev->left_origin + dev->x, dev->top_margin + dev->y + (5.0 / PCL_DPI), dev->pen_ink);
}

static void
handle_text(pcl3_t *dev, uint8_t ch)
{
    if (dev->display_functions) {
        print_char(dev, ch);
        if (ch == 0x0d) {
            carriage_return(dev);
            line_feed(dev);
        }
        return;
    }

    switch (ch) {
        case 0x08: /* BS */
            dev->x -= char_advance(dev, 0.1);
            if (dev->x < dev->left_margin)
                dev->x = dev->left_margin;
            break;

        case 0x09: /* HT */
            {
                const double tab = 8.0 * char_advance(dev, 0.1);

                dev->x = dev->left_margin + (floor(((dev->x - dev->left_margin) / tab) + 1.0) * tab);
            }
            break;

        case 0x0a: /* LF */
            if (dev->line_termination & 2)
                carriage_return(dev);
            line_feed(dev);
            break;

        case 0x0c: /* FF */
            if (dev->line_termination & 2)
                carriage_return(dev);
            eject_page(dev);
            break;

        case 0x0d: /* CR */
            carriage_return(dev);
            if (dev->line_termination & 1)
                line_feed(dev);
            break;

        case 0x0e: /* SO */
            dev->font_active = 1;
            dev->face_valid  = false;
            break;

        case 0x0f: /* SI */
            dev->font_active = 0;
            dev->face_valid  = false;
            break;

        default:
            if (ch >= 0x20)
                print_char(dev, ch);
            break;
    }
}

static void
page_defaults(pcl3_t *dev)
{
    dev->left_origin      = 0.25;
    dev->top_margin       = 0.5;
    dev->left_margin      = 0.0;
    dev->right_margin     = dev->page_width - 0.5;
    dev->vmi              = 1.0 / 6.0;
    dev->hmi              = 0.1;
    dev->hmi_set          = false;
    dev->wrap             = true;
    dev->line_termination = 0;
    dev->underline        = false;
    dev->landscape        = false;
    dev->push_count       = 0;
    dev->res              = 75;
    dev->compression      = 0;
    dev->raster_width     = 0;
    dev->raster_x_offset  = 0;
    dev->raster_shift     = 0.0;
    dev->raster_active    = false;
    dev->rect_w           = 0.0;
    dev->rect_h           = 0.0;
    dev->pen_ink          = INK_K;
    dev->font_active      = 0;
    dev->face_valid       = false;

    for (int i = 0; i < 2; i++) {
        dev->font[i].symbol_set = 8 * 32 + 21;
        dev->font[i].spacing    = 0;
        dev->font[i].pitch      = 10.0;
        dev->font[i].height     = 12.0;
        dev->font[i].style      = 0;
        dev->font[i].weight     = 0;
        dev->font[i].typeface   = 3;
        dev->font[i].soft_id    = -1;
    }

    set_palette_planes(dev, 1);
    clear_seeds(dev);
    home_cursor(dev);
}

/* ESC E: ends the current page, and returns to the power-up settings. */
static void
printer_reset(pcl3_t *dev)
{
    if (dev->dirty)
        eject_page(dev);

    set_page_size(dev, dev->default_width, dev->default_height);
    page_defaults(dev);
}

static void
select_paper(pcl3_t *dev, int id)
{
    double width;
    double height;

    switch (abs(id)) {
        case 1:  width = EXECUTIVE_PAGE_WIDTH;     height = EXECUTIVE_PAGE_HEIGHT;     break;
        case 2:  width = LETTER_PAGE_WIDTH;        height = LETTER_PAGE_HEIGHT;        break;
        case 3:  width = LEGAL_PAGE_WIDTH;         height = LEGAL_PAGE_HEIGHT;         break;
        case 6:  width = LEDGER_PAGE_WIDTH;        height = LEDGER_PAGE_HEIGHT;        break;
        case 26: width = A4_PAGE_WIDTH;            height = A4_PAGE_HEIGHT;            break;
        case 27: width = A3_PAGE_WIDTH;            height = A3_PAGE_HEIGHT;            break;
        case 80: width = ENVELOPE_MONARCH_PAGE_WIDTH; height = ENVELOPE_MONARCH_PAGE_HEIGHT; break;
        case 81: width = ENVELOPE_NO10_PAGE_WIDTH; height = ENVELOPE_NO10_PAGE_HEIGHT; break;
        case 90: width = ENVELOPE_DL_PAGE_WIDTH;   height = ENVELOPE_DL_PAGE_HEIGHT;   break;
        case 91: width = ENVELOPE_C5_PAGE_WIDTH;   height = ENVELOPE_C5_PAGE_HEIGHT;   break;
        case 100: width = B5_PAGE_WIDTH;           height = B5_PAGE_HEIGHT;            break;
        default:
            return;
    }

    if (dev->dirty)
        eject_page(dev);

    set_page_size(dev, width, height);
    dev->right_margin = dev->page_width - 0.5;
    home_cursor(dev);
}

static void
move_cursor(double *pos, double amount, bool relative)
{
    if (relative)
        *pos += amount;
    else
        *pos = amount;
}

static void
cmd_ampersand(pcl3_t *dev, uint8_t group, uint8_t term, double value, bool relative)
{
    const int ival = (int) value;

    switch ((group << 8) | term) {
        case ('a' << 8) | 'C': /* cursor column */
            move_cursor(&dev->x, value * char_advance(dev, 0.1), relative);
            break;
        case ('a' << 8) | 'R': /* cursor row */
            move_cursor(&dev->y, (value * dev->vmi) + (relative ? 0.0 : (0.75 * dev->vmi)), relative);
            break;
        case ('a' << 8) | 'H': /* decipoints */
            move_cursor(&dev->x, value / 720.0, relative);
            break;
        case ('a' << 8) | 'V':
            move_cursor(&dev->y, value / 720.0, relative);
            break;
        case ('a' << 8) | 'L': /* left margin */
            dev->left_margin = value * char_advance(dev, 0.1);
            if (dev->x < dev->left_margin)
                dev->x = dev->left_margin;
            break;
        case ('a' << 8) | 'M': /* right margin */
            dev->right_margin = (value + 1.0) * char_advance(dev, 0.1);
            break;

        case ('d' << 8) | 'D': /* underline */
            dev->underline = true;
            break;
        case ('d' << 8) | '@':
            dev->underline = false;
            break;

        case ('f' << 8) | 'S': /* push or pop the cursor position */
            if ((ival == 0) && (dev->push_count < 20)) {
                dev->push_x[dev->push_count] = dev->x;
                dev->push_y[dev->push_count] = dev->y;
                dev->push_count++;
            } else if ((ival == 1) && (dev->push_count > 0)) {
                dev->push_count--;
                dev->x = dev->push_x[dev->push_count];
                dev->y = dev->push_y[dev->push_count];
            }
            break;

        case ('k' << 8) | 'G': /* line termination */
            dev->line_termination = ival & 3;
            break;
        case ('k' << 8) | 'H': /* HMI, in 1/120 inch */
            dev->hmi     = value / 120.0;
            dev->hmi_set = value > 0.0;
            break;

        case ('l' << 8) | 'A': /* page size */
            select_paper(dev, ival);
            break;
        case ('l' << 8) | 'C': /* VMI, in 1/48 inch */
            dev->vmi = value / 48.0;
            break;
        case ('l' << 8) | 'D': /* lines per inch */
            if (value > 0.0)
                dev->vmi = 1.0 / value;
            break;
        case ('l' << 8) | 'E': /* top margin, in lines */
            dev->top_margin = value * dev->vmi;
            home_cursor(dev);
            break;
        case ('l' << 8) | 'O': /* orientation */
            dev->landscape = ival == 1;
            break;
        case ('l' << 8) | 'H': /* paper source; 0 ejects the page */
            if (ival == 0)
                eject_page(dev);
            break;

        case ('s' << 8) | 'C': /* end of line wrap */
            dev->wrap = (ival == 0);
            break;

        default:
            break;
    }
}

static void
cmd_asterisk(pcl3_t *dev, uint8_t group, uint8_t term, double value, bool relative)
{
    const int ival = (int) value;

    switch ((group << 8) | term) {
        case ('p' << 8) | 'X':
            move_cursor(&dev->x, value / PCL_DPI, relative);
            break;
        case ('p' << 8) | 'Y':
            move_cursor(&dev->y, value / PCL_DPI, relative);
            break;

        case ('t' << 8) | 'R': /* raster resolution */
            if (ival > 0)
                dev->res = ival;
            break;

        case ('r' << 8) | 'A': /* start raster graphics */
            start_raster(dev, ival);
            break;
        case ('r' << 8) | 'C': /* end raster graphics, and reset its settings */
            dev->compression     = 0;
            dev->raster_x_offset = 0;
            fallthrough;
        case ('r' << 8) | 'B': /* end raster graphics */
            dev->raster_active = false;
            dev->curr_plane    = 0;
            break;
        case ('r' << 8) | 'S': /* raster width */
            dev->raster_width = (ival > 0) ? ival : 0;
            break;
        case ('r' << 8) | 'U': /* planes and palette */
            set_palette_planes(dev, ival);
            break;

        case ('b' << 8) | 'M': /* raster compression */
            dev->compression = ival;
            break;
        case ('b' << 8) | 'X': /* raster X offset, in dots */
            dev->raster_x_offset = ival;
            break;
        case ('b' << 8) | 'Y': /* raster Y offset, in rows */
            if (!dev->raster_active)
                start_raster(dev, 0);
            dev->y += (double) ival / (dev->plane_vres[0] ? dev->plane_vres[0] : dev->res);
            clear_seeds(dev);
            break;

        case ('c' << 8) | 'A': /* rectangle size, in dots */
            dev->rect_w = value / PCL_DPI;
            break;
        case ('c' << 8) | 'B':
            dev->rect_h = value / PCL_DPI;
            break;
        case ('c' << 8) | 'P': /* fill rectangle */
            if ((dev->rect_w > 0.0) && (dev->rect_h > 0.0))
                fill_rect(dev, dev->left_origin + dev->x, dev->top_margin + dev->y,
                          dev->left_origin + dev->x + dev->rect_w,
                          dev->top_margin + dev->y + dev->rect_h,
                          (ival == 1) ? 0x00 : dev->pen_ink);
            break;

        case ('c' << 8) | 'D': /* soft font ID */
            dev->font_id = ival;
            break;
        case ('c' << 8) | 'E': /* soft font character code */
            dev->char_code = ival;
            break;
        case ('c' << 8) | 'F': /* soft font control */
            if ((ival == 0) || (ival == 1)) {
                for (int i = 0; i < MAX_SOFTFONT; i++) {
                    if (dev->softfonts[i].id >= 0)
                        free_softfont(dev, &dev->softfonts[i]);
                }
            } else if (ival == 2) {
                pcl3_softfont_t *font = find_softfont(dev, dev->font_id, false);

                if (font != NULL)
                    free_softfont(dev, font);
            } else if ((ival == 3) && (dev->char_code >= 0) && (dev->char_code < 256)) {
                pcl3_softfont_t *font = find_softfont(dev, dev->font_id, false);

                if (font != NULL)
                    delete_glyph(&font->glyphs[dev->char_code]);
            }
            break;

        case ('v' << 8) | 'S': /* foreground color */
            if (dev->palette_planes == -4)
                dev->pen_ink = ((ival & 1) ? INK_K : 0) | ((ival & 2) ? INK_C : 0) |
                               ((ival & 4) ? INK_M : 0) | ((ival & 8) ? INK_Y : 0);
            else if (dev->palette_planes == -3)
                dev->pen_ink = ((ival & 1) ? INK_C : 0) | ((ival & 2) ? INK_M : 0) |
                               ((ival & 4) ? INK_Y : 0);
            else if (dev->palette_planes == 3)
                dev->pen_ink = ((ival & 1) ? 0 : INK_C) | ((ival & 2) ? 0 : INK_M) |
                               ((ival & 4) ? 0 : INK_Y);
            break;

        default:
            break;
    }
}

static void
cmd_font(pcl3_t *dev, uint8_t which, uint8_t group, uint8_t term, double value)
{
    pcl3_fontsel_t *sel = &dev->font[which];

    dev->face_valid = false;

    if (group == 0) {
        if (term == 'X') { /* select a soft font by its ID */
            sel->soft_id = (int) value;
        } else if (term != '@') { /* symbol set */
            sel->symbol_set = ((int) value * 32) + (term - 64);
            sel->soft_id    = -1;
        }
        return;
    }

    if (group != 's')
        return;

    sel->soft_id = -1;

    switch (term) {
        case 'P':
            sel->spacing = (int) value;
            break;
        case 'H':
            if (value > 0.0)
                sel->pitch = value;
            break;
        case 'V':
            if (value > 0.0)
                sel->height = value;
            break;
        case 'S':
            sel->style = (int) value;
            break;
        case 'B':
            sel->weight = (int) value;
            break;
        case 'T':
            sel->typeface = (int) value;
            break;
        default:
            break;
    }
}

/* Whether this command is followed by value bytes of binary data. */
static bool
takes_data(const pcl3_t *dev, uint8_t term)
{
    const uint8_t param = dev->param_char;
    const uint8_t group = dev->group_char;

    if ((param == '*') && (group == 'b'))
        return (term == 'W') || (term == 'V');
    if ((param == '*') && ((group == 'g') || (group == 'o') || (group == 'c')))
        return term == 'W';
    if (((param == '(') || (param == ')')) && (group == 's'))
        return term == 'W';
    if ((param == '&') && ((group == 'b') || (group == 'n')))
        return term == 'W';

    return false;
}

static void
finish_data(pcl3_t *dev)
{
    const uint8_t  param = dev->param_char;
    const uint8_t  group = dev->group_char;
    const uint32_t kept  = (dev->data_len > MAX_DATA) ? MAX_DATA : dev->data_len;

    if ((param == '*') && (group == 'b')) {
        /* The DeskJet 500 and 500C take no graphics on a landscape page. */
        if (!dev->landscape || dev->landscape_graphics) {
            if (!dev->raster_active)
                start_raster(dev, 0);

            decode_plane(dev, dev->data, kept);
            dev->curr_plane++;

            if (dev->data_term == 'W') {
                print_row(dev, dev->curr_plane);
                dev->curr_plane = 0;
            }
        }
    } else if ((param == '*') && (group == 'g'))
        configure_raster_data(dev, dev->data, kept);
    else if ((param == '*') && (group == 'o')) {
        /* Driver configuration, format 3 function 7: the A4 left offset of the
           DeskJet 520 and 560C. The driver sends 36 here, and the same move as 18
           dots at 300 dpi to the other models. Printers ignore all the others. */
        if ((kept >= 5) && (dev->data[0] == 3) && (dev->data[1] == 7)) {
            dev->raster_shift = (double) ((dev->data[3] << 8) | dev->data[4]) / 600.0;
            if (dev->data[2] == '-')
                dev->raster_shift = -dev->raster_shift;
        }
    } else if ((param == ')') && (group == 's'))
        store_font_descriptor(dev, dev->data, kept);
    else if ((param == '(') && (group == 's'))
        store_glyph(dev, dev->data, kept);

    if (dev->data_continues) {
        dev->state          = STATE_VALUE;
        dev->value_seen     = false;
        dev->value_negative = false;
        dev->value_signed   = false;
        dev->value_fraction = false;
        dev->value          = 0.0;
    } else
        dev->state = STATE_TEXT;
}

static void
run_command(pcl3_t *dev, uint8_t term)
{
    const double value    = dev->value_negative ? -dev->value : dev->value;
    const bool   relative = dev->value_signed;

    switch (dev->param_char) {
        case '&':
            cmd_ampersand(dev, dev->group_char, term, value, relative);
            break;
        case '*':
            cmd_asterisk(dev, dev->group_char, term, value, relative);
            break;
        case '(':
        case ')':
            cmd_font(dev, dev->param_char == ')', dev->group_char, term, dev->value);
            break;
        default:
            break;
    }
}

static void
handle_value(pcl3_t *dev, uint8_t ch)
{
    if ((ch >= '0') && (ch <= '9')) {
        dev->value_seen = true;
        if (dev->value_fraction) {
            dev->value_scale /= 10.0;
            dev->value += (ch - '0') * dev->value_scale;
        } else
            dev->value = (dev->value * 10.0) + (ch - '0');
        return;
    }

    if ((ch == '+') || (ch == '-')) {
        dev->value_signed   = true;
        dev->value_negative = (ch == '-');
        return;
    }

    if (ch == '.') {
        dev->value_fraction = true;
        dev->value_scale    = 1.0;
        return;
    }

    /* Anything else ends this parameter. Lower case continues the sequence. */
    if ((ch >= 0x60) && (ch <= 0x7e)) {
        const uint8_t term = ch - 0x20;

        if (takes_data(dev, term)) {
            dev->data_continues = true;
            dev->data_term      = term;
            dev->data_len       = (uint32_t) dev->value;
            dev->data_pos       = 0;
            if (dev->data_len == 0)
                finish_data(dev);
            else
                dev->state = STATE_DATA;
            return;
        }

        run_command(dev, term);
        dev->value_seen     = false;
        dev->value_negative = false;
        dev->value_signed   = false;
        dev->value_fraction = false;
        dev->value          = 0.0;
        return;
    }

    if ((ch >= 0x40) && (ch <= 0x5e)) {
        if (takes_data(dev, ch)) {
            dev->data_continues = false;
            dev->data_term      = ch;
            dev->data_len       = (uint32_t) dev->value;
            dev->data_pos       = 0;
            if (dev->data_len == 0)
                finish_data(dev);
            else
                dev->state = STATE_DATA;
            return;
        }

        if ((dev->param_char == '&') && (dev->group_char == 'p') && (ch == 'X')) {
            dev->transparent_left = (uint32_t) dev->value;
            dev->state            = dev->transparent_left ? STATE_TRANSPARENT : STATE_TEXT;
            return;
        }

        run_command(dev, ch);
        dev->state = STATE_TEXT;
        return;
    }

    /* Not a valid sequence. */
    dev->state = STATE_TEXT;
}

void
pcl3_write(void *priv, uint8_t ch)
{
    pcl3_t *dev = (pcl3_t *) priv;

    switch (dev->state) {
        case STATE_TEXT:
            if (ch == 0x1b)
                dev->state = STATE_ESC;
            else
                handle_text(dev, ch);
            break;

        case STATE_ESC:
            dev->state = STATE_TEXT;

            if ((ch >= 0x21) && (ch <= 0x2f)) {
                dev->param_char     = ch;
                dev->group_char     = 0;
                dev->value_seen     = false;
                dev->value_negative = false;
                dev->value_signed   = false;
                dev->value_fraction = false;
                dev->value          = 0.0;
                dev->state          = STATE_GROUP;
                break;
            }

            switch (ch) {
                case 'E': /* reset */
                    printer_reset(dev);
                    break;
                case '9': /* clear margins */
                    dev->left_margin  = 0.0;
                    dev->right_margin = dev->page_width - 0.5;
                    break;
                case '=': /* half line feed */
                    dev->y += dev->vmi / 2.0;
                    break;
                case 'Y': /* display functions */
                    dev->display_functions = true;
                    break;
                case 'Z':
                    dev->display_functions = false;
                    break;
                case '?': /* the driver pads its data with this */
                    dev->state = STATE_SWALLOW;
                    break;
                default:
                    break;
            }
            break;

        case STATE_GROUP:
            dev->state = STATE_VALUE;
            if ((ch >= 0x60) && (ch <= 0x7e))
                dev->group_char = ch;
            else
                handle_value(dev, ch);
            break;

        case STATE_VALUE:
            handle_value(dev, ch);
            break;

        case STATE_DATA:
            if (dev->data_pos < MAX_DATA)
                dev->data[dev->data_pos] = ch;
            if (++dev->data_pos >= dev->data_len)
                finish_data(dev);
            break;

        case STATE_TRANSPARENT:
            print_char(dev, ch);
            if (--dev->transparent_left == 0)
                dev->state = STATE_TEXT;
            break;

        case STATE_SWALLOW:
            dev->state = STATE_TEXT;
            break;

        default:
            dev->state = STATE_TEXT;
            break;
    }
}

/* Keep what has been printed if the data stops; the sheet stays in the printer. */
void
pcl3_flush(void *priv)
{
    pcl3_t *dev = (pcl3_t *) priv;

    if (dev->dirty)
        save_page(dev);
}

void
pcl3_reset(void *priv)
{
    pcl3_t *dev = (pcl3_t *) priv;

    dev->state = STATE_TEXT;
    printer_reset(dev);
}

void *
pcl3_open(double width, double height, int dpi)
{
    pcl3_t *dev = (pcl3_t *) calloc(1, sizeof(pcl3_t));

    if (dev == NULL)
        return NULL;

    dev->dpi                = dpi;
    dev->color              = true;
    dev->landscape_graphics = true;
    dev->deskjet_fonts      = true;
    dev->default_width  = width;
    dev->default_height = height;
    dev->data           = (uint8_t *) malloc(MAX_DATA);

    for (int i = 0; i < MAX_PLANES; i++)
        dev->seed[i] = (uint8_t *) calloc(1, MAX_ROW);

    for (int i = 0; i < MAX_SOFTFONT; i++)
        dev->softfonts[i].id = -1;

    if (ft_lib == NULL) {
        if (FT_Init_FreeType(&ft_lib))
            ft_lib = NULL;
    }

    rom_get_full_path(dev->font_path, "roms/printer/fonts/");
    path_slash(dev->font_path);

    path_append_filename(dev->page_path, usr_path, "printer");
    if (!plat_dir_check(dev->page_path))
        plat_dir_create(dev->page_path);
    path_slash(dev->page_path);

    /* One entry per combination of the C, M, Y and K ink bits; K covers all. */
    for (int i = 0; i < 256; i++) {
        dev->palette[i].r = (i & (INK_C | INK_K)) ? 0 : 255;
        dev->palette[i].g = (i & (INK_M | INK_K)) ? 0 : 255;
        dev->palette[i].b = (i & (INK_Y | INK_K)) ? 0 : 255;
    }

    set_page_size(dev, width, height);
    page_defaults(dev);

    return dev;
}

void
pcl3_set_model(void *priv, int model)
{
    pcl3_t *dev = (pcl3_t *) priv;

    dev->color              = (model == PCL3_MODEL_500C) || (model == PCL3_MODEL_540) ||
                              (model == PCL3_MODEL_550C) || (model == PCL3_MODEL_560C);
    dev->landscape_graphics = (model != PCL3_MODEL_500) && (model != PCL3_MODEL_500C) && (model != PCL3_MODEL_510);
    dev->deskjet_fonts      = model != PCL3_MODEL_540;
    /* The DeskJet 540 keeps soft fonts in its internal buffer, of an unknown size. */
    dev->ram_limited        = model != PCL3_MODEL_540;
}

void
pcl3_add_ram_cartridge(void *priv, int kilobytes)
{
    pcl3_t *dev = (pcl3_t *) priv;

    for (int i = 0; (i < (kilobytes / 64)) && (dev->ram_pages < MAX_RAM_PAGE); i++)
        dev->ram_free[dev->ram_pages++] = RAM_PAGE;
}

/* A set name such as 8U, as the text at offset 7 of a recreation ends with. */
static int
cart_symbol_set(const uint8_t *rom)
{
    int end = 37;
    int start;
    int value = 0;

    while ((end > 7) && (rom[end] == ' '))
        end--;
    start = end;
    while ((start > 7) && (rom[start - 1] >= '0') && (rom[start - 1] <= '9'))
        start--;
    if ((start == end) || (rom[end] < 'A') || (rom[end] > 'Z'))
        return -1;
    for (int i = start; i < end; i++)
        value = (value * 10) + (rom[i] - '0');

    return (value * 32) + (rom[end] - 64);
}

/* The name field is followed by sizes in text: the first is the pitch. */
static double
cart_pitch(const uint8_t *descriptor)
{
    int pitch = 0;

    for (int i = 0x3C; i < 0x41; i++) {
        if ((descriptor[i] >= '0') && (descriptor[i] <= '9'))
            pitch = (pitch * 10) + (descriptor[i] - '0');
    }

    return ((pitch >= 5) && (pitch <= 30)) ? pitch : 12.0;
}

static uint16_t
cart_word(const uint8_t *rom, uint32_t at)
{
    return rom[at] | (rom[at + 1] << 8);
}

/* Recreated cartridges have a block of their own after the four words of a font, as
   nothing is known of how HP stores portrait or proportional fonts. */
#define CART_EXT_SIZE 26

static void
load_cart_ext(pcl3_softfont_t *font, const uint8_t *ext)
{
    const uint8_t flags = ext[4];
    const int     hmi   = cart_word(ext, 24);

    font->landscape    = (flags & 1) != 0;
    font->proportional = (flags & 2) != 0;
    font->bold         = (flags & 4) != 0;
    font->italic       = (flags & 8) != 0;
    font->passes       = ext[5];
    font->format       = 12;
    font->pool         = false;
    font->pitch        = cart_word(ext, 14) / 100.0;
    font->height       = cart_word(ext, 16) / 100.0;
    font->typeface     = cart_word(ext, 18);
    font->symbol_set   = cart_word(ext, 20);
    font->hmi          = hmi / 1200.0;

    for (int i = 0; i < 4; i++)
        font->pass_base[i] = cart_word(ext, 6 + (i * 2));

    font->baseline = font->pass_base[0];
}

/* The bytes of one pass of a character: a flag byte per column, then its data. */
static uint32_t
cart_pass_size(const uint8_t *rom, uint32_t from, int width, uint32_t to)
{
    uint32_t size = width;

    for (int i = 0; (i < width) && ((from + i) < to); i++) {
        for (int box = 0; box < DJ_BOXES; box++)
            size += (rom[from + i] >> box) & 1;
    }

    return size;
}

/* Returns how much of the cartridge from this part the font takes up. */
static uint32_t
load_cart_font(pcl3_t *dev, const uint8_t *rom, uint32_t limit, uint32_t record)
{
    const uint32_t descriptor = cart_word(rom, record);
    const uint32_t widths     = cart_word(rom, record + 2);
    const uint32_t offsets    = cart_word(rom, record + 4);
    const uint32_t pads       = cart_word(rom, record + 6);
    const uint32_t ext_at     = cart_word(rom, record + 8);
    const bool     real       = memcmp(rom + 7, "RECREATION ", 11) != 0;
    const bool     ext        = !real && ((ext_at + CART_EXT_SIZE) <= CART_BANK) && !memcmp(rom + ext_at, "RX1", 4);
    uint32_t       rights     = 0;
    uint32_t       scale      = 1; /* very large fonts count in 16 byte steps */
    uint32_t       end        = 0;
    uint8_t       *block;
    pcl3_softfont_t *font;
    char           name[17];

    if (ext) {
        rights = cart_word(rom, ext_at + 22);
        if (rom[ext_at + 4] & 0x10)
            scale = 16;
    }

    if ((dev->cart_fonts >= MAX_CART_FONT) || ((descriptor + 0x54) > CART_BANK) ||
        ((widths + 256) > CART_BANK) || ((offsets + 514) > CART_BANK) || ((pads + 256) > CART_BANK) ||
        ((rights + 256) > CART_BANK))
        return 0;

    font  = (pcl3_softfont_t *) calloc(1, sizeof(pcl3_softfont_t));
    block = (uint8_t *) malloc(8 + CART_BANK);
    if ((font == NULL) || (block == NULL)) {
        free(font);
        free(block);
        return 0;
    }

    /* Its characters are the same as those of a landscape soft font of format 9. */
    font->deskjet      = true;
    font->landscape    = true;
    font->format       = 9;
    font->passes       = 1;
    font->baseline     = CART_BASELINE;
    font->pass_base[0] = CART_BASELINE;
    font->res_x        = 600.0;
    font->res_y        = 300.0;
    font->pool         = real;
    font->bold         = cart_word(rom, descriptor + 10) != 0;
    font->pitch        = cart_pitch(rom + descriptor);
    font->hmi          = 1.0 / font->pitch;
    font->symbol_set   = real ? -1 : cart_symbol_set(rom);

    memcpy(name, rom + descriptor + 0x2A, 16);
    name[16]       = 0;
    font->typeface = strstr(name, "Letter Gothic") ? 6 : strstr(name, "Courier") ? 3 : -1;

    if (ext) {
        load_cart_ext(font, rom + ext_at);

        /* Tall cells would make for huge characters. */
        if ((font->passes < 1) || (font->passes > 4) || (font->hmi <= 0.0)) {
            free(font);
            free(block);
            return 0;
        }
        for (int i = 0; i < font->passes; i++) {
            if ((font->pass_base[i] < 1) || (font->pass_base[i] > DJ_MAX_BASE) ||
                (i && (font->pass_base[i] > font->pass_base[i - 1]))) {
                free(font);
                free(block);
                return 0;
            }
        }
    }

    for (int i = 0; i < 256; i++) {
        uint32_t       from  = cart_word(rom, offsets + (i * 2)) * scale;
        const uint32_t to    = cart_word(rom, offsets + (i * 2) + 2) * scale;
        const uint8_t  width = rom[widths + i];
        pcl3_glyph_t  *glyph = &font->glyphs[real ? i : ((i + 0x20) & 0xFF)];

        if ((width == 255) || (!ext && ((width == 0) || (to <= from))) || (to > limit))
            continue;

        if (to > end)
            end = to;

        for (int pass = 0; pass < font->passes; pass++) {
            const uint32_t size = cart_pass_size(rom, from, width, to);

            if (((from + size) > to) || (size > CART_BANK))
                break;

            block[0] = font->format;
            block[1] = 0;
            block[2] = 6;
            block[3] = (font->passes > 1) ? (pass + 2) : 0;
            block[4] = width;
            block[5] = rom[pads + i];
            block[6] = rom[pads + i];
            block[7] = ext ? rom[rights + i] : 0;
            memcpy(block + 8, rom + from, size);
            store_deskjet_glyph(font, glyph, block, 8 + size);
            from += size;
        }
    }

    free(block);
    dev->carts[dev->cart_fonts++] = font;

    return end;
}

/* A cartridge is 64 KB parts, each with a list of font records from 0x26. */
void
pcl3_load_cartridge(void *priv, const char *fn)
{
    pcl3_t  *dev = (pcl3_t *) priv;
    FILE    *fp  = fopen(fn, "rb");
    uint8_t *rom;
    size_t   size;

    if (fp == NULL)
        return;

    rom = (uint8_t *) malloc(CART_BANK * MAX_CART_BANK);
    if (rom == NULL) {
        fclose(fp);
        return;
    }

    size = fread(rom, 1, CART_BANK * MAX_CART_BANK, fp);
    fclose(fp);

    for (uint32_t bank = 0; (bank + CART_BANK) <= size;) {
        const uint8_t *part  = rom + bank;
        const uint32_t first = cart_word(part, 0x26);
        uint32_t       used  = CART_BANK;

        for (uint32_t list = 0x26; (list < first) && (list < 0x100); list += 2) {
            const uint32_t record = cart_word(part, list);

            if (record == 0)
                break;
            if ((record + 10) <= CART_BANK) {
                const uint32_t end = load_cart_font(dev, part, (uint32_t) (size - bank), record);

                if (end > used)
                    used = end;
            }
        }

        bank += (used + CART_BANK - 1) & ~(CART_BANK - 1);
    }

    free(rom);
}

void
pcl3_close(void *priv)
{
    pcl3_t *dev = (pcl3_t *) priv;

    if (dev->dirty)
        save_page(dev);

    if (dev->face != NULL)
        FT_Done_Face(dev->face);

    for (int i = 0; i < MAX_SOFTFONT; i++) {
        if (dev->softfonts[i].id >= 0)
            delete_softfont(&dev->softfonts[i]);
    }

    for (int i = 0; i < dev->cart_fonts; i++) {
        delete_softfont(dev->carts[i]);
        free(dev->carts[i]);
    }

    for (int i = 0; i < MAX_PLANES; i++)
        free(dev->seed[i]);

    free(dev->data);
    free(dev->pixels);
    free(dev);
}
