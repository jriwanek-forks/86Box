/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Definitions for the command line interface.
 *
 *
 *
 * Authors: RichardG, <richardg867@gmail.com>
 *
 *          Copyright 2021-2025 RichardG.
 */
#ifndef EMU_CLI_H
#define EMU_CLI_H

#define CLI_RENDER_OUTPUT stderr

#ifdef USE_CLI
#    define CLI_RENDER_MAX_LINES    60
#    define CLI_RENDER_FB_SIZE      150
#    define CLI_RENDER_ANSIBUF_SIZE 4096 /* good for a fully packed SVGA 150-column line with some margin */
#    define CLI_RENDER_GFXBUF_W     (2048 + 64)
#    define CLI_RENDER_GFXBUF_H     (2048 + 64)

enum {
    /* No color capability. */
    TERM_COLOR_NONE = 0,
    /* 8 ANSI colors. */
    TERM_COLOR_3BIT = 3,
    /* 8 ANSI colors in dark and bright variants. */
    TERM_COLOR_4BIT = 4,
    /* xterm 256-color palette. */
    TERM_COLOR_8BIT = 8,
    /* True color with arbitrary RGB values. */
    TERM_COLOR_24BIT = 24
};

enum {
    /* SGR 6 provides a faster blink rate, more in line with IBM PC
       video cards, where supported. We can't enable both 5 and 6
       simultaneously, as they don't cancel each other out on mintty
       and possibly other terminals, resulting in irregular blinking. */
    TERM_CTL_RAPIDBLINK = 0x01,
    /* Printing through aux port CSIs. */
    TERM_CTL_PRINT = 0x02
};

enum {
    /* DEC Sixel graphics. */
    TERM_GFX_SIXEL = 0x01,
    /* PNG image rendering through the iTerm2 protocol. */
    TERM_GFX_PNG = 0x02,
    /* PNG image rendering through the kitty protocol. */
    TERM_GFX_PNG_KITTY = 0x04
};

typedef struct {
    uint8_t color_level, ctl_level, gfx_level,
        can_input, can_utf8, cpr, decrqss_color,
        kitty_input, size_x, size_y;
    unsigned int decrqss_cursor, sixel_color_regs;

    int (*setcolor)(char *p, uint8_t index, uint8_t is_background);
} cli_term_t;

/* cli.c */
extern cli_term_t cli_term;

/* cli_render.c */
extern const uint8_t cga_ansi_palette[];

/* video.c */
extern int cli_blit;

/* cli.c */
extern void cli_term_setcolor(uint8_t level, char *source);
extern void cli_term_setsize(int size_x, int size_y, char *source);
extern void cli_term_updatesize(int runtime);

extern void cli_init(void);
extern void cli_close(void);

/* cli_input.c */
extern void cli_input_init(void);
extern void cli_input_close(void);

/* cli_render.c */
extern void cli_render_blank(void);
extern void cli_render_gfx(char *str);
extern void cli_render_gfx_box(char *str);
#    ifdef EMU_VIDEO_H
extern void cli_render_gfx_blit(bitmap_t *bitmap, int x, int y, int w, int h);
#    endif
extern void cli_render_cga(uint8_t cy, uint8_t rowcount,
                           int xlimit, int xinc,
                           const uint8_t *fb, uint32_t fb_base, uint32_t fb_mask, uint8_t fb_step,
                           uint8_t do_render, uint8_t do_blink,
                           uint32_t ca, uint8_t con);
extern void cli_render_mda(int xlimit, uint8_t rowcount,
                           const uint8_t *fb, uint16_t fb_base,
                           uint8_t do_render, uint8_t do_blink,
                           uint16_t ca, uint8_t con);

extern void cli_render_write(char *s);
extern void cli_render_write_title(wchar_t *s);

extern void cli_render_monitorenter(void);
extern void cli_render_monitorexit(void);

extern int  cli_render_setcolor_none(char *p, uint8_t index, uint8_t is_background);
extern void cli_render_setcolorlevel(void);
extern void cli_render_setpal(uint8_t index, uint32_t color);
extern void cli_render_updatescreen(void);

extern void cli_render_process_screenshot(char *path, uint32_t *buf, int start_x, int start_y, int w, int h, int row_len);

extern void cli_render_init(void);
extern void cli_render_close(void);
#endif

enum {
    VT_SHIFT      = 0x01,
    VT_ALT        = 0x02,
    VT_CTRL       = 0x04,
    VT_SUPER      = 0x08, /* Meta on xterm */
    VT_HYPER      = 0x10,
    VT_META       = 0x20,
    VT_CAPSLOCK   = 0x40,
    VT_NUMLOCK    = 0x80,
    VT_MODS_ONLY  = VT_SHIFT | VT_ALT | VT_CTRL | VT_SUPER | VT_HYPER | VT_META | VT_CAPSLOCK | VT_NUMLOCK,
    VT_SHIFT_FAKE = 0x100,
    VT_KEY_DOWN   = 0x200,
    VT_KEY_UP     = 0x400
};

/* cli_input.c */
extern const uint16_t ascii_seqs[128];

extern void cli_input_send(uint16_t code, uint16_t modifier);

/* cli_monitor.c */
extern void cli_monitor_thread(void *priv);
extern void cli_monitor_init(int independent);
extern void cli_monitor_close(void);

#endif
