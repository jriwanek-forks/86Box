/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Emulation of the EGA and Chips & Technologies SuperEGA
 *          graphics cards.
 *
 * Authors: Sarah Walker, <https://pcem-emulator.co.uk/>
 *          Miran Grca, <mgrca8@gmail.com>
 *
 *          Copyright 2008-2020 Sarah Walker.
 *          Copyright 2016-2020 Miran Grca.
 */
#ifndef VIDEO_EGA_H
#define VIDEO_EGA_H

#if defined(EMU_MEM_H) && defined(EMU_ROM_H)
typedef struct ega_t {
    mem_mapping_t mapping;

    rom_t bios_rom;

    uint8_t crtcreg;
    uint8_t gdcaddr;
    uint8_t attraddr;
    uint8_t attrff;
    uint8_t attr_palette_enable;
    uint8_t seqaddr;
    uint8_t miscout;
    uint8_t writemask;
    uint8_t la;
    uint8_t lb;
    uint8_t lc;
    uint8_t ld;
    uint8_t status;
    uint8_t colourcompare;
    uint8_t colournocare;
    uint8_t scrblank;
    uint8_t plane_mask;
    uint8_t ctl_mode;
    uint8_t color_mux;
    uint8_t dot;
    uint8_t crtc[256];
    uint8_t gdcreg[256];
    uint8_t attrregs[32];
    uint8_t seqregs[64];
    uint8_t egapal[16];
    uint8_t regs[256];
    uint8_t *vram;

    uint16_t light_pen;

    int vidclock;
    int fast;
    int extvram;
    int vres;
    int readmode;
    int writemode;
    int readplane;
    int vrammask;
    int chain4;
    int chain2_read;
    int chain2_write;
    int cursorvisible;
    int oddeven_page;
    int oddeven_chain;
    int vc;
    int real_vc;
    int scanline;
    int dispon;
    int hdisp_on;
    int cursoron;
    int blink;
    int fullchange;
    int linepos;
    int vslines;
    int linecountff;
    int oddeven;
    int lowres;
    int interlace;
    int linedbl;
    int lindebl;
    int rowcount;
    int vtotal;
    int dispend;
    int vsyncstart;
    int split;
    int hdisp;
    int hdisp_old;
    int htotal;
    int hdisp_time;
    int rowoffset;
    int vblankstart;
    int scrollcache;
    int firstline;
    int lastline;
    int firstline_draw;
    int lastline_draw;
    int x_add;
    int y_add;
    int displine;
    int res_x;
    int res_y;
    int bpp;
    int index;
    int remap_required;
    int actual_type;
    int chipset;
    int mono_display;

    int mda_attr_to_color_table[256][2][2];

    uint32_t charseta;
    uint32_t charsetb;
    uint32_t memaddr_latch;
    uint32_t memaddr;
    uint32_t memaddr_backup;
    uint32_t cursoraddr;
    uint32_t vram_limit;
    uint32_t overscan_color;
    uint32_t cca;

    uint32_t *pallook;

    uint64_t   dispontime;
    uint64_t   dispofftime;

    uint64_t   dot_time;

    pc_timer_t timer;
    pc_timer_t dot_timer;

    double     dot_clock;

    void *     eeprom;

    uint32_t   (*remap_func)(struct ega_t *ega, uint32_t in_addr);
    void       (*render)(struct ega_t *svga);

    /* If set then another device is driving the monitor output and the EGA
      card should not attempt to display anything. */
    void       (*render_override)(void *priv);
    void *     priv_parent;

    uint8_t    alt_addr; /* 0 for 0x3XX range, 1 for 0x2XX range */

    /* Set when an MDA (or Hercules) shares the machine: the 3Bx ports and the
       B0000 window belong to it, so the EGA must never decode them. */
    uint8_t    mono_io_disabled;

    int        monitor_index; /* Monitor this card renders to. */

    /* Feature connector (see EGA_FEATURE_* below). */
    uint8_t    feature_card;  /* Card plugged into the feature connector. */
    uint8_t    feat_ctrl;     /* FC0/FC1, written through the Feature Control register. */
    uint8_t    feat_in;       /* FEAT0/FEAT1, driven by the card, read back at 3C2h. */
    uint8_t    drivers_off;   /* Misc Output bit 4 set with nothing else driving a display. */
    double     ext_clock;     /* Dot clock supplied by the card, 0 if none. */
    void      *pal;           /* PAL encoder/decoder state (ega_pal_t), NULL if unused. */
} ega_t;
#endif

#ifdef EMU_DEVICE_H
extern const device_t ega_device;
extern const device_t cpqega_device;
extern const device_t sega_device;
extern const device_t atiega800p_device;
extern const device_t iskra_ega_device;
extern const device_t et2000_device;
extern const device_t jega_device;
extern const device_t jvga_device;
#endif

extern int update_overscan;

#define DISPLAY_RGB          0
#define DISPLAY_COMPOSITE    1
#define DISPLAY_RGB_NO_BROWN 2
#define DISPLAY_GREEN        3
#define DISPLAY_AMBER        4
#define DISPLAY_WHITE        5

/* IBM EGA configuration switch values, SW1 in bit 0 and SW4 in bit 3, with a set
   bit meaning the switch is OFF (open). This is the value the BIOS reads back
   through bit 4 of Input Status 0 (3C2h). */
#define EGA_SW_MDA_PRI_CD40   0x00 /* MDA primary,  EGA secondary on 5153 40x25   */
#define EGA_SW_MDA_PRI_CD80   0x01 /* MDA primary,  EGA secondary on 5153 80x25   */
#define EGA_SW_MDA_PRI_ECD_N  0x02 /* MDA primary,  EGA secondary on 5154 normal  */
#define EGA_SW_MDA_PRI_ECD_E  0x03 /* MDA primary,  EGA secondary on 5154 enhanced*/
#define EGA_SW_CGA40_PRI_MONO 0x04 /* CGA 40x25 primary, EGA secondary on 5151    */
#define EGA_SW_CGA80_PRI_MONO 0x05 /* CGA 80x25 primary, EGA secondary on 5151    */
#define EGA_SW_CD40           0x06 /* EGA primary on 5153 40x25 (MDA optional)    */
#define EGA_SW_CD80           0x07 /* EGA primary on 5153 80x25 (MDA optional)    */
#define EGA_SW_ECD_NORMAL     0x08 /* EGA primary on 5154 normal (MDA optional)   */
#define EGA_SW_ECD_ENHANCED   0x09 /* EGA primary on 5154 enhanced (MDA optional) */
#define EGA_SW_MONO_CGA40     0x0a /* EGA primary on 5151, CGA secondary 40x25    */
#define EGA_SW_MONO           0x0b /* EGA primary on 5151 (CGA 80x25 optional)    */
#define EGA_SW_AUTO           0x10 /* Not a switch setting: derive from the setup */

/* Cards for the IBM EGA's 32-pin feature connector. */
#define EGA_FEATURE_NONE           0
#define EGA_FEATURE_132COL         1 /* External dot clock for 132-column modes */
#define EGA_FEATURE_MONO_COMP_W    2 /* RGBI to monochrome composite, white      */
#define EGA_FEATURE_MONO_COMP_G    3 /* RGBI to monochrome composite, green      */
#define EGA_FEATURE_MONO_COMP_A    4 /* RGBI to monochrome composite, amber      */
#define EGA_FEATURE_COLOR_COMP     5 /* RGBI to NTSC colour composite            */
#define EGA_FEATURE_COLOR_COMP_PAL 6 /* RGBI to PAL colour composite             */

#define EGA_FEATURE_IS_COMPOSITE(c) (((c) == EGA_FEATURE_COLOR_COMP) || ((c) == EGA_FEATURE_COLOR_COMP_PAL))

/* PAL colour subcarrier (Hz). Unlike NTSC it is not a sub-multiple of any EGA dot
   clock (14.318 MHz / 4.434 MHz = 3.229 samples per cycle), so the cross-colour
   patterns crawl from line to line and frame to frame instead of standing still. */
#define EGA_PAL_FSC                4433618.75

/* Dot clock of the 132-column feature card, selected by programming Misc Output
   bits 3-2 to 10b (external clock). No period card's documentation has turned up
   yet, so this is derived rather than measured: 16.257 MHz x 132 / 80, which gives
   a 132-column mode exactly the horizontal timing of the standard 80-column
   Enhanced Color Display modes once the CRTC horizontal registers are scaled by
   the same 132/80. Replace it with the crystal of a real card when one is known. */
#define EGA_FEATURE_132COL_CLOCK   26824050.0

#define EGA_SW_IS_MONO(sw)    (((sw) == EGA_SW_CGA40_PRI_MONO) || ((sw) == EGA_SW_CGA80_PRI_MONO) || \
                               ((sw) == EGA_SW_MONO_CGA40) || ((sw) == EGA_SW_MONO))

#if defined(EMU_MEM_H) && defined(EMU_ROM_H)
extern void ega_init(ega_t *ega, int monitor_type, int is_mono);
extern void ega_recalctimings(struct ega_t *ega);
extern void ega_recalc_remap_func(struct ega_t *ega);
#endif

extern void    ega_out(uint16_t addr, uint8_t val, void *priv);
extern uint8_t ega_in(uint16_t addr, void *priv);
extern void    ega_poll(void *priv);
extern void    ega_write(uint32_t addr, uint8_t val, void *priv);
extern uint8_t ega_read(uint32_t addr, void *priv);
extern void    ega_set_type(void *priv, uint32_t local);

extern int firstline_draw;
extern int lastline_draw;
extern int displine;
extern int scanline;

extern uint32_t memaddr;
extern uint32_t cursoraddr;
extern int      cursorvisible;
extern int      cursoron;
extern int      cgablink;

extern int scrollcache;

extern uint8_t edatlookup[4][4];
extern uint8_t egaremap2bpp[256];

#if defined(EMU_MEM_H) && defined(EMU_ROM_H)
void ega_render_blank(ega_t *ega);

void ega_render_overscan_left(ega_t *ega);
void ega_render_overscan_right(ega_t *ega);

void ega_render_text(ega_t *ega);
void ega_render_graphics(ega_t *ega);
#endif

enum {
  EGA_IBM = 0,
  EGA_COMPAQ,
  EGA_SUPEREGA,
  EGA_ATI800P,
  EGA_ISKRA,
  EGA_TSENG
};

enum {
  EGA_TYPE_IBM    = 0,
  EGA_TYPE_OTHER  = 1,
  EGA_TYPE_COMPAQ = 2
};

#endif /*VIDEO_EGA_H*/
