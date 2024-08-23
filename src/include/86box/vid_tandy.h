#ifndef VIDEO_TANDY_H
#define VIDEO_TANDY_H

typedef struct t1kvid_t {
    mem_mapping_t mapping;
    mem_mapping_t vram_mapping;

    uint8_t crtc[32];
    int     crtcreg;

    int     array_index;
    uint8_t array[256];
    int     memctrl;
    uint8_t mode;
    uint8_t col;
    uint8_t stat;

    uint8_t *vram;
    uint8_t *b8000;
    uint32_t b8000_mask;
    uint32_t b8000_limit;
    uint8_t  planar_ctrl;

    int      linepos;
    int      displine;
    int      sc;
    int      vc;
    int      dispon;
    int      con;
    int      coff;
    int      cursoron;
    int      blink;
    int      fullchange;
    int      vsynctime;
    int      vadj;
    uint16_t ma;
    uint16_t maback;

    uint64_t   dispontime;
    uint64_t   dispofftime;
    pc_timer_t timer;
    int        firstline;
    int        lastline;

    int composite;
} t1kvid_t;

#ifdef EMU_DEVICE_H
extern const device_t tandy_vid_device;
extern const device_t tandy_vid_device_hx;
extern const device_t tandy_vid_device_sl;
#endif

static void tandy_vid_init(tandy_t *dev);
static void tandy_recalc_address(tandy_t *dev);
static void tandy_recalc_address_sl(tandy_t *dev);
static void tandy_vid_out(uint16_t addr, uint8_t val, void *priv);
static uint8_t tandy_vid_in(uint16_t addr, void *priv);
static void tandy_vid_write(uint32_t addr, uint8_t val, void *priv);
static uint8_t tandy_vid_read(uint32_t addr, void *priv);

#endif /*VIDEO_TANDY_H*/
