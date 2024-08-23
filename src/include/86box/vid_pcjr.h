#ifndef VIDEO_PCJR_H
#define VIDEO_PCJR_H

typedef struct pcjr_vid_t {
    /* Video Controller stuff. */
    mem_mapping_t mapping;
    uint8_t       crtc[32];
    int           crtcreg;
    int           array_index;
    uint8_t       array[32];
    int           array_ff;
    int           memctrl;
    uint8_t       stat;
    int           addr_mode;
    uint8_t      *vram;
    uint8_t      *b8000;
    int           linepos;
    int           displine;
    int           sc;
    int           vc;
    int           dispon;
    int           con;
    int           coff;
    int           cursoron;
    int           blink;
    int           vsynctime;
    int           fullchange;
    int           vadj;
    uint16_t      ma;
    uint16_t      maback;
    uint64_t      dispontime;
    uint64_t      dispofftime;
    pc_timer_t    timer;
    int           firstline;
    int           lastline;
    int           composite;
    pcjr_kbd_t    *kbd; /* Hack for now */
} pcjr_vid_t;

static video_timings_t timing_pcjr_dram = { VIDEO_BUS, 0, 0, 0, 0, 0, 0 }; /*No additional waitstates*/

#endif /*VIDEO_PCJR_H*/
