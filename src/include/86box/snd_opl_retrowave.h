/*
 * 86Box	A hypervisor and IBM PC system emulator that specializes in
 *		running old operating systems and software designed for IBM
 *		PC systems and compatibles from 1981 through fairly recent
 *		system designs based on the PCI bus.
 *
 *		This file is part of the 86Box distribution.
 *
 *		Definitions for the RetroWave OPL driver.
 *
 * Version:	@(#)snd_opl_retrowave.h	1.0.0	2021/12/22
 *
 * Authors:	Fred N. van Kempen, <decwiz@yahoo.com>
 *		Miran Grca, <mgrca8@gmail.com>
 *
 *		Copyright 2017-2020 Fred N. van Kempen.
 *		Copyright 2016-2019 Miran Grca.
 */
#ifndef SOUND_OPL_RETROWAVE_H
# define SOUND_OPL_RETROWAVE_H

typedef struct retrowave_drv_s {
    void    *opl;
    int8_t    flags, pad;

    uint16_t    port;
    uint8_t    status, timer_ctrl;
    uint16_t    timer_count[2],
        timer_cur_count[2];

    pc_timer_t    timers[2];

    int        pos;
    int32_t    buffer[SOUNDBUFLEN * 2];
} retrowave_drv_t;

enum {
    FLAG_CYCLES = 0x02,
    FLAG_OPL3   = 0x01
};

enum {
    STAT_TMR_OVER  = 0x60,
    STAT_TMR1_OVER = 0x40,
    STAT_TMR2_OVER = 0x20,
    STAT_TMR_ANY   = 0x80
};

enum {
    CTRL_RESET      = 0x80,
    CTRL_TMR_MASK   = 0x60,
    CTRL_TMR1_MASK  = 0x40,
    CTRL_TMR2_MASK  = 0x20,
    CTRL_TMR2_START = 0x02,
    CTRL_TMR1_START = 0x01
};

extern void *retrowave_86box_module_init(void);
extern void	retrowave_close(void *);

extern uint16_t	retrowave_write_addr(void *, uint16_t port, uint8_t val);
extern void	retrowave_write_reg(void *, uint16_t reg, uint8_t v);

extern void	retrowave_generate_stream(void *, int32_t *sndptr, uint32_t num);

#endif	/*SOUND_OPL_RETROWAVE_H*/
