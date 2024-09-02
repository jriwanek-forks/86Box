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


extern void *	retrowave_86box_module_init(void);
extern void	retrowave_close(void *);

extern uint16_t	retrowave_write_addr(void *, uint16_t port, uint8_t val);
extern void	retrowave_write_reg(void *, uint16_t reg, uint8_t v);

extern void	retrowave_generate_stream(void *, int32_t *sndptr, uint32_t num);

#endif	/*SOUND_OPL_RETROWAVE_H*/
