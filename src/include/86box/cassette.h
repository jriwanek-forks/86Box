/*****************************************************************************
 * pce                                                                       *
 *****************************************************************************/

/*****************************************************************************
 * File name:   src/ibmpc/cassette.h                                         *
 * Created:     2008-11-25 by Hampa Hug <hampa@hampa.ch>                     *
 * Copyright:   (C) 2008-2019 Hampa Hug <hampa@hampa.ch>                     *
 *****************************************************************************/

/*****************************************************************************
 * This program is free software. You can redistribute it and / or modify it *
 * under the terms of the GNU General Public License version 2 as  published *
 * by the Free Software Foundation.                                          *
 *                                                                           *
 * This program is distributed in the hope  that  it  will  be  useful,  but *
 * WITHOUT  ANY   WARRANTY,   without   even   the   implied   warranty   of *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU  General *
 * Public License for more details.                                          *
 *****************************************************************************/

#ifndef PCE_IBMPC_CASSETTE_H
#define PCE_IBMPC_CASSETTE_H 1

#include <stdio.h>

typedef struct pc_cassette_t {
    char save;
    char pcm;

    unsigned char motor;

    uint64_t      position;

    uint64_t      position_save;
    uint64_t      position_load;

    unsigned char data_out;
    unsigned char data_inp;

    uint32_t      pcm_out_vol;
    uint32_t      pcm_out_val;

    uint32_t      cas_out_cnt;
    unsigned char cas_out_buf;

    uint32_t      cas_inp_cnt;
    unsigned char cas_inp_buf;
    unsigned char cas_inp_bit;

    int32_t       pcm_inp_fir[3];

    uint64_t      clk;

    uint64_t      clk_pcm;

    uint64_t      clk_out;
    uint64_t      clk_inp;

    uint64_t      srate;

    char          close;
    char         *fname;
    FILE         *fp;
    pc_timer_t    timer;
} pc_cassette_t;

void pc_cas_init(pc_cassette_t *cas);
void pc_cas_free(pc_cassette_t *cas);

pc_cassette_t *pc_cas_new(void);
void           pc_cas_del(pc_cassette_t *cas);

/*!***************************************************************************
 * @short  Set the cassette file
 * @return True on error, false otherwise
 *****************************************************************************/
uint32_t pc_cas_set_fname(pc_cassette_t *cas, const char *fname);

/*!***************************************************************************
 * @short  Get the cassette mode
 * @return True if in save mode, false if in load mode
 *****************************************************************************/
uint32_t pc_cas_get_mode(const pc_cassette_t *cas);

/*!***************************************************************************
 * @short Set the cassette mode
 * @param save If true set save mode, otherwise set load mode
 *****************************************************************************/
void pc_cas_set_mode(pc_cassette_t *cas, uint32_t save);

/*!***************************************************************************
 * @short  Get the cassette pcm mode
 * @return True if in pcm mode, false if in binary mode
 *****************************************************************************/
uint32_t pc_cas_get_pcm(const pc_cassette_t *cas);

/*!***************************************************************************
 * @short Set the cassette pcm mode
 * @param pcm If true set pcm mode, otherwise set binary mode
 *****************************************************************************/
void pc_cas_set_pcm(pc_cassette_t *cas, uint32_t pcm);

/*!***************************************************************************
 * @short  Get the pcm sample rate
 * @return The sample rate in Hz
 *****************************************************************************/
uint64_t pc_cas_get_srate(const pc_cassette_t *cas);

/*!***************************************************************************
 * @short Set the pcm sample rate
 * @param pcm The sample rate in Hz
 *****************************************************************************/
void pc_cas_set_srate(pc_cassette_t *cas, uint64_t srate);

/*!***************************************************************************
 * @short Rewind the cassette
 *****************************************************************************/
void pc_cas_rewind(pc_cassette_t *cas);

/*!***************************************************************************
 * @short Fast forward to the end of the cassette
 *****************************************************************************/
void pc_cas_append(pc_cassette_t *cas);

/*!***************************************************************************
 * @short Get the current load/save position
 *****************************************************************************/
uint64_t pc_cas_get_position(const pc_cassette_t *cas);

/*!***************************************************************************
 * @short Set the current load/save position
 *****************************************************************************/
uint32_t pc_cas_set_position(pc_cassette_t *cas, uint64_t pos);

/*!***************************************************************************
 * @short Set the cassette motor status
 *****************************************************************************/
void pc_cas_set_motor(pc_cassette_t *cas, unsigned char val);

/*!***************************************************************************
 * @short Get the current input from the cassette
 *****************************************************************************/
unsigned char pc_cas_get_inp(const pc_cassette_t *cas);

/*!***************************************************************************
 * @short Set the current output to the cassette
 *****************************************************************************/
void pc_cas_set_out(pc_cassette_t *cas, unsigned char val);

void pc_cas_print_state(const pc_cassette_t *cas);

void pc_cas_clock(pc_cassette_t *cas, uint64_t cnt);
void pc_cas_advance(pc_cassette_t *cas);

extern pc_cassette_t *cassette;

extern char     cassette_fname[512];
extern char     cassette_mode[512];
extern uint64_t cassette_pos;
extern uint64_t cassette_srate;
extern uint32_t cassette_enable;
extern uint32_t cassette_append;
extern uint32_t cassette_pcm;
extern uint32_t cassette_ui_writeprot;

extern const device_t cassette_device;

#endif /*PCE_IBMPC_CASSETTE_H*/
