/* Copyright holders: Sarah Walker, Tenshi
   see COPYING for more details
*/
void d86f_init(void);
void d86f_load(int drive, char *fn);
void d86f_close(int drive);
void d86f_seek(int drive, int track);
int d86f_hole(int drive);
double d86f_byteperiod(int drive);
void d86f_stop(int drive);
void d86f_poll(int drive);
int d86f_realtrack(int track, int drive);
void d86f_reset(int drive, int side);
void d86f_readsector(int drive, int sector, int track, int side, int density, int sector_size);
void d86f_writesector(int drive, int sector, int track, int side, int density, int sector_size);
void d86f_comparesector(int drive, int sector, int track, int side, int rate, int sector_size);
void d86f_readaddress(int drive, int side, int density);
void d86f_format(int drive, int side, int density, uint8_t fill);

void d86f_prepare_track_layout(int drive, int side);

#define length_gap0	80
#define length_gap1	50
#define length_sync	12
#define length_am	4
#define length_crc	2

#define IBM
#define MFM
#ifdef IBM
#define pre_gap1	length_gap0 + length_sync + length_am
#else
#define pre_gap1	0
#endif
  
#define pre_track	pre_gap1 + length_gap1
#define pre_gap		length_sync + length_am + 4 + length_crc
#define pre_data	length_sync + length_am
#define post_gap	length_crc

#if 0
extern int raw_tsize[2];
extern int gap2_size[2];
extern int gap3_size[2];
extern int gap4_size[2];
#endif
