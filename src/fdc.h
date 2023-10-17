/* Copyright holders: Sarah Walker, Tenshi
   see COPYING for more details
*/
void fdc_init(void);
void fdc_add(void);
void fdc_add_for_superio(void);
void fdc_add_pcjr(void);
void fdc_add_tandy(void);
void fdc_remove(void);
void fdc_reset(void);
void fdc_poll(void);
void fdc_abort(void);
void fdc_discchange_clear(int drive);
void fdc_set_dskchg_activelow(void);
void fdc_3f1_enable(int enable);
void fdc_set_ps1(void);
int fdc_get_bit_rate(void);
int fdc_get_bitcell_period(void);

/* A few functions to communicate between Super I/O chips and the FDC. */
void fdc_update_is_nsc(int is_nsc);
void fdc_update_max_track(int max_track);
void fdc_update_enh_mode(int enh_mode);
int fdc_get_rwc(int drive);
void fdc_update_rwc(int drive, int rwc);
int fdc_get_boot_drive(void);
void fdc_update_boot_drive(int boot_drive);
void fdc_update_densel_polarity(int densel_polarity);
uint8_t fdc_get_densel_polarity(void);
void fdc_update_densel_force(int densel_force);
void fdc_update_drvrate(int drive, int drvrate);
void fdc_update_drv2en(int drv2en);

void fdc_noidam(void);
void fdc_nosector(void);
void fdc_nodataam(void);
void fdc_cannotformat(void);
void fdc_wrongcylinder(void);
void fdc_badcylinder(void);

sector_id_t fdc_get_read_track_sector(void);
int fdc_get_compare_condition(void);
int fdc_is_deleted(void);
int fdc_is_sk(void);
void fdc_set_wrong_am(void);
int fdc_get_drive(void);
int fdc_get_perp(void);
int fdc_get_format_n();
int fdc_is_mfm(void);
double fdc_get_hut(void);
double fdc_get_hlt(void);
void fdc_request_next_sector_id(void);
void fdc_stop_id_request(void);
int fdc_get_gap(void);
int fdc_get_gap2(int drive);
int fdc_get_dtl(void);
int fdc_get_format_sectors(void);

void fdc_finishcompare(int satisfying);
void fdc_finishread(void);
void fdc_sector_finishcompare(int satisfying);
void fdc_sector_finishread(void);
void fdc_track_finishread(int condition);
int fdc_is_verify(void);

int real_drive(int drive);
