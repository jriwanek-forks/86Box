/* Copyright holders: Sarah Walker, Tenshi, SA1988
   see COPYING for more details
*/
#ifndef __IDE__
#define __IDE__

typedef struct __attribute__((__packed__)) IDE
{
        int type;
        int board;
        uint8_t atastat;
        uint8_t error;
        int secount,sector,cylinder,head,drive,cylprecomp;
        uint8_t command;
        uint8_t fdisk;
        int pos;
        int packlen;
        int spt,hpc;
        int tracks;
        int packetstatus;
        uint8_t asc;
        int reset;
        FILE *hdfile;
        uint16_t buffer[65536];
        int irqstat;
        int service;
        int lba;
		int channel;
        uint32_t lba_addr;
        int skip512;
        int blocksize, blockcount;
		uint16_t dma_identify_data[3];
		int hdi,base;
		int hdc_num;
		uint8_t specify_success;
} IDE;

extern void writeide(int ide_board, uint16_t addr, uint8_t val);
extern void writeidew(int ide_board, uint16_t val);
extern uint8_t readide(int ide_board, uint16_t addr);
extern uint16_t readidew(int ide_board);
extern void callbackide(int ide_board);
extern void resetide(void);
extern void ide_init(void);
extern void ide_ter_init(void);
extern void ide_qua_init(void);
extern void ide_pri_enable(void);
extern void ide_sec_enable(void);
extern void ide_ter_enable(void);
extern void ide_qua_enable(void);
extern void ide_pri_disable(void);
extern void ide_sec_disable(void);
extern void ide_ter_disable(void);
extern void ide_qua_disable(void);
extern void ide_set_bus_master(int (*read)(int channel, uint8_t *data, int transfer_length), int (*write)(int channel, uint8_t *data, int transfer_length), void (*set_irq)(int channel));

extern int ideboard;

extern int ide_enable[4];
extern int ide_irq[4];

extern int idecallback[4];

extern char ide_fn[IDE_NUM][512];

void ide_irq_raise(IDE *ide);
void ide_irq_lower(IDE *ide);

IDE ide_drives[IDE_NUM];

void ide_padstr8(uint8_t *buf, int buf_size, const char *src);

void win_cdrom_eject(uint8_t id);
void win_cdrom_reload(uint8_t id);

#endif //__IDE__
