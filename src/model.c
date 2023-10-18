/* Copyright holders: Sarah Walker
   see COPYING for more details
*/
#include <stdint.h>
#include <stdio.h>

#include "ibm.h"
#include "cdrom.h"
#include "cpu.h"
#include "mem.h"
#include "model.h"
#include "mouse.h"
#include "io.h"
#include "rom.h"

// #include "acer386sx.h"
#include "acerm3a.h"
#include "ali1429.h"
#include "amstrad.h"
#include "compaq.h"
#include "device.h"
#include "disc.h"
#include "dma.h"
#include "fdc.h"
#include "fdc37c665.h"
#include "fdc37c932fr.h"
#include "gameport.h"
#include "headland.h"
#include "i430fx.h"
#include "i430hx.h"
#include "i430lx.h"
#include "i430nx.h"
#include "i430vx.h"
#include "i440fx.h"
#include "i82335.h"
#include "ide.h"
#include "intel.h"
#include "intel_flash.h"
#include "jim.h"
#include "keyboard_amstrad.h"
#include "keyboard_at.h"
#include "keyboard_olim24.h"
#include "keyboard_pcjr.h"
#include "keyboard_xt.h"
#include "lpt.h"
#include "memregs.h"
#include "neat.h"
#include "nmi.h"
#include "nvr.h"
#include "olivetti_m24.h"
#include "opti495.h"
#include "pc87306.h"
#include "pci.h"
#include "pic.h"
#include "piix.h"
#include "pit.h"
#include "ps1.h"
#include "ps2.h"
#include "scat.h"
#include "serial.h"
#include "sis496.h"
#include "sis85c471.h"
#include "sio.h"
#include "sound_ps1.h"
#include "sound_pssj.h"
#include "sound_sn76489.h"
#include "tandy_eeprom.h"
#include "tandy_rom.h"
#include "um8669f.h"
// #include "um8881f.h"
#include "vid_pcjr.h"
#include "vid_tandy.h"
#include "w83877f.h"
#include "wd76c10.h"
#include "xtide.h"

void            xt_init(void);
void          pcjr_init(void);
void       tandy1k_init(void);
void    tandy1ksl2_init(void);
void           ams_init(void);
void        europc_init(void);
void        olim24_init(void);
void            at_init(void);
void    deskpro386_init(void);
void     ps1_m2011_init(void);
void     ps1_m2121_init(void);
void   ps2_m30_286_init(void);
void       at_neat_init(void);
void       at_scat_init(void);
// void  at_acer386sx_init(void);
// void      at_82335_init(void);
void    at_wd76c10_init(void);
void    at_ali1429_init(void);
void   at_headland_init(void);
void    at_opti495_init(void);
// void    at_um8881f_init(void);
void     at_sis496_init(void);
void     at_i430vx_init(void);
void     at_batman_init(void);
void   at_endeavor_init(void);

void     at_dtk486_init(void);
void       at_r418_init(void);
void     at_586mc1_init(void);
void      at_plato_init(void);
void     at_mb500n_init(void);
#if 0
void at_powermate_v_init(void);
#endif
void   at_p54tp4xe_init(void);
void    at_acerm3a_init(void);
void   at_acerv35n_init(void);
void    at_p55t2p4_init(void);
void    at_p55tvp4_init(void);
// void       at_marl_init(void);
void      at_p55va_init(void);
void     at_i440fx_init(void);

int model;

int AMSTRAD, AT, PCI, TANDY;

PCI_RESET pci_reset_handler;

MODEL models[] =
{
        {"IBM PC",			ROM_IBMPC,		"ibmpc",		{ "",      cpus_8088,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, 0,                     64, 640,  64,          xt_init, NULL},
        {"IBM XT",			ROM_IBMXT,		"ibmxt",		{ "",      cpus_8088,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, 0,                     64, 640,  64,          xt_init, NULL},
        {"IBM PCjr",			ROM_IBMPCJR,		"ibmpcjr",		{ "",      cpus_pcjr,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 1, 0,                    128, 640, 128,        pcjr_init, &pcjr_device},
        {"Generic XT clone",		ROM_GENXT,		"genxt",		{ "",      cpus_8088,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, 0,                     64, 640,  64,          xt_init, NULL},
        {"AMI XT clone",		ROM_AMIXT,		"amixt",		{ "",      cpus_8088,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, 0,                     64, 640,  64,          xt_init, NULL},
        {"DTK XT clone",		ROM_DTKXT,		"dtk",			{ "",      cpus_8088,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, 0,                     64, 640,  64,          xt_init, NULL},        
        {"VTech Laser Turbo XT",	ROM_LTXT,		"ltxt",			{ "",      cpus_8088,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, 0,                     64, 640,  64,          xt_init, NULL},
	{"VTech Laser XT3",		ROM_LXT3,		"lxt3",			{ "",      cpus_8088,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, 0,                     64, 640,  64,          xt_init, NULL},
        {"Phoenix XT clone",		ROM_PXXT,		"pxxt",			{ "",      cpus_8088,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, 0,                     64, 640,  64,          xt_init, NULL},
        {"Juko XT clone",		ROM_JUKOPC,		"jukopc",		{ "",      cpus_8088,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, 0,                     64, 640,  64,          xt_init, NULL},
        {"Tandy 1000",			ROM_TANDY,		"tandy",		{ "",      cpus_8088,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 1, 0,                    128, 640, 128,     tandy1k_init, &tandy1000_device},
        {"Tandy 1000 HX",		ROM_TANDY1000HX,	"tandy1000hx",		{ "",      cpus_8088,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 1, 0,                    256, 640, 128,     tandy1k_init, &tandy1000hx_device},
        {"Tandy 1000 SL/2",		ROM_TANDY1000SL2,	"tandy1000sl2",		{ "",      cpus_8086,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 1, 0,                    512, 768, 128,  tandy1ksl2_init, NULL},
        {"Amstrad PC1512",		ROM_PC1512,		"pc1512",		{ "",      cpus_pc1512,      "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 1, MODEL_AMSTRAD,        512, 640, 128,         ams_init, NULL},
        {"Sinclair PC200",		ROM_PC200,		"pc200",		{ "",      cpus_8086,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 1, MODEL_AMSTRAD,        512, 640, 128,         ams_init, NULL},
        {"Euro PC",			ROM_EUROPC,		"europc",		{ "",      cpus_8086,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, 0,                    512, 640, 128,      europc_init, NULL},
        {"Olivetti M24",		ROM_OLIM24,		"olivetti_m24",		{ "",      cpus_8086,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 1, MODEL_OLIM24,         128, 640, 128,      olim24_init, NULL},
        {"Amstrad PC1640",		ROM_PC1640,		"pc1640",		{ "",      cpus_8086,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 1, MODEL_AMSTRAD,        640, 640,   0,         ams_init, NULL},
        {"Amstrad PC2086",		ROM_PC2086,		"pc2086",		{ "",      cpus_8086,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 1, MODEL_AMSTRAD,        640, 640,   0,         ams_init, NULL},
        {"Amstrad PC3086",		ROM_PC3086,		"pc3086",		{ "",      cpus_8086,        "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 1, MODEL_AMSTRAD,        640, 640,   0,         ams_init, NULL},
        {"IBM AT",			ROM_IBMAT,		"ibmat",		{ "",      cpus_ibmat,       "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, MODEL_AT,               1,  16,   1,          at_init, NULL},
        {"Commodore PC 30 III",		ROM_CMDPC30,		"cmdpc30",		{ "",      cpus_286,         "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, MODEL_AT,               1,  16,   1,          at_init, NULL},
        {"AMI 286 clone",		ROM_AMI286,		"ami286",		{ "",      cpus_286,         "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, MODEL_AT,               1,  16,   1,     at_neat_init, NULL},        
        {"Award 286 clone",		ROM_AWARD286,		"award286",		{ "",      cpus_286,         "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, MODEL_AT,               1,  16,   1,     at_scat_init, NULL},
        {"DELL System 200",		ROM_DELL200,		"dells200",		{ "",      cpus_286,         "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, MODEL_AT,               1,  16,   1,          at_init, NULL},
        {"Hyundai Super-286TR",		ROM_SUPER286TR,		"super286tr",		{ "",      cpus_286,         "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, MODEL_AT,               1,  16,   1,     at_scat_init, NULL},
        {"Samsung SPC-4200P",		ROM_SPC4200P,		"spc4200p",		{ "",      cpus_286,         "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, MODEL_AT,               1,  16,   1,     at_scat_init, NULL},
        {"IBM PS/1 model 2011",		ROM_IBMPS1_2011,	"ibmps1es",		{ "",      cpus_ps1_m2011,   "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 1, MODEL_AT | MODEL_PS2,   1,  16,   1,   ps1_m2011_init, NULL},
        {"IBM PS/2 Model 30-286",	ROM_IBMPS2_M30_286,	"ibmps2_m30_286",	{ "",      cpus_ps2_m30_286, "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, MODEL_AT | MODEL_PS2,   1,  16,   1, ps2_m30_286_init, NULL},
        {"IBM PS/1 model 2121",		ROM_IBMPS1_2121,	"ibmps1_2121",		{ "Intel", cpus_i386SX,      "AMD", cpus_Am386SX, "Cyrix", cpus_486SLC, "",      NULL,     "",      NULL}, 1, MODEL_AT | MODEL_PS2,   1,  16,   1,   ps1_m2121_init, NULL},
        {"IBM PS/1 m.2121 + ISA",	ROM_IBMPS1_2121_ISA,	"ibmps1_2121_isa",	{ "Intel", cpus_i386DX,      "AMD", cpus_Am386DX, "Cyrix", cpus_486DLC, "",      NULL,     "",      NULL}, 0, MODEL_AT | MODEL_PS2,   1,  16,   1,   ps1_m2121_init, NULL},
        {"Compaq Deskpro 386",		ROM_DESKPRO_386,	"dekspro386",		{ "Intel", cpus_i386DX,      "AMD", cpus_Am386DX, "Cyrix", cpus_486DLC, "",      NULL,     "",      NULL}, 0, MODEL_AT,               1,  15,   1,  deskpro386_init, NULL},
        {"DTK 386SX clone",		ROM_DTK386,		"dtk386",		{ "Intel", cpus_i386SX,      "AMD", cpus_Am386SX, "Cyrix", cpus_486SLC, "",      NULL,     "",      NULL}, 0, MODEL_AT,               1,  16,   1,     at_neat_init, NULL},
        {"Amstrad MegaPC",		ROM_MEGAPC,		"megapc",		{ "Intel", cpus_i386SX,      "AMD", cpus_Am386SX, "Cyrix", cpus_486SLC, "",      NULL,     "",      NULL}, 1, MODEL_AT | MODEL_PS2,   1,  16,   1,  at_wd76c10_init, NULL},
        {"AMI 386SX clone",		ROM_AMI386SX,		"ami386",		{ "Intel", cpus_i386SX,      "AMD", cpus_Am386SX, "Cyrix", cpus_486SLC, "",      NULL,     "",      NULL}, 0, MODEL_AT,               1, 256,  1,  at_headland_init, NULL},
        {"Commodore PC 60 III",		ROM_CMDPC60,		"cmdpc60",		{ "Intel", cpus_i386DX,      "AMD", cpus_Am386DX, "Cyrix", cpus_486DLC, "",      NULL,     "",      NULL}, 0, MODEL_AT,               1,  16,   1,          at_init, NULL},
/* The MegaPC manual says 386DX model of the Amstrad PC70386 exists, but Sarah Walker just *had* to remove 386DX CPU's from some boards. */
        {"Amstrad MegaPC 386DX",	ROM_MEGAPCDX,		"megapcdx",		{ "Intel", cpus_i386DX,      "AMD", cpus_Am386DX, "Cyrix", cpus_486DLC, "",      NULL,     "",      NULL}, 1, MODEL_AT | MODEL_PS2,   1,  16,  1,   at_wd76c10_init, NULL},
        {"MR 386DX clone",		ROM_MR386DX_OPTI495,	"mr386dx",		{ "Intel", cpus_i386DX,      "AMD", cpus_Am386DX, "Cyrix", cpus_486DLC, "",      NULL,     "",      NULL}, 0, MODEL_AT,               1, 256,  1,   at_opti495_init, NULL},
        {"AMI 386DX clone",		ROM_AMI386DX_OPTI495,	"ami386dx",		{ "Intel", cpus_i386DX,      "AMD", cpus_Am386DX, "Cyrix", cpus_486DLC, "",      NULL,     "",      NULL}, 0, MODEL_AT,               1, 256,  1,   at_opti495_init, NULL},
        {"AMI 486 clone",		ROM_AMI486,		"ami486",		{ "Intel", cpus_i486,        "AMD", cpus_Am486,   "Cyrix", cpus_Cx486,  "",      NULL,     "",      NULL}, 0, MODEL_AT,               1, 256,  1,   at_ali1429_init, NULL},
        {"AMI WinBIOS 486",		ROM_WIN486,		"win486",		{ "Intel", cpus_i486,        "AMD", cpus_Am486,   "Cyrix", cpus_Cx486,  "",      NULL,     "",      NULL}, 0, MODEL_AT,               1, 256,  1,   at_ali1429_init, NULL},
        {"DTK PKM-0038S E-2",		ROM_DTK486,		"dtk486",		{ "Intel", cpus_i486,        "AMD", cpus_Am486,   "Cyrix", cpus_Cx486,  "",      NULL,     "",      NULL}, 0, MODEL_AT,               1, 256,  1,    at_dtk486_init, NULL},
        {"Award SiS 496/497",		ROM_SIS496,		"sis496",		{ "Intel", cpus_i486,        "AMD", cpus_Am486,   "Cyrix", cpus_Cx486,  "",      NULL,     "",      NULL}, 0, MODEL_AT,               1, 256,  1,    at_sis496_init, NULL},
        {"Rise Computer R418",		ROM_R418,		"r418",			{ "Intel", cpus_i486,        "AMD", cpus_Am486,   "Cyrix", cpus_Cx486,  "",      NULL,     "",      NULL}, 0, MODEL_AT,               1, 256,  1,      at_r418_init, NULL},
        {"Intel Premiere/PCI",		ROM_REVENGE,		"revenge",		{ "Intel", cpus_Pentium5V,   "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, MODEL_AT | MODEL_PS2,   1, 128,  1,    at_batman_init, NULL},
        {"Micro Star 586MC1",		ROM_586MC1,		"586mc1",		{ "Intel", cpus_Pentium5V50, "",    NULL,          "",     NULL,        "",      NULL,     "",      NULL}, 0, MODEL_AT,               1, 128,  1,    at_586mc1_init, NULL},
        {"Intel Premiere/PCI II",	ROM_PLATO,		"plato",		{ "Intel", cpus_PentiumS5,   "IDT", cpus_WinChip, "AMD",   cpus_K5,     "",      NULL,     "",      NULL}, 0, MODEL_AT | MODEL_PS2,   1, 128,  1,     at_plato_init, NULL},
        {"Intel Advanced/EV",		ROM_ENDEAVOR,		"endeavor",		{ "Intel", cpus_PentiumS5,   "IDT", cpus_WinChip, "AMD",   cpus_K5,     "",      NULL,     "",      NULL}, 0, MODEL_AT | MODEL_PS2,   1, 128,  1,  at_endeavor_init, NULL},
        {"Intel Advanced/ZP",		ROM_ZAPPA,		"zappa",		{ "Intel", cpus_PentiumS5,   "IDT", cpus_WinChip, "AMD",   cpus_K5,     "",      NULL,     "",      NULL}, 0, MODEL_AT | MODEL_PS2,   1, 128,  1,  at_endeavor_init, NULL},
        {"PC Partner MB500N",		ROM_MB500N,		"mb500n",		{ "Intel", cpus_PentiumS5,   "IDT", cpus_WinChip, "AMD",   cpus_K5,     "",      NULL,     "",      NULL}, 0, MODEL_AT | MODEL_PS2,   1, 128,  1,    at_mb500n_init, NULL},
        {"Intel Advanced/ATX",		ROM_THOR,		"thor",			{ "Intel", cpus_Pentium,     "IDT", cpus_WinChip, "Cyrix", cpus_6x86,   "AMD",   cpus_K56, "",      NULL}, 0, MODEL_AT | MODEL_PS2,   1, 256,  1,  at_endeavor_init, NULL},
        {"MR Intel Advanced/ATX",	ROM_MRTHOR,		"mrthor",		{ "Intel", cpus_Pentium,     "IDT", cpus_WinChip, "Cyrix", cpus_6x86,   "AMD",   cpus_K56, "",      NULL}, 0, MODEL_AT | MODEL_PS2,   1, 256,  1,  at_endeavor_init, NULL},
        {"ASUS P/I-P54TP4XE",		ROM_P54TP4XE,		"p54tp4xe",		{ "Intel", cpus_PentiumS5,   "IDT", cpus_WinChip, "AMD",   cpus_K5,     "",      NULL,     "",      NULL}, 0, MODEL_AT | MODEL_PS2,   1, 512,  1,  at_p54tp4xe_init, NULL},
        {"Acer M3a",			ROM_ACERM3A,		"acerm3a",		{ "Intel", cpus_Pentium,     "IDT", cpus_WinChip, "Cyrix", cpus_6x86,   "AMD",   cpus_K56, "",      NULL}, 0, MODEL_AT | MODEL_PS2,   1, 512,  1,   at_acerm3a_init, NULL},
        {"Acer V35N",			ROM_ACERV35N,		"acerv3n",		{ "Intel", cpus_Pentium,     "IDT", cpus_WinChip, "Cyrix", cpus_6x86,   "AMD",   cpus_K56, "",      NULL}, 0, MODEL_AT | MODEL_PS2,   1, 512,  1,  at_acerv35n_init, NULL},
        {"ASUS P/I-P55T2P4",		ROM_P55T2P4,		"p55r2p4",		{ "Intel", cpus_Pentium,     "IDT", cpus_WinChip, "Cyrix", cpus_6x86,   "AMD",   cpus_K56, "",      NULL}, 0, MODEL_AT | MODEL_PS2,   1, 512,  1,   at_p55t2p4_init, NULL},
        {"Award 430VX PCI",		ROM_430VX,		"430vx",		{ "Intel", cpus_Pentium,     "IDT", cpus_WinChip, "Cyrix", cpus_6x86,   "AMD",   cpus_K56, "",      NULL}, 0, MODEL_AT | MODEL_PS2,   1, 256,  1,    at_i430vx_init, NULL},
        {"Epox P55-VA",			ROM_P55VA,		"p55va",		{ "Intel", cpus_Pentium,     "IDT", cpus_WinChip, "Cyrix", cpus_6x86,   "AMD",   cpus_K56, "",      NULL}, 0, MODEL_AT | MODEL_PS2,   1, 256,  1,     at_p55va_init, NULL},
        {"ASUS P/I-P55TVP4",		ROM_P55TVP4,		"p55tvp4",		{ "Intel", cpus_Pentium,     "IDT", cpus_WinChip, "Cyrix", cpus_6x86,   "AMD",   cpus_K56, "",      NULL}, 0, MODEL_AT | MODEL_PS2,   1, 256,  1,   at_p55tvp4_init, NULL},
        {"Award 440FX PCI",		ROM_440FX,		"440fx",		{ "Intel", cpus_PentiumPro,  "",    NULL,         "",      NULL,        "",      NULL,     "",      NULL}, 0, MODEL_AT | MODEL_PS2,   1, 1024, 1,    at_i440fx_init, NULL},
        {"", -1, "", {"", 0, "", 0, "", 0}, 0,0,0, 0}
};

int model_count(void)
{
        return (sizeof(models) / sizeof(MODEL)) - 1;
}

int model_getromset(void)
{
        return models[model].id;
}

int model_getmodel(int romset)
{
	int c = 0;
	
	while (models[c].id != -1)
	{
		if (models[c].id == romset)
			return c;
		c++;
	}
	
	return 0;
}

char *model_getname(void)
{
        return models[model].name;
}


device_t *model_getdevice(int model)
{
        return models[model].device;
}

char *model_get_internal_name(void)
{
        return models[model].internal_name;
}

int model_get_model_from_internal_name(char *s)
{
	int c = 0;
	
	while (models[c].id != -1)
	{
		if (!strcmp(models[c].internal_name, s))
			return c;
		c++;
	}
	
	return 0;
}

void common_init(void)
{
        dma_init();
        fdc_add();
        lpt_init();
        pic_init();
        pit_init();
        serial1_init(0x3f8, 4);
        serial2_init(0x2f8, 3);
}

void xt_init(void)
{
        common_init();
	mem_add_bios();
        pit_set_out_func(1, pit_refresh_timer_xt);
        keyboard_xt_init();
        xtide_init();
	nmi_init();
	if (joystick_type != 7)  device_add(&gameport_device);
}

void pcjr_init(void)
{
	mem_add_bios();
        fdc_add_pcjr();
        pic_init();
        pit_init();
        pit_set_out_func(0, pit_irq0_timer_pcjr);
        serial1_init(0x2f8, 3);
        keyboard_pcjr_init();
        device_add(&sn76489_device);
	nmi_mask = 0x80;
}

void tandy1k_init(void)
{
        TANDY = 1;
        common_init();
	mem_add_bios();
        keyboard_tandy_init();
        if (romset == ROM_TANDY)
                device_add(&sn76489_device);
        else
                device_add(&ncr8496_device);
        xtide_init();
	nmi_init();
	if (romset != ROM_TANDY)
                device_add(&tandy_eeprom_device);
	if (joystick_type != 7)  device_add(&gameport_device);
}
void tandy1ksl2_init(void)
{
//        TANDY = 1;
        common_init();
	mem_add_bios();
        keyboard_tandy_init();
        device_add(&pssj_device);
        xtide_init();
	nmi_init();
        device_add(&tandy_rom_device);
        device_add(&tandy_eeprom_device);
	if (joystick_type != 7)  device_add(&gameport_device);
}

void ams_init(void)
{
        AMSTRAD = 1;
        common_init();
	mem_add_bios();
        amstrad_init();
        keyboard_amstrad_init();
        nvr_init();
        xtide_init();
	nmi_init();
	fdc_set_dskchg_activelow();
	if (joystick_type != 7)  device_add(&gameport_device);
}

void europc_init(void)
{
        common_init();
	mem_add_bios();
        jim_init();
        keyboard_xt_init();
        xtide_init();
	nmi_init();
	if (joystick_type != 7)  device_add(&gameport_device);
}

void olim24_init(void)
{
        common_init();
	mem_add_bios();
        keyboard_olim24_init();
        nvr_init();
        olivetti_m24_init();
        xtide_init();
	nmi_init();
	if (joystick_type != 7)  device_add(&gameport_device);
}

void at_init(void)
{
	AT = 1;
        common_init();
	lpt2_remove();
	mem_add_bios();
        pit_set_out_func(1, pit_refresh_timer_at);
        dma16_init();
        ide_init();
        keyboard_at_init();
        nvr_init();
        pic2_init();
	if (joystick_type != 7)  device_add(&gameport_device);
}

void deskpro386_init(void)
{
        at_init();
        compaq_init();
}

void ps1_common_init(void)
{
        AT = 1;
        common_init();
	mem_add_bios();
        pit_set_out_func(1, pit_refresh_timer_at);
        dma16_init();
        ide_init();
        keyboard_at_init();
        nvr_init();
        pic2_init();
        fdc_set_dskchg_activelow();
        device_add(&ps1_audio_device);
        /*PS/1 audio uses ports 200h and 202-207h, so only initialise gameport on 201h*/
        if (joystick_type != 7)  device_add(&gameport_201_device);
}
 
void ps1_m2011_init(void)
{
        ps1_common_init();
        ps1mb_init();
}

void ps1_m2121_init(void)
{
        ps1_common_init();
        ps1mb_m2121_init();
        fdc_set_ps1();
}

void ps2_m30_286_init(void)
{
        AT = 1;
        common_init();
        mem_add_bios();
        pit_set_out_func(1, pit_refresh_timer_at);
        dma16_init();
        ide_init();
        keyboard_at_init();
        nvr_init();
        pic2_init();
        ps2board_init();
        fdc_set_dskchg_activelow();
        fdc_set_ps1();
}

void at_neat_init(void)
{
        at_init();
        neat_init();
}

void at_scat_init(void)
{
        at_init();
        scat_init();
}

/* void at_acer386sx_init(void)
{
        at_init();
        acer386sx_init();
}

void at_82335_init(void)
{
        at_init();
        i82335_init();
} */

void at_wd76c10_init(void)
{
        at_init();
        wd76c10_init();
}

void at_headland_init(void)
{
        at_init();
        headland_init();
}

void at_opti495_init(void)
{
        at_init();
        opti495_init();
}

void secondary_ide_check(void)
{
	int i = 0;
	int secondary_cdroms = 0;

	for (i = 0; i < CDROM_NUM; i++)
	{
		if ((cdrom_drives[i].ide_channel >= 2) && (cdrom_drives[i].ide_channel <= 3) && !cdrom_drives[i].bus_type)
		{
			secondary_cdroms++;
		}
		if (!secondary_cdroms)  ide_sec_disable();
	}
}

void at_ali1429_init(void)
{

        at_init();
        ali1429_init();

	secondary_ide_check();
}

/* void at_um8881f_init(void)
{
        at_init();
        pci_init(PCI_CONFIG_TYPE_1, 0, 31);
        um8881f_init();
} */

void at_dtk486_init(void)
{
        at_init();
	memregs_init();
	sis85c471_init();
	secondary_ide_check();
}

void at_sis496_init(void)
{
        at_init();
	memregs_init();
        pci_init(PCI_CONFIG_TYPE_1, 0, 31);
        device_add(&sis496_device);
}

void at_r418_init(void)
{
	at_sis496_init();
        fdc37c665_init();
}

void at_premiere_common_init(void)
{
        at_init();
	memregs_init();
        pci_init(PCI_CONFIG_TYPE_2, 0xd, 0x10);
 	sio_init(1);
        fdc37c665_init();
        intel_batman_init();
        device_add(&intel_flash_bxt_ami_device);
}

void at_batman_init(void)
{
	at_premiere_common_init();
        i430lx_init();
}

void at_586mc1_init(void)
{
        at_init();
	memregs_init();
        pci_init(PCI_CONFIG_TYPE_2, 0xd, 0x10);
        i430lx_init();
	sio_init(1);
        device_add(&intel_flash_bxt_device);
	secondary_ide_check();
}

void at_plato_init(void)
{
	at_premiere_common_init();
        i430nx_init();
}

void at_advanced_common_init(void)
{
        at_init();
	memregs_init();
        pci_init(PCI_CONFIG_TYPE_1, 0xd, 0x10);
        i430fx_init();
        piix_init(7);
        pc87306_init();
}

void at_endeavor_init(void)
{
        at_advanced_common_init();
        device_add(&intel_flash_bxt_ami_device);
}

#if 0
void at_marl_init(void)
{
        at_advanced_common_init();
        // device_add(&intel_flash_100bxt_ami_device);
}
#endif

void at_mb500n_init(void)
{
        at_init();
        pci_init(PCI_CONFIG_TYPE_1, 0xd, 0x10);
        i430fx_init();
        piix_init(7);
        fdc37c665_init();
        device_add(&intel_flash_bxt_device);
}

#if 0
void at_powermate_v_init(void)
{
        at_init();
	powermate_memregs_init();
        pci_init(PCI_CONFIG_TYPE_1, 0, 31);
        i430fx_init();
        piix_init(7);
        fdc37c665_init();
        acerm3a_io_init();
        device_add(&intel_flash_bxt_device);
}
#endif

void at_p54tp4xe_init(void)
{
        at_init();
	memregs_init();
        pci_init(PCI_CONFIG_TYPE_1, 0xd, 0x10);
        i430fx_init();
        piix_init(7);
        fdc37c665_init();
        device_add(&intel_flash_bxt_device);
}

void at_acerm3a_init(void)
{
        at_init();
	memregs_init();
	powermate_memregs_init();
        pci_init(PCI_CONFIG_TYPE_1, 0xd, 0x10);
        i430hx_init();
        piix3_init(7);
        fdc37c932fr_init();
        acerm3a_io_init();
        device_add(&intel_flash_bxb_device);
}

void at_acerv35n_init(void)
{
        at_init();
	memregs_init();
	powermate_memregs_init();
        pci_init(PCI_CONFIG_TYPE_1, 0xd, 0x10);
        i430hx_init();
        piix3_init(7);
        fdc37c932fr_init();
        acerm3a_io_init();
        device_add(&intel_flash_bxb_device);
}

void at_p55t2p4_init(void)
{
        at_init();
	memregs_init();
        pci_init(PCI_CONFIG_TYPE_1, 0, 31);
        i430hx_init();
        piix3_init(7);
        w83877f_init();
        device_add(&intel_flash_bxt_device);
}

void at_i430vx_init(void)
{
        at_init();
	memregs_init();
        pci_init(PCI_CONFIG_TYPE_1, 0, 31);
        i430vx_init();
        piix3_init(7);
        um8669f_init();
        device_add(&intel_flash_bxt_device);
}

void at_p55tvp4_init(void)
{
        at_init();
	memregs_init();
        pci_init(PCI_CONFIG_TYPE_1, 0, 31);
        i430vx_init();
        piix3_init(7);
        w83877f_init();
        device_add(&intel_flash_bxt_device);
}

void at_p55va_init(void)
{
        at_init();
	memregs_init();
        pci_init(PCI_CONFIG_TYPE_1, 0, 31);
        i430vx_init();
        piix3_init(7);
        fdc37c932fr_init();
        device_add(&intel_flash_bxt_device);
}

void at_i440fx_init(void)
{
        at_init();
	memregs_init();
        pci_init(PCI_CONFIG_TYPE_1, 0, 31);
        i440fx_init();
        piix3_init(7);
	fdc37c665_init();
        device_add(&intel_flash_bxt_device);
}

void model_init(void)
{
        pclog("Initting as %s\n", model_getname());
        AMSTRAD = AT = PCI = TANDY = 0;
        io_init();
        
	pci_reset_handler.pci_master_reset = NULL;
	pci_reset_handler.pci_set_reset = NULL;
	pci_reset_handler.super_io_reset = NULL;
	fdc_update_is_nsc(0);
        models[model].init();
        if (models[model].device)
                device_add(models[model].device);
}
