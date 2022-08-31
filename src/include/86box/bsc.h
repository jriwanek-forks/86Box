/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Definitons for IBM BSC emulation.
 *
 * Authors: Jasmine Iwanek, <jasmine@iwanek.co.uk>
 *
 *          Copyright 2022-2025 Jasmine Iwanek.
 */

#ifndef EMU_BSC_H
#define EMU_BSC_H

#define BSC_FIFO_SIZE 16

/* Default settings for the ports. */
#define BSC1_ADDR 0x03a0
#define BSC1_IRQ  4
#define BSC2_ADDR 0x0380
#define BSC2_IRQ  3

struct bsc_device_s;
struct bsc_s;

typedef struct bsc_s {
    uint8_t lsr;
    uint8_t thr;
    uint8_t mctrl;
    uint8_t rcr;
    uint8_t iir;
    uint8_t ier;
    uint8_t lcr;
    uint8_t msr;
    uint8_t dat;
    uint8_t int_status;
    uint8_t scratch;
    uint8_t fcr;
    uint8_t irq;
    uint8_t type;
    uint8_t inst;
    uint8_t transmit_enabled;
    uint8_t fifo_enabled;
    uint8_t rcvr_fifo_len;
    uint8_t bits;
    uint8_t data_bits;
    uint8_t baud_cycles;
    uint8_t rcvr_fifo_full;
    uint8_t txsr, pad;

    uint16_t dlab;
    uint16_t base_address;

    uint8_t rcvr_fifo_pos;
    uint8_t xmit_fifo_pos;
    uint8_t pad0;
    uint8_t pad1;
    uint8_t rcvr_fifo[BSC_FIFO_SIZE];
    uint8_t xmit_fifo[BSC_FIFO_SIZE];

    pc_timer_t transmit_timer;
    pc_timer_t timeout_timer;
    double     clock_src;
    double     transmit_period;

    struct bsc_device_s *bd;
} bsc_t;

typedef struct bsc_device_s {
    void (*rcr_callback)(struct bsc_s *bsc, void *priv);
    void (*dev_write)(struct bsc_s *bsc, void *priv, uint8_t data);
    void  *priv;
    bsc_t *bsc;
} bsc_device_t;

typedef struct {
    uint8_t enabled;
} bsc_port_t;

extern bsc_port_t	bsc_ports[BSC_MAX];

extern bsc_t *bsc_attach(int port,
                         void (*rcr_callback)(struct bsc_s *bsc, void *priv),
                         void (*dev_write)(struct bsc_s *bsc, void *priv, uint8_t data),
                         void *priv);
#if 0
extern void      bsc_remove(bsc_t *dev);
#endif
extern void      bsc_set_type(bsc_t *dev, int type);
extern void      bsc_setup(bsc_t *dev, uint16_t addr, uint8_t irq);
extern void      bsc_clear_fifo(bsc_t *dev);
extern void      bsc_write_fifo(bsc_t *dev, uint8_t dat);
extern void      bsc_set_next_inst(int ni);
extern void      bsc_standalone_init(void);
extern void      bsc_set_clock_src(bsc_t *dev, double clock_src);
#if 0
extern void      bsc_reset_port(bsc_t *dev);
#endif

extern const device_t ibm_bsc_device;

#endif /*EMU_BSC_H*/
