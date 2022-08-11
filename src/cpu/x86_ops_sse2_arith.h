#include <stdlib.h>
#include <math.h>

static int
opSQRTPD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = sqrt(XMM[cpu_rm].d[0]);
        XMM[cpu_reg].d[1] = sqrt(XMM[cpu_rm].d[1]);
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0]       = *(double *) &src[0];
        src_real[1]       = *(double *) &src[1];
        XMM[cpu_reg].d[0] = sqrt(src_real[0]);
        XMM[cpu_reg].d[1] = sqrt(src_real[1]);
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opSQRTPD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = sqrt(XMM[cpu_rm].d[0]);
        XMM[cpu_reg].d[1] = sqrt(XMM[cpu_rm].d[1]);
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0]       = *(double *) &src[0];
        src_real[1]       = *(double *) &src[1];
        XMM[cpu_reg].d[0] = sqrt(src_real[0]);
        XMM[cpu_reg].d[1] = sqrt(src_real[1]);
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opSQRTSD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = sqrt(XMM[cpu_rm].d[0]);
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real          = *(double *) &src;
        XMM[cpu_reg].d[0] = sqrt(src_real);
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opSQRTSD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = sqrt(XMM[cpu_rm].d[0]);
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real          = *(double *) &src;
        XMM[cpu_reg].d[0] = sqrt(src_real);
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opRSQRTPD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = 1.0 / sqrt(XMM[cpu_rm].d[0]);
        XMM[cpu_reg].d[1] = 1.0 / sqrt(XMM[cpu_rm].d[1]);
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0]       = *(double *) &src[0];
        src_real[1]       = *(double *) &src[1];
        XMM[cpu_reg].d[0] = 1.0 / sqrt(src_real[0]);
        XMM[cpu_reg].d[1] = 1.0 / sqrt(src_real[1]);
    }
    return 0;
}

static int
opRSQRTPD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = 1.0 / sqrt(XMM[cpu_rm].d[0]);
        XMM[cpu_reg].d[1] = 1.0 / sqrt(XMM[cpu_rm].d[1]);
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0]       = *(double *) &src[0];
        src_real[1]       = *(double *) &src[1];
        XMM[cpu_reg].d[0] = 1.0 / sqrt(src_real[0]);
        XMM[cpu_reg].d[1] = 1.0 / sqrt(src_real[1]);
    }
    return 0;
}

static int
opRSQRTSD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = 1.0 / sqrt(XMM[cpu_rm].d[0]);
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real          = *(double *) &src;
        XMM[cpu_reg].d[0] = 1.0 / sqrt(src_real);
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opRSQRTSD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = 1.0 / sqrt(XMM[cpu_rm].d[0]);
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real          = *(double *) &src;
        XMM[cpu_reg].d[0] = 1.0 / sqrt(src_real);
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opRCPSD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = 1.0 / XMM[cpu_rm].d[0];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real          = *(double *) &src;
        XMM[cpu_reg].d[0] = 1.0 / src_real;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opRCPSD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = 1.0 / XMM[cpu_rm].d[0];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real          = *(double *) &src;
        XMM[cpu_reg].d[0] = 1.0 / src_real;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opRCPPD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = 1.0 / XMM[cpu_rm].d[0];
        XMM[cpu_reg].d[1] = 1.0 / XMM[cpu_rm].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 4);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0]       = *(double *) &src[0];
        src_real[1]       = *(double *) &src[1];
        XMM[cpu_reg].d[0] = 1.0 / src_real[0];
        XMM[cpu_reg].d[1] = 1.0 / src_real[1];
    }
    return 0;
}

static int
opRCPPD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = 1.0 / XMM[cpu_rm].d[0];
        XMM[cpu_reg].d[1] = 1.0 / XMM[cpu_rm].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 4);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0]       = *(double *) &src[0];
        src_real[1]       = *(double *) &src[1];
        XMM[cpu_reg].d[0] = 1.0 / src_real[0];
        XMM[cpu_reg].d[1] = 1.0 / src_real[1];
    }
    return 0;
}

static int
opADDPD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] += XMM[cpu_rm].d[0];
        XMM[cpu_reg].d[1] += XMM[cpu_rm].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0] = *(double *) &src[0];
        src_real[1] = *(double *) &src[1];
        XMM[cpu_reg].d[0] += src_real[0];
        XMM[cpu_reg].d[1] += src_real[1];
    }
    return 0;
}

static int
opADDPD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] += XMM[cpu_rm].d[0];
        XMM[cpu_reg].d[1] += XMM[cpu_rm].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0] = *(double *) &src[0];
        src_real[1] = *(double *) &src[1];
        XMM[cpu_reg].d[0] += src_real[0];
        XMM[cpu_reg].d[1] += src_real[1];
    }
    return 0;
}

static int
opADDSD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] += XMM[cpu_rm].d[0];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real = *(double *) &src;
        XMM[cpu_reg].d[0] += src_real;
    }
    return 0;
}

static int
opADDSD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] += XMM[cpu_rm].d[0];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real = *(double *) &src;
        XMM[cpu_reg].d[0] += src_real;
    }
    return 0;
}

static int
opMULPD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] *= XMM[cpu_rm].d[0];
        XMM[cpu_reg].d[1] *= XMM[cpu_rm].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0] = *(double *) &src[0];
        src_real[1] = *(double *) &src[1];
        XMM[cpu_reg].d[0] *= src_real[0];
        XMM[cpu_reg].d[1] *= src_real[1];
    }
    return 0;
}

static int
opMULPD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] *= XMM[cpu_rm].d[0];
        XMM[cpu_reg].d[1] *= XMM[cpu_rm].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0] = *(double *) &src[0];
        src_real[1] = *(double *) &src[1];
        XMM[cpu_reg].d[0] *= src_real[0];
        XMM[cpu_reg].d[1] *= src_real[1];
    }
    return 0;
}

static int
opMULSD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] *= XMM[cpu_rm].d[0];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real = *(double *) &src;
        XMM[cpu_reg].d[0] *= src_real;
    }
    return 0;
}

static int
opMULSD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] *= XMM[cpu_rm].d[0];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real = *(double *) &src;
        XMM[cpu_reg].d[0] *= src_real;
    }
    return 0;
}

static int
opSUBPD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] -= XMM[cpu_rm].d[0];
        XMM[cpu_reg].d[1] -= XMM[cpu_rm].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0] = *(double *) &src[0];
        src_real[1] = *(double *) &src[1];
        XMM[cpu_reg].d[0] -= src_real[0];
        XMM[cpu_reg].d[1] -= src_real[1];
    }
    return 0;
}

static int
opSUBPD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] -= XMM[cpu_rm].d[0];
        XMM[cpu_reg].d[1] -= XMM[cpu_rm].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0] = *(double *) &src[0];
        src_real[1] = *(double *) &src[1];
        XMM[cpu_reg].d[0] -= src_real[0];
        XMM[cpu_reg].d[1] -= src_real[1];
    }
    return 0;
}

static int
opSUBSD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] -= XMM[cpu_rm].d[0];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real = *(double *) &src;
        XMM[cpu_reg].d[0] -= src_real;
    }
    return 0;
}

static int
opSUBSD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] -= XMM[cpu_rm].d[0];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real = *(double *) &src;
        XMM[cpu_reg].d[0] -= src_real;
    }
    return 0;
}

static int
opMINPD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = fmin(XMM[cpu_rm].d[0], XMM[cpu_reg].d[0]);
        XMM[cpu_reg].d[1] = fmin(XMM[cpu_rm].d[1], XMM[cpu_reg].d[1]);
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0]       = *(double *) &src[0];
        src_real[1]       = *(double *) &src[1];
        XMM[cpu_reg].d[0] = fmin(XMM[cpu_reg].d[0], src_real[0]);
        XMM[cpu_reg].d[1] = fmin(XMM[cpu_reg].d[1], src_real[1]);
    }
    return 0;
}

static int
opMINPD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = fmin(XMM[cpu_rm].d[0], XMM[cpu_reg].d[0]);
        XMM[cpu_reg].d[1] = fmin(XMM[cpu_rm].d[1], XMM[cpu_reg].d[1]);
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0]       = *(double *) &src[0];
        src_real[1]       = *(double *) &src[1];
        XMM[cpu_reg].d[0] = fmin(XMM[cpu_reg].d[0], src_real[0]);
        XMM[cpu_reg].d[1] = fmin(XMM[cpu_reg].d[1], src_real[1]);
    }
    return 0;
}

static int
opMINSD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = fmin(XMM[cpu_rm].d[0], XMM[cpu_reg].d[0]);
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real          = *(double *) &src;
        XMM[cpu_reg].d[0] = fmin(XMM[cpu_reg].d[0], src_real);
    }
    return 0;
}

static int
opMINSD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = fmin(XMM[cpu_rm].d[0], XMM[cpu_reg].d[0]);
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real          = *(double *) &src;
        XMM[cpu_reg].d[0] = fmin(XMM[cpu_reg].d[0], src_real);
    }
    return 0;
}

static int
opDIVPD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] /= XMM[cpu_rm].d[0];
        XMM[cpu_reg].d[1] /= XMM[cpu_rm].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0] = *(double *) &src[0];
        src_real[1] = *(double *) &src[1];
        XMM[cpu_reg].d[0] /= src_real[0];
        XMM[cpu_reg].d[1] /= src_real[1];
    }
    return 0;
}

static int
opDIVPD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] /= XMM[cpu_rm].d[0];
        XMM[cpu_reg].d[1] /= XMM[cpu_rm].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0] = *(double *) &src[0];
        src_real[1] = *(double *) &src[1];
        XMM[cpu_reg].d[0] /= src_real[0];
        XMM[cpu_reg].d[1] /= src_real[1];
    }
    return 0;
}

static int
opDIVSD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] /= XMM[cpu_rm].d[0];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real = *(double *) &src;
        XMM[cpu_reg].d[0] /= src_real;
    }
    return 0;
}

static int
opDIVSD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] /= XMM[cpu_rm].d[0];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real = *(double *) &src;
        XMM[cpu_reg].d[0] /= src_real;
    }
    return 0;
}

static int
opMAXPD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = fmax(XMM[cpu_rm].d[0], XMM[cpu_reg].d[0]);
        XMM[cpu_reg].d[1] = fmax(XMM[cpu_rm].d[1], XMM[cpu_reg].d[1]);
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0]       = *(double *) &src[0];
        src_real[1]       = *(double *) &src[1];
        XMM[cpu_reg].d[0] = fmax(XMM[cpu_reg].d[0], src_real[0]);
        XMM[cpu_reg].d[1] = fmax(XMM[cpu_reg].d[1], src_real[1]);
    }
    return 0;
}

static int
opMAXPD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = fmax(XMM[cpu_rm].d[0], XMM[cpu_reg].d[0]);
        XMM[cpu_reg].d[1] = fmax(XMM[cpu_rm].d[1], XMM[cpu_reg].d[1]);
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0]       = *(double *) &src[0];
        src_real[1]       = *(double *) &src[1];
        XMM[cpu_reg].d[0] = fmax(XMM[cpu_reg].d[0], src_real[0]);
        XMM[cpu_reg].d[1] = fmax(XMM[cpu_reg].d[1], src_real[1]);
    }
    return 0;
}

static int
opMAXSD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = fmax(XMM[cpu_rm].d[0], XMM[cpu_reg].d[0]);
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real          = *(double *) &src;
        XMM[cpu_reg].d[0] = fmax(XMM[cpu_reg].d[0], src_real);
    }
    return 0;
}

static int
opMAXSD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = fmax(XMM[cpu_rm].d[0], XMM[cpu_reg].d[0]);
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real          = *(double *) &src;
        XMM[cpu_reg].d[0] = fmax(XMM[cpu_reg].d[0], src_real);
    }
    return 0;
}

static int
opCMPPD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    uint8_t  imm = getbyte();
    uint64_t cmp[2] = { 0, 0 };
    if (cpu_mod == 3) {
        switch (imm & 7) {
            case 0:
                {
                    cmp[0] = XMM[cpu_reg].d[0] == XMM[cpu_rm].d[0] ? ~0 : 0;
                    cmp[1] = XMM[cpu_reg].d[1] == XMM[cpu_rm].d[1] ? ~0 : 0;
                    break;
                }
            case 1:
                {
                    cmp[0] = XMM[cpu_reg].d[0] < XMM[cpu_rm].d[0] ? ~0 : 0;
                    cmp[1] = XMM[cpu_reg].d[1] < XMM[cpu_rm].d[1] ? ~0 : 0;
                    break;
                }
            case 2:
                {
                    cmp[0] = XMM[cpu_reg].d[0] <= XMM[cpu_rm].d[0] ? ~0 : 0;
                    cmp[1] = XMM[cpu_reg].d[1] <= XMM[cpu_rm].d[1] ? ~0 : 0;
                    break;
                }
            case 3:
                {
                    // TODO: NaNs
                    cmp[0] = 0;
                    cmp[1] = 0;
                    break;
                }
            case 4:
                {
                    cmp[0] = XMM[cpu_reg].d[0] != XMM[cpu_rm].d[0] ? ~0 : 0;
                    cmp[1] = XMM[cpu_reg].d[1] != XMM[cpu_rm].d[1] ? ~0 : 0;
                    break;
                }
            case 5:
                {
                    cmp[0] = !(XMM[cpu_reg].d[0] < XMM[cpu_rm].d[0]) ? ~0 : 0;
                    cmp[1] = !(XMM[cpu_reg].d[1] < XMM[cpu_rm].d[1]) ? ~0 : 0;
                    break;
                }
            case 6:
                {
                    cmp[0] = !(XMM[cpu_reg].d[0] <= XMM[cpu_rm].d[0]) ? ~0 : 0;
                    cmp[1] = !(XMM[cpu_reg].d[1] <= XMM[cpu_rm].d[1]) ? ~0 : 0;
                    break;
                }
            case 7:
                {
                    // TODO: NaNs
                    cmp[0] = ~0;
                    cmp[1] = ~0;
                    break;
                }
        }
        XMM[cpu_reg].q[0] = cmp[0];
        XMM[cpu_reg].q[1] = cmp[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0] = *(double *) &src[0];
        src_real[1] = *(double *) &src[1];
        switch (imm & 7) {
            case 0:
                {
                    cmp[0] = XMM[cpu_reg].d[0] == src_real[0] ? ~0 : 0;
                    cmp[1] = XMM[cpu_reg].d[1] == src_real[1] ? ~0 : 0;
                    break;
                }
            case 1:
                {
                    cmp[0] = XMM[cpu_reg].d[0] < src_real[0] ? ~0 : 0;
                    cmp[1] = XMM[cpu_reg].d[1] < src_real[1] ? ~0 : 0;
                    break;
                }
            case 2:
                {
                    cmp[0] = XMM[cpu_reg].d[0] <= src_real[0] ? ~0 : 0;
                    cmp[1] = XMM[cpu_reg].d[1] <= src_real[1] ? ~0 : 0;
                    break;
                }
            case 3:
                {
                    // TODO: NaNs
                    cmp[0] = 0;
                    cmp[1] = 0;
                    break;
                }
            case 4:
                {
                    cmp[0] = XMM[cpu_reg].d[0] != src_real[0] ? ~0 : 0;
                    cmp[1] = XMM[cpu_reg].d[1] != src_real[1] ? ~0 : 0;
                    break;
                }
            case 5:
                {
                    cmp[0] = !(XMM[cpu_reg].d[0] < src_real[0]) ? ~0 : 0;
                    cmp[1] = !(XMM[cpu_reg].d[1] < src_real[1]) ? ~0 : 0;
                    break;
                }
            case 6:
                {
                    cmp[0] = !(XMM[cpu_reg].d[0] <= src_real[0]) ? ~0 : 0;
                    cmp[1] = !(XMM[cpu_reg].d[1] <= src_real[1]) ? ~0 : 0;
                    break;
                }
            case 7:
                {
                    // TODO: NaNs
                    cmp[0] = ~0;
                    cmp[1] = ~0;
                    break;
                }
        }
        XMM[cpu_reg].q[0] = cmp[0];
        XMM[cpu_reg].q[1] = cmp[1];
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opCMPPD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    uint8_t  imm = getbyte();
    uint64_t cmp[2] = { 0, 0 };
    if (cpu_mod == 3) {
        switch (imm & 7) {
            case 0:
                {
                    cmp[0] = XMM[cpu_reg].d[0] == XMM[cpu_rm].d[0] ? ~0 : 0;
                    cmp[1] = XMM[cpu_reg].d[1] == XMM[cpu_rm].d[1] ? ~0 : 0;
                    break;
                }
            case 1:
                {
                    cmp[0] = XMM[cpu_reg].d[0] < XMM[cpu_rm].d[0] ? ~0 : 0;
                    cmp[1] = XMM[cpu_reg].d[1] < XMM[cpu_rm].d[1] ? ~0 : 0;
                    break;
                }
            case 2:
                {
                    cmp[0] = XMM[cpu_reg].d[0] <= XMM[cpu_rm].d[0] ? ~0 : 0;
                    cmp[1] = XMM[cpu_reg].d[1] <= XMM[cpu_rm].d[1] ? ~0 : 0;
                    break;
                }
            case 3:
                {
                    // TODO: NaNs
                    cmp[0] = 0;
                    cmp[1] = 0;
                    break;
                }
            case 4:
                {
                    cmp[0] = XMM[cpu_reg].d[0] != XMM[cpu_rm].d[0] ? ~0 : 0;
                    cmp[1] = XMM[cpu_reg].d[1] != XMM[cpu_rm].d[1] ? ~0 : 0;
                    break;
                }
            case 5:
                {
                    cmp[0] = !(XMM[cpu_reg].d[0] < XMM[cpu_rm].d[0]) ? ~0 : 0;
                    cmp[1] = !(XMM[cpu_reg].d[1] < XMM[cpu_rm].d[1]) ? ~0 : 0;
                    break;
                }
            case 6:
                {
                    cmp[0] = !(XMM[cpu_reg].d[0] <= XMM[cpu_rm].d[0]) ? ~0 : 0;
                    cmp[1] = !(XMM[cpu_reg].d[1] <= XMM[cpu_rm].d[1]) ? ~0 : 0;
                    break;
                }
            case 7:
                {
                    // TODO: NaNs
                    cmp[0] = ~0;
                    cmp[1] = ~0;
                    break;
                }
        }
        XMM[cpu_reg].q[0] = cmp[0];
        XMM[cpu_reg].q[1] = cmp[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t src[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        double src_real[2];
        src_real[0] = *(double *) &src[0];
        src_real[1] = *(double *) &src[1];
        switch (imm & 7) {
            case 0:
                {
                    cmp[0] = XMM[cpu_reg].d[0] == src_real[0] ? ~0 : 0;
                    cmp[1] = XMM[cpu_reg].d[1] == src_real[1] ? ~0 : 0;
                    break;
                }
            case 1:
                {
                    cmp[0] = XMM[cpu_reg].d[0] < src_real[0] ? ~0 : 0;
                    cmp[1] = XMM[cpu_reg].d[1] < src_real[1] ? ~0 : 0;
                    break;
                }
            case 2:
                {
                    cmp[0] = XMM[cpu_reg].d[0] <= src_real[0] ? ~0 : 0;
                    cmp[1] = XMM[cpu_reg].d[1] <= src_real[1] ? ~0 : 0;
                    break;
                }
            case 3:
                {
                    // TODO: NaNs
                    cmp[0] = 0;
                    cmp[1] = 0;
                    break;
                }
            case 4:
                {
                    cmp[0] = XMM[cpu_reg].d[0] != src_real[0] ? ~0 : 0;
                    cmp[1] = XMM[cpu_reg].d[1] != src_real[1] ? ~0 : 0;
                    break;
                }
            case 5:
                {
                    cmp[0] = !(XMM[cpu_reg].d[0] < src_real[0]) ? ~0 : 0;
                    cmp[1] = !(XMM[cpu_reg].d[1] < src_real[1]) ? ~0 : 0;
                    break;
                }
            case 6:
                {
                    cmp[0] = !(XMM[cpu_reg].d[0] <= src_real[0]) ? ~0 : 0;
                    cmp[1] = !(XMM[cpu_reg].d[1] <= src_real[1]) ? ~0 : 0;
                    break;
                }
            case 7:
                {
                    // TODO: NaNs
                    cmp[0] = ~0;
                    cmp[1] = ~0;
                    break;
                }
        }
        XMM[cpu_reg].q[0] = cmp[0];
        XMM[cpu_reg].q[1] = cmp[1];
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opCMPSD_xmm_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    uint8_t  imm = getbyte();
    uint64_t cmp = 0;
    if (cpu_mod == 3) {
        switch (imm & 7) {
            case 0:
                {
                    cmp = XMM[cpu_reg].d[0] == XMM[cpu_rm].d[0] ? ~0 : 0;
                    break;
                }
            case 1:
                {
                    cmp = XMM[cpu_reg].d[0] < XMM[cpu_rm].d[0] ? ~0 : 0;
                    break;
                }
            case 2:
                {
                    cmp = XMM[cpu_reg].d[0] <= XMM[cpu_rm].d[0] ? ~0 : 0;
                    break;
                }
            case 3:
                {
                    // TODO: NaNs
                    cmp = 0;
                    break;
                }
            case 4:
                {
                    cmp = XMM[cpu_reg].d[0] != XMM[cpu_rm].d[0] ? ~0 : 0;
                    break;
                }
            case 5:
                {
                    cmp = !(XMM[cpu_reg].d[0] < XMM[cpu_rm].d[0]) ? ~0 : 0;
                    break;
                }
            case 6:
                {
                    cmp = !(XMM[cpu_reg].d[0] <= XMM[cpu_rm].d[0]) ? ~0 : 0;
                    break;
                }
            case 7:
                {
                    // TODO: NaNs
                    cmp = ~0;
                    break;
                }
        }
        XMM[cpu_reg].q[0] = cmp;
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real = *(double *) &src;
        switch (imm & 7) {
            case 0:
                {
                    cmp = XMM[cpu_reg].d[0] == src_real ? ~0 : 0;
                    break;
                }
            case 1:
                {
                    cmp = XMM[cpu_reg].d[0] < src_real ? ~0 : 0;
                    break;
                }
            case 2:
                {
                    cmp = XMM[cpu_reg].d[0] <= src_real ? ~0 : 0;
                    break;
                }
            case 3:
                {
                    // TODO: NaNs
                    cmp = 0;
                    break;
                }
            case 4:
                {
                    cmp = XMM[cpu_reg].d[0] != src_real ? ~0 : 0;
                    break;
                }
            case 5:
                {
                    cmp = !(XMM[cpu_reg].d[0] < src_real) ? ~0 : 0;
                    break;
                }
            case 6:
                {
                    cmp = !(XMM[cpu_reg].d[0] <= src_real) ? ~0 : 0;
                    break;
                }
            case 7:
                {
                    // TODO: NaNs
                    cmp = ~0;
                    break;
                }
        }
        XMM[cpu_reg].q[0] = cmp;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opCMPSD_xmm_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    uint8_t  imm = getbyte();
    uint64_t cmp = 0;
    if (cpu_mod == 3) {
        switch (imm & 7) {
            case 0:
                {
                    cmp = XMM[cpu_reg].d[0] == XMM[cpu_rm].d[0] ? ~0 : 0;
                    break;
                }
            case 1:
                {
                    cmp = XMM[cpu_reg].d[0] < XMM[cpu_rm].d[0] ? ~0 : 0;
                    break;
                }
            case 2:
                {
                    cmp = XMM[cpu_reg].d[0] <= XMM[cpu_rm].d[0] ? ~0 : 0;
                    break;
                }
            case 3:
                {
                    // TODO: NaNs
                    cmp = 0;
                    break;
                }
            case 4:
                {
                    cmp = XMM[cpu_reg].d[0] != XMM[cpu_rm].d[0] ? ~0 : 0;
                    break;
                }
            case 5:
                {
                    cmp = !(XMM[cpu_reg].d[0] < XMM[cpu_rm].d[0]) ? ~0 : 0;
                    break;
                }
            case 6:
                {
                    cmp = !(XMM[cpu_reg].d[0] <= XMM[cpu_rm].d[0]) ? ~0 : 0;
                    break;
                }
            case 7:
                {
                    // TODO: NaNs
                    cmp = ~0;
                    break;
                }
        }
        XMM[cpu_reg].q[0] = cmp;
        CLOCK_CYCLES(1);
    } else {
        uint64_t src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        double src_real;
        src_real = *(double *) &src;
        switch (imm & 7) {
            case 0:
                {
                    cmp = XMM[cpu_reg].d[0] == src_real ? ~0 : 0;
                    break;
                }
            case 1:
                {
                    cmp = XMM[cpu_reg].d[0] < src_real ? ~0 : 0;
                    break;
                }
            case 2:
                {
                    cmp = XMM[cpu_reg].d[0] <= src_real ? ~0 : 0;
                    break;
                }
            case 3:
                {
                    // TODO: NaNs
                    cmp = 0;
                    break;
                }
            case 4:
                {
                    cmp = XMM[cpu_reg].d[0] != src_real ? ~0 : 0;
                    break;
                }
            case 5:
                {
                    cmp = !(XMM[cpu_reg].d[0] < src_real) ? ~0 : 0;
                    break;
                }
            case 6:
                {
                    cmp = !(XMM[cpu_reg].d[0] <= src_real) ? ~0 : 0;
                    break;
                }
            case 7:
                {
                    // TODO: NaNs
                    cmp = ~0;
                    break;
                }
        }
        XMM[cpu_reg].q[0] = cmp;
        CLOCK_CYCLES(2);
    }
    return 0;
}
