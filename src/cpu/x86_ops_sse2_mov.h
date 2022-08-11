static int
opMOVUPD_q_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = XMM[cpu_rm].d[0];
        XMM[cpu_reg].d[1] = XMM[cpu_rm].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t dst[4];

        SEG_CHECK_READ(cpu_state.ea_seg);
        dst[0] = readmeml(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        dst[1] = readmeml(easeg, cpu_state.eaaddr + 4);
        if (cpu_state.abrt)
            return 1;
        dst[2] = readmeml(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        dst[3] = readmeml(easeg, cpu_state.eaaddr + 12);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].l[0] = dst[0];
        XMM[cpu_reg].l[1] = dst[1];
        XMM[cpu_reg].l[2] = dst[2];
        XMM[cpu_reg].l[3] = dst[3];

        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVUPD_q_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = XMM[cpu_rm].d[0];
        XMM[cpu_reg].d[1] = XMM[cpu_rm].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t dst[4];

        SEG_CHECK_READ(cpu_state.ea_seg);
        dst[0] = readmeml(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        dst[1] = readmeml(easeg, cpu_state.eaaddr + 4);
        if (cpu_state.abrt)
            return 1;
        dst[2] = readmeml(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        dst[3] = readmeml(easeg, cpu_state.eaaddr + 12);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].l[0] = dst[0];
        XMM[cpu_reg].l[1] = dst[1];
        XMM[cpu_reg].l[2] = dst[2];
        XMM[cpu_reg].l[3] = dst[3];

        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVUPD_xmm_q_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_rm].d[0] = XMM[cpu_reg].d[0];
        XMM[cpu_rm].d[1] = XMM[cpu_reg].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint32_t rm[4] = { XMM[cpu_reg].l[0], XMM[cpu_reg].l[1], XMM[cpu_reg].l[2], XMM[cpu_reg].l[3] };
        SEG_CHECK_WRITE(cpu_state.ea_seg);
        writememl(easeg, cpu_state.eaaddr, rm[0]);
        if (cpu_state.abrt)
            return 1;
        writememl(easeg, cpu_state.eaaddr + 4, rm[1]);
        if (cpu_state.abrt)
            return 1;
        writememl(easeg, cpu_state.eaaddr + 8, rm[2]);
        if (cpu_state.abrt)
            return 1;
        writememl(easeg, cpu_state.eaaddr + 12, rm[3]);
        if (cpu_state.abrt)
            return 1;

        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVUPD_xmm_q_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_rm].d[0] = XMM[cpu_reg].d[0];
        XMM[cpu_rm].d[1] = XMM[cpu_reg].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint32_t rm[4] = { XMM[cpu_reg].l[0], XMM[cpu_reg].l[1], XMM[cpu_reg].l[2], XMM[cpu_reg].l[3] };
        SEG_CHECK_WRITE(cpu_state.ea_seg);
        writememl(easeg, cpu_state.eaaddr, rm[0]);
        if (cpu_state.abrt)
            return 1;
        writememl(easeg, cpu_state.eaaddr + 4, rm[1]);
        if (cpu_state.abrt)
            return 1;
        writememl(easeg, cpu_state.eaaddr + 8, rm[2]);
        if (cpu_state.abrt)
            return 1;
        writememl(easeg, cpu_state.eaaddr + 12, rm[3]);
        if (cpu_state.abrt)
            return 1;

        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVSD_f_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = XMM[cpu_rm].d[0];
        CLOCK_CYCLES(1);
    } else {
        uint64_t dst;

        SEG_CHECK_READ(cpu_state.ea_seg);
        dst = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].q[0] = dst;
        XMM[cpu_reg].q[1] = 0;

        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVSD_f_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = XMM[cpu_rm].d[0];
        CLOCK_CYCLES(1);
    } else {
        uint64_t dst;

        SEG_CHECK_READ(cpu_state.ea_seg);
        dst = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].q[0] = dst;
        XMM[cpu_reg].q[1] = 0;

        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVSD_xmm_f_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_rm].d[0] = XMM[cpu_reg].d[0];
        CLOCK_CYCLES(1);
    } else {
        uint64_t rm = XMM[cpu_reg].q[0];

        SEG_CHECK_WRITE(cpu_state.ea_seg);
        writememq(easeg, cpu_state.eaaddr, rm);
        if (cpu_state.abrt)
            return 1;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVSD_xmm_f_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_rm].d[0] = XMM[cpu_reg].d[0];
        CLOCK_CYCLES(1);
    } else {
        uint64_t rm = XMM[cpu_reg].q[0];

        SEG_CHECK_WRITE(cpu_state.ea_seg);
        writememq(easeg, cpu_state.eaaddr, rm);
        if (cpu_state.abrt)
            return 1;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVLPD_f_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    ILLEGAL_ON(cpu_mod == 3);
    uint64_t dst;

    SEG_CHECK_READ(cpu_state.ea_seg);
    dst = readmemq(easeg, cpu_state.eaaddr);
    if (cpu_state.abrt)
        return 1;
    XMM[cpu_reg].q[0] = dst;
    CLOCK_CYCLES(2);
    return 0;
}

static int
opMOVLPD_f_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    ILLEGAL_ON(cpu_mod == 3);
    uint64_t dst;

    SEG_CHECK_READ(cpu_state.ea_seg);
    dst = readmemq(easeg, cpu_state.eaaddr);
    if (cpu_state.abrt)
        return 1;
    XMM[cpu_reg].q[0] = dst;
    CLOCK_CYCLES(2);
    return 0;
}

static int
opMOVLPD_xmm_f_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_rm].q[0] = XMM[cpu_reg].q[0];
        CLOCK_CYCLES(1);
    } else {
        SEG_CHECK_WRITE(cpu_state.ea_seg);
        writememq(easeg, cpu_state.eaaddr, XMM[cpu_rm].q[0]);
        if (cpu_state.abrt)
            return 1;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVLPD_xmm_f_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_rm].q[0] = XMM[cpu_reg].q[0];
        CLOCK_CYCLES(1);
    } else {
        SEG_CHECK_WRITE(cpu_state.ea_seg);
        writememq(easeg, cpu_state.eaaddr, XMM[cpu_rm].q[0]);
        if (cpu_state.abrt)
            return 1;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opUNPCKLPD_f_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].q[0] = XMM[cpu_rm].q[0];
        XMM[cpu_reg].q[1] = XMM[cpu_rm].q[0];
        CLOCK_CYCLES(1);
    } else {
        uint64_t dst[2];
        SEG_CHECK_READ(cpu_state.ea_seg);
        dst[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        dst[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].q[0] = dst[0];
        XMM[cpu_reg].q[1] = dst[0];
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opUNPCKLPD_f_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].q[0] = XMM[cpu_rm].q[0];
        XMM[cpu_reg].q[1] = XMM[cpu_rm].q[0];
        CLOCK_CYCLES(1);
    } else {
        uint64_t dst[2];
        SEG_CHECK_READ(cpu_state.ea_seg);
        dst[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        dst[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].q[0] = dst[0];
        XMM[cpu_reg].q[1] = dst[0];
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opUNPCKHPD_f_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].q[0] = XMM[cpu_rm].q[1];
        XMM[cpu_reg].q[1] = XMM[cpu_rm].q[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t dst[2];
        SEG_CHECK_READ(cpu_state.ea_seg);
        dst[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        dst[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].q[0] = dst[1];
        XMM[cpu_reg].q[1] = dst[1];
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opUNPCKHPD_f_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].q[0] = XMM[cpu_rm].q[1];
        XMM[cpu_reg].q[1] = XMM[cpu_rm].q[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t dst[2];
        SEG_CHECK_READ(cpu_state.ea_seg);
        dst[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        dst[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].q[0] = dst[1];
        XMM[cpu_reg].q[1] = dst[1];
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVHPD_f_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    ILLEGAL_ON(cpu_mod == 3);
    uint64_t dst;

    SEG_CHECK_READ(cpu_state.ea_seg);
    dst = readmemq(easeg, cpu_state.eaaddr);
    if (cpu_state.abrt)
        return 1;
    XMM[cpu_reg].q[1] = dst;
    CLOCK_CYCLES(2);
    return 0;
}

static int
opMOVHPD_f_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    ILLEGAL_ON(cpu_mod == 3);
    uint64_t dst;

    SEG_CHECK_READ(cpu_state.ea_seg);
    dst = readmemq(easeg, cpu_state.eaaddr);
    if (cpu_state.abrt)
        return 1;
    XMM[cpu_reg].q[1] = dst;
    CLOCK_CYCLES(2);
    return 0;
}

static int
opMOVHPD_xmm_f_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_rm].q[1] = XMM[cpu_reg].q[0];
        CLOCK_CYCLES(1);
    } else {
        SEG_CHECK_WRITE(cpu_state.ea_seg);
        writememq(easeg, cpu_state.eaaddr + 8, XMM[cpu_rm].q[0]);
        if (cpu_state.abrt)
            return 1;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVHPD_xmm_f_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_rm].q[1] = XMM[cpu_reg].q[0];
        CLOCK_CYCLES(1);
    } else {
        SEG_CHECK_WRITE(cpu_state.ea_seg);
        writememq(easeg, cpu_state.eaaddr + 8, XMM[cpu_rm].q[0]);
        if (cpu_state.abrt)
            return 1;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVAPD_q_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = XMM[cpu_rm].d[0];
        XMM[cpu_reg].d[1] = XMM[cpu_rm].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t dst[4];

        SEG_CHECK_READ(cpu_state.ea_seg);
        if (cpu_state.eaaddr & 0xf)
            x86gpf(NULL, 0);
        dst[0] = readmeml(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        dst[1] = readmeml(easeg, cpu_state.eaaddr + 4);
        if (cpu_state.abrt)
            return 1;
        dst[2] = readmeml(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        dst[3] = readmeml(easeg, cpu_state.eaaddr + 12);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].l[0] = dst[0];
        XMM[cpu_reg].l[1] = dst[1];
        XMM[cpu_reg].l[2] = dst[2];
        XMM[cpu_reg].l[3] = dst[3];

        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVAPD_q_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = XMM[cpu_rm].d[0];
        XMM[cpu_reg].d[1] = XMM[cpu_rm].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t dst[4];

        SEG_CHECK_READ(cpu_state.ea_seg);
        if (cpu_state.eaaddr & 0xf)
            x86gpf(NULL, 0);
        dst[0] = readmeml(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        dst[1] = readmeml(easeg, cpu_state.eaaddr + 4);
        if (cpu_state.abrt)
            return 1;
        dst[2] = readmeml(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        dst[3] = readmeml(easeg, cpu_state.eaaddr + 12);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].l[0] = dst[0];
        XMM[cpu_reg].l[1] = dst[1];
        XMM[cpu_reg].l[2] = dst[2];
        XMM[cpu_reg].l[3] = dst[3];

        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVAPD_xmm_q_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_rm].d[0] = XMM[cpu_reg].d[0];
        XMM[cpu_rm].d[1] = XMM[cpu_reg].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint32_t rm[4] = { XMM[cpu_reg].l[0], XMM[cpu_reg].l[1], XMM[cpu_reg].l[2], XMM[cpu_reg].l[3] };
        SEG_CHECK_WRITE(cpu_state.ea_seg);
        if (cpu_state.eaaddr & 0xf)
            x86gpf(NULL, 0);
        writememl(easeg, cpu_state.eaaddr, rm[0]);
        if (cpu_state.abrt)
            return 1;
        writememl(easeg, cpu_state.eaaddr + 4, rm[1]);
        if (cpu_state.abrt)
            return 1;
        writememl(easeg, cpu_state.eaaddr + 8, rm[2]);
        if (cpu_state.abrt)
            return 1;
        writememl(easeg, cpu_state.eaaddr + 12, rm[3]);
        if (cpu_state.abrt)
            return 1;

        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVAPD_xmm_q_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_rm].d[0] = XMM[cpu_reg].d[0];
        XMM[cpu_rm].d[1] = XMM[cpu_reg].d[1];
        CLOCK_CYCLES(1);
    } else {
        uint32_t rm[4] = { XMM[cpu_reg].l[0], XMM[cpu_reg].l[1], XMM[cpu_reg].l[2], XMM[cpu_reg].l[3] };
        SEG_CHECK_WRITE(cpu_state.ea_seg);
        if (cpu_state.eaaddr & 0xf)
            x86gpf(NULL, 0);
        writememl(easeg, cpu_state.eaaddr, rm[0]);
        if (cpu_state.abrt)
            return 1;
        writememl(easeg, cpu_state.eaaddr + 4, rm[1]);
        if (cpu_state.abrt)
            return 1;
        writememl(easeg, cpu_state.eaaddr + 8, rm[2]);
        if (cpu_state.abrt)
            return 1;
        writememl(easeg, cpu_state.eaaddr + 12, rm[3]);
        if (cpu_state.abrt)
            return 1;

        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVNTPD_xmm_q_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    ILLEGAL_ON(cpu_mod == 3);

    uint32_t rm[4] = { XMM[cpu_reg].l[0], XMM[cpu_reg].l[1], XMM[cpu_reg].l[2], XMM[cpu_reg].l[3] };
    SEG_CHECK_WRITE(cpu_state.ea_seg);
    if (cpu_state.eaaddr & 0xf)
        x86gpf(NULL, 0);
    writememl(easeg, cpu_state.eaaddr, rm[0]);
    if (cpu_state.abrt)
        return 1;
    writememl(easeg, cpu_state.eaaddr + 4, rm[1]);
    if (cpu_state.abrt)
        return 1;
    writememl(easeg, cpu_state.eaaddr + 8, rm[2]);
    if (cpu_state.abrt)
        return 1;
    writememl(easeg, cpu_state.eaaddr + 12, rm[3]);
    if (cpu_state.abrt)
        return 1;

    CLOCK_CYCLES(2);
    return 0;
}

static int
opMOVNTPD_xmm_q_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    ILLEGAL_ON(cpu_mod == 3);

    uint32_t rm[4] = { XMM[cpu_reg].l[0], XMM[cpu_reg].l[1], XMM[cpu_reg].l[2], XMM[cpu_reg].l[3] };
    SEG_CHECK_WRITE(cpu_state.ea_seg);
    if (cpu_state.eaaddr & 0xf)
        x86gpf(NULL, 0);
    writememl(easeg, cpu_state.eaaddr, rm[0]);
    if (cpu_state.abrt)
        return 1;
    writememl(easeg, cpu_state.eaaddr + 4, rm[1]);
    if (cpu_state.abrt)
        return 1;
    writememl(easeg, cpu_state.eaaddr + 8, rm[2]);
    if (cpu_state.abrt)
        return 1;
    writememl(easeg, cpu_state.eaaddr + 12, rm[3]);
    if (cpu_state.abrt)
        return 1;

    CLOCK_CYCLES(2);
    return 0;
}

static int
opMOVMSKPD_l_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    ILLEGAL_ON(cpu_mod != 3);
    if (cpu_mod == 3) {
        uint32_t result = 0;
        if (XMM[cpu_rm].q[0] & (1ULL << 63))
            result |= 1;
        if (XMM[cpu_rm].q[1] & (1ULL << 63))
            result |= 2;
        setr32(cpu_reg, result);
        CLOCK_CYCLES(1);
    }
    return 0;
}

static int
opMOVMSKPD_l_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    ILLEGAL_ON(cpu_mod != 3);
    if (cpu_mod == 3) {
        uint32_t result = 0;
        if (XMM[cpu_rm].q[0] & (1ULL << 63))
            result |= 1;
        if (XMM[cpu_rm].q[1] & (1ULL << 63))
            result |= 2;
        setr32(cpu_reg, result);
        CLOCK_CYCLES(1);
    }
    return 0;
}

static int
opMOVD_l_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].l[0] = cpu_state.regs[cpu_rm].l;
        XMM[cpu_reg].l[1] = 0;
        XMM[cpu_reg].l[2] = 0;
        XMM[cpu_reg].l[3] = 0;
        CLOCK_CYCLES(1);
    } else {
        uint32_t dst;

        SEG_CHECK_READ(cpu_state.ea_seg);
        dst = readmeml(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].l[0] = dst;
        XMM[cpu_reg].l[1] = 0;
        XMM[cpu_reg].l[2] = 0;
        XMM[cpu_reg].l[3] = 0;

        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVD_l_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].l[0] = cpu_state.regs[cpu_rm].l;
        XMM[cpu_reg].l[1] = 0;
        XMM[cpu_reg].l[2] = 0;
        XMM[cpu_reg].l[3] = 0;
        CLOCK_CYCLES(1);
    } else {
        uint32_t dst;

        SEG_CHECK_READ(cpu_state.ea_seg);
        dst = readmeml(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].l[0] = dst;
        XMM[cpu_reg].l[1] = 0;
        XMM[cpu_reg].l[2] = 0;
        XMM[cpu_reg].l[3] = 0;

        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVDQA_l_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].q[0] = XMM[cpu_rm].q[0];
        XMM[cpu_reg].q[1] = XMM[cpu_rm].q[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t dst[2];

        if (cpu_state.eaaddr & 0xf) {
            x86gpf(NULL, 0);
            if (cpu_state.abrt)
                return 1;
        }

        SEG_CHECK_READ(cpu_state.ea_seg);
        dst[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        dst[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].q[0] = dst[0];
        XMM[cpu_reg].q[1] = dst[1];

        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVDQA_l_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].q[0] = XMM[cpu_rm].q[0];
        XMM[cpu_reg].q[1] = XMM[cpu_rm].q[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t dst[2];

        if (cpu_state.eaaddr & 0xf) {
            x86gpf(NULL, 0);
            if (cpu_state.abrt)
                return 1;
        }

        SEG_CHECK_READ(cpu_state.ea_seg);
        dst[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        dst[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].q[0] = dst[0];
        XMM[cpu_reg].q[1] = dst[1];

        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVDQU_l_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].q[0] = XMM[cpu_rm].q[0];
        XMM[cpu_reg].q[1] = XMM[cpu_rm].q[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t dst[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        dst[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        dst[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].q[0] = dst[0];
        XMM[cpu_reg].q[1] = dst[1];

        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVDQU_l_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].q[0] = XMM[cpu_rm].q[0];
        XMM[cpu_reg].q[1] = XMM[cpu_rm].q[1];
        CLOCK_CYCLES(1);
    } else {
        uint64_t dst[2];

        SEG_CHECK_READ(cpu_state.ea_seg);
        dst[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        dst[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].q[0] = dst[0];
        XMM[cpu_reg].q[1] = dst[1];

        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opPSHUFLW_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    uint8_t imm = getbyte();
    if (cpu_state.abrt)
        return 1;
    if (cpu_mod == 3) {
        XMM[cpu_reg].w[0] = XMM[cpu_rm].w[(imm & 3)];
        XMM[cpu_reg].w[1] = XMM[cpu_rm].w[((imm >> 2) & 3)];
        XMM[cpu_reg].w[2] = XMM[cpu_rm].w[((imm >> 4) & 3)];
        XMM[cpu_reg].w[3] = XMM[cpu_rm].w[((imm >> 6) & 3)];
        XMM[cpu_reg].q[1] = XMM[cpu_rm].q[1];
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
        XMM[cpu_reg].w[0] = (src[0] >> ((imm & 3) << 4)) & 0xffff;
        XMM[cpu_reg].w[1] = (src[0] >> (((imm >> 2) & 3) << 4)) & 0xffff;
        XMM[cpu_reg].w[2] = (src[0] >> (((imm >> 4) & 3) << 4)) & 0xffff;
        XMM[cpu_reg].w[3] = (src[0] >> (((imm >> 6) & 3) << 4)) & 0xffff;
        XMM[cpu_reg].q[1] = src[1];
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opPSHUFLW_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    uint8_t imm = getbyte();
    if (cpu_state.abrt)
        return 1;
    if (cpu_mod == 3) {
        XMM[cpu_reg].w[0] = XMM[cpu_rm].w[(imm & 3)];
        XMM[cpu_reg].w[1] = XMM[cpu_rm].w[((imm >> 2) & 3)];
        XMM[cpu_reg].w[2] = XMM[cpu_rm].w[((imm >> 4) & 3)];
        XMM[cpu_reg].w[3] = XMM[cpu_rm].w[((imm >> 6) & 3)];
        XMM[cpu_reg].q[1] = XMM[cpu_rm].q[1];
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
        XMM[cpu_reg].w[0] = (src[0] >> ((imm & 3) << 4)) & 0xffff;
        XMM[cpu_reg].w[1] = (src[0] >> (((imm >> 2) & 3) << 4)) & 0xffff;
        XMM[cpu_reg].w[2] = (src[0] >> (((imm >> 4) & 3) << 4)) & 0xffff;
        XMM[cpu_reg].w[3] = (src[0] >> (((imm >> 6) & 3) << 4)) & 0xffff;
        XMM[cpu_reg].q[1] = src[1];
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opPSHUFHW_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    uint8_t imm = getbyte();
    if (cpu_state.abrt)
        return 1;
    if (cpu_mod == 3) {
        XMM[cpu_reg].q[0] = XMM[cpu_rm].q[0];
        XMM[cpu_reg].w[4] = XMM[cpu_rm].w[(imm & 3) + 4];
        XMM[cpu_reg].w[5] = XMM[cpu_rm].w[((imm >> 2) & 3) + 4];
        XMM[cpu_reg].w[6] = XMM[cpu_rm].w[((imm >> 4) & 3) + 4];
        XMM[cpu_reg].w[7] = XMM[cpu_rm].w[((imm >> 6) & 3) + 4];
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
        XMM[cpu_reg].q[0] = src[0];
        XMM[cpu_reg].w[4] = (src[1] >> ((imm & 3) << 4)) & 0xffff;
        XMM[cpu_reg].w[5] = (src[1] >> (((imm >> 2) & 3) << 4)) & 0xffff;
        XMM[cpu_reg].w[6] = (src[1] >> (((imm >> 4) & 3) << 4)) & 0xffff;
        XMM[cpu_reg].w[7] = (src[1] >> (((imm >> 6) & 3) << 4)) & 0xffff;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opPSHUFHW_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    uint8_t imm = getbyte();
    if (cpu_state.abrt)
        return 1;
    if (cpu_mod == 3) {
        XMM[cpu_reg].q[0] = XMM[cpu_rm].q[0];
        XMM[cpu_reg].w[4] = XMM[cpu_rm].w[(imm & 3) + 4];
        XMM[cpu_reg].w[5] = XMM[cpu_rm].w[((imm >> 2) & 3) + 4];
        XMM[cpu_reg].w[6] = XMM[cpu_rm].w[((imm >> 4) & 3) + 4];
        XMM[cpu_reg].w[7] = XMM[cpu_rm].w[((imm >> 6) & 3) + 4];
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
        XMM[cpu_reg].q[0] = src[0];
        XMM[cpu_reg].w[4] = (src[1] >> ((imm & 3) << 4)) & 0xffff;
        XMM[cpu_reg].w[5] = (src[1] >> (((imm >> 2) & 3) << 4)) & 0xffff;
        XMM[cpu_reg].w[6] = (src[1] >> (((imm >> 4) & 3) << 4)) & 0xffff;
        XMM[cpu_reg].w[7] = (src[1] >> (((imm >> 6) & 3) << 4)) & 0xffff;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opPSHUFD_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    uint8_t imm = getbyte();
    if (cpu_state.abrt)
        return 1;
    if (cpu_mod == 3) {
        XMM[cpu_reg].l[0] = XMM[cpu_rm].l[(imm & 3)];
        XMM[cpu_reg].l[1] = XMM[cpu_rm].l[((imm >> 2) & 3)];
        XMM[cpu_reg].l[2] = XMM[cpu_rm].l[((imm >> 4) & 3)];
        XMM[cpu_reg].l[3] = XMM[cpu_rm].l[((imm >> 6) & 3)];
        CLOCK_CYCLES(1);
    } else {
        uint32_t src[4];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmeml(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmeml(easeg, cpu_state.eaaddr + 4);
        if (cpu_state.abrt)
            return 1;
        src[2] = readmeml(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        src[3] = readmeml(easeg, cpu_state.eaaddr + 12);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].l[0] = src[(imm & 3)];
        XMM[cpu_reg].l[1] = src[((imm >> 2) & 3)];
        XMM[cpu_reg].l[2] = src[((imm >> 4) & 3)];
        XMM[cpu_reg].l[3] = src[((imm >> 6) & 3)];
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opPSHUFD_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    uint8_t imm = getbyte();
    if (cpu_state.abrt)
        return 1;
    if (cpu_mod == 3) {
        XMM[cpu_reg].l[0] = XMM[cpu_rm].l[(imm & 3)];
        XMM[cpu_reg].l[1] = XMM[cpu_rm].l[((imm >> 2) & 3)];
        XMM[cpu_reg].l[2] = XMM[cpu_rm].l[((imm >> 4) & 3)];
        XMM[cpu_reg].l[3] = XMM[cpu_rm].l[((imm >> 6) & 3)];
        CLOCK_CYCLES(1);
    } else {
        uint32_t src[4];

        SEG_CHECK_READ(cpu_state.ea_seg);
        src[0] = readmeml(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        src[1] = readmeml(easeg, cpu_state.eaaddr + 4);
        if (cpu_state.abrt)
            return 1;
        src[2] = readmeml(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 1;
        src[3] = readmeml(easeg, cpu_state.eaaddr + 12);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].l[0] = src[(imm & 3)];
        XMM[cpu_reg].l[1] = src[((imm >> 2) & 3)];
        XMM[cpu_reg].l[2] = src[((imm >> 4) & 3)];
        XMM[cpu_reg].l[3] = src[((imm >> 6) & 3)];
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVD_xmm_l_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        cpu_state.regs[cpu_rm].l = XMM[cpu_reg].l[0];
        CLOCK_CYCLES(1);
    } else {
        SEG_CHECK_WRITE(cpu_state.ea_seg);
        CHECK_WRITE_COMMON(cpu_state.ea_seg, cpu_state.eaaddr, cpu_state.eaaddr + 3);
        writememl(easeg, cpu_state.eaaddr, XMM[cpu_reg].l[0]);
        if (cpu_state.abrt)
            return 1;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVD_xmm_l_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        cpu_state.regs[cpu_rm].l = XMM[cpu_reg].l[0];
        CLOCK_CYCLES(1);
    } else {
        SEG_CHECK_WRITE(cpu_state.ea_seg);
        CHECK_WRITE_COMMON(cpu_state.ea_seg, cpu_state.eaaddr, cpu_state.eaaddr + 3);
        writememl(easeg, cpu_state.eaaddr, XMM[cpu_reg].l[0]);
        if (cpu_state.abrt)
            return 1;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVQ_xmm_q_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_rm].q[0] = XMM[cpu_reg].q[0];
        CLOCK_CYCLES(1);
    } else {
        SEG_CHECK_WRITE(cpu_state.ea_seg);
        CHECK_WRITE_COMMON(cpu_state.ea_seg, cpu_state.eaaddr, cpu_state.eaaddr + 7);
        writememq(easeg, cpu_state.eaaddr, XMM[cpu_reg].q[0]);
        if (cpu_state.abrt)
            return 1;
        CLOCK_CYCLES(2);
    }
    return 0;
}
static int
opMOVQ_xmm_q_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_rm].q[0] = XMM[cpu_reg].q[0];
        CLOCK_CYCLES(1);
    } else {
        SEG_CHECK_WRITE(cpu_state.ea_seg);
        CHECK_WRITE_COMMON(cpu_state.ea_seg, cpu_state.eaaddr, cpu_state.eaaddr + 7);
        writememq(easeg, cpu_state.eaaddr, XMM[cpu_reg].q[0]);
        if (cpu_state.abrt)
            return 1;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVDQA_xmm_q_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_rm].q[0] = XMM[cpu_reg].q[0];
        XMM[cpu_rm].q[1] = XMM[cpu_reg].q[1];
        CLOCK_CYCLES(1);
    } else {
        if (cpu_state.eaaddr & 0xf) {
            x86gpf(NULL, 0);
            if (cpu_state.abrt)
                return 1;
        }

        SEG_CHECK_WRITE(cpu_state.ea_seg);
        CHECK_WRITE_COMMON(cpu_state.ea_seg, cpu_state.eaaddr, cpu_state.eaaddr + 15);
        writememq(easeg, cpu_state.eaaddr, XMM[cpu_reg].q[0]);
        if (cpu_state.abrt)
            return 1;
        writememq(easeg, cpu_state.eaaddr + 8, XMM[cpu_reg].q[1]);
        if (cpu_state.abrt)
            return 1;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVDQA_xmm_q_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_rm].q[0] = XMM[cpu_reg].q[0];
        XMM[cpu_rm].q[1] = XMM[cpu_reg].q[1];
        CLOCK_CYCLES(1);
    } else {
        if (cpu_state.eaaddr & 0xf) {
            x86gpf(NULL, 0);
            if (cpu_state.abrt)
                return 1;
        }

        SEG_CHECK_WRITE(cpu_state.ea_seg);
        CHECK_WRITE_COMMON(cpu_state.ea_seg, cpu_state.eaaddr, cpu_state.eaaddr + 15);
        writememq(easeg, cpu_state.eaaddr, XMM[cpu_reg].q[0]);
        if (cpu_state.abrt)
            return 1;
        writememq(easeg, cpu_state.eaaddr + 8, XMM[cpu_reg].q[1]);
        if (cpu_state.abrt)
            return 1;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVDQU_xmm_q_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_rm].q[0] = XMM[cpu_reg].q[0];
        XMM[cpu_rm].q[1] = XMM[cpu_reg].q[1];
        CLOCK_CYCLES(1);
    } else {
        SEG_CHECK_WRITE(cpu_state.ea_seg);
        CHECK_WRITE_COMMON(cpu_state.ea_seg, cpu_state.eaaddr, cpu_state.eaaddr + 15);
        writememq(easeg, cpu_state.eaaddr, XMM[cpu_reg].q[0]);
        if (cpu_state.abrt)
            return 1;
        writememq(easeg, cpu_state.eaaddr + 8, XMM[cpu_reg].q[1]);
        if (cpu_state.abrt)
            return 1;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVDQU_xmm_q_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_rm].q[0] = XMM[cpu_reg].q[0];
        XMM[cpu_rm].q[1] = XMM[cpu_reg].q[1];
        CLOCK_CYCLES(1);
    } else {
        SEG_CHECK_WRITE(cpu_state.ea_seg);
        CHECK_WRITE_COMMON(cpu_state.ea_seg, cpu_state.eaaddr, cpu_state.eaaddr + 15);
        writememq(easeg, cpu_state.eaaddr, XMM[cpu_reg].q[0]);
        if (cpu_state.abrt)
            return 1;
        writememq(easeg, cpu_state.eaaddr + 8, XMM[cpu_reg].q[1]);
        if (cpu_state.abrt)
            return 1;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVNTI_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    ILLEGAL_ON(cpu_mod == 3);
    SEG_CHECK_WRITE(cpu_state.ea_seg);
    CHECK_WRITE_COMMON(cpu_state.ea_seg, cpu_state.eaaddr, cpu_state.eaaddr + 3);
    writememl(easeg, cpu_state.eaaddr, getr32(cpu_reg));
    if (cpu_state.abrt)
        return 1;
    CLOCK_CYCLES(1);
    return 0;
}

static int
opMOVNTI_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    ILLEGAL_ON(cpu_mod == 3);
    SEG_CHECK_WRITE(cpu_state.ea_seg);
    CHECK_WRITE_COMMON(cpu_state.ea_seg, cpu_state.eaaddr, cpu_state.eaaddr + 3);
    writememl(easeg, cpu_state.eaaddr, getr32(cpu_reg));
    if (cpu_state.abrt)
        return 1;
    CLOCK_CYCLES(1);
    return 0;
}

static int
opSHUFPD_xmm_w_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    uint8_t imm = getbyte();
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = XMM[cpu_rm].f[imm & 1];
        XMM[cpu_reg].d[1] = XMM[cpu_rm].f[(imm >> 1) & 1];
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
        XMM[cpu_reg].q[0] = src[imm & 1];
        XMM[cpu_reg].q[1] = src[(imm >> 1) & 1];
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opSHUFPD_xmm_w_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    uint8_t imm = getbyte();
    if (cpu_mod == 3) {
        XMM[cpu_reg].d[0] = XMM[cpu_rm].f[imm & 1];
        XMM[cpu_reg].d[1] = XMM[cpu_rm].f[(imm >> 1) & 1];
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
        XMM[cpu_reg].q[0] = src[imm & 1];
        XMM[cpu_reg].q[1] = src[(imm >> 1) & 1];
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVQ_q_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].q[0] = XMM[cpu_rm].q[0];
        XMM[cpu_reg].q[1] = 0;
        CLOCK_CYCLES(1);
    } else {
        SEG_CHECK_READ(cpu_state.ea_seg);
        CHECK_WRITE_COMMON(cpu_state.ea_seg, cpu_state.eaaddr, cpu_state.eaaddr + 7);
        XMM[cpu_reg].q[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].q[1] = 0;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVQ_q_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].q[0] = XMM[cpu_rm].q[0];
        XMM[cpu_reg].q[1] = 0;
        CLOCK_CYCLES(1);
    } else {
        SEG_CHECK_READ(cpu_state.ea_seg);
        CHECK_WRITE_COMMON(cpu_state.ea_seg, cpu_state.eaaddr, cpu_state.eaaddr + 7);
        XMM[cpu_reg].q[0] = readmemq(easeg, cpu_state.eaaddr);
        if (cpu_state.abrt)
            return 1;
        XMM[cpu_reg].q[1] = 0;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opMOVQ2DQ_a16(uint32_t fetchdat)
{
    MMX_ENTER();
    fetch_ea_16(fetchdat);
    ILLEGAL_ON(cpu_mod != 3);

    XMM[cpu_rm].q[0] = cpu_state.MM[cpu_reg].q;
    XMM[cpu_rm].q[1] = 0;
    CLOCK_CYCLES(1);
    return 0;
}

static int
opMOVQ2DQ_a32(uint32_t fetchdat)
{
    MMX_ENTER();
    fetch_ea_32(fetchdat);
    ILLEGAL_ON(cpu_mod != 3);

    XMM[cpu_rm].q[0] = cpu_state.MM[cpu_reg].q;
    XMM[cpu_rm].q[1] = 0;
    CLOCK_CYCLES(1);
    return 0;
}

static int
opMOVDQ2Q_a16(uint32_t fetchdat)
{
    MMX_ENTER();
    fetch_ea_16(fetchdat);
    ILLEGAL_ON(cpu_mod != 3);

    cpu_state.MM[cpu_rm].q = XMM[cpu_reg].q[0];
    CLOCK_CYCLES(1);
    return 0;
}

static int
opMOVDQ2Q_a32(uint32_t fetchdat)
{
    MMX_ENTER();
    fetch_ea_32(fetchdat);
    ILLEGAL_ON(cpu_mod != 3);

    cpu_state.MM[cpu_rm].q = XMM[cpu_reg].q[0];
    CLOCK_CYCLES(1);
    return 0;
}

static int
opMOVNTDQ_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    ILLEGAL_ON(cpu_mod == 3);

    SEG_CHECK_WRITE(cpu_state.ea_seg);
    CHECK_WRITE_COMMON(cpu_state.ea_seg, cpu_state.eaaddr, cpu_state.eaaddr + 15);
    writememq(easeg, cpu_state.eaaddr, XMM[cpu_reg].q[0]);
    if (cpu_state.abrt)
        return 1;
    writememq(easeg, cpu_state.eaaddr + 8, XMM[cpu_reg].q[1]);
    if (cpu_state.abrt)
        return 1;
    CLOCK_CYCLES(2);
    return 0;
}

static int
opMOVNTDQ_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    ILLEGAL_ON(cpu_mod == 3);

    SEG_CHECK_WRITE(cpu_state.ea_seg);
    CHECK_WRITE_COMMON(cpu_state.ea_seg, cpu_state.eaaddr, cpu_state.eaaddr + 15);
    writememq(easeg, cpu_state.eaaddr, XMM[cpu_reg].q[0]);
    if (cpu_state.abrt)
        return 1;
    writememq(easeg, cpu_state.eaaddr + 8, XMM[cpu_reg].q[1]);
    if (cpu_state.abrt)
        return 1;
    CLOCK_CYCLES(2);
    return 0;
}

static int
opMASKMOVDQU_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    ILLEGAL_ON(cpu_mod != 3);
    if (cpu_mod == 3) {
        SEG_CHECK_WRITE(cpu_state.ea_seg);
        if (XMM[cpu_rm].b[0] & (1 << 7)) {
            writememb(ds, DI, XMM[cpu_reg].b[0]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[1] & (1 << 7)) {
            writememb(ds, DI + 1, XMM[cpu_reg].b[1]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[2] & (1 << 7)) {
            writememb(ds, DI + 2, XMM[cpu_reg].b[2]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[3] & (1 << 7)) {
            writememb(ds, DI + 3, XMM[cpu_reg].b[3]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[4] & (1 << 7)) {
            writememb(ds, DI + 4, XMM[cpu_reg].b[4]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[5] & (1 << 7)) {
            writememb(ds, DI + 5, XMM[cpu_reg].b[5]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[6] & (1 << 7)) {
            writememb(ds, DI + 6, XMM[cpu_reg].b[6]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[7] & (1 << 7)) {
            writememb(ds, DI + 7, XMM[cpu_reg].b[7]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[8] & (1 << 7)) {
            writememb(ds, DI + 8, XMM[cpu_reg].b[8]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[9] & (1 << 7)) {
            writememb(ds, DI + 9, XMM[cpu_reg].b[9]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[10] & (1 << 7)) {
            writememb(ds, DI + 10, XMM[cpu_reg].b[10]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[11] & (1 << 7)) {
            writememb(ds, DI + 11, XMM[cpu_reg].b[11]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[12] & (1 << 7)) {
            writememb(ds, DI + 12, XMM[cpu_reg].b[12]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[13] & (1 << 7)) {
            writememb(ds, DI + 13, XMM[cpu_reg].b[13]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[14] & (1 << 7)) {
            writememb(ds, DI + 14, XMM[cpu_reg].b[14]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[15] & (1 << 7)) {
            writememb(ds, DI + 15, XMM[cpu_reg].b[15]);
            if (cpu_state.abrt)
                return 1;
        }
        CLOCK_CYCLES(1);
    }
    return 0;
}

static int
opMASKMOVDQU_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    ILLEGAL_ON(cpu_mod != 3);
    if (cpu_mod == 3) {
        SEG_CHECK_WRITE(cpu_state.ea_seg);
        if (XMM[cpu_rm].b[0] & (1 << 7)) {
            writememb(ds, DI, XMM[cpu_reg].b[0]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[1] & (1 << 7)) {
            writememb(ds, DI + 1, XMM[cpu_reg].b[1]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[2] & (1 << 7)) {
            writememb(ds, DI + 2, XMM[cpu_reg].b[2]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[3] & (1 << 7)) {
            writememb(ds, DI + 3, XMM[cpu_reg].b[3]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[4] & (1 << 7)) {
            writememb(ds, DI + 4, XMM[cpu_reg].b[4]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[5] & (1 << 7)) {
            writememb(ds, DI + 5, XMM[cpu_reg].b[5]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[6] & (1 << 7)) {
            writememb(ds, DI + 6, XMM[cpu_reg].b[6]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[7] & (1 << 7)) {
            writememb(ds, DI + 7, XMM[cpu_reg].b[7]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[8] & (1 << 7)) {
            writememb(ds, DI + 8, XMM[cpu_reg].b[8]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[9] & (1 << 7)) {
            writememb(ds, DI + 9, XMM[cpu_reg].b[9]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[10] & (1 << 7)) {
            writememb(ds, DI + 10, XMM[cpu_reg].b[10]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[11] & (1 << 7)) {
            writememb(ds, DI + 11, XMM[cpu_reg].b[11]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[12] & (1 << 7)) {
            writememb(ds, DI + 12, XMM[cpu_reg].b[12]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[13] & (1 << 7)) {
            writememb(ds, DI + 13, XMM[cpu_reg].b[13]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[14] & (1 << 7)) {
            writememb(ds, DI + 14, XMM[cpu_reg].b[14]);
            if (cpu_state.abrt)
                return 1;
        }
        if (XMM[cpu_rm].b[15] & (1 << 7)) {
            writememb(ds, DI + 15, XMM[cpu_reg].b[15]);
            if (cpu_state.abrt)
                return 1;
        }
        CLOCK_CYCLES(1);
    }
    return 0;
}
