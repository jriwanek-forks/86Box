static int
opPADDQ_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].q[0] += src.q[0];
    XMM[cpu_reg].q[1] += src.q[1];

    return 0;
}

static int
opPADDQ_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].q[0] += src.q[0];
    XMM[cpu_reg].q[1] += src.q[1];

    return 0;
}

static int
opPADDQ_a16(uint32_t fetchdat)
{
    if (sse_xmm)
        return opPADDQ_xmm_a16(fetchdat);
    MMX_REG src;
    MMX_ENTER();

    fetch_ea_16(fetchdat);
    MMX_GETSRC();

    cpu_state.MM[cpu_reg].q += src.q;

    return 0;
}

static int
opPADDQ_a32(uint32_t fetchdat)
{
    if (sse_xmm)
        return opPADDQ_xmm_a32(fetchdat);
    MMX_REG src;
    MMX_ENTER();

    fetch_ea_32(fetchdat);
    MMX_GETSRC();

    cpu_state.MM[cpu_reg].q += src.q;

    return 0;
}

static int
opPMULLW_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].w[0] *= XMM[cpu_rm].w[0];
        XMM[cpu_reg].w[1] *= XMM[cpu_rm].w[1];
        XMM[cpu_reg].w[2] *= XMM[cpu_rm].w[2];
        XMM[cpu_reg].w[3] *= XMM[cpu_rm].w[3];
        XMM[cpu_reg].w[4] *= XMM[cpu_rm].w[4];
        XMM[cpu_reg].w[5] *= XMM[cpu_rm].w[5];
        XMM[cpu_reg].w[6] *= XMM[cpu_rm].w[6];
        XMM[cpu_reg].w[7] *= XMM[cpu_rm].w[7];
        CLOCK_CYCLES(1);
    } else {
        SSE_REG src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src.q[0] = readmemq(easeg, cpu_state.eaaddr);
        src.q[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 0;
        XMM[cpu_reg].w[0] *= src.w[0];
        XMM[cpu_reg].w[1] *= src.w[1];
        XMM[cpu_reg].w[2] *= src.w[2];
        XMM[cpu_reg].w[3] *= src.w[3];
        XMM[cpu_reg].w[4] *= src.w[4];
        XMM[cpu_reg].w[5] *= src.w[5];
        XMM[cpu_reg].w[6] *= src.w[6];
        XMM[cpu_reg].w[7] *= src.w[7];
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opPMULLW_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].w[0] *= XMM[cpu_rm].w[0];
        XMM[cpu_reg].w[1] *= XMM[cpu_rm].w[1];
        XMM[cpu_reg].w[2] *= XMM[cpu_rm].w[2];
        XMM[cpu_reg].w[3] *= XMM[cpu_rm].w[3];
        XMM[cpu_reg].w[4] *= XMM[cpu_rm].w[4];
        XMM[cpu_reg].w[5] *= XMM[cpu_rm].w[5];
        XMM[cpu_reg].w[6] *= XMM[cpu_rm].w[6];
        XMM[cpu_reg].w[7] *= XMM[cpu_rm].w[7];
        CLOCK_CYCLES(1);
    } else {
        SSE_REG src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src.q[0] = readmemq(easeg, cpu_state.eaaddr);
        src.q[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 0;
        XMM[cpu_reg].w[0] *= src.w[0];
        XMM[cpu_reg].w[1] *= src.w[1];
        XMM[cpu_reg].w[2] *= src.w[2];
        XMM[cpu_reg].w[3] *= src.w[3];
        XMM[cpu_reg].w[4] *= src.w[4];
        XMM[cpu_reg].w[5] *= src.w[5];
        XMM[cpu_reg].w[6] *= src.w[6];
        XMM[cpu_reg].w[7] *= src.w[7];
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opPSUBUSB_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].b[0]  = USATB(XMM[cpu_reg].b[0] - src.b[0]);
    XMM[cpu_reg].b[1]  = USATB(XMM[cpu_reg].b[1] - src.b[1]);
    XMM[cpu_reg].b[2]  = USATB(XMM[cpu_reg].b[2] - src.b[2]);
    XMM[cpu_reg].b[3]  = USATB(XMM[cpu_reg].b[3] - src.b[3]);
    XMM[cpu_reg].b[4]  = USATB(XMM[cpu_reg].b[4] - src.b[4]);
    XMM[cpu_reg].b[5]  = USATB(XMM[cpu_reg].b[5] - src.b[5]);
    XMM[cpu_reg].b[6]  = USATB(XMM[cpu_reg].b[6] - src.b[6]);
    XMM[cpu_reg].b[7]  = USATB(XMM[cpu_reg].b[7] - src.b[7]);
    XMM[cpu_reg].b[8]  = USATB(XMM[cpu_reg].b[8] - src.b[8]);
    XMM[cpu_reg].b[9]  = USATB(XMM[cpu_reg].b[9] - src.b[9]);
    XMM[cpu_reg].b[10] = USATB(XMM[cpu_reg].b[10] - src.b[10]);
    XMM[cpu_reg].b[11] = USATB(XMM[cpu_reg].b[11] - src.b[11]);
    XMM[cpu_reg].b[12] = USATB(XMM[cpu_reg].b[12] - src.b[12]);
    XMM[cpu_reg].b[13] = USATB(XMM[cpu_reg].b[13] - src.b[13]);
    XMM[cpu_reg].b[14] = USATB(XMM[cpu_reg].b[14] - src.b[14]);
    XMM[cpu_reg].b[15] = USATB(XMM[cpu_reg].b[15] - src.b[15]);

    return 0;
}

static int
opPSUBUSB_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].b[0]  = USATB(XMM[cpu_reg].b[0] - src.b[0]);
    XMM[cpu_reg].b[1]  = USATB(XMM[cpu_reg].b[1] - src.b[1]);
    XMM[cpu_reg].b[2]  = USATB(XMM[cpu_reg].b[2] - src.b[2]);
    XMM[cpu_reg].b[3]  = USATB(XMM[cpu_reg].b[3] - src.b[3]);
    XMM[cpu_reg].b[4]  = USATB(XMM[cpu_reg].b[4] - src.b[4]);
    XMM[cpu_reg].b[5]  = USATB(XMM[cpu_reg].b[5] - src.b[5]);
    XMM[cpu_reg].b[6]  = USATB(XMM[cpu_reg].b[6] - src.b[6]);
    XMM[cpu_reg].b[7]  = USATB(XMM[cpu_reg].b[7] - src.b[7]);
    XMM[cpu_reg].b[8]  = USATB(XMM[cpu_reg].b[8] - src.b[8]);
    XMM[cpu_reg].b[9]  = USATB(XMM[cpu_reg].b[9] - src.b[9]);
    XMM[cpu_reg].b[10] = USATB(XMM[cpu_reg].b[10] - src.b[10]);
    XMM[cpu_reg].b[11] = USATB(XMM[cpu_reg].b[11] - src.b[11]);
    XMM[cpu_reg].b[12] = USATB(XMM[cpu_reg].b[12] - src.b[12]);
    XMM[cpu_reg].b[13] = USATB(XMM[cpu_reg].b[13] - src.b[13]);
    XMM[cpu_reg].b[14] = USATB(XMM[cpu_reg].b[14] - src.b[14]);
    XMM[cpu_reg].b[15] = USATB(XMM[cpu_reg].b[15] - src.b[15]);

    return 0;
}

static int
opPSUBUSW_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].w[0] = USATB(XMM[cpu_reg].w[0] - src.w[0]);
    XMM[cpu_reg].w[1] = USATB(XMM[cpu_reg].w[1] - src.w[1]);
    XMM[cpu_reg].w[2] = USATB(XMM[cpu_reg].w[2] - src.w[2]);
    XMM[cpu_reg].w[3] = USATB(XMM[cpu_reg].w[3] - src.w[3]);
    XMM[cpu_reg].w[4] = USATB(XMM[cpu_reg].w[4] - src.w[4]);
    XMM[cpu_reg].w[5] = USATB(XMM[cpu_reg].w[5] - src.w[5]);
    XMM[cpu_reg].w[6] = USATB(XMM[cpu_reg].w[6] - src.w[6]);
    XMM[cpu_reg].w[7] = USATB(XMM[cpu_reg].w[7] - src.w[7]);

    return 0;
}

static int
opPSUBUSW_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].w[0] = USATB(XMM[cpu_reg].w[0] - src.w[0]);
    XMM[cpu_reg].w[1] = USATB(XMM[cpu_reg].w[1] - src.w[1]);
    XMM[cpu_reg].w[2] = USATB(XMM[cpu_reg].w[2] - src.w[2]);
    XMM[cpu_reg].w[3] = USATB(XMM[cpu_reg].w[3] - src.w[3]);
    XMM[cpu_reg].w[4] = USATB(XMM[cpu_reg].w[4] - src.w[4]);
    XMM[cpu_reg].w[5] = USATB(XMM[cpu_reg].w[5] - src.w[5]);
    XMM[cpu_reg].w[6] = USATB(XMM[cpu_reg].w[6] - src.w[6]);
    XMM[cpu_reg].w[7] = USATB(XMM[cpu_reg].w[7] - src.w[7]);

    return 0;
}

static int
opPADDUSB_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].b[0]  = USATB(XMM[cpu_reg].b[0] + src.b[0]);
    XMM[cpu_reg].b[1]  = USATB(XMM[cpu_reg].b[1] + src.b[1]);
    XMM[cpu_reg].b[2]  = USATB(XMM[cpu_reg].b[2] + src.b[2]);
    XMM[cpu_reg].b[3]  = USATB(XMM[cpu_reg].b[3] + src.b[3]);
    XMM[cpu_reg].b[4]  = USATB(XMM[cpu_reg].b[4] + src.b[4]);
    XMM[cpu_reg].b[5]  = USATB(XMM[cpu_reg].b[5] + src.b[5]);
    XMM[cpu_reg].b[6]  = USATB(XMM[cpu_reg].b[6] + src.b[6]);
    XMM[cpu_reg].b[7]  = USATB(XMM[cpu_reg].b[7] + src.b[7]);
    XMM[cpu_reg].b[8]  = USATB(XMM[cpu_reg].b[8] + src.b[8]);
    XMM[cpu_reg].b[9]  = USATB(XMM[cpu_reg].b[9] + src.b[9]);
    XMM[cpu_reg].b[10] = USATB(XMM[cpu_reg].b[10] + src.b[10]);
    XMM[cpu_reg].b[11] = USATB(XMM[cpu_reg].b[11] + src.b[11]);
    XMM[cpu_reg].b[12] = USATB(XMM[cpu_reg].b[12] + src.b[12]);
    XMM[cpu_reg].b[13] = USATB(XMM[cpu_reg].b[13] + src.b[13]);
    XMM[cpu_reg].b[14] = USATB(XMM[cpu_reg].b[14] + src.b[14]);
    XMM[cpu_reg].b[15] = USATB(XMM[cpu_reg].b[15] + src.b[15]);

    return 0;
}

static int
opPADDUSB_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].b[0]  = USATB(XMM[cpu_reg].b[0] + src.b[0]);
    XMM[cpu_reg].b[1]  = USATB(XMM[cpu_reg].b[1] + src.b[1]);
    XMM[cpu_reg].b[2]  = USATB(XMM[cpu_reg].b[2] + src.b[2]);
    XMM[cpu_reg].b[3]  = USATB(XMM[cpu_reg].b[3] + src.b[3]);
    XMM[cpu_reg].b[4]  = USATB(XMM[cpu_reg].b[4] + src.b[4]);
    XMM[cpu_reg].b[5]  = USATB(XMM[cpu_reg].b[5] + src.b[5]);
    XMM[cpu_reg].b[6]  = USATB(XMM[cpu_reg].b[6] + src.b[6]);
    XMM[cpu_reg].b[7]  = USATB(XMM[cpu_reg].b[7] + src.b[7]);
    XMM[cpu_reg].b[8]  = USATB(XMM[cpu_reg].b[8] + src.b[8]);
    XMM[cpu_reg].b[9]  = USATB(XMM[cpu_reg].b[9] + src.b[9]);
    XMM[cpu_reg].b[10] = USATB(XMM[cpu_reg].b[10] + src.b[10]);
    XMM[cpu_reg].b[11] = USATB(XMM[cpu_reg].b[11] + src.b[11]);
    XMM[cpu_reg].b[12] = USATB(XMM[cpu_reg].b[12] + src.b[12]);
    XMM[cpu_reg].b[13] = USATB(XMM[cpu_reg].b[13] + src.b[13]);
    XMM[cpu_reg].b[14] = USATB(XMM[cpu_reg].b[14] + src.b[14]);
    XMM[cpu_reg].b[15] = USATB(XMM[cpu_reg].b[15] + src.b[15]);
    return 0;
}

static int
opPADDUSW_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].w[0] = USATB(XMM[cpu_reg].w[0] + src.w[0]);
    XMM[cpu_reg].w[1] = USATB(XMM[cpu_reg].w[1] + src.w[1]);
    XMM[cpu_reg].w[2] = USATB(XMM[cpu_reg].w[2] + src.w[2]);
    XMM[cpu_reg].w[3] = USATB(XMM[cpu_reg].w[3] + src.w[3]);
    XMM[cpu_reg].w[4] = USATB(XMM[cpu_reg].w[4] + src.w[4]);
    XMM[cpu_reg].w[5] = USATB(XMM[cpu_reg].w[5] + src.w[5]);
    XMM[cpu_reg].w[6] = USATB(XMM[cpu_reg].w[6] + src.w[6]);
    XMM[cpu_reg].w[7] = USATB(XMM[cpu_reg].w[7] + src.w[7]);

    return 0;
}

static int
opPADDUSW_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].w[0] = USATB(XMM[cpu_reg].w[0] + src.w[0]);
    XMM[cpu_reg].w[1] = USATB(XMM[cpu_reg].w[1] + src.w[1]);
    XMM[cpu_reg].w[2] = USATB(XMM[cpu_reg].w[2] + src.w[2]);
    XMM[cpu_reg].w[3] = USATB(XMM[cpu_reg].w[3] + src.w[3]);
    XMM[cpu_reg].w[4] = USATB(XMM[cpu_reg].w[4] + src.w[4]);
    XMM[cpu_reg].w[5] = USATB(XMM[cpu_reg].w[5] + src.w[5]);
    XMM[cpu_reg].w[6] = USATB(XMM[cpu_reg].w[6] + src.w[6]);
    XMM[cpu_reg].w[7] = USATB(XMM[cpu_reg].w[7] + src.w[7]);

    return 0;
}

static int
opPMULHW_xmm_a16(uint32_t fetchdat)
{
    fetch_ea_16(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].w[0] = ((int32_t) XMM[cpu_reg].sw[0] * (int32_t) XMM[cpu_rm].sw[0]) >> 16;
        XMM[cpu_reg].w[1] = ((int32_t) XMM[cpu_reg].sw[1] * (int32_t) XMM[cpu_rm].sw[1]) >> 16;
        XMM[cpu_reg].w[2] = ((int32_t) XMM[cpu_reg].sw[2] * (int32_t) XMM[cpu_rm].sw[2]) >> 16;
        XMM[cpu_reg].w[3] = ((int32_t) XMM[cpu_reg].sw[3] * (int32_t) XMM[cpu_rm].sw[3]) >> 16;
        XMM[cpu_reg].w[4] = ((int32_t) XMM[cpu_reg].sw[4] * (int32_t) XMM[cpu_rm].sw[4]) >> 16;
        XMM[cpu_reg].w[5] = ((int32_t) XMM[cpu_reg].sw[5] * (int32_t) XMM[cpu_rm].sw[5]) >> 16;
        XMM[cpu_reg].w[6] = ((int32_t) XMM[cpu_reg].sw[6] * (int32_t) XMM[cpu_rm].sw[6]) >> 16;
        XMM[cpu_reg].w[7] = ((int32_t) XMM[cpu_reg].sw[7] * (int32_t) XMM[cpu_rm].sw[7]) >> 16;
        CLOCK_CYCLES(1);
    } else {
        SSE_REG src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src.q[0] = readmemq(easeg, cpu_state.eaaddr);
        src.q[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 0;
        XMM[cpu_reg].w[0] = ((int32_t) XMM[cpu_reg].sw[0] * (int32_t) src.sw[0]) >> 16;
        XMM[cpu_reg].w[1] = ((int32_t) XMM[cpu_reg].sw[1] * (int32_t) src.sw[1]) >> 16;
        XMM[cpu_reg].w[2] = ((int32_t) XMM[cpu_reg].sw[2] * (int32_t) src.sw[2]) >> 16;
        XMM[cpu_reg].w[3] = ((int32_t) XMM[cpu_reg].sw[3] * (int32_t) src.sw[3]) >> 16;
        XMM[cpu_reg].w[4] = ((int32_t) XMM[cpu_reg].sw[4] * (int32_t) src.sw[4]) >> 16;
        XMM[cpu_reg].w[5] = ((int32_t) XMM[cpu_reg].sw[5] * (int32_t) src.sw[5]) >> 16;
        XMM[cpu_reg].w[6] = ((int32_t) XMM[cpu_reg].sw[6] * (int32_t) src.sw[6]) >> 16;
        XMM[cpu_reg].w[7] = ((int32_t) XMM[cpu_reg].sw[7] * (int32_t) src.sw[7]) >> 16;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opPMULHW_xmm_a32(uint32_t fetchdat)
{
    fetch_ea_32(fetchdat);
    if (cpu_mod == 3) {
        XMM[cpu_reg].w[0] = ((int32_t) XMM[cpu_reg].sw[0] * (int32_t) XMM[cpu_rm].sw[0]) >> 16;
        XMM[cpu_reg].w[1] = ((int32_t) XMM[cpu_reg].sw[1] * (int32_t) XMM[cpu_rm].sw[1]) >> 16;
        XMM[cpu_reg].w[2] = ((int32_t) XMM[cpu_reg].sw[2] * (int32_t) XMM[cpu_rm].sw[2]) >> 16;
        XMM[cpu_reg].w[3] = ((int32_t) XMM[cpu_reg].sw[3] * (int32_t) XMM[cpu_rm].sw[3]) >> 16;
        XMM[cpu_reg].w[4] = ((int32_t) XMM[cpu_reg].sw[4] * (int32_t) XMM[cpu_rm].sw[4]) >> 16;
        XMM[cpu_reg].w[5] = ((int32_t) XMM[cpu_reg].sw[5] * (int32_t) XMM[cpu_rm].sw[5]) >> 16;
        XMM[cpu_reg].w[6] = ((int32_t) XMM[cpu_reg].sw[6] * (int32_t) XMM[cpu_rm].sw[6]) >> 16;
        XMM[cpu_reg].w[7] = ((int32_t) XMM[cpu_reg].sw[7] * (int32_t) XMM[cpu_rm].sw[7]) >> 16;
        CLOCK_CYCLES(1);
    } else {
        SSE_REG src;

        SEG_CHECK_READ(cpu_state.ea_seg);
        src.q[0] = readmemq(easeg, cpu_state.eaaddr);
        src.q[1] = readmemq(easeg, cpu_state.eaaddr + 8);
        if (cpu_state.abrt)
            return 0;
        XMM[cpu_reg].w[0] = ((int32_t) XMM[cpu_reg].sw[0] * (int32_t) src.sw[0]) >> 16;
        XMM[cpu_reg].w[1] = ((int32_t) XMM[cpu_reg].sw[1] * (int32_t) src.sw[1]) >> 16;
        XMM[cpu_reg].w[2] = ((int32_t) XMM[cpu_reg].sw[2] * (int32_t) src.sw[2]) >> 16;
        XMM[cpu_reg].w[3] = ((int32_t) XMM[cpu_reg].sw[3] * (int32_t) src.sw[3]) >> 16;
        XMM[cpu_reg].w[4] = ((int32_t) XMM[cpu_reg].sw[4] * (int32_t) src.sw[4]) >> 16;
        XMM[cpu_reg].w[5] = ((int32_t) XMM[cpu_reg].sw[5] * (int32_t) src.sw[5]) >> 16;
        XMM[cpu_reg].w[6] = ((int32_t) XMM[cpu_reg].sw[6] * (int32_t) src.sw[6]) >> 16;
        XMM[cpu_reg].w[7] = ((int32_t) XMM[cpu_reg].sw[7] * (int32_t) src.sw[7]) >> 16;
        CLOCK_CYCLES(2);
    }
    return 0;
}

static int
opPSUBSB_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].sb[0]  = SSATB(XMM[cpu_reg].sb[0] - src.sb[0]);
    XMM[cpu_reg].sb[1]  = SSATB(XMM[cpu_reg].sb[1] - src.sb[1]);
    XMM[cpu_reg].sb[2]  = SSATB(XMM[cpu_reg].sb[2] - src.sb[2]);
    XMM[cpu_reg].sb[3]  = SSATB(XMM[cpu_reg].sb[3] - src.sb[3]);
    XMM[cpu_reg].sb[4]  = SSATB(XMM[cpu_reg].sb[4] - src.sb[4]);
    XMM[cpu_reg].sb[5]  = SSATB(XMM[cpu_reg].sb[5] - src.sb[5]);
    XMM[cpu_reg].sb[6]  = SSATB(XMM[cpu_reg].sb[6] - src.sb[6]);
    XMM[cpu_reg].sb[7]  = SSATB(XMM[cpu_reg].sb[7] - src.sb[7]);
    XMM[cpu_reg].sb[8]  = SSATB(XMM[cpu_reg].sb[8] - src.sb[8]);
    XMM[cpu_reg].sb[9]  = SSATB(XMM[cpu_reg].sb[9] - src.sb[9]);
    XMM[cpu_reg].sb[10] = SSATB(XMM[cpu_reg].sb[10] - src.sb[10]);
    XMM[cpu_reg].sb[11] = SSATB(XMM[cpu_reg].sb[11] - src.sb[11]);
    XMM[cpu_reg].sb[12] = SSATB(XMM[cpu_reg].sb[12] - src.sb[12]);
    XMM[cpu_reg].sb[13] = SSATB(XMM[cpu_reg].sb[13] - src.sb[13]);
    XMM[cpu_reg].sb[14] = SSATB(XMM[cpu_reg].sb[14] - src.sb[14]);
    XMM[cpu_reg].sb[15] = SSATB(XMM[cpu_reg].sb[15] - src.sb[15]);

    return 0;
}

static int
opPSUBSB_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].sb[0]  = SSATB(XMM[cpu_reg].sb[0] - src.sb[0]);
    XMM[cpu_reg].sb[1]  = SSATB(XMM[cpu_reg].sb[1] - src.sb[1]);
    XMM[cpu_reg].sb[2]  = SSATB(XMM[cpu_reg].sb[2] - src.sb[2]);
    XMM[cpu_reg].sb[3]  = SSATB(XMM[cpu_reg].sb[3] - src.sb[3]);
    XMM[cpu_reg].sb[4]  = SSATB(XMM[cpu_reg].sb[4] - src.sb[4]);
    XMM[cpu_reg].sb[5]  = SSATB(XMM[cpu_reg].sb[5] - src.sb[5]);
    XMM[cpu_reg].sb[6]  = SSATB(XMM[cpu_reg].sb[6] - src.sb[6]);
    XMM[cpu_reg].sb[7]  = SSATB(XMM[cpu_reg].sb[7] - src.sb[7]);
    XMM[cpu_reg].sb[8]  = SSATB(XMM[cpu_reg].sb[8] - src.sb[8]);
    XMM[cpu_reg].sb[9]  = SSATB(XMM[cpu_reg].sb[9] - src.sb[9]);
    XMM[cpu_reg].sb[10] = SSATB(XMM[cpu_reg].sb[10] - src.sb[10]);
    XMM[cpu_reg].sb[11] = SSATB(XMM[cpu_reg].sb[11] - src.sb[11]);
    XMM[cpu_reg].sb[12] = SSATB(XMM[cpu_reg].sb[12] - src.sb[12]);
    XMM[cpu_reg].sb[13] = SSATB(XMM[cpu_reg].sb[13] - src.sb[13]);
    XMM[cpu_reg].sb[14] = SSATB(XMM[cpu_reg].sb[14] - src.sb[14]);
    XMM[cpu_reg].sb[15] = SSATB(XMM[cpu_reg].sb[15] - src.sb[15]);

    return 0;
}

static int
opPSUBSW_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].sw[0] = SSATB(XMM[cpu_reg].sw[0] - src.sw[0]);
    XMM[cpu_reg].sw[1] = SSATB(XMM[cpu_reg].sw[1] - src.sw[1]);
    XMM[cpu_reg].sw[2] = SSATB(XMM[cpu_reg].sw[2] - src.sw[2]);
    XMM[cpu_reg].sw[3] = SSATB(XMM[cpu_reg].sw[3] - src.sw[3]);
    XMM[cpu_reg].sw[4] = SSATB(XMM[cpu_reg].sw[4] - src.sw[4]);
    XMM[cpu_reg].sw[5] = SSATB(XMM[cpu_reg].sw[5] - src.sw[5]);
    XMM[cpu_reg].sw[6] = SSATB(XMM[cpu_reg].sw[6] - src.sw[6]);
    XMM[cpu_reg].sw[7] = SSATB(XMM[cpu_reg].sw[7] - src.sw[7]);

    return 0;
}

static int
opPSUBSW_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].sw[0] = SSATB(XMM[cpu_reg].sw[0] - src.sw[0]);
    XMM[cpu_reg].sw[1] = SSATB(XMM[cpu_reg].sw[1] - src.sw[1]);
    XMM[cpu_reg].sw[2] = SSATB(XMM[cpu_reg].sw[2] - src.sw[2]);
    XMM[cpu_reg].sw[3] = SSATB(XMM[cpu_reg].sw[3] - src.sw[3]);
    XMM[cpu_reg].sw[4] = SSATB(XMM[cpu_reg].sw[4] - src.sw[4]);
    XMM[cpu_reg].sw[5] = SSATB(XMM[cpu_reg].sw[5] - src.sw[5]);
    XMM[cpu_reg].sw[6] = SSATB(XMM[cpu_reg].sw[6] - src.sw[6]);
    XMM[cpu_reg].sw[7] = SSATB(XMM[cpu_reg].sw[7] - src.sw[7]);

    return 0;
}

static int
opPADDSB_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].sb[0]  = SSATB(XMM[cpu_reg].sb[0] + src.sb[0]);
    XMM[cpu_reg].sb[1]  = SSATB(XMM[cpu_reg].sb[1] + src.sb[1]);
    XMM[cpu_reg].sb[2]  = SSATB(XMM[cpu_reg].sb[2] + src.sb[2]);
    XMM[cpu_reg].sb[3]  = SSATB(XMM[cpu_reg].sb[3] + src.sb[3]);
    XMM[cpu_reg].sb[4]  = SSATB(XMM[cpu_reg].sb[4] + src.sb[4]);
    XMM[cpu_reg].sb[5]  = SSATB(XMM[cpu_reg].sb[5] + src.sb[5]);
    XMM[cpu_reg].sb[6]  = SSATB(XMM[cpu_reg].sb[6] + src.sb[6]);
    XMM[cpu_reg].sb[7]  = SSATB(XMM[cpu_reg].sb[7] + src.sb[7]);
    XMM[cpu_reg].sb[8]  = SSATB(XMM[cpu_reg].sb[8] + src.sb[8]);
    XMM[cpu_reg].sb[9]  = SSATB(XMM[cpu_reg].sb[9] + src.sb[9]);
    XMM[cpu_reg].sb[10] = SSATB(XMM[cpu_reg].sb[10] + src.sb[10]);
    XMM[cpu_reg].sb[11] = SSATB(XMM[cpu_reg].sb[11] + src.sb[11]);
    XMM[cpu_reg].sb[12] = SSATB(XMM[cpu_reg].sb[12] + src.sb[12]);
    XMM[cpu_reg].sb[13] = SSATB(XMM[cpu_reg].sb[13] + src.sb[13]);
    XMM[cpu_reg].sb[14] = SSATB(XMM[cpu_reg].sb[14] + src.sb[14]);
    XMM[cpu_reg].sb[15] = SSATB(XMM[cpu_reg].sb[15] + src.sb[15]);

    return 0;
}

static int
opPADDSB_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].sb[0]  = SSATB(XMM[cpu_reg].sb[0] + src.sb[0]);
    XMM[cpu_reg].sb[1]  = SSATB(XMM[cpu_reg].sb[1] + src.sb[1]);
    XMM[cpu_reg].sb[2]  = SSATB(XMM[cpu_reg].sb[2] + src.sb[2]);
    XMM[cpu_reg].sb[3]  = SSATB(XMM[cpu_reg].sb[3] + src.sb[3]);
    XMM[cpu_reg].sb[4]  = SSATB(XMM[cpu_reg].sb[4] + src.sb[4]);
    XMM[cpu_reg].sb[5]  = SSATB(XMM[cpu_reg].sb[5] + src.sb[5]);
    XMM[cpu_reg].sb[6]  = SSATB(XMM[cpu_reg].sb[6] + src.sb[6]);
    XMM[cpu_reg].sb[7]  = SSATB(XMM[cpu_reg].sb[7] + src.sb[7]);
    XMM[cpu_reg].sb[8]  = SSATB(XMM[cpu_reg].sb[8] + src.sb[8]);
    XMM[cpu_reg].sb[9]  = SSATB(XMM[cpu_reg].sb[9] + src.sb[9]);
    XMM[cpu_reg].sb[10] = SSATB(XMM[cpu_reg].sb[10] + src.sb[10]);
    XMM[cpu_reg].sb[11] = SSATB(XMM[cpu_reg].sb[11] + src.sb[11]);
    XMM[cpu_reg].sb[12] = SSATB(XMM[cpu_reg].sb[12] + src.sb[12]);
    XMM[cpu_reg].sb[13] = SSATB(XMM[cpu_reg].sb[13] + src.sb[13]);
    XMM[cpu_reg].sb[14] = SSATB(XMM[cpu_reg].sb[14] + src.sb[14]);
    XMM[cpu_reg].sb[15] = SSATB(XMM[cpu_reg].sb[15] + src.sb[15]);

    return 0;
}

static int
opPADDSW_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].sw[0] = SSATB(XMM[cpu_reg].sw[0] + src.sw[0]);
    XMM[cpu_reg].sw[1] = SSATB(XMM[cpu_reg].sw[1] + src.sw[1]);
    XMM[cpu_reg].sw[2] = SSATB(XMM[cpu_reg].sw[2] + src.sw[2]);
    XMM[cpu_reg].sw[3] = SSATB(XMM[cpu_reg].sw[3] + src.sw[3]);
    XMM[cpu_reg].sw[4] = SSATB(XMM[cpu_reg].sw[4] + src.sw[4]);
    XMM[cpu_reg].sw[5] = SSATB(XMM[cpu_reg].sw[5] + src.sw[5]);
    XMM[cpu_reg].sw[6] = SSATB(XMM[cpu_reg].sw[6] + src.sw[6]);
    XMM[cpu_reg].sw[7] = SSATB(XMM[cpu_reg].sw[7] + src.sw[7]);

    return 0;
}

static int
opPADDSW_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].sw[0] = SSATB(XMM[cpu_reg].sw[0] + src.sw[0]);
    XMM[cpu_reg].sw[1] = SSATB(XMM[cpu_reg].sw[1] + src.sw[1]);
    XMM[cpu_reg].sw[2] = SSATB(XMM[cpu_reg].sw[2] + src.sw[2]);
    XMM[cpu_reg].sw[3] = SSATB(XMM[cpu_reg].sw[3] + src.sw[3]);
    XMM[cpu_reg].sw[4] = SSATB(XMM[cpu_reg].sw[4] + src.sw[4]);
    XMM[cpu_reg].sw[5] = SSATB(XMM[cpu_reg].sw[5] + src.sw[5]);
    XMM[cpu_reg].sw[6] = SSATB(XMM[cpu_reg].sw[6] + src.sw[6]);
    XMM[cpu_reg].sw[7] = SSATB(XMM[cpu_reg].sw[7] + src.sw[7]);

    return 0;
}

static int
opPMULUDQ_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].q[0] = (uint64_t) XMM[cpu_reg].l[0] * (uint64_t) src.l[0];
    XMM[cpu_reg].q[1] = (uint64_t) XMM[cpu_reg].l[2] * (uint64_t) src.l[2];

    return 0;
}

static int
opPMULUDQ_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].q[0] = (uint64_t) XMM[cpu_reg].l[0] * (uint64_t) src.l[0];
    XMM[cpu_reg].q[1] = (uint64_t) XMM[cpu_reg].l[2] * (uint64_t) src.l[2];

    return 0;
}

static int
opPMULUDQ_mm_a16(uint32_t fetchdat)
{
    if (sse_xmm)
        return opPMULUDQ_xmm_a16(fetchdat);
    MMX_REG src;

    fetch_ea_16(fetchdat);
    MMX_GETSRC();

    cpu_state.MM[cpu_reg].q = (uint64_t) cpu_state.MM[cpu_reg].l[0] * (uint64_t) src.l[0];

    return 0;
}

static int
opPMULUDQ_mm_a32(uint32_t fetchdat)
{
    if (sse_xmm)
        return opPMULUDQ_xmm_a32(fetchdat);
    MMX_REG src;

    fetch_ea_32(fetchdat);
    MMX_GETSRC();

    cpu_state.MM[cpu_reg].q = (uint64_t) cpu_state.MM[cpu_reg].l[0] * (uint64_t) src.l[0];

    return 0;
}

static int
opPMADDWD_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    if (XMM[cpu_reg].l[0] == 0x80008000 && src.l[0] == 0x80008000)
        XMM[cpu_reg].l[0] = 0x80000000;
    else
        XMM[cpu_reg].sl[0] = ((int32_t) XMM[cpu_reg].sw[0] * (int32_t) src.sw[0]) + ((int32_t) XMM[cpu_reg].sw[1] * (int32_t) src.sw[1]);

    if (XMM[cpu_reg].l[1] == 0x80008000 && src.l[1] == 0x80008000)
        XMM[cpu_reg].l[1] = 0x80000000;
    else
        XMM[cpu_reg].sl[1] = ((int32_t) XMM[cpu_reg].sw[2] * (int32_t) src.sw[2]) + ((int32_t) XMM[cpu_reg].sw[3] * (int32_t) src.sw[3]);

    if (XMM[cpu_reg].l[2] == 0x80008000 && src.l[2] == 0x80008000)
        XMM[cpu_reg].l[2] = 0x80000000;
    else
        XMM[cpu_reg].sl[2] = ((int32_t) XMM[cpu_reg].sw[4] * (int32_t) src.sw[4]) + ((int32_t) XMM[cpu_reg].sw[5] * (int32_t) src.sw[5]);

    if (XMM[cpu_reg].l[3] == 0x80008000 && src.l[3] == 0x80008000)
        XMM[cpu_reg].l[3] = 0x80000000;
    else
        XMM[cpu_reg].sl[3] = ((int32_t) XMM[cpu_reg].sw[6] * (int32_t) src.sw[6]) + ((int32_t) XMM[cpu_reg].sw[7] * (int32_t) src.sw[7]);

    return 0;
}

static int
opPMADDWD_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    if (XMM[cpu_reg].l[0] == 0x80008000 && src.l[0] == 0x80008000)
        XMM[cpu_reg].l[0] = 0x80000000;
    else
        XMM[cpu_reg].sl[0] = ((int32_t) XMM[cpu_reg].sw[0] * (int32_t) src.sw[0]) + ((int32_t) XMM[cpu_reg].sw[1] * (int32_t) src.sw[1]);

    if (XMM[cpu_reg].l[1] == 0x80008000 && src.l[1] == 0x80008000)
        XMM[cpu_reg].l[1] = 0x80000000;
    else
        XMM[cpu_reg].sl[1] = ((int32_t) XMM[cpu_reg].sw[2] * (int32_t) src.sw[2]) + ((int32_t) XMM[cpu_reg].sw[3] * (int32_t) src.sw[3]);

    if (XMM[cpu_reg].l[2] == 0x80008000 && src.l[2] == 0x80008000)
        XMM[cpu_reg].l[2] = 0x80000000;
    else
        XMM[cpu_reg].sl[2] = ((int32_t) XMM[cpu_reg].sw[4] * (int32_t) src.sw[4]) + ((int32_t) XMM[cpu_reg].sw[5] * (int32_t) src.sw[5]);

    if (XMM[cpu_reg].l[3] == 0x80008000 && src.l[3] == 0x80008000)
        XMM[cpu_reg].l[3] = 0x80000000;
    else
        XMM[cpu_reg].sl[3] = ((int32_t) XMM[cpu_reg].sw[6] * (int32_t) src.sw[6]) + ((int32_t) XMM[cpu_reg].sw[7] * (int32_t) src.sw[7]);

    return 0;
}

static int
opPSUBB_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].b[0] -= src.b[0];
    XMM[cpu_reg].b[1] -= src.b[1];
    XMM[cpu_reg].b[2] -= src.b[2];
    XMM[cpu_reg].b[3] -= src.b[3];
    XMM[cpu_reg].b[4] -= src.b[4];
    XMM[cpu_reg].b[5] -= src.b[5];
    XMM[cpu_reg].b[6] -= src.b[6];
    XMM[cpu_reg].b[7] -= src.b[7];
    XMM[cpu_reg].b[8] -= src.b[8];
    XMM[cpu_reg].b[9] -= src.b[9];
    XMM[cpu_reg].b[10] -= src.b[10];
    XMM[cpu_reg].b[11] -= src.b[11];
    XMM[cpu_reg].b[12] -= src.b[12];
    XMM[cpu_reg].b[13] -= src.b[13];
    XMM[cpu_reg].b[14] -= src.b[14];
    XMM[cpu_reg].b[15] -= src.b[15];

    return 0;
}

static int
opPSUBB_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].b[0] -= src.b[0];
    XMM[cpu_reg].b[1] -= src.b[1];
    XMM[cpu_reg].b[2] -= src.b[2];
    XMM[cpu_reg].b[3] -= src.b[3];
    XMM[cpu_reg].b[4] -= src.b[4];
    XMM[cpu_reg].b[5] -= src.b[5];
    XMM[cpu_reg].b[6] -= src.b[6];
    XMM[cpu_reg].b[7] -= src.b[7];
    XMM[cpu_reg].b[8] -= src.b[8];
    XMM[cpu_reg].b[9] -= src.b[9];
    XMM[cpu_reg].b[10] -= src.b[10];
    XMM[cpu_reg].b[11] -= src.b[11];
    XMM[cpu_reg].b[12] -= src.b[12];
    XMM[cpu_reg].b[13] -= src.b[13];
    XMM[cpu_reg].b[14] -= src.b[14];
    XMM[cpu_reg].b[15] -= src.b[15];

    return 0;
}

static int
opPSUBW_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].w[0] -= src.w[0];
    XMM[cpu_reg].w[1] -= src.w[1];
    XMM[cpu_reg].w[2] -= src.w[2];
    XMM[cpu_reg].w[3] -= src.w[3];
    XMM[cpu_reg].w[4] -= src.w[4];
    XMM[cpu_reg].w[5] -= src.w[5];
    XMM[cpu_reg].w[6] -= src.w[6];
    XMM[cpu_reg].w[7] -= src.w[7];

    return 0;
}

static int
opPSUBW_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].w[0] -= src.w[0];
    XMM[cpu_reg].w[1] -= src.w[1];
    XMM[cpu_reg].w[2] -= src.w[2];
    XMM[cpu_reg].w[3] -= src.w[3];
    XMM[cpu_reg].w[4] -= src.w[4];
    XMM[cpu_reg].w[5] -= src.w[5];
    XMM[cpu_reg].w[6] -= src.w[6];
    XMM[cpu_reg].w[7] -= src.w[7];

    return 0;
}

static int
opPSUBD_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].l[0] -= src.l[0];
    XMM[cpu_reg].l[1] -= src.l[1];
    XMM[cpu_reg].l[2] -= src.l[2];
    XMM[cpu_reg].l[3] -= src.l[3];

    return 0;
}

static int
opPSUBD_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].l[0] -= src.l[0];
    XMM[cpu_reg].l[1] -= src.l[1];
    XMM[cpu_reg].l[2] -= src.l[2];
    XMM[cpu_reg].l[3] -= src.l[3];

    return 0;
}

static int
opPSUBQ_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].q[0] -= src.q[0];
    XMM[cpu_reg].q[1] -= src.q[1];

    return 0;
}

static int
opPSUBQ_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].q[0] -= src.q[0];
    XMM[cpu_reg].q[1] -= src.q[1];

    return 0;
}

static int
opPADDB_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].b[0] += src.b[0];
    XMM[cpu_reg].b[1] += src.b[1];
    XMM[cpu_reg].b[2] += src.b[2];
    XMM[cpu_reg].b[3] += src.b[3];
    XMM[cpu_reg].b[4] += src.b[4];
    XMM[cpu_reg].b[5] += src.b[5];
    XMM[cpu_reg].b[6] += src.b[6];
    XMM[cpu_reg].b[7] += src.b[7];
    XMM[cpu_reg].b[8] += src.b[8];
    XMM[cpu_reg].b[9] += src.b[9];
    XMM[cpu_reg].b[10] += src.b[10];
    XMM[cpu_reg].b[11] += src.b[11];
    XMM[cpu_reg].b[12] += src.b[12];
    XMM[cpu_reg].b[13] += src.b[13];
    XMM[cpu_reg].b[14] += src.b[14];
    XMM[cpu_reg].b[15] += src.b[15];

    return 0;
}

static int
opPADDB_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].b[0] += src.b[0];
    XMM[cpu_reg].b[1] += src.b[1];
    XMM[cpu_reg].b[2] += src.b[2];
    XMM[cpu_reg].b[3] += src.b[3];
    XMM[cpu_reg].b[4] += src.b[4];
    XMM[cpu_reg].b[5] += src.b[5];
    XMM[cpu_reg].b[6] += src.b[6];
    XMM[cpu_reg].b[7] += src.b[7];
    XMM[cpu_reg].b[8] += src.b[8];
    XMM[cpu_reg].b[9] += src.b[9];
    XMM[cpu_reg].b[10] += src.b[10];
    XMM[cpu_reg].b[11] += src.b[11];
    XMM[cpu_reg].b[12] += src.b[12];
    XMM[cpu_reg].b[13] += src.b[13];
    XMM[cpu_reg].b[14] += src.b[14];
    XMM[cpu_reg].b[15] += src.b[15];

    return 0;
}

static int
opPADDW_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].w[0] += src.w[0];
    XMM[cpu_reg].w[1] += src.w[1];
    XMM[cpu_reg].w[2] += src.w[2];
    XMM[cpu_reg].w[3] += src.w[3];
    XMM[cpu_reg].w[4] += src.w[4];
    XMM[cpu_reg].w[5] += src.w[5];
    XMM[cpu_reg].w[6] += src.w[6];
    XMM[cpu_reg].w[7] += src.w[7];

    return 0;
}

static int
opPADDW_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].w[0] += src.w[0];
    XMM[cpu_reg].w[1] += src.w[1];
    XMM[cpu_reg].w[2] += src.w[2];
    XMM[cpu_reg].w[3] += src.w[3];
    XMM[cpu_reg].w[4] += src.w[4];
    XMM[cpu_reg].w[5] += src.w[5];
    XMM[cpu_reg].w[6] += src.w[6];
    XMM[cpu_reg].w[7] += src.w[7];

    return 0;
}

static int
opPADDD_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].l[0] += src.l[0];
    XMM[cpu_reg].l[1] += src.l[1];
    XMM[cpu_reg].l[2] += src.l[2];
    XMM[cpu_reg].l[3] += src.l[3];

    return 0;
}

static int
opPADDD_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].l[0] += src.l[0];
    XMM[cpu_reg].l[1] += src.l[1];
    XMM[cpu_reg].l[2] += src.l[2];
    XMM[cpu_reg].l[3] += src.l[3];

    return 0;
}
