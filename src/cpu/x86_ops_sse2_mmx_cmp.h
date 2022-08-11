static int
opPCMPGTB_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].b[0]  = (XMM[cpu_reg].sb[0] > src.sb[0]) ? 0xff : 0;
    XMM[cpu_reg].b[1]  = (XMM[cpu_reg].sb[1] > src.sb[1]) ? 0xff : 0;
    XMM[cpu_reg].b[2]  = (XMM[cpu_reg].sb[2] > src.sb[2]) ? 0xff : 0;
    XMM[cpu_reg].b[3]  = (XMM[cpu_reg].sb[3] > src.sb[3]) ? 0xff : 0;
    XMM[cpu_reg].b[4]  = (XMM[cpu_reg].sb[4] > src.sb[4]) ? 0xff : 0;
    XMM[cpu_reg].b[5]  = (XMM[cpu_reg].sb[5] > src.sb[5]) ? 0xff : 0;
    XMM[cpu_reg].b[6]  = (XMM[cpu_reg].sb[6] > src.sb[6]) ? 0xff : 0;
    XMM[cpu_reg].b[7]  = (XMM[cpu_reg].sb[7] > src.sb[7]) ? 0xff : 0;
    XMM[cpu_reg].b[8]  = (XMM[cpu_reg].sb[8] > src.sb[8]) ? 0xff : 0;
    XMM[cpu_reg].b[9]  = (XMM[cpu_reg].sb[9] > src.sb[9]) ? 0xff : 0;
    XMM[cpu_reg].b[10] = (XMM[cpu_reg].sb[10] > src.sb[10]) ? 0xff : 0;
    XMM[cpu_reg].b[11] = (XMM[cpu_reg].sb[11] > src.sb[11]) ? 0xff : 0;
    XMM[cpu_reg].b[12] = (XMM[cpu_reg].sb[12] > src.sb[12]) ? 0xff : 0;
    XMM[cpu_reg].b[13] = (XMM[cpu_reg].sb[13] > src.sb[13]) ? 0xff : 0;
    XMM[cpu_reg].b[14] = (XMM[cpu_reg].sb[14] > src.sb[14]) ? 0xff : 0;
    XMM[cpu_reg].b[15] = (XMM[cpu_reg].sb[15] > src.sb[15]) ? 0xff : 0;

    return 0;
}

static int
opPCMPGTB_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].b[0]  = (XMM[cpu_reg].sb[0] > src.sb[0]) ? 0xff : 0;
    XMM[cpu_reg].b[1]  = (XMM[cpu_reg].sb[1] > src.sb[1]) ? 0xff : 0;
    XMM[cpu_reg].b[2]  = (XMM[cpu_reg].sb[2] > src.sb[2]) ? 0xff : 0;
    XMM[cpu_reg].b[3]  = (XMM[cpu_reg].sb[3] > src.sb[3]) ? 0xff : 0;
    XMM[cpu_reg].b[4]  = (XMM[cpu_reg].sb[4] > src.sb[4]) ? 0xff : 0;
    XMM[cpu_reg].b[5]  = (XMM[cpu_reg].sb[5] > src.sb[5]) ? 0xff : 0;
    XMM[cpu_reg].b[6]  = (XMM[cpu_reg].sb[6] > src.sb[6]) ? 0xff : 0;
    XMM[cpu_reg].b[7]  = (XMM[cpu_reg].sb[7] > src.sb[7]) ? 0xff : 0;
    XMM[cpu_reg].b[8]  = (XMM[cpu_reg].sb[8] > src.sb[8]) ? 0xff : 0;
    XMM[cpu_reg].b[9]  = (XMM[cpu_reg].sb[9] > src.sb[9]) ? 0xff : 0;
    XMM[cpu_reg].b[10] = (XMM[cpu_reg].sb[10] > src.sb[10]) ? 0xff : 0;
    XMM[cpu_reg].b[11] = (XMM[cpu_reg].sb[11] > src.sb[11]) ? 0xff : 0;
    XMM[cpu_reg].b[12] = (XMM[cpu_reg].sb[12] > src.sb[12]) ? 0xff : 0;
    XMM[cpu_reg].b[13] = (XMM[cpu_reg].sb[13] > src.sb[13]) ? 0xff : 0;
    XMM[cpu_reg].b[14] = (XMM[cpu_reg].sb[14] > src.sb[14]) ? 0xff : 0;
    XMM[cpu_reg].b[15] = (XMM[cpu_reg].sb[15] > src.sb[15]) ? 0xff : 0;

    return 0;
}

static int
opPCMPGTW_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].w[0] = (XMM[cpu_reg].sw[0] > src.sw[0]) ? 0xff : 0;
    XMM[cpu_reg].w[1] = (XMM[cpu_reg].sw[1] > src.sw[1]) ? 0xff : 0;
    XMM[cpu_reg].w[2] = (XMM[cpu_reg].sw[2] > src.sw[2]) ? 0xff : 0;
    XMM[cpu_reg].w[3] = (XMM[cpu_reg].sw[3] > src.sw[3]) ? 0xff : 0;
    XMM[cpu_reg].w[4] = (XMM[cpu_reg].sw[4] > src.sw[4]) ? 0xff : 0;
    XMM[cpu_reg].w[5] = (XMM[cpu_reg].sw[5] > src.sw[5]) ? 0xff : 0;
    XMM[cpu_reg].w[6] = (XMM[cpu_reg].sw[6] > src.sw[6]) ? 0xff : 0;
    XMM[cpu_reg].w[7] = (XMM[cpu_reg].sw[7] > src.sw[7]) ? 0xff : 0;

    return 0;
}

static int
opPCMPGTW_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].w[0] = (XMM[cpu_reg].sw[0] > src.sw[0]) ? 0xff : 0;
    XMM[cpu_reg].w[1] = (XMM[cpu_reg].sw[1] > src.sw[1]) ? 0xff : 0;
    XMM[cpu_reg].w[2] = (XMM[cpu_reg].sw[2] > src.sw[2]) ? 0xff : 0;
    XMM[cpu_reg].w[3] = (XMM[cpu_reg].sw[3] > src.sw[3]) ? 0xff : 0;
    XMM[cpu_reg].w[4] = (XMM[cpu_reg].sw[4] > src.sw[4]) ? 0xff : 0;
    XMM[cpu_reg].w[5] = (XMM[cpu_reg].sw[5] > src.sw[5]) ? 0xff : 0;
    XMM[cpu_reg].w[6] = (XMM[cpu_reg].sw[6] > src.sw[6]) ? 0xff : 0;
    XMM[cpu_reg].w[7] = (XMM[cpu_reg].sw[7] > src.sw[7]) ? 0xff : 0;

    return 0;
}

static int
opPCMPGTD_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].l[0] = (XMM[cpu_reg].sl[0] > src.sl[0]) ? 0xff : 0;
    XMM[cpu_reg].l[1] = (XMM[cpu_reg].sl[1] > src.sl[1]) ? 0xff : 0;
    XMM[cpu_reg].l[2] = (XMM[cpu_reg].sl[2] > src.sl[2]) ? 0xff : 0;
    XMM[cpu_reg].l[3] = (XMM[cpu_reg].sl[3] > src.sl[3]) ? 0xff : 0;

    return 0;
}

static int
opPCMPGTD_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].l[0] = (XMM[cpu_reg].sl[0] > src.sl[0]) ? 0xff : 0;
    XMM[cpu_reg].l[1] = (XMM[cpu_reg].sl[1] > src.sl[1]) ? 0xff : 0;
    XMM[cpu_reg].l[2] = (XMM[cpu_reg].sl[2] > src.sl[2]) ? 0xff : 0;
    XMM[cpu_reg].l[3] = (XMM[cpu_reg].sl[3] > src.sl[3]) ? 0xff : 0;

    return 0;
}

static int
opPCMPEQB_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].b[0]  = (XMM[cpu_reg].sb[0] == src.sb[0]) ? 0xff : 0;
    XMM[cpu_reg].b[1]  = (XMM[cpu_reg].sb[1] == src.sb[1]) ? 0xff : 0;
    XMM[cpu_reg].b[2]  = (XMM[cpu_reg].sb[2] == src.sb[2]) ? 0xff : 0;
    XMM[cpu_reg].b[3]  = (XMM[cpu_reg].sb[3] == src.sb[3]) ? 0xff : 0;
    XMM[cpu_reg].b[4]  = (XMM[cpu_reg].sb[4] == src.sb[4]) ? 0xff : 0;
    XMM[cpu_reg].b[5]  = (XMM[cpu_reg].sb[5] == src.sb[5]) ? 0xff : 0;
    XMM[cpu_reg].b[6]  = (XMM[cpu_reg].sb[6] == src.sb[6]) ? 0xff : 0;
    XMM[cpu_reg].b[7]  = (XMM[cpu_reg].sb[7] == src.sb[7]) ? 0xff : 0;
    XMM[cpu_reg].b[8]  = (XMM[cpu_reg].sb[8] == src.sb[8]) ? 0xff : 0;
    XMM[cpu_reg].b[9]  = (XMM[cpu_reg].sb[9] == src.sb[9]) ? 0xff : 0;
    XMM[cpu_reg].b[10] = (XMM[cpu_reg].sb[10] == src.sb[10]) ? 0xff : 0;
    XMM[cpu_reg].b[11] = (XMM[cpu_reg].sb[11] == src.sb[11]) ? 0xff : 0;
    XMM[cpu_reg].b[12] = (XMM[cpu_reg].sb[12] == src.sb[12]) ? 0xff : 0;
    XMM[cpu_reg].b[13] = (XMM[cpu_reg].sb[13] == src.sb[13]) ? 0xff : 0;
    XMM[cpu_reg].b[14] = (XMM[cpu_reg].sb[14] == src.sb[14]) ? 0xff : 0;
    XMM[cpu_reg].b[15] = (XMM[cpu_reg].sb[15] == src.sb[15]) ? 0xff : 0;

    return 0;
}

static int
opPCMPEQB_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].b[0]  = (XMM[cpu_reg].sb[0] == src.sb[0]) ? 0xff : 0;
    XMM[cpu_reg].b[1]  = (XMM[cpu_reg].sb[1] == src.sb[1]) ? 0xff : 0;
    XMM[cpu_reg].b[2]  = (XMM[cpu_reg].sb[2] == src.sb[2]) ? 0xff : 0;
    XMM[cpu_reg].b[3]  = (XMM[cpu_reg].sb[3] == src.sb[3]) ? 0xff : 0;
    XMM[cpu_reg].b[4]  = (XMM[cpu_reg].sb[4] == src.sb[4]) ? 0xff : 0;
    XMM[cpu_reg].b[5]  = (XMM[cpu_reg].sb[5] == src.sb[5]) ? 0xff : 0;
    XMM[cpu_reg].b[6]  = (XMM[cpu_reg].sb[6] == src.sb[6]) ? 0xff : 0;
    XMM[cpu_reg].b[7]  = (XMM[cpu_reg].sb[7] == src.sb[7]) ? 0xff : 0;
    XMM[cpu_reg].b[8]  = (XMM[cpu_reg].sb[8] == src.sb[8]) ? 0xff : 0;
    XMM[cpu_reg].b[9]  = (XMM[cpu_reg].sb[9] == src.sb[9]) ? 0xff : 0;
    XMM[cpu_reg].b[10] = (XMM[cpu_reg].sb[10] == src.sb[10]) ? 0xff : 0;
    XMM[cpu_reg].b[11] = (XMM[cpu_reg].sb[11] == src.sb[11]) ? 0xff : 0;
    XMM[cpu_reg].b[12] = (XMM[cpu_reg].sb[12] == src.sb[12]) ? 0xff : 0;
    XMM[cpu_reg].b[13] = (XMM[cpu_reg].sb[13] == src.sb[13]) ? 0xff : 0;
    XMM[cpu_reg].b[14] = (XMM[cpu_reg].sb[14] == src.sb[14]) ? 0xff : 0;
    XMM[cpu_reg].b[15] = (XMM[cpu_reg].sb[15] == src.sb[15]) ? 0xff : 0;

    return 0;
}

static int
opPCMPEQW_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].w[0] = (XMM[cpu_reg].sw[0] == src.sw[0]) ? 0xff : 0;
    XMM[cpu_reg].w[1] = (XMM[cpu_reg].sw[1] == src.sw[1]) ? 0xff : 0;
    XMM[cpu_reg].w[2] = (XMM[cpu_reg].sw[2] == src.sw[2]) ? 0xff : 0;
    XMM[cpu_reg].w[3] = (XMM[cpu_reg].sw[3] == src.sw[3]) ? 0xff : 0;
    XMM[cpu_reg].w[4] = (XMM[cpu_reg].sw[4] == src.sw[4]) ? 0xff : 0;
    XMM[cpu_reg].w[5] = (XMM[cpu_reg].sw[5] == src.sw[5]) ? 0xff : 0;
    XMM[cpu_reg].w[6] = (XMM[cpu_reg].sw[6] == src.sw[6]) ? 0xff : 0;
    XMM[cpu_reg].w[7] = (XMM[cpu_reg].sw[7] == src.sw[7]) ? 0xff : 0;

    return 0;
}

static int
opPCMPEQW_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].w[0] = (XMM[cpu_reg].sw[0] == src.sw[0]) ? 0xff : 0;
    XMM[cpu_reg].w[1] = (XMM[cpu_reg].sw[1] == src.sw[1]) ? 0xff : 0;
    XMM[cpu_reg].w[2] = (XMM[cpu_reg].sw[2] == src.sw[2]) ? 0xff : 0;
    XMM[cpu_reg].w[3] = (XMM[cpu_reg].sw[3] == src.sw[3]) ? 0xff : 0;
    XMM[cpu_reg].w[4] = (XMM[cpu_reg].sw[4] == src.sw[4]) ? 0xff : 0;
    XMM[cpu_reg].w[5] = (XMM[cpu_reg].sw[5] == src.sw[5]) ? 0xff : 0;
    XMM[cpu_reg].w[6] = (XMM[cpu_reg].sw[6] == src.sw[6]) ? 0xff : 0;
    XMM[cpu_reg].w[7] = (XMM[cpu_reg].sw[7] == src.sw[7]) ? 0xff : 0;

    return 0;
}

static int
opPCMPEQD_xmm_a16(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].l[0] = (XMM[cpu_reg].sl[0] == src.sl[0]) ? 0xff : 0;
    XMM[cpu_reg].l[1] = (XMM[cpu_reg].sl[1] == src.sl[1]) ? 0xff : 0;
    XMM[cpu_reg].l[2] = (XMM[cpu_reg].sl[2] == src.sl[2]) ? 0xff : 0;
    XMM[cpu_reg].l[3] = (XMM[cpu_reg].sl[3] == src.sl[3]) ? 0xff : 0;

    return 0;
}

static int
opPCMPEQD_xmm_a32(uint32_t fetchdat)
{
    SSE_REG src;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    XMM[cpu_reg].l[0] = (XMM[cpu_reg].sl[0] == src.sl[0]) ? 0xff : 0;
    XMM[cpu_reg].l[1] = (XMM[cpu_reg].sl[1] == src.sl[1]) ? 0xff : 0;
    XMM[cpu_reg].l[2] = (XMM[cpu_reg].sl[2] == src.sl[2]) ? 0xff : 0;
    XMM[cpu_reg].l[3] = (XMM[cpu_reg].sl[3] == src.sl[3]) ? 0xff : 0;

    return 0;
}
