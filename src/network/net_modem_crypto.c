/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Cryptographic primitives for modem authentication protocols.
 *          MD4 (RFC 1320), MD5 (RFC 1321), SHA-1 (FIPS 180-1), and
 *          DES (FIPS 46-3) implementations for PAP, CHAP, MS-CHAP,
 *          and MS-CHAPv2.
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2025-2026 Jasmine Iwanek.
 */
#include <string.h>
#include <86box/net_modem_crypto.h>

#define ROTL32(x, n) (((x) << (n)) | ((x) >> (32 - (n))))

/* Little-endian helpers (MD4, MD5) */
static inline uint32_t
le32_load(const uint8_t *p)
{
    return (uint32_t) p[0] | ((uint32_t) p[1] << 8)
         | ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24);
}

static inline void
le32_store(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t) v;
    p[1] = (uint8_t) (v >> 8);
    p[2] = (uint8_t) (v >> 16);
    p[3] = (uint8_t) (v >> 24);
}

static inline void
le64_store(uint8_t *p, uint64_t v)
{
    le32_store(p, (uint32_t) v);
    le32_store(p + 4, (uint32_t) (v >> 32));
}

/* Big-endian helpers (SHA-1) */
static inline uint32_t
be32_load(const uint8_t *p)
{
    return ((uint32_t) p[0] << 24) | ((uint32_t) p[1] << 16)
         | ((uint32_t) p[2] << 8) | (uint32_t) p[3];
}

static inline void
be32_store(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t) (v >> 24);
    p[1] = (uint8_t) (v >> 16);
    p[2] = (uint8_t) (v >> 8);
    p[3] = (uint8_t) v;
}

static inline void
be64_store(uint8_t *p, uint64_t v)
{
    be32_store(p, (uint32_t) (v >> 32));
    be32_store(p + 4, (uint32_t) v);
}

/* ========== MD4 (RFC 1320) ========== */

#define MD4_F(x, y, z) (((x) & (y)) | (~(x) & (z)))
#define MD4_G(x, y, z) (((x) & (y)) | ((x) & (z)) | ((y) & (z)))
#define MD4_H(x, y, z) ((x) ^ (y) ^ (z))

#define MD4_R1(a, b, c, d, k, s) \
    do { (a) += MD4_F((b), (c), (d)) + X[(k)]; (a) = ROTL32((a), (s)); } while (0)
#define MD4_R2(a, b, c, d, k, s) \
    do { (a) += MD4_G((b), (c), (d)) + X[(k)] + 0x5A827999u; (a) = ROTL32((a), (s)); } while (0)
#define MD4_R3(a, b, c, d, k, s) \
    do { (a) += MD4_H((b), (c), (d)) + X[(k)] + 0x6ED9EBA1u; (a) = ROTL32((a), (s)); } while (0)

static void
md4_transform(uint32_t state[4], const uint8_t block[64])
{
    uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
    uint32_t X[16];

    for (int i = 0; i < 16; i++)
        X[i] = le32_load(block + i * 4);

    /* Round 1 */
    MD4_R1(a, b, c, d,  0,  3); MD4_R1(d, a, b, c,  1,  7);
    MD4_R1(c, d, a, b,  2, 11); MD4_R1(b, c, d, a,  3, 19);
    MD4_R1(a, b, c, d,  4,  3); MD4_R1(d, a, b, c,  5,  7);
    MD4_R1(c, d, a, b,  6, 11); MD4_R1(b, c, d, a,  7, 19);
    MD4_R1(a, b, c, d,  8,  3); MD4_R1(d, a, b, c,  9,  7);
    MD4_R1(c, d, a, b, 10, 11); MD4_R1(b, c, d, a, 11, 19);
    MD4_R1(a, b, c, d, 12,  3); MD4_R1(d, a, b, c, 13,  7);
    MD4_R1(c, d, a, b, 14, 11); MD4_R1(b, c, d, a, 15, 19);

    /* Round 2 */
    MD4_R2(a, b, c, d,  0,  3); MD4_R2(d, a, b, c,  4,  5);
    MD4_R2(c, d, a, b,  8,  9); MD4_R2(b, c, d, a, 12, 13);
    MD4_R2(a, b, c, d,  1,  3); MD4_R2(d, a, b, c,  5,  5);
    MD4_R2(c, d, a, b,  9,  9); MD4_R2(b, c, d, a, 13, 13);
    MD4_R2(a, b, c, d,  2,  3); MD4_R2(d, a, b, c,  6,  5);
    MD4_R2(c, d, a, b, 10,  9); MD4_R2(b, c, d, a, 14, 13);
    MD4_R2(a, b, c, d,  3,  3); MD4_R2(d, a, b, c,  7,  5);
    MD4_R2(c, d, a, b, 11,  9); MD4_R2(b, c, d, a, 15, 13);

    /* Round 3 */
    MD4_R3(a, b, c, d,  0,  3); MD4_R3(d, a, b, c,  8,  9);
    MD4_R3(c, d, a, b,  4, 11); MD4_R3(b, c, d, a, 12, 15);
    MD4_R3(a, b, c, d,  2,  3); MD4_R3(d, a, b, c, 10,  9);
    MD4_R3(c, d, a, b,  6, 11); MD4_R3(b, c, d, a, 14, 15);
    MD4_R3(a, b, c, d,  1,  3); MD4_R3(d, a, b, c,  9,  9);
    MD4_R3(c, d, a, b,  5, 11); MD4_R3(b, c, d, a, 13, 15);
    MD4_R3(a, b, c, d,  3,  3); MD4_R3(d, a, b, c, 11,  9);
    MD4_R3(c, d, a, b,  7, 11); MD4_R3(b, c, d, a, 15, 15);

    state[0] += a; state[1] += b; state[2] += c; state[3] += d;
}

void
modem_md4_init(modem_md4_ctx_t *ctx)
{
    ctx->state[0] = 0x67452301;
    ctx->state[1] = 0xEFCDAB89;
    ctx->state[2] = 0x98BADCFE;
    ctx->state[3] = 0x10325476;
    ctx->count    = 0;
}

void
modem_md4_update(modem_md4_ctx_t *ctx, const uint8_t *data, size_t len)
{
    size_t idx = (size_t) (ctx->count & 0x3F);
    ctx->count += len;

    for (size_t i = 0; i < len; i++) {
        ctx->buffer[idx++] = data[i];
        if (idx == 64) {
            md4_transform(ctx->state, ctx->buffer);
            idx = 0;
        }
    }
}

void
modem_md4_final(modem_md4_ctx_t *ctx, uint8_t digest[MD4_DIGEST_LENGTH])
{
    uint8_t pad[64];
    size_t  idx    = (size_t) (ctx->count & 0x3F);
    uint64_t bits  = ctx->count * 8;

    memset(pad, 0, sizeof(pad));
    pad[0] = 0x80;

    if (idx < 56)
        modem_md4_update(ctx, pad, 56 - idx);
    else
        modem_md4_update(ctx, pad, 120 - idx);

    le64_store(pad, bits);
    modem_md4_update(ctx, pad, 8);

    for (int i = 0; i < 4; i++)
        le32_store(digest + i * 4, ctx->state[i]);
}

void
modem_md4(const uint8_t *data, size_t len, uint8_t digest[MD4_DIGEST_LENGTH])
{
    modem_md4_ctx_t ctx;
    modem_md4_init(&ctx);
    modem_md4_update(&ctx, data, len);
    modem_md4_final(&ctx, digest);
}

/* ========== MD5 (RFC 1321) ========== */

#define MD5_F(x, y, z) (((x) & (y)) | (~(x) & (z)))
#define MD5_G(x, y, z) (((x) & (z)) | ((y) & ~(z)))
#define MD5_H(x, y, z) ((x) ^ (y) ^ (z))
#define MD5_I(x, y, z) ((y) ^ ((x) | ~(z)))

#define MD5_STEP(f, a, b, c, d, x, t, s) \
    do { (a) += f((b), (c), (d)) + (x) + (t); (a) = ROTL32((a), (s)) + (b); } while (0)

static void
md5_transform(uint32_t state[4], const uint8_t block[64])
{
    uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
    uint32_t X[16];

    for (int i = 0; i < 16; i++)
        X[i] = le32_load(block + i * 4);

    /* Round 1 */
    MD5_STEP(MD5_F, a, b, c, d, X[ 0], 0xD76AA478,  7);
    MD5_STEP(MD5_F, d, a, b, c, X[ 1], 0xE8C7B756, 12);
    MD5_STEP(MD5_F, c, d, a, b, X[ 2], 0x242070DB, 17);
    MD5_STEP(MD5_F, b, c, d, a, X[ 3], 0xC1BDCEEE, 22);
    MD5_STEP(MD5_F, a, b, c, d, X[ 4], 0xF57C0FAF,  7);
    MD5_STEP(MD5_F, d, a, b, c, X[ 5], 0x4787C62A, 12);
    MD5_STEP(MD5_F, c, d, a, b, X[ 6], 0xA8304613, 17);
    MD5_STEP(MD5_F, b, c, d, a, X[ 7], 0xFD469501, 22);
    MD5_STEP(MD5_F, a, b, c, d, X[ 8], 0x698098D8,  7);
    MD5_STEP(MD5_F, d, a, b, c, X[ 9], 0x8B44F7AF, 12);
    MD5_STEP(MD5_F, c, d, a, b, X[10], 0xFFFF5BB1, 17);
    MD5_STEP(MD5_F, b, c, d, a, X[11], 0x895CD7BE, 22);
    MD5_STEP(MD5_F, a, b, c, d, X[12], 0x6B901122,  7);
    MD5_STEP(MD5_F, d, a, b, c, X[13], 0xFD987193, 12);
    MD5_STEP(MD5_F, c, d, a, b, X[14], 0xA679438E, 17);
    MD5_STEP(MD5_F, b, c, d, a, X[15], 0x49B40821, 22);

    /* Round 2 */
    MD5_STEP(MD5_G, a, b, c, d, X[ 1], 0xF61E2562,  5);
    MD5_STEP(MD5_G, d, a, b, c, X[ 6], 0xC040B340,  9);
    MD5_STEP(MD5_G, c, d, a, b, X[11], 0x265E5A51, 14);
    MD5_STEP(MD5_G, b, c, d, a, X[ 0], 0xE9B6C7AA, 20);
    MD5_STEP(MD5_G, a, b, c, d, X[ 5], 0xD62F105D,  5);
    MD5_STEP(MD5_G, d, a, b, c, X[10], 0x02441453,  9);
    MD5_STEP(MD5_G, c, d, a, b, X[15], 0xD8A1E681, 14);
    MD5_STEP(MD5_G, b, c, d, a, X[ 4], 0xE7D3FBC8, 20);
    MD5_STEP(MD5_G, a, b, c, d, X[ 9], 0x21E1CDE6,  5);
    MD5_STEP(MD5_G, d, a, b, c, X[14], 0xC33707D6,  9);
    MD5_STEP(MD5_G, c, d, a, b, X[ 3], 0xF4D50D87, 14);
    MD5_STEP(MD5_G, b, c, d, a, X[ 8], 0x455A14ED, 20);
    MD5_STEP(MD5_G, a, b, c, d, X[13], 0xA9E3E905,  5);
    MD5_STEP(MD5_G, d, a, b, c, X[ 2], 0xFCEFA3F8,  9);
    MD5_STEP(MD5_G, c, d, a, b, X[ 7], 0x676F02D9, 14);
    MD5_STEP(MD5_G, b, c, d, a, X[12], 0x8D2A4C8A, 20);

    /* Round 3 */
    MD5_STEP(MD5_H, a, b, c, d, X[ 5], 0xFFFA3942,  4);
    MD5_STEP(MD5_H, d, a, b, c, X[ 8], 0x8771F681, 11);
    MD5_STEP(MD5_H, c, d, a, b, X[11], 0x6D9D6122, 16);
    MD5_STEP(MD5_H, b, c, d, a, X[14], 0xFDE5380C, 23);
    MD5_STEP(MD5_H, a, b, c, d, X[ 1], 0xA4BEEA44,  4);
    MD5_STEP(MD5_H, d, a, b, c, X[ 4], 0x4BDECFA9, 11);
    MD5_STEP(MD5_H, c, d, a, b, X[ 7], 0xF6BB4B60, 16);
    MD5_STEP(MD5_H, b, c, d, a, X[10], 0xBEBFBC70, 23);
    MD5_STEP(MD5_H, a, b, c, d, X[13], 0x289B7EC6,  4);
    MD5_STEP(MD5_H, d, a, b, c, X[ 0], 0xEAA127FA, 11);
    MD5_STEP(MD5_H, c, d, a, b, X[ 3], 0xD4EF3085, 16);
    MD5_STEP(MD5_H, b, c, d, a, X[ 6], 0x04881D05, 23);
    MD5_STEP(MD5_H, a, b, c, d, X[ 9], 0xD9D4D039,  4);
    MD5_STEP(MD5_H, d, a, b, c, X[12], 0xE6DB99E5, 11);
    MD5_STEP(MD5_H, c, d, a, b, X[15], 0x1FA27CF8, 16);
    MD5_STEP(MD5_H, b, c, d, a, X[ 2], 0xC4AC5665, 23);

    /* Round 4 */
    MD5_STEP(MD5_I, a, b, c, d, X[ 0], 0xF4292244,  6);
    MD5_STEP(MD5_I, d, a, b, c, X[ 7], 0x432AFF97, 10);
    MD5_STEP(MD5_I, c, d, a, b, X[14], 0xAB9423A7, 15);
    MD5_STEP(MD5_I, b, c, d, a, X[ 5], 0xFC93A039, 21);
    MD5_STEP(MD5_I, a, b, c, d, X[12], 0x655B59C3,  6);
    MD5_STEP(MD5_I, d, a, b, c, X[ 3], 0x8F0CCC92, 10);
    MD5_STEP(MD5_I, c, d, a, b, X[10], 0xFFEFF47D, 15);
    MD5_STEP(MD5_I, b, c, d, a, X[ 1], 0x85845DD1, 21);
    MD5_STEP(MD5_I, a, b, c, d, X[ 8], 0x6FA87E4F,  6);
    MD5_STEP(MD5_I, d, a, b, c, X[15], 0xFE2CE6E0, 10);
    MD5_STEP(MD5_I, c, d, a, b, X[ 6], 0xA3014314, 15);
    MD5_STEP(MD5_I, b, c, d, a, X[13], 0x4E0811A1, 21);
    MD5_STEP(MD5_I, a, b, c, d, X[ 4], 0xF7537E82,  6);
    MD5_STEP(MD5_I, d, a, b, c, X[11], 0xBD3AF235, 10);
    MD5_STEP(MD5_I, c, d, a, b, X[ 2], 0x2AD7D2BB, 15);
    MD5_STEP(MD5_I, b, c, d, a, X[ 9], 0xEB86D391, 21);

    state[0] += a; state[1] += b; state[2] += c; state[3] += d;
}

void
modem_md5_init(modem_md5_ctx_t *ctx)
{
    ctx->state[0] = 0x67452301;
    ctx->state[1] = 0xEFCDAB89;
    ctx->state[2] = 0x98BADCFE;
    ctx->state[3] = 0x10325476;
    ctx->count    = 0;
}

void
modem_md5_update(modem_md5_ctx_t *ctx, const uint8_t *data, size_t len)
{
    size_t idx = (size_t) (ctx->count & 0x3F);
    ctx->count += len;

    for (size_t i = 0; i < len; i++) {
        ctx->buffer[idx++] = data[i];
        if (idx == 64) {
            md5_transform(ctx->state, ctx->buffer);
            idx = 0;
        }
    }
}

void
modem_md5_final(modem_md5_ctx_t *ctx, uint8_t digest[MD5_DIGEST_LENGTH])
{
    uint8_t  pad[64];
    size_t   idx  = (size_t) (ctx->count & 0x3F);
    uint64_t bits = ctx->count * 8;

    memset(pad, 0, sizeof(pad));
    pad[0] = 0x80;

    if (idx < 56)
        modem_md5_update(ctx, pad, 56 - idx);
    else
        modem_md5_update(ctx, pad, 120 - idx);

    le64_store(pad, bits);
    modem_md5_update(ctx, pad, 8);

    for (int i = 0; i < 4; i++)
        le32_store(digest + i * 4, ctx->state[i]);
}

void
modem_md5(const uint8_t *data, size_t len, uint8_t digest[MD5_DIGEST_LENGTH])
{
    modem_md5_ctx_t ctx;
    modem_md5_init(&ctx);
    modem_md5_update(&ctx, data, len);
    modem_md5_final(&ctx, digest);
}

/* ========== SHA-1 (FIPS 180-1) ========== */

static void
sha1_transform(uint32_t state[5], const uint8_t block[64])
{
    uint32_t W[80];
    uint32_t a, b, c, d, e, temp;

    for (int i = 0; i < 16; i++)
        W[i] = be32_load(block + i * 4);
    for (int i = 16; i < 80; i++)
        W[i] = ROTL32(W[i - 3] ^ W[i - 8] ^ W[i - 14] ^ W[i - 16], 1);

    a = state[0]; b = state[1]; c = state[2]; d = state[3]; e = state[4];

    for (int i = 0; i < 80; i++) {
        if (i < 20)
            temp = ROTL32(a, 5) + ((b & c) | (~b & d)) + e + W[i] + 0x5A827999;
        else if (i < 40)
            temp = ROTL32(a, 5) + (b ^ c ^ d) + e + W[i] + 0x6ED9EBA1;
        else if (i < 60)
            temp = ROTL32(a, 5) + ((b & c) | (b & d) | (c & d)) + e + W[i] + 0x8F1BBCDC;
        else
            temp = ROTL32(a, 5) + (b ^ c ^ d) + e + W[i] + 0xCA62C1D6;
        e = d; d = c; c = ROTL32(b, 30); b = a; a = temp;
    }

    state[0] += a; state[1] += b; state[2] += c; state[3] += d; state[4] += e;
}

void
modem_sha1_init(modem_sha1_ctx_t *ctx)
{
    ctx->state[0] = 0x67452301;
    ctx->state[1] = 0xEFCDAB89;
    ctx->state[2] = 0x98BADCFE;
    ctx->state[3] = 0x10325476;
    ctx->state[4] = 0xC3D2E1F0;
    ctx->count    = 0;
}

void
modem_sha1_update(modem_sha1_ctx_t *ctx, const uint8_t *data, size_t len)
{
    size_t idx = (size_t) (ctx->count & 0x3F);
    ctx->count += len;

    for (size_t i = 0; i < len; i++) {
        ctx->buffer[idx++] = data[i];
        if (idx == 64) {
            sha1_transform(ctx->state, ctx->buffer);
            idx = 0;
        }
    }
}

void
modem_sha1_final(modem_sha1_ctx_t *ctx, uint8_t digest[SHA1_DIGEST_LENGTH])
{
    uint8_t  pad[64];
    size_t   idx  = (size_t) (ctx->count & 0x3F);
    uint64_t bits = ctx->count * 8;

    memset(pad, 0, sizeof(pad));
    pad[0] = 0x80;

    if (idx < 56)
        modem_sha1_update(ctx, pad, 56 - idx);
    else
        modem_sha1_update(ctx, pad, 120 - idx);

    be64_store(pad, bits);
    modem_sha1_update(ctx, pad, 8);

    for (int i = 0; i < 5; i++)
        be32_store(digest + i * 4, ctx->state[i]);
}

void
modem_sha1(const uint8_t *data, size_t len, uint8_t digest[SHA1_DIGEST_LENGTH])
{
    modem_sha1_ctx_t ctx;
    modem_sha1_init(&ctx);
    modem_sha1_update(&ctx, data, len);
    modem_sha1_final(&ctx, digest);
}

/* ========== DES (FIPS 46-3) ========== */

/* Initial Permutation */
static const uint8_t des_ip[64] = {
    58, 50, 42, 34, 26, 18, 10, 2,
    60, 52, 44, 36, 28, 20, 12, 4,
    62, 54, 46, 38, 30, 22, 14, 6,
    64, 56, 48, 40, 32, 24, 16, 8,
    57, 49, 41, 33, 25, 17,  9, 1,
    59, 51, 43, 35, 27, 19, 11, 3,
    61, 53, 45, 37, 29, 21, 13, 5,
    63, 55, 47, 39, 31, 23, 15, 7
};

/* Final Permutation (IP^-1) */
static const uint8_t des_fp[64] = {
    40, 8, 48, 16, 56, 24, 64, 32,
    39, 7, 47, 15, 55, 23, 63, 31,
    38, 6, 46, 14, 54, 22, 62, 30,
    37, 5, 45, 13, 53, 21, 61, 29,
    36, 4, 44, 12, 52, 20, 60, 28,
    35, 3, 43, 11, 51, 19, 59, 27,
    34, 2, 42, 10, 50, 18, 58, 26,
    33, 1, 41,  9, 49, 17, 57, 25
};

/* Expansion permutation */
static const uint8_t des_e[48] = {
    32,  1,  2,  3,  4,  5,
     4,  5,  6,  7,  8,  9,
     8,  9, 10, 11, 12, 13,
    12, 13, 14, 15, 16, 17,
    16, 17, 18, 19, 20, 21,
    20, 21, 22, 23, 24, 25,
    24, 25, 26, 27, 28, 29,
    28, 29, 30, 31, 32,  1
};

/* P permutation */
static const uint8_t des_p[32] = {
    16,  7, 20, 21, 29, 12, 28, 17,
     1, 15, 23, 26,  5, 18, 31, 10,
     2,  8, 24, 14, 32, 27,  3,  9,
    19, 13, 30,  6, 22, 11,  4, 25
};

/* S-boxes */
static const uint8_t des_sbox[8][64] = {
    /* S1 */
    { 14,  4, 13,  1,  2, 15, 11,  8,  3, 10,  6, 12,  5,  9,  0,  7,
       0, 15,  7,  4, 14,  2, 13,  1, 10,  6, 12, 11,  9,  5,  3,  8,
       4,  1, 14,  8, 13,  6,  2, 11, 15, 12,  9,  7,  3, 10,  5,  0,
      15, 12,  8,  2,  4,  9,  1,  7,  5, 11,  3, 14, 10,  0,  6, 13 },
    /* S2 */
    { 15,  1,  8, 14,  6, 11,  3,  4,  9,  7,  2, 13, 12,  0,  5, 10,
       3, 13,  4,  7, 15,  2,  8, 14, 12,  0,  1, 10,  6,  9, 11,  5,
       0, 14,  7, 11, 10,  4, 13,  1,  5,  8, 12,  6,  9,  3,  2, 15,
      13,  8, 10,  1,  3, 15,  4,  2, 11,  6,  7, 12,  0,  5, 14,  9 },
    /* S3 */
    { 10,  0,  9, 14,  6,  3, 15,  5,  1, 13, 12,  7, 11,  4,  2,  8,
      13,  7,  0,  9,  3,  4,  6, 10,  2,  8,  5, 14, 12, 11, 15,  1,
      13,  6,  4,  9,  8, 15,  3,  0, 11,  1,  2, 12,  5, 10, 14,  7,
       1, 10, 13,  0,  6,  9,  8,  7,  4, 15, 14,  3, 11,  5,  2, 12 },
    /* S4 */
    {  7, 13, 14,  3,  0,  6,  9, 10,  1,  2,  8,  5, 11, 12,  4, 15,
      13,  8, 11,  5,  6, 15,  0,  3,  4,  7,  2, 12,  1, 10, 14,  9,
      10,  6,  9,  0, 12, 11,  7, 13, 15,  1,  3, 14,  5,  2,  8,  4,
       3, 15,  0,  6, 10,  1, 13,  8,  9,  4,  5, 11, 12,  7,  2, 14 },
    /* S5 */
    {  2, 12,  4,  1,  7, 10, 11,  6,  8,  5,  3, 15, 13,  0, 14,  9,
      14, 11,  2, 12,  4,  7, 13,  1,  5,  0, 15, 10,  3,  9,  8,  6,
       4,  2,  1, 11, 10, 13,  7,  8, 15,  9, 12,  5,  6,  3,  0, 14,
      11,  8, 12,  7,  1, 14,  2, 13,  6, 15,  0,  9, 10,  4,  5,  3 },
    /* S6 */
    { 12,  1, 10, 15,  9,  2,  6,  8,  0, 13,  3,  4, 14,  7,  5, 11,
      10, 15,  4,  2,  7, 12,  9,  5,  6,  1, 13, 14,  0, 11,  3,  8,
       9, 14, 15,  5,  2,  8, 12,  3,  7,  0,  4, 10,  1, 13, 11,  6,
       4,  3,  2, 12,  9,  5, 15, 10, 11, 14,  1,  7,  6,  0,  8, 13 },
    /* S7 */
    {  4, 11,  2, 14, 15,  0,  8, 13,  3, 12,  9,  7,  5, 10,  6,  1,
      13,  0, 11,  7,  4,  9,  1, 10, 14,  3,  5, 12,  2, 15,  8,  6,
       1,  4, 11, 13, 12,  3,  7, 14, 10, 15,  6,  8,  0,  5,  9,  2,
       6, 11, 13,  8,  1,  4, 10,  7,  9,  5,  0, 15, 14,  2,  3, 12 },
    /* S8 */
    { 13,  2,  8,  4,  6, 15, 11,  1, 10,  9,  3, 14,  5,  0, 12,  7,
       1, 15, 13,  8, 10,  3,  7,  4, 12,  5,  6,  2,  0, 14,  9, 11,
       7, 11,  4,  1,  9, 12, 14,  2,  0,  6, 10, 13, 15,  3,  5,  8,
       2,  1, 14,  7,  4, 10,  8, 13, 15, 12,  9,  0,  3,  5,  6, 11 }
};

/* Permuted Choice 1 */
static const uint8_t des_pc1[56] = {
    57, 49, 41, 33, 25, 17,  9,
     1, 58, 50, 42, 34, 26, 18,
    10,  2, 59, 51, 43, 35, 27,
    19, 11,  3, 60, 52, 44, 36,
    63, 55, 47, 39, 31, 23, 15,
     7, 62, 54, 46, 38, 30, 22,
    14,  6, 61, 53, 45, 37, 29,
    21, 13,  5, 28, 20, 12,  4
};

/* Permuted Choice 2 */
static const uint8_t des_pc2[48] = {
    14, 17, 11, 24,  1,  5,
     3, 28, 15,  6, 21, 10,
    23, 19, 12,  4, 26,  8,
    16,  7, 27, 20, 13,  2,
    41, 52, 31, 37, 47, 55,
    30, 40, 51, 45, 33, 48,
    44, 49, 39, 56, 34, 53,
    46, 42, 50, 36, 29, 32
};

/* Key schedule left shifts */
static const uint8_t des_shifts[16] = {
    1, 1, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1
};

/* Get bit n (1-based, MSB first) from data */
static inline int
des_get_bit(const uint8_t *data, int n)
{
    return (data[(n - 1) >> 3] >> (7 - ((n - 1) & 7))) & 1;
}

/* Set bit n (1-based, MSB first) in data */
static inline void
des_set_bit(uint8_t *data, int n)
{
    data[(n - 1) >> 3] |= (1 << (7 - ((n - 1) & 7)));
}

/* Apply a bit permutation */
static void
des_permute(const uint8_t *in, uint8_t *out, const uint8_t *table, int n)
{
    memset(out, 0, (n + 7) / 8);
    for (int i = 0; i < n; i++) {
        if (des_get_bit(in, table[i]))
            des_set_bit(out, i + 1);
    }
}

/* Left-rotate a 28-bit value stored in the top 28 bits of a uint32_t */
static inline uint32_t
des_rotl28(uint32_t v, int n)
{
    return ((v << n) | (v >> (28 - n))) & 0x0FFFFFFF;
}

void
modem_des_encrypt_block(const uint8_t key[7], const uint8_t plaintext[8], uint8_t ciphertext[8])
{
    uint8_t  des_key[8];
    uint8_t  key56[7]; /* 56-bit permuted key */
    uint8_t  subkeys[16][6];
    uint8_t  block[8];
    uint32_t C, D;

    /* Expand 7-byte key to 8-byte DES key (set parity bits) */
    des_key[0] = (key[0] >> 1);
    des_key[1] = ((key[0] & 0x01) << 6) | (key[1] >> 2);
    des_key[2] = ((key[1] & 0x03) << 5) | (key[2] >> 3);
    des_key[3] = ((key[2] & 0x07) << 4) | (key[3] >> 4);
    des_key[4] = ((key[3] & 0x0F) << 3) | (key[4] >> 5);
    des_key[5] = ((key[4] & 0x1F) << 2) | (key[5] >> 6);
    des_key[6] = ((key[5] & 0x3F) << 1) | (key[6] >> 7);
    des_key[7] = (key[6] & 0x7F);
    for (int i = 0; i < 8; i++)
        des_key[i] = (des_key[i] << 1) & 0xFE;

    /* Apply PC-1 to get 56-bit key */
    des_permute(des_key, key56, des_pc1, 56);

    /* Split into two 28-bit halves */
    C = ((uint32_t) key56[0] << 20) | ((uint32_t) key56[1] << 12)
      | ((uint32_t) key56[2] << 4) | ((uint32_t) key56[3] >> 4);
    D = ((uint32_t) (key56[3] & 0x0F) << 24) | ((uint32_t) key56[4] << 16)
      | ((uint32_t) key56[5] << 8) | (uint32_t) key56[6];

    /* Generate 16 subkeys */
    for (int round = 0; round < 16; round++) {
        uint8_t cd[7];

        C = des_rotl28(C, des_shifts[round]);
        D = des_rotl28(D, des_shifts[round]);

        /* Combine C and D back into 56 bits */
        cd[0] = (uint8_t) (C >> 20);
        cd[1] = (uint8_t) (C >> 12);
        cd[2] = (uint8_t) (C >> 4);
        cd[3] = (uint8_t) ((C << 4) | (D >> 24));
        cd[4] = (uint8_t) (D >> 16);
        cd[5] = (uint8_t) (D >> 8);
        cd[6] = (uint8_t) D;

        /* Apply PC-2 to get 48-bit subkey */
        des_permute(cd, subkeys[round], des_pc2, 48);
    }

    /* Apply Initial Permutation */
    des_permute(plaintext, block, des_ip, 64);

    /* 16 Feistel rounds */
    uint32_t L = ((uint32_t) block[0] << 24) | ((uint32_t) block[1] << 16)
               | ((uint32_t) block[2] << 8) | (uint32_t) block[3];
    uint32_t R = ((uint32_t) block[4] << 24) | ((uint32_t) block[5] << 16)
               | ((uint32_t) block[6] << 8) | (uint32_t) block[7];

    for (int round = 0; round < 16; round++) {
        uint8_t  expanded[6];
        uint8_t  R_bytes[4];
        uint32_t f_result = 0;

        /* Convert R to bytes for expansion */
        R_bytes[0] = (uint8_t) (R >> 24);
        R_bytes[1] = (uint8_t) (R >> 16);
        R_bytes[2] = (uint8_t) (R >> 8);
        R_bytes[3] = (uint8_t) R;

        /* E-expand R from 32 to 48 bits */
        des_permute(R_bytes, expanded, des_e, 48);

        /* XOR with subkey */
        for (int i = 0; i < 6; i++)
            expanded[i] ^= subkeys[round][i];

        /* S-box substitution */
        for (int i = 0; i < 8; i++) {
            int bit_offset = i * 6;
            int byte_off   = bit_offset >> 3;
            int bit_pos    = bit_offset & 7;
            uint8_t val;

            /* Extract 6 bits */
            if (bit_pos <= 2) {
                val = (expanded[byte_off] >> (2 - bit_pos)) & 0x3F;
            } else {
                val = ((expanded[byte_off] << (bit_pos - 2)) | (expanded[byte_off + 1] >> (10 - bit_pos))) & 0x3F;
            }

            /* Row = bits 0,5; Column = bits 1..4 */
            int row = ((val >> 5) << 1) | (val & 1);
            int col = (val >> 1) & 0x0F;

            f_result = (f_result << 4) | des_sbox[i][row * 16 + col];
        }

        /* Apply P permutation */
        {
            uint8_t f_bytes[4];
            uint8_t p_bytes[4];
            f_bytes[0] = (uint8_t) (f_result >> 24);
            f_bytes[1] = (uint8_t) (f_result >> 16);
            f_bytes[2] = (uint8_t) (f_result >> 8);
            f_bytes[3] = (uint8_t) f_result;
            des_permute(f_bytes, p_bytes, des_p, 32);
            f_result = ((uint32_t) p_bytes[0] << 24) | ((uint32_t) p_bytes[1] << 16)
                     | ((uint32_t) p_bytes[2] << 8) | (uint32_t) p_bytes[3];
        }

        /* Feistel step */
        uint32_t temp = R;
        R = L ^ f_result;
        L = temp;
    }

    /* Combine R,L (swapped) */
    block[0] = (uint8_t) (R >> 24); block[1] = (uint8_t) (R >> 16);
    block[2] = (uint8_t) (R >> 8);  block[3] = (uint8_t) R;
    block[4] = (uint8_t) (L >> 24); block[5] = (uint8_t) (L >> 16);
    block[6] = (uint8_t) (L >> 8);  block[7] = (uint8_t) L;

    /* Apply Final Permutation */
    des_permute(block, ciphertext, des_fp, 64);
}
