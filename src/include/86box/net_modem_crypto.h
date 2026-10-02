/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Cryptographic primitives for modem authentication protocols.
 *          MD4, MD5, SHA-1, and DES implementations for PAP, CHAP,
 *          MS-CHAP, and MS-CHAPv2.
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2025-2026 Jasmine Iwanek.
 */
#ifndef NET_MODEM_CRYPTO_H
#define NET_MODEM_CRYPTO_H

#include <stddef.h>
#include <stdint.h>

/* MD4 - used by MS-CHAP NT hash */
#define MD4_DIGEST_LENGTH 16
#define MD4_BLOCK_LENGTH  64

typedef struct {
    uint32_t state[4];
    uint64_t count;
    uint8_t  buffer[MD4_BLOCK_LENGTH];
} modem_md4_ctx_t;

void modem_md4_init(modem_md4_ctx_t *ctx);
void modem_md4_update(modem_md4_ctx_t *ctx, const uint8_t *data, size_t len);
void modem_md4_final(modem_md4_ctx_t *ctx, uint8_t digest[MD4_DIGEST_LENGTH]);
void modem_md4(const uint8_t *data, size_t len, uint8_t digest[MD4_DIGEST_LENGTH]);

/* MD5 - used by CHAP */
#define MD5_DIGEST_LENGTH 16
#define MD5_BLOCK_LENGTH  64

typedef struct {
    uint32_t state[4];
    uint64_t count;
    uint8_t  buffer[MD5_BLOCK_LENGTH];
} modem_md5_ctx_t;

void modem_md5_init(modem_md5_ctx_t *ctx);
void modem_md5_update(modem_md5_ctx_t *ctx, const uint8_t *data, size_t len);
void modem_md5_final(modem_md5_ctx_t *ctx, uint8_t digest[MD5_DIGEST_LENGTH]);
void modem_md5(const uint8_t *data, size_t len, uint8_t digest[MD5_DIGEST_LENGTH]);

/* SHA-1 - used by MS-CHAPv2 */
#define SHA1_DIGEST_LENGTH 20
#define SHA1_BLOCK_LENGTH  64

typedef struct {
    uint32_t state[5];
    uint64_t count;
    uint8_t  buffer[SHA1_BLOCK_LENGTH];
} modem_sha1_ctx_t;

void modem_sha1_init(modem_sha1_ctx_t *ctx);
void modem_sha1_update(modem_sha1_ctx_t *ctx, const uint8_t *data, size_t len);
void modem_sha1_final(modem_sha1_ctx_t *ctx, uint8_t digest[SHA1_DIGEST_LENGTH]);
void modem_sha1(const uint8_t *data, size_t len, uint8_t digest[SHA1_DIGEST_LENGTH]);

/* DES - used by MS-CHAP challenge-response */
void modem_des_encrypt_block(const uint8_t key[7], const uint8_t plaintext[8], uint8_t ciphertext[8]);

#endif /* NET_MODEM_CRYPTO_H */
