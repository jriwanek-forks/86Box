/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          SLIP authentication - optional text-based login prompt
 *          before entering SLIP data mode. This is an emulator-specific
 *          login extension; SLIP framing is specified by RFC 1055.
 *
 * Authors: Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Copyright 2025-2026 Jasmine Iwanek.
 */
#ifndef MODEM_SLIP_AUTH_H
#define MODEM_SLIP_AUTH_H

/* SLIP auth states */
typedef enum {
    SLIP_AUTH_SEND_USERNAME_PROMPT,
    SLIP_AUTH_RECV_USERNAME,
    SLIP_AUTH_SEND_PASSWORD_PROMPT,
    SLIP_AUTH_RECV_PASSWORD,
    SLIP_AUTH_DONE_OK,
    SLIP_AUTH_DONE_FAIL
} slip_auth_state_t;

#define SLIP_AUTH_BUF_SIZE 64

typedef struct {
    slip_auth_state_t state;
    char              username_buf[SLIP_AUTH_BUF_SIZE];
    int               username_pos;
    char              password_buf[SLIP_AUTH_BUF_SIZE];
    int               password_pos;
    bool              invalid_input;
    bool              ignore_lf;
    char              expected_user[SLIP_AUTH_BUF_SIZE];
    char              expected_pass[SLIP_AUTH_BUF_SIZE];
    bool              active;

    /* Callback to push bytes to the serial line */
    void             *modem;
    void             *log;
    void            (*serial_push)(void *modem, const uint8_t *data, int len);
} slip_auth_ctx_t;

#ifdef __cplusplus
extern "C" {
#endif

slip_auth_ctx_t *slip_auth_init(void *modem,
                                void *log,
                                void (*serial_push)(void *, const uint8_t *, int),
                                const char *username,
                                const char *password);
void             slip_auth_close(slip_auth_ctx_t *ctx);
void             slip_auth_start(slip_auth_ctx_t *ctx);

/* Process one byte from the serial line during SLIP auth.
   Returns true when auth is done (check ctx->state for result). */
bool             slip_auth_rx_byte(slip_auth_ctx_t *ctx, uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* MODEM_SLIP_AUTH_H */
