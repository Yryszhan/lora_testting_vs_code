/**
  * @file    lorawan.c
  * @brief   LoRaWAN 1.0 ABP uplink. Ключи берутся из config.h.
  */
#include "lorawan.h"
#include "aes.h"
#include "config.h"
#include <string.h>

static const uint8_t DEVADDR[4] = CFG_DEVADDR;
static const uint8_t NWKSKEY[16] = CFG_NWKSKEY;
static const uint8_t APPSKEY[16] = CFG_APPSKEY;

/* Шифрование FRMPayload по LoRaWAN (AES-CTR, uplink dir=0) */
static void encrypt_payload(uint8_t *data, int len,
                            const uint8_t devaddr_le[4], uint32_t fcnt)
{
    uint8_t rk[176];
    aes_key_expand(APPSKEY, rk);
    int k = (len + 15) / 16;
    for (int i = 1; i <= k; i++) {
        uint8_t A[16] = {
            0x01, 0, 0, 0, 0, 0x00,          /* dir=0 uplink */
            devaddr_le[0], devaddr_le[1], devaddr_le[2], devaddr_le[3],
            (uint8_t)(fcnt & 0xFF), (uint8_t)((fcnt >> 8) & 0xFF),
            (uint8_t)((fcnt >> 16) & 0xFF), (uint8_t)((fcnt >> 24) & 0xFF),
            0x00, (uint8_t)i
        };
        uint8_t S[16];
        memcpy(S, A, 16);
        aes_encrypt_block(rk, S);
        int blockstart = (i - 1) * 16;
        int n = len - blockstart; if (n > 16) n = 16;
        for (int j = 0; j < n; j++) data[blockstart + j] ^= S[j];
    }
}

/* MIC по LoRaWAN 1.0: CMAC(NwkSKey, B0 || msg), первые 4 байта */
static void compute_mic(const uint8_t *msg, int len,
                        const uint8_t devaddr_le[4], uint32_t fcnt,
                        uint8_t mic[4])
{
    uint8_t buf[64];
    uint8_t B0[16] = {
        0x49, 0, 0, 0, 0, 0x00,              /* dir=0 */
        devaddr_le[0], devaddr_le[1], devaddr_le[2], devaddr_le[3],
        (uint8_t)(fcnt & 0xFF), (uint8_t)((fcnt >> 8) & 0xFF),
        (uint8_t)((fcnt >> 16) & 0xFF), (uint8_t)((fcnt >> 24) & 0xFF),
        0x00, (uint8_t)len
    };
    memcpy(buf, B0, 16);
    memcpy(buf + 16, msg, len);
    uint8_t full[16];
    aes_cmac(NWKSKEY, buf, 16 + len, full);
    memcpy(mic, full, 4);
}

int lorawan_build_uplink(uint8_t *out, const uint8_t *app_payload,
                         int app_len, uint8_t fport, uint32_t fcnt)
{
    uint8_t devaddr_le[4] = { DEVADDR[3], DEVADDR[2], DEVADDR[1], DEVADDR[0] };
    int p = 0;

    out[p++] = 0x40;                        /* MHDR: unconfirmed data up */
    out[p++] = devaddr_le[0];               /* DevAddr little-endian */
    out[p++] = devaddr_le[1];
    out[p++] = devaddr_le[2];
    out[p++] = devaddr_le[3];
    out[p++] = 0x00;                        /* FCtrl */
    out[p++] = (uint8_t)(fcnt & 0xFF);      /* FCnt little-endian */
    out[p++] = (uint8_t)((fcnt >> 8) & 0xFF);
    out[p++] = fport;                       /* FPort */

    /* payload: копируем и шифруем на месте */
    uint8_t enc[32];
    memcpy(enc, app_payload, app_len);
    encrypt_payload(enc, app_len, devaddr_le, fcnt);
    memcpy(out + p, enc, app_len);
    p += app_len;

    /* MIC по MHDR..FRMPayload */
    uint8_t mic[4];
    compute_mic(out, p, devaddr_le, fcnt, mic);
    memcpy(out + p, mic, 4);
    p += 4;

    return p;
}