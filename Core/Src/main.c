/* USER CODE BEGIN Header */
/**
  * @file    main.c
  * @brief   SX1262 (Ra-01SH) — LoRaWAN ABP uplink "HI" с AES (MIC + шифрование).
  *          868.1 МГц, SF12 BW125, 22 dBm, LDO.
  */
/* USER CODE END Header */
#include "main.h"
#include <string.h>

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
volatile uint8_t  sx_status     = 0;
volatile uint8_t  reg320        = 0;
volatile uint8_t  err_lsb       = 0;
volatile uint8_t  mode_after_tx = 0;
volatile uint8_t  tx_reached    = 0;
volatile uint32_t tx_count      = 0;

/* ====== LoRaWAN ABP параметры (СОВПАДАЮТ с ChirpStack) ====== */
/* DevAddr 00886B53 — в кадре идёт little-endian: 53 6B 88 00 */
static const uint8_t DEVADDR[4] = {0x00,0x88,0x6B,0x53};   /* big-endian запись */

/* NwkSKey = 00112233445566778899AABBCCDDEEFF */
static const uint8_t NWKSKEY[16] = {
    0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
    0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF
};
/* AppSKey = 0102030405060708090A0B0C0D0E0F10 */
static const uint8_t APPSKEY[16] = {
    0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,
    0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10
};

static uint32_t fcnt_up = 0;   /* счётчик кадров uplink */
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* ============================================================
 *                    AES-128 (компактный)
 * ============================================================ */
/* USER CODE BEGIN AES */
static const uint8_t aes_sbox[256] = {
0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16
};

static uint8_t aes_rcon[11] = {
0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1b,0x36
};

static uint8_t xtime(uint8_t x){ return (uint8_t)((x<<1) ^ ((x>>7)*0x1b)); }

/* Расширение ключа: 11 round-ключей по 16 байт */
static void aes_key_expand(const uint8_t key[16], uint8_t rk[176])
{
    memcpy(rk, key, 16);
    uint8_t t[4];
    for (int i = 16; i < 176; i += 4) {
        memcpy(t, rk + i - 4, 4);
        if (i % 16 == 0) {
            uint8_t tmp = t[0];
            t[0] = aes_sbox[t[1]] ^ aes_rcon[i/16];
            t[1] = aes_sbox[t[2]];
            t[2] = aes_sbox[t[3]];
            t[3] = aes_sbox[tmp];
        }
        for (int j = 0; j < 4; j++)
            rk[i+j] = rk[i-16+j] ^ t[j];
    }
}

/* Шифрование одного блока 16 байт (in-place) */
static void aes_encrypt_block(const uint8_t rk[176], uint8_t s[16])
{
    /* AddRoundKey 0 */
    for (int i=0;i<16;i++) s[i]^=rk[i];

    for (int round=1; round<10; round++){
        /* SubBytes */
        for(int i=0;i<16;i++) s[i]=aes_sbox[s[i]];
        /* ShiftRows */
        uint8_t t;
        t=s[1];  s[1]=s[5];  s[5]=s[9];  s[9]=s[13];  s[13]=t;
        t=s[2];  s[2]=s[10]; s[10]=t; t=s[6]; s[6]=s[14]; s[14]=t;
        t=s[15]; s[15]=s[11];s[11]=s[7];s[7]=s[3];  s[3]=t;
        /* MixColumns */
        for(int c=0;c<4;c++){
            uint8_t *col=s+c*4;
            uint8_t a0=col[0],a1=col[1],a2=col[2],a3=col[3];
            uint8_t h=a0^a1^a2^a3;
            col[0]^=h^xtime(a0^a1);
            col[1]^=h^xtime(a1^a2);
            col[2]^=h^xtime(a2^a3);
            col[3]^=h^xtime(a3^a0);
        }
        /* AddRoundKey */
        for(int i=0;i<16;i++) s[i]^=rk[round*16+i];
    }
    /* финальный раунд (без MixColumns) */
    for(int i=0;i<16;i++) s[i]=aes_sbox[s[i]];
    uint8_t t;
    t=s[1];  s[1]=s[5];  s[5]=s[9];  s[9]=s[13];  s[13]=t;
    t=s[2];  s[2]=s[10]; s[10]=t; t=s[6]; s[6]=s[14]; s[14]=t;
    t=s[15]; s[15]=s[11];s[11]=s[7];s[7]=s[3];  s[3]=t;
    for(int i=0;i<16;i++) s[i]^=rk[160+i];
}

/* ============================================================
 *               AES-CMAC (для MIC)
 * ============================================================ */
static void leftshift_onebit(const uint8_t *in, uint8_t *out)
{
    uint8_t overflow=0;
    for(int i=15;i>=0;i--){
        out[i]=(uint8_t)((in[i]<<1)|overflow);
        overflow=(in[i]&0x80)?1:0;
    }
}

static void cmac_subkey(const uint8_t rk[176], uint8_t k1[16], uint8_t k2[16])
{
    uint8_t L[16]={0};
    aes_encrypt_block(rk, L);          /* AES(0^16) */
    leftshift_onebit(L,k1);
    if(L[0]&0x80) k1[15]^=0x87;
    leftshift_onebit(k1,k2);
    if(k1[0]&0x80) k2[15]^=0x87;
}

/* CMAC по сообщению msg[len] ключом key, результат mac[16] */
static void aes_cmac(const uint8_t key[16], const uint8_t *msg, int len, uint8_t mac[16])
{
    uint8_t rk[176];
    aes_key_expand(key, rk);
    uint8_t k1[16],k2[16];
    cmac_subkey(rk,k1,k2);

    int n=(len+15)/16;
    int flag_complete;
    if(n==0){ n=1; flag_complete=0; }
    else flag_complete=(len%16==0);

    uint8_t X[16]={0};
    uint8_t M_last[16];

    /* последний блок */
    int last_start=(n-1)*16;
    if(flag_complete){
        for(int i=0;i<16;i++) M_last[i]=msg[last_start+i]^k1[i];
    } else {
        int rem=len-last_start;
        for(int i=0;i<16;i++){
            uint8_t b;
            if(i<rem) b=msg[last_start+i];
            else if(i==rem) b=0x80;
            else b=0x00;
            M_last[i]=b^k2[i];
        }
    }

    /* блоки 0..n-2 */
    for(int i=0;i<n-1;i++){
        for(int j=0;j<16;j++) X[j]^=msg[i*16+j];
        aes_encrypt_block(rk,X);
    }
    for(int j=0;j<16;j++) X[j]^=M_last[j];
    aes_encrypt_block(rk,X);
    memcpy(mac,X,16);
}

/* ============================================================
 *      LoRaWAN: шифрование payload (AES-CTR) + MIC (B0)
 * ============================================================ */

/* Шифрование/дешифрование FRMPayload по LoRaWAN (FCntUp в dir=0) */
static void lorawan_encrypt_payload(uint8_t *data, int len,
                                    const uint8_t appskey[16],
                                    const uint8_t devaddr_le[4],
                                    uint32_t fcnt)
{
    uint8_t rk[176];
    aes_key_expand(appskey, rk);
    int k=(len+15)/16;
    for(int i=1;i<=k;i++){
        uint8_t A[16]={0x01,0,0,0,0, 0x00, /* dir=0 uplink */
                       devaddr_le[0],devaddr_le[1],devaddr_le[2],devaddr_le[3],
                       (uint8_t)(fcnt&0xFF),(uint8_t)((fcnt>>8)&0xFF),
                       (uint8_t)((fcnt>>16)&0xFF),(uint8_t)((fcnt>>24)&0xFF),
                       0x00, (uint8_t)i};
        uint8_t S[16];
        memcpy(S,A,16);
        aes_encrypt_block(rk,S);
        int blockstart=(i-1)*16;
        int n=len-blockstart; if(n>16) n=16;
        for(int j=0;j<n;j++) data[blockstart+j]^=S[j];
    }
}

/* MIC по LoRaWAN 1.0: B0 || msg, CMAC с NwkSKey, первые 4 байта */
static void lorawan_compute_mic(const uint8_t *msg, int len,
                                const uint8_t nwkskey[16],
                                const uint8_t devaddr_le[4],
                                uint32_t fcnt, uint8_t mic[4])
{
    uint8_t buf[64];
    /* блок B0 */
    uint8_t B0[16]={0x49,0,0,0,0, 0x00, /* dir=0 */
                    devaddr_le[0],devaddr_le[1],devaddr_le[2],devaddr_le[3],
                    (uint8_t)(fcnt&0xFF),(uint8_t)((fcnt>>8)&0xFF),
                    (uint8_t)((fcnt>>16)&0xFF),(uint8_t)((fcnt>>24)&0xFF),
                    0x00,(uint8_t)len};
    memcpy(buf,B0,16);
    memcpy(buf+16,msg,len);
    uint8_t full[16];
    aes_cmac(nwkskey, buf, 16+len, full);
    memcpy(mic,full,4);
}

/* Собирает готовый LoRaWAN-кадр в out[], возвращает длину */
static int lorawan_build_uplink(uint8_t *out, const uint8_t *app_payload,
                                int app_len, uint8_t fport, uint32_t fcnt)
{
    uint8_t devaddr_le[4]={DEVADDR[3],DEVADDR[2],DEVADDR[1],DEVADDR[0]};
    int p=0;
    out[p++]=0x40;                 /* MHDR: unconfirmed data up */
    out[p++]=devaddr_le[0];        /* DevAddr little-endian */
    out[p++]=devaddr_le[1];
    out[p++]=devaddr_le[2];
    out[p++]=devaddr_le[3];
    out[p++]=0x00;                 /* FCtrl */
    out[p++]=(uint8_t)(fcnt&0xFF); /* FCnt little-endian */
    out[p++]=(uint8_t)((fcnt>>8)&0xFF);
    out[p++]=fport;                /* FPort */

    /* копируем payload и шифруем его на месте */
    uint8_t enc[32];
    memcpy(enc, app_payload, app_len);
    lorawan_encrypt_payload(enc, app_len, APPSKEY, devaddr_le, fcnt);
    memcpy(out+p, enc, app_len);
    p+=app_len;

    /* MIC по всему, что собрали (MHDR..FRMPayload) */
    uint8_t mic[4];
    lorawan_compute_mic(out, p, NWKSKEY, devaddr_le, fcnt, mic);
    memcpy(out+p, mic, 4);
    p+=4;

    return p;  /* итоговая длина кадра */
}
/* USER CODE END AES */

/* USER CODE BEGIN 0 */
static void busy(void) { HAL_Delay(5); }

static void sx_cmd(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    busy();
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, 100);
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
}

static void sx_reset(void)
{
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
}

static uint8_t sx_chipmode(void)
{
    uint8_t tx[2] = {0xC0, 0x00};
    uint8_t rx[2] = {0};
    sx_cmd(tx, rx, 2);
    return (rx[1] >> 4) & 0x7;
}

static uint8_t sx_readreg(uint16_t addr)
{
    uint8_t tx[5] = {0x1D, (addr>>8)&0xFF, addr&0xFF, 0x00, 0x00};
    uint8_t rx[5] = {0};
    sx_cmd(tx, rx, 5);
    return rx[4];
}

static void sx_radio_config(void)
{
    uint8_t rx[10] = {0};
    { uint8_t t[2]={0x80,0x00}; sx_cmd(t,rx,2); HAL_Delay(5); }
    { uint8_t t[2]={0x89,0x7F}; sx_cmd(t,rx,2); HAL_Delay(20); }
    { uint8_t t[1]={0x07}; sx_cmd(t,rx,1); HAL_Delay(5); }

    /* Регулятор LDO */
    { uint8_t t[2]={0x96,0x00}; sx_cmd(t,rx,2); HAL_Delay(5); }
    /* DIO2 управляет RF-свитчем */
    { uint8_t t[2]={0x9D,0x01}; sx_cmd(t,rx,2); HAL_Delay(5); }
    /* CalibrateImage 863-870 */
    { uint8_t t[3]={0x98,0xD7,0xDB}; sx_cmd(t,rx,3); HAL_Delay(5); }
    /* PacketType LoRa */
    { uint8_t t[2]={0x8A,0x01}; sx_cmd(t,rx,2); HAL_Delay(5); }

    /* Частота 868.100 МГц */
    { uint8_t t[5]={0x86,0x36,0x40,0x3D,0x60}; sx_cmd(t,rx,5); HAL_Delay(5); }

    /* PA config +22 dBm */
    { uint8_t t[5]={0x95,0x04,0x07,0x00,0x01}; sx_cmd(t,rx,5); HAL_Delay(5); }
    /* TX params 22 dBm, ramp 200us */
    { uint8_t t[3]={0x8E,22,0x04}; sx_cmd(t,rx,3); HAL_Delay(5); }

    /* Модуляция SF12 BW125 CR4/5 LDRO on */
    { uint8_t t[5]={0x8B,0x0C,0x04,0x01,0x01}; sx_cmd(t,rx,5); HAL_Delay(5); }
    /* Синхрослово LoRaWAN public 0x3444 */
    { uint8_t t[5]={0x0D,0x07,0x40,0x34,0x44}; sx_cmd(t,rx,5); HAL_Delay(5); }
}

static void sx_send_packet(void)
{
    uint8_t rx[10] = {0};

    /* === собираем LoRaWAN-кадр с AES === */
    uint8_t app[2] = {'H','I'};            /* открытый payload */
    uint8_t frame[32];
    int flen = lorawan_build_uplink(frame, app, 2, 0x01, fcnt_up);

    { uint8_t t[2]={0x80,0x00}; sx_cmd(t,rx,2); HAL_Delay(5); }
    { uint8_t t[3]={0x8F,0x00,0x00}; sx_cmd(t,rx,3); HAL_Delay(5); }
    /* packet params: preamble 8, explicit, payload=flen, CRC on, IQ std */
    { uint8_t t[7]={0x8C,0x00,0x08,0x00,(uint8_t)flen,0x01,0x00}; sx_cmd(t,rx,7); HAL_Delay(5); }
    {
        uint8_t wr[1]={0x0E}; uint8_t addr[1]={0x00};
        busy();
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, wr,   1, 100);     /* WriteBuffer opcode */
        HAL_SPI_Transmit(&hspi1, addr, 1, 100);     /* offset 0 */
        HAL_SPI_Transmit(&hspi1, frame, flen, 100); /* сам кадр */
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
        HAL_Delay(5);
    }
    { uint8_t t[3]={0x02,0xFF,0xFF}; sx_cmd(t,rx,3); HAL_Delay(5); }
    { uint8_t t[4]={0x83,0x00,0x00,0x00}; sx_cmd(t,rx,4); }
    HAL_Delay(2);
    mode_after_tx = sx_chipmode();
    tx_reached = (mode_after_tx == 0x06) ? 1 : 0;
    HAL_Delay(2000);
    { uint8_t t[4]={0x17,0x00,0x00,0x00}; uint8_t r[4]={0};
      sx_cmd(t,r,4); err_lsb = r[3]; }

    fcnt_up++;      /* увеличиваем счётчик кадров */
    tx_count++;
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
  HAL_Delay(500);

  sx_reset();
  reg320    = sx_readreg(0x0320);
  sx_status = sx_chipmode();
  sx_radio_config();

  while(1)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    sx_send_packet();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(3000);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = NSS_Pin;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = RST_Pin;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO1_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}