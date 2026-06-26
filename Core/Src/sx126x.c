/**
  * @file    sx126x.c
  * @brief   Драйвер SX1262 (Ra-01SH). Параметры радио берутся из config.h.
  */
#include "sx126x.h"
#include "config.h"

/* ====== Опкоды команд SX1262 (из даташита) ====== */
#define SX_SET_STANDBY        0x80
#define SX_SET_TX             0x83
#define SX_SET_REGULATOR      0x96
#define SX_SET_DIO2_RFSW      0x9D
#define SX_CALIBRATE          0x89
#define SX_CALIBRATE_IMAGE    0x98
#define SX_SET_PACKET_TYPE    0x8A
#define SX_SET_RF_FREQUENCY   0x86
#define SX_SET_PA_CONFIG      0x95
#define SX_SET_TX_PARAMS      0x8E
#define SX_SET_MODULATION     0x8B
#define SX_SET_PACKET_PARAMS  0x8C
#define SX_SET_BUFFER_BASE    0x8F
#define SX_WRITE_BUFFER       0x0E
#define SX_WRITE_REGISTER     0x0D
#define SX_SET_DIO_IRQ        0x08
#define SX_CLR_IRQ_STATUS     0x02
#define SX_GET_STATUS         0xC0
#define SX_READ_REGISTER      0x1D
#define SX_GET_DEVICE_ERRORS  0x17

static SPI_HandleTypeDef *hspi = 0;

void sx_init(SPI_HandleTypeDef *h) { hspi = h; }

/* Программная задержка вместо аппаратного BUSY (пин BUSY не разведён). */
static void busy(void) { HAL_Delay(5); }

/* Базовая SPI-транзакция: NSS↓ → обмен → NSS↑ */
static void sx_cmd(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    busy();
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx, rx, len, 100);
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
}

void sx_reset(void)
{
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
}

uint8_t sx_chipmode(void)
{
    uint8_t tx[2] = { SX_GET_STATUS, 0x00 };
    uint8_t rx[2] = {0};
    sx_cmd(tx, rx, 2);
    return (rx[1] >> 4) & 0x7;
}

uint8_t sx_readreg(uint16_t addr)
{
    uint8_t tx[5] = { SX_READ_REGISTER, (addr >> 8) & 0xFF, addr & 0xFF, 0x00, 0x00 };
    uint8_t rx[5] = {0};
    sx_cmd(tx, rx, 5);
    return rx[4];
}

void sx_radio_config(void)
{
    uint8_t rx[10] = {0};

    { uint8_t t[2] = { SX_SET_STANDBY, 0x00 }; sx_cmd(t, rx, 2); HAL_Delay(5); }
    { uint8_t t[2] = { SX_CALIBRATE, 0x7F };   sx_cmd(t, rx, 2); HAL_Delay(20); }
    { uint8_t t[1] = { 0x07 };                 sx_cmd(t, rx, 1); HAL_Delay(5); }

    /* Регулятор LDO (Ra-01SH не имеет DC-DC) */
    { uint8_t t[2] = { SX_SET_REGULATOR, 0x00 }; sx_cmd(t, rx, 2); HAL_Delay(5); }
    /* DIO2 управляет RF-свитчем */
    { uint8_t t[2] = { SX_SET_DIO2_RFSW, 0x01 }; sx_cmd(t, rx, 2); HAL_Delay(5); }
    /* CalibrateImage 863-870 МГц */
    { uint8_t t[3] = { SX_CALIBRATE_IMAGE, 0xD7, 0xDB }; sx_cmd(t, rx, 3); HAL_Delay(5); }
    /* PacketType = LoRa */
    { uint8_t t[2] = { SX_SET_PACKET_TYPE, 0x01 }; sx_cmd(t, rx, 2); HAL_Delay(5); }

    /* Частота (из config.h) */
    { uint8_t t[5] = { SX_SET_RF_FREQUENCY,
                       CFG_FREQ_B1, CFG_FREQ_B2, CFG_FREQ_B3, CFG_FREQ_B4 };
      sx_cmd(t, rx, 5); HAL_Delay(5); }

    /* PA config +22 dBm */
    { uint8_t t[5] = { SX_SET_PA_CONFIG, 0x04, 0x07, 0x00, 0x01 };
      sx_cmd(t, rx, 5); HAL_Delay(5); }
    /* TX params: мощность из config, ramp 200us */
    { uint8_t t[3] = { SX_SET_TX_PARAMS, CFG_TX_POWER, 0x04 };
      sx_cmd(t, rx, 3); HAL_Delay(5); }

    /* Модуляция: SF/BW/CR из config, LDRO on */
    { uint8_t t[5] = { SX_SET_MODULATION, CFG_SF, CFG_BW, CFG_CR, 0x01 };
      sx_cmd(t, rx, 5); HAL_Delay(5); }
    /* Синхрослово LoRaWAN public 0x3444 */
    { uint8_t t[5] = { SX_WRITE_REGISTER, 0x07, 0x40, 0x34, 0x44 };
      sx_cmd(t, rx, 5); HAL_Delay(5); }
}

void sx_send(const uint8_t *frame, uint8_t len)
{
    uint8_t rx[10] = {0};

    { uint8_t t[2] = { SX_SET_STANDBY, 0x00 }; sx_cmd(t, rx, 2); HAL_Delay(5); }
    { uint8_t t[3] = { SX_SET_BUFFER_BASE, 0x00, 0x00 }; sx_cmd(t, rx, 3); HAL_Delay(5); }
    /* packet params: preamble 8, explicit header, payload=len, CRC on, IQ std */
    { uint8_t t[7] = { SX_SET_PACKET_PARAMS, 0x00, 0x08, 0x00, len, 0x01, 0x00 };
      sx_cmd(t, rx, 7); HAL_Delay(5); }

    /* запись кадра в буфер чипа */
    {
        uint8_t wr[1]   = { SX_WRITE_BUFFER };
        uint8_t addr[1] = { 0x00 };
        busy();
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(hspi, wr,   1,   100);
        HAL_SPI_Transmit(hspi, addr, 1,   100);
        HAL_SPI_Transmit(hspi, (uint8_t *)frame, len, 100);
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
        HAL_Delay(5);
    }

    /* разрешить IRQ и запустить передачу */
    { uint8_t t[3] = { SX_CLR_IRQ_STATUS, 0xFF, 0xFF }; sx_cmd(t, rx, 3); HAL_Delay(5); }
    { uint8_t t[4] = { SX_SET_TX, 0x00, 0x00, 0x00 };   sx_cmd(t, rx, 4); }
}