/*
 * SX1262.c — полный драйвер для LoRaWAN uplink (EU868, SF12BW125)
 * Шаг 1: физический уровень. AES/MIC будут отдельным слоем.
 * Базовый автор: Saba_Abiri, переработка под Ra-01SH + ChirpStack.
 */

#include "SX1262.h"
#include "string.h"
#include "math.h"

#define XTAL_FREQUENCY (double)32000000
#define FREQ_STEP      (double)(XTAL_FREQUENCY / 524288.0)

SX1262 SX_stc = {0};
uint8_t inReceiveMode = 0;
volatile uint8_t Status_Now = 0;
HAL_StatusTypeDef err22 = 0;

/* ---------- базовый низкий уровень ---------- */

SX1262 *SX1262_Get_st(void) { return(&SX_stc); }

void SX1262_CSLow(void)  { HAL_GPIO_WritePin(SX_stc.NSS_Port, SX_stc.NSS_Pin_, GPIO_PIN_RESET); }
void SX1262_CSHigh(void) { HAL_GPIO_WritePin(SX_stc.NSS_Port, SX_stc.NSS_Pin_, GPIO_PIN_SET); }

int SX1262_IsBusy(void) { return 0; }

/* Софт-замена BUSY: на твоей плате пин не разведён, поэтому ждём с запасом */
void SX1262_BusyWait(void)
{
    HAL_Delay(5);
}

void SX1262_TxWait(void) { while(SX_stc.State == RADIO_TX) {} }

void SX1262_Set_Command(uint8_t *cmnd_, uint8_t *ans_, uint16_t Len,
                        uint32_t Time_out, uint16_t Delay)
{
    SX1262_BusyWait();
    SX1262_CSLow();
    err22 = HAL_SPI_TransmitReceive(&SX_stc.SPI, cmnd_, ans_, Len, Time_out);
    SX1262_CSHigh();
    if(Delay) HAL_Delay(Delay);
}

/* ---------- статус / ожидание ---------- */

uint8_t SX1262_getstatus(void)
{
    uint8_t c[2] = {0xC0, 0x00};
    uint8_t a[2] = {0};
    SX1262_Set_Command(c, a, 2, 100, 0);
    uint8_t chipMode = (a[1] >> 4) & 0x7;
    Status_Now = chipMode;
    return(chipMode);
}

uint8_t SX1262_waitForRadioCommandCompletion(uint32_t timeout)
{
    uint32_t startTime = HAL_GetTick();
    uint8_t c[2] = {0xC0, 0x00};
    uint8_t a[2] = {0};
    while (1)
    {
        SX1262_Set_Command(c, a, 2, 100, 0);
        uint8_t chipMode = (a[1] >> 4) & 0x7;
        // 0x02 = STBY_RC, 0x03 = STBY_XOSC — значит TX/команда завершилась
        if (chipMode == 0x02 || chipMode == 0x03) return 0;
        if (HAL_GetTick() - startTime >= timeout) return 1;
    }
}

void SX1262_setModeStandby(void)
{
    uint8_t c[2] = {0x80, 0x00};   // STDBY_RC
    uint8_t a[2] = {0};
    SX1262_Set_Command(c, a, 2, 100, 0);
}

/* ---------- проверка чипа ---------- */

uint8_t SX1262_Check_Correct(void)
{
    // Чтение версии-сигнатуры из 0x0320 (рабочий ответ 'S' = 0x53)
    uint8_t c[5] = {0x1D, 0x03, 0x20, 0x00, 0x00};
    uint8_t a[5] = {0};
    SX1262_Set_Command(c, a, 5, 100, 0);
    if(err22) return(2);
    if(a[4] == 0x53) return(0);
    return(1);
}

/* ---------- инициализация ---------- */

void SX1262_Init(void)
{
    SX1262_CSHigh();

    // Hardware reset
    HAL_GPIO_WritePin(SX_stc.Reset_Port, SX_stc.Reset_Pin_, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(SX_stc.Reset_Port, SX_stc.Reset_Pin_, GPIO_PIN_SET);
    HAL_Delay(100);

    // Standby RC
    uint8_t stb[2] = {0x80, 0x00};
    uint8_t a[4] = {0};
    SX1262_Set_Command(stb, a, 2, 100, 10);

    // TCXO через DIO3: 3.3В, таймаут старта ~10мс (0x64 * 15.625us)
    uint8_t tcxo[4] = {0x97, 0x07, 0x00, 0x64};
    SX1262_Set_Command(tcxo, a, 4, 100, 20);

    // Калибровка всех блоков ПОСЛЕ запуска TCXO
    uint8_t cal[2] = {0x89, 0x7F};
    SX1262_Set_Command(cal, a, 2, 100, 20);

    // DC-DC регулятор
    uint8_t reg[2] = {0x96, 0x01};
    SX1262_Set_Command(reg, a, 2, 100, 10);

    if(SX1262_Check_Correct() == 0)
    {
        SX1262_Radio_essental_Config();
    }
}

/* ---------- основной конфиг радиотракта ---------- */

void SX1262_Radio_essental_Config(void)
{
    uint8_t c[10], a[10];

    // DIO2 управляет антенным RF-свитчем
    c[0]=0x9D; c[1]=0x01; SX1262_Set_Command(c,a,2,100,10);

    // CalibrateImage для 863–870 МГц
    c[0]=0x98; c[1]=0xD7; c[2]=0xDB; SX1262_Set_Command(c,a,3,100,10);

    // PacketType = LoRa
    c[0]=0x8A; c[1]=0x01; SX1262_Set_Command(c,a,2,100,10);

    // Частота 868.1 МГц (канал 0 шлюза)
    c[0]=0x86; c[1]=0x36; c[2]=0x40; c[3]=0xCC; c[4]=0xCC;
    SX1262_Set_Command(c,a,5,100,10);

    // PA config: +22 dBm для SX1262 (deviceSel=0)
    c[0]=0x95; c[1]=0x04; c[2]=0x07; c[3]=0x00; c[4]=0x01;
    SX1262_Set_Command(c,a,5,100,10);

    // TX params: 22 dBm, ramp 200us
    c[0]=0x8E; c[1]=22; c[2]=0x04; SX1262_Set_Command(c,a,3,100,10);

    // Модуляция: SF12, BW125, CR4/5, LowDataRateOptimize ON
    c[0]=0x8B; c[1]=0x0C; c[2]=0x04; c[3]=0x01; c[4]=0x01;
    SX1262_Set_Command(c,a,5,100,10);

    // DIO IRQ: TxDone(bit0) + Timeout(bit9) на DIO1
    c[0]=0x08; c[1]=0x00; c[2]=0x02; c[3]=0x00; c[4]=0x02;
    c[5]=0x00; c[6]=0x00; c[7]=0x00; c[8]=0x00;
    SX1262_Set_Command(c,a,9,100,10);

    // Публичное синхрослово LoRaWAN 0x3444 (регистры 0x0740/0x0741)
    uint8_t s[5] = {0x0D, 0x07, 0x40, 0x34, 0x44};
    SX1262_Set_Command(s,a,5,100,10);
}

/* ---------- частота вручную ---------- */

void SX1262_SetFrequency(uint32_t frequency)
{
    uint8_t buf[5], la[5] = {0};
    uint32_t freq = (uint32_t)((double)frequency / FREQ_STEP);
    buf[0]=0x86;
    buf[1]=(freq>>24)&0xFF; buf[2]=(freq>>16)&0xFF;
    buf[3]=(freq>>8)&0xFF;  buf[4]=freq&0xFF;
    SX1262_Set_Command(buf, la, 5, 100, 0);
}

/* ---------- ВРЕМЕННЫЙ ТЕСТ: чистая несущая ---------- */
/* Вызови один раз вместо передачи. Чип будет лить CW на 868.1 МГц.
   Проверяй RTL-SDR / спектроанализатором. Несущая есть = TX-тракт жив. */
void SX1262_TestCW(void)
{
    uint8_t a[5];
    uint8_t pt[2]={0x8A,0x01};                 SX1262_Set_Command(pt,a,2,100,10);
    uint8_t sw[2]={0x9D,0x01};                 SX1262_Set_Command(sw,a,2,100,10);
    uint8_t fr[5]={0x86,0x36,0x40,0xCC,0xCC};  SX1262_Set_Command(fr,a,5,100,10);
    uint8_t pa[5]={0x95,0x04,0x07,0x00,0x01};  SX1262_Set_Command(pa,a,5,100,10);
    uint8_t tp[3]={0x8E,22,0x04};              SX1262_Set_Command(tp,a,3,100,10);
    uint8_t cw[1]={0xD1};                      SX1262_Set_Command(cw,a,1,100,0); // SetTxContinuousWave
}

/* ---------- передача пакета ---------- */

void SX1262_Transmit(uint8_t* data, uint8_t len)
{
    uint8_t a[10];

    // Standby перед загрузкой
    uint8_t stb[2]={0x80,0x00}; SX1262_Set_Command(stb,a,2,100,5);

    // Базовый адрес буфера: TX=0x00, RX=0x00
    uint8_t bb[3]={0x8F,0x00,0x00}; SX1262_Set_Command(bb,a,3,100,5);

    // Параметры пакета LoRa (6 байт):
    // preambleMSB, preambleLSB, headerType, payloadLen, CRC, InvertIQ
    uint8_t pp[7];
    pp[0]=0x8C;
    pp[1]=0x00;     // preamble MSB
    pp[2]=0x08;     // preamble = 8 символов (стандарт LoRaWAN)
    pp[3]=0x00;     // explicit header
    pp[4]=len;      // payload length
    pp[5]=0x01;     // CRC ON  <-- ключевая правка
    pp[6]=0x00;     // InvertIQ OFF (правильно для uplink)
    SX1262_Set_Command(pp,a,7,100,5);

    // Запись payload в FIFO с адреса 0x00
    uint8_t wr_cmd[1]={0x0E};
    uint8_t addr[1]={0x00};
    uint8_t dummy[256]={0};
    SX1262_BusyWait();
    SX1262_CSLow();
    HAL_SPI_TransmitReceive(&SX_stc.SPI, wr_cmd, dummy, 1, 100);
    HAL_SPI_TransmitReceive(&SX_stc.SPI, addr,   dummy, 1, 100);
    HAL_SPI_TransmitReceive(&SX_stc.SPI, data,   dummy, len, 100);
    SX1262_CSHigh();
    HAL_Delay(5);

    // Очистка IRQ-флагов
    uint8_t clr[3]={0x02,0xFF,0xFF}; SX1262_Set_Command(clr,a,3,100,5);

    // SetTx, timeout=0 (без аппаратного таймаута)
    uint8_t tx[4]={0x83,0x00,0x00,0x00}; SX1262_Set_Command(tx,a,4,100,0);

    // SF12 + ~15 байт ≈ 1.3 с Time-on-Air. Ждём с запасом.
    HAL_Delay(2000);

    // Возврат в standby
    SX1262_Set_Command(stb,a,2,100,5);
    inReceiveMode = 0;
}

/* ---------- приём (оставлено для будущего) ---------- */

void SX1262_setRX(void)
{
    uint8_t a[7];
    uint8_t pp[7]={0x8C,0x00,0x08,0x00,0xFF,0x01,0x00};
    SX1262_Set_Command(pp,a,7,100,0);
    uint8_t rx[4]={0x82,0xFF,0xFF,0xFF}; // continuous
    SX1262_Set_Command(rx,a,4,100,0);
    inReceiveMode = 1;
}

void SX1262_setModeReceive(void) { SX1262_setRX(); }

void SX1262_HandleCallback(uint8_t *buf, uint8_t *len)
{
    uint8_t bs[4]={0x13,0xFF,0xFF,0xFF};
    uint8_t ba[4]={0};
    SX1262_Set_Command(bs,ba,4,100,0);
    uint8_t payloadLen   = ba[2];
    uint8_t startAddress = ba[3];

    uint8_t rd[3]={0x1E,startAddress,0x00};
    uint8_t out[256]={0};
    SX1262_BusyWait();
    SX1262_CSLow();
    HAL_SPI_TransmitReceive(&SX_stc.SPI, rd, out, 3, 100);
    if(payloadLen>0){
        uint8_t dummy[256]={0};
        HAL_SPI_TransmitReceive(&SX_stc.SPI, dummy, out, payloadLen, 100);
    }
    SX1262_CSHigh();
    memcpy(buf,out,payloadLen);
    *len=payloadLen;
}