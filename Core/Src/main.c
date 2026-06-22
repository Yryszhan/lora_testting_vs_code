/* USER CODE BEGIN Header */
/**
  * @file    main.c
  * @brief   SX1262 (Ra-01SH) — передача, 22 dBm, LDO, точно 868.1 МГц.
  */
/* USER CODE END Header */
#include "main.h"

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
volatile uint8_t sx_status     = 0;
volatile uint8_t reg320        = 0;
volatile uint8_t err_lsb       = 0;
volatile uint8_t mode_after_tx = 0;
volatile uint8_t tx_reached    = 0;
volatile uint32_t tx_count     = 0;
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

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

    // Регулятор LDO
    { uint8_t t[2]={0x96,0x00}; sx_cmd(t,rx,2); HAL_Delay(5); }
    // DIO2 управляет RF-свитчем
    { uint8_t t[2]={0x9D,0x01}; sx_cmd(t,rx,2); HAL_Delay(5); }
    // CalibrateImage 863-870
    { uint8_t t[3]={0x98,0xD7,0xDB}; sx_cmd(t,rx,3); HAL_Delay(5); }
    // PacketType LoRa
    { uint8_t t[2]={0x8A,0x01}; sx_cmd(t,rx,2); HAL_Delay(5); }

    // Частота: ТОЧНО 868.100 МГц (было 868.135, смещение убрано)
    { uint8_t t[5]={0x86,0x36,0x40,0x3D,0x60}; sx_cmd(t,rx,5); HAL_Delay(5); }

    // PA config +22 dBm
    { uint8_t t[5]={0x95,0x04,0x07,0x00,0x01}; sx_cmd(t,rx,5); HAL_Delay(5); }
    // TX params 22 dBm, ramp 200us
    { uint8_t t[3]={0x8E,22,0x04}; sx_cmd(t,rx,3); HAL_Delay(5); }

    // Модуляция SF12 BW125 CR4/5 LDRO on
    { uint8_t t[5]={0x8B,0x0C,0x04,0x01,0x01}; sx_cmd(t,rx,5); HAL_Delay(5); }
    // Синхрослово LoRaWAN 0x3444
    { uint8_t t[5]={0x0D,0x07,0x40,0x34,0x44}; sx_cmd(t,rx,5); HAL_Delay(5); }
}

static void sx_send_packet(void)
{
    uint8_t rx[10] = {0};
    { uint8_t t[2]={0x80,0x00}; sx_cmd(t,rx,2); HAL_Delay(5); }
    { uint8_t t[3]={0x8F,0x00,0x00}; sx_cmd(t,rx,3); HAL_Delay(5); }
    { uint8_t t[7]={0x8C,0x00,0x08,0x00,15,0x01,0x00}; sx_cmd(t,rx,7); HAL_Delay(5); }
    {
        uint8_t payload[15] = {0x40,0x53,0x6b,0x88,0x00,0x00,0x00,0x00,
                               0x01,'H','I',0x00,0x00,0x00,0x00};
        uint8_t wr[1]={0x0E}; uint8_t addr[1]={0x00}; uint8_t d[16]={0};
        busy();
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(&hspi1, wr,   d, 1, 100);
        HAL_SPI_TransmitReceive(&hspi1, addr, d, 1, 100);
        HAL_SPI_TransmitReceive(&hspi1, payload, d, 15, 100);
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