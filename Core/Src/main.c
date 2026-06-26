/* USER CODE BEGIN Header */
/**
  * @file    main.c
  * @brief   QUASAR LoRaWAN node — прикладная логика.
  *          Вся реализация вынесена в библиотеки:
  *            aes.*        — AES-128 (шифрование + CMAC)
  *            lorawan.*    — сборка LoRaWAN-кадра
  *            sx126x.*     — драйвер радио Ra-01SH
  *            fcnt_store.* — хранение FCnt во Flash
  *            config.h     — все настройки и ключи
  *
  *  Watchdog (wdt.*) временно отключён — включить после активации
  *  IWDG в CubeMX (System Core -> IWDG -> Activated -> Generate Code).
  */
/* USER CODE END Header */
#include "main.h"
#include "config.h"
#include "sx126x.h"
#include "lorawan.h"
#include "fcnt_store.h"

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
static uint32_t fcnt_up = 0;   /* счётчик кадров uplink */
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN 0 */
static void led_on(void)  { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); }
static void led_off(void) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); }

/* Отправить одно LoRaWAN-сообщение */
static void send_message(const uint8_t *payload, int len)
{
    uint8_t frame[32];
    int flen = lorawan_build_uplink(frame, payload, len, 0x01, fcnt_up);
    sx_send(frame, (uint8_t)flen);
    HAL_Delay(50);          /* дать время на передачу */
    fcnt_up++;
    fcnt_save(fcnt_up);     /* сохранить счётчик в Flash */
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

    /* инициализация радио */
    sx_init(&hspi1);
    sx_reset();
    sx_radio_config();

    /* восстановить счётчик кадров из Flash */
    fcnt_up = fcnt_load();

    while (1)
    {
        uint8_t payload[2] = { 'H', 'I' };
        led_on();
        send_message(payload, 2);
        led_off();

        HAL_Delay(CFG_TX_INTERVAL_MS);
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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