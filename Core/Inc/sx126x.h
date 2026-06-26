/**
  * @file    sx126x.h
  * @brief   Драйвер SX1262 (Ra-01SH) по голому SPI для LoRaWAN-передачи.
  *
  *  Зависит от HAL (SPI + GPIO). SPI-хэндл передаётся через sx_init().
  *  Пины NSS/RST/DIO1 определены в main.h (генерация CubeMX).
  */
#ifndef SX126X_H
#define SX126X_H

#include <stdint.h>
#include "main.h"     /* для SPI_HandleTypeDef и определений пинов */

/* Передать драйверу хэндл SPI. Вызвать один раз перед использованием. */
void sx_init(SPI_HandleTypeDef *hspi);

/* Аппаратный сброс чипа через RST. */
void sx_reset(void);

/* Чтение текущего режима чипа (для диагностики): TX=6, STBY=2 и т.д. */
uint8_t sx_chipmode(void);

/* Чтение регистра чипа (для проверки сигнатуры). */
uint8_t sx_readreg(uint16_t addr);

/* Полная конфигурация радио (частота, SF, мощность, свитч). */
void sx_radio_config(void);

/* Передать готовый кадр frame[len] в эфир. */
void sx_send(const uint8_t *frame, uint8_t len);

#endif /* SX126X_H */