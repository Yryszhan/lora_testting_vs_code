/**
  * @file    wdt.h
  * @brief   Независимый watchdog IWDG (объявления).
  */
#ifndef WDT_H
#define WDT_H

#include <stdint.h>

/* Инициализация IWDG (~26 c таймаут). Вызвать один раз при старте. */
void wdt_init(void);

/* Сброс watchdog. Вызывать в цикле чаще таймаута (< 26 c). */
void wdt_kick(void);

/* Задержка с периодическим сбросом watchdog (замена длинного HAL_Delay). */
void wdt_delay(uint32_t total_ms);

#endif /* WDT_H */