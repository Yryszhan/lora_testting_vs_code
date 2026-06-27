/**
  * @file    config.h
  * @brief   Конфигурация QUASAR LoRaWAN node: радио, ключи ABP, интервалы.
  *
  *  ВНИМАНИЕ: здесь лежат секретные ключи. Если выкладываешь проект
  *  в публичный репозиторий — замени реальные ключи на заглушки,
  *  либо вынеси этот файл в .gitignore.
  */
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* ============== Параметры радио ============== */
/* Частота 868.100 МГц = 0x3D6036 (для команды SetRfFrequency) */
#define CFG_FREQ_B1   0x36
#define CFG_FREQ_B2   0x40
#define CFG_FREQ_B3   0x3D
#define CFG_FREQ_B4   0x60

#define CFG_TX_POWER  14        /* dBm */
#define CFG_SF        0x0C      /* SF12 */
#define CFG_BW        0x04      /* BW125 */
#define CFG_CR        0x01      /* CR 4/5 */

/* ============== Интервал передачи ============== */
#define CFG_TX_INTERVAL_MS   60000   /* пауза между uplink'ами, мс */

/* ============== LoRaWAN ABP ключи ==============
 * Должны совпадать с устройством в ChirpStack. */

/* DevAddr 00886B53 (big-endian запись, в кадр пойдёт little-endian) */
#define CFG_DEVADDR  { 0x00, 0x88, 0x6B, 0x53 }

/* NwkSKey = 00112233445566778899AABBCCDDEEFF */
#define CFG_NWKSKEY  { 0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77, \
                       0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF }

/* AppSKey = 0102030405060708090A0B0C0D0E0F10 */
#define CFG_APPSKEY  { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08, \
                       0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10 }

#endif /* CONFIG_H */