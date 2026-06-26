/**
  * @file    lorawan.h
  * @brief   LoRaWAN 1.0 ABP uplink: шифрование payload + MIC + сборка кадра.
  *          Использует AES-слой (aes.h) и ключи из config.h.
  */
#ifndef LORAWAN_H
#define LORAWAN_H

#include <stdint.h>

/**
  * @brief Собрать готовый LoRaWAN uplink-кадр.
  * @param out          буфер результата (>= app_len + 13 байт)
  * @param app_payload  открытые данные приложения
  * @param app_len      длина данных
  * @param fport        порт (обычно 1)
  * @param fcnt         текущий счётчик кадров
  * @return длина собранного кадра в байтах
  */
int lorawan_build_uplink(uint8_t *out, const uint8_t *app_payload,
                         int app_len, uint8_t fport, uint32_t fcnt);

#endif /* LORAWAN_H */