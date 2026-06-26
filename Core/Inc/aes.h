/**
  * @file    aes.h
  * @brief   AES-128: шифрование блока и CMAC. Чистая реализация без зависимостей.
  */
#ifndef AES_H
#define AES_H

#include <stdint.h>

/* Расширение ключа: на выходе 176 байт (11 round-ключей по 16). */
void aes_key_expand(const uint8_t key[16], uint8_t rk[176]);

/* Шифрование одного блока 16 байт (in-place) расширенным ключом rk. */
void aes_encrypt_block(const uint8_t rk[176], uint8_t state[16]);

/* AES-CMAC: вычисляет mac[16] по сообщению msg[len] ключом key[16]. */
void aes_cmac(const uint8_t key[16], const uint8_t *msg, int len, uint8_t mac[16]);

#endif /* AES_H */