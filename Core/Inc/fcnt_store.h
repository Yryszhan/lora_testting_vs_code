/**
  * @file    fcnt_store.h
  * @brief   Хранение LoRaWAN FCnt во Flash (объявления).
  */
#ifndef FCNT_STORE_H
#define FCNT_STORE_H

#include <stdint.h>

/* Загрузить последний сохранённый FCnt (0 если хранилище пусто). */
uint32_t fcnt_load(void);

/* Сохранить новое значение FCnt во Flash. */
void fcnt_save(uint32_t fcnt);

#endif /* FCNT_STORE_H */