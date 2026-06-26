/**
  * @file    fcnt_store.c
  * @brief   Хранение LoRaWAN FCnt в Flash STM32F103 с wear-leveling.
  *
  *  Идея: вместо стирания страницы на каждую запись, пишем значения
  *  последовательно в свободные слоты одной страницы. Стираем страницу
  *  только когда она заполнилась. Это многократно продлевает ресурс Flash.
  *
  *  Формат слота (8 байт): [ uint32 fcnt ][ uint32 magic=0xCAFEFC01 ]
  *  Пустой (стёртый) слот = 0xFFFFFFFF 0xFFFFFFFF.
  *  Актуальное значение = последний слот с корректным magic.
  */
#include "main.h"
#include <string.h>
#include "stdint.h"
/* ВАЖНО: выбери страницу Flash, которая НЕ занята программой.
 * Для STM32F103C8 (64 КБ Flash, страница 1 КБ) последняя страница = 0x0800FC00.
 * Для STM32F103CB (128 КБ) последняя = 0x0801FC00.
 * Проверь свой объём Flash и размер прошивки (.map файл)! */
#define FCNT_PAGE_ADDR    0x0800FC00U          /* адрес страницы хранения  */
#define FCNT_PAGE_SIZE    1024U                /* размер страницы (F103=1К)*/
#define FCNT_SLOT_SIZE    8U                   /* 4 байта fcnt + 4 magic   */
#define FCNT_MAGIC        0xCAFEFC01U          /* метка валидного слота    */
#define FCNT_SLOTS        (FCNT_PAGE_SIZE / FCNT_SLOT_SIZE)

/* Чтение 32-бит из Flash */
static inline uint32_t flash_rd32(uint32_t addr)
{
    return *(volatile uint32_t *)addr;
}

/* Стирание страницы хранения FCnt */
static void fcnt_erase_page(void)
{
    FLASH_EraseInitTypeDef er = {0};
    uint32_t page_error = 0;
    er.TypeErase   = FLASH_TYPEERASE_PAGES;
    er.PageAddress = FCNT_PAGE_ADDR;
    er.NbPages     = 1;
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&er, &page_error);
    HAL_FLASH_Lock();
}

/**
  * @brief Загрузить последний сохранённый FCnt.
  * @return сохранённое значение, либо 0 если страница пуста/повреждена.
  */
uint32_t fcnt_load(void)
{
    uint32_t last = 0;
    int found = 0;
    for (uint32_t i = 0; i < FCNT_SLOTS; i++) {
        uint32_t addr  = FCNT_PAGE_ADDR + i * FCNT_SLOT_SIZE;
        uint32_t value = flash_rd32(addr);
        uint32_t magic = flash_rd32(addr + 4);
        if (magic == FCNT_MAGIC) {
            last  = value;     /* перезаписываем — нужен ПОСЛЕДНИЙ валидный */
            found = 1;
        } else if (value == 0xFFFFFFFFU && magic == 0xFFFFFFFFU) {
            break;             /* дошли до пустого слота — дальше тоже пусто */
        }
    }
    return found ? last : 0;
}

/**
  * @brief Сохранить новое значение FCnt.
  *        Пишет в следующий свободный слот; если страница заполнена —
  *        стирает её, пишет в начало.
  */
void fcnt_save(uint32_t fcnt)
{
    /* найти первый свободный слот */
    uint32_t free_idx = FCNT_SLOTS;  /* FCNT_SLOTS == "не найдено" */
    for (uint32_t i = 0; i < FCNT_SLOTS; i++) {
        uint32_t addr = FCNT_PAGE_ADDR + i * FCNT_SLOT_SIZE;
        if (flash_rd32(addr) == 0xFFFFFFFFU &&
            flash_rd32(addr + 4) == 0xFFFFFFFFU) {
            free_idx = i;
            break;
        }
    }

    /* страница заполнена — стереть и начать сначала */
    if (free_idx == FCNT_SLOTS) {
        fcnt_erase_page();
        free_idx = 0;
    }

    uint32_t addr = FCNT_PAGE_ADDR + free_idx * FCNT_SLOT_SIZE;
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr,     fcnt);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 4, FCNT_MAGIC);
    HAL_FLASH_Lock();
}