#ifndef TEST_EFLASH_H
#define TEST_EFLASH_H

#include <stdint.h>

#define EFLASH_PAGE_SIZE       256U
#define EFLASH_ERASABLE_SIZE   4096U

int erase_flash_sector(uint32_t addr);
int write_flash(uint32_t addr, const uint8_t *data, uint32_t size);

#endif
