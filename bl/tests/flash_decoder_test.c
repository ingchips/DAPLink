#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "error.h"
#include "flash_decoder.h"
#include "sys_init.h"
#include "validation.h"

static uint32_t erase_addresses[8];
static uint32_t erase_count;
static uint32_t write_count;

int erase_flash_sector(uint32_t addr)
{
    erase_addresses[erase_count++] = addr;
    return 0;
}

int write_flash(uint32_t addr, const uint8_t *data, uint32_t size)
{
    (void)addr;
    (void)data;
    (void)size;
    write_count++;
    return 0;
}

static int check(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "%s\n", message);
        return 0;
    }
    return 1;
}

int main(void)
{
    uint8_t data[1024];
    uint32_t vectors[4] = {
        APP_RAM_END,
        (APP_ADDR + 0x100001U) | 1U,
        0U,
        0U,
    };

    memset(data, 0xa5, sizeof(data));
    if (!check(validate_bin_nvic((const uint8_t *)vectors) == 1,
               "large application vector was not detected")) return 1;

    if (!check(flash_decoder_open() == ERROR_SUCCESS, "open failed")) return 1;
    if (!check(flash_decoder_write(APP_ADDR, data, 48U) == ERROR_SUCCESS,
               "first write failed")) return 1;
    if (!check(flash_decoder_write(APP_ADDR + 48U, data, 464U) == ERROR_SUCCESS,
               "second write failed")) return 1;
    if (!check(flash_decoder_write(APP_ADDR + 512U, data, 512U) == ERROR_SUCCESS,
               "third write failed")) return 1;
    if (!check((erase_count == 1U) && (erase_addresses[0] == APP_ADDR),
               "same sector was not erased exactly once")) return 1;

    if (!check(flash_decoder_write(APP_ADDR + 0x0f00U, data, 512U) == ERROR_SUCCESS,
               "cross-sector write failed")) return 1;
    if (!check((erase_count == 2U) && (erase_addresses[1] == APP_ADDR + 0x1000U),
               "next sector was not erased")) return 1;
    if (!check(flash_decoder_close() == ERROR_SUCCESS, "close failed")) return 1;

    if (!check(flash_decoder_open() == ERROR_SUCCESS, "second open failed")) return 1;
    if (!check(flash_decoder_write(APP_ADDR, data, 4U) == ERROR_SUCCESS,
               "write after reopen failed")) return 1;
    if (!check((erase_count == 3U) && (erase_addresses[2] == APP_ADDR),
               "erase state was not reset for a new stream")) return 1;
    if (!check(flash_decoder_close() == ERROR_SUCCESS, "second close failed")) return 1;

    printf("decoder accepted repeated writes: erases=%u writes=%u\n", erase_count, write_count);
    return 0;
}
