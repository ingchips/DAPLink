#ifndef __SWD_SPI_H__
#define __SWD_SPI_H__

#include "stdint.h"

void swd_spi_init(void);
void swd_spi_set_clock(uint32_t clock_hz);
uint32_t swd_target_reset_aircr(void);
uint8_t SWD_Transfer(uint32_t request, uint32_t *data);
void SWJ_Sequence(uint32_t count, const uint8_t *data);
void SWD_Sequence(uint32_t info, const uint8_t *swdo, uint8_t *swdi);

#endif
