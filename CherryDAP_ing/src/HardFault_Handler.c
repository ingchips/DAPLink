#include <stdio.h>
#include "ingsoc.h"

void HardFault_Handler(void)
{
    printf("HARDFAULT!\n");
    for (;;);
}

void NMI_Handler(void)
{
    for (;;);
}

void MemManage_Handler(void)
{
    for (;;);
}

void BusFault_Handler(void)
{
    for (;;);
}

void UsageFault_Handler(void)
{
    for (;;);
}

void DebugMon_Handler(void)
{
}
