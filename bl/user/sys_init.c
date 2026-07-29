#include "soc.h"
#include "sys_init.h"

void fpu_init(void)
{

    SCB->CPACR |= (0xFU << 20); 
    __DSB(); 
    __ISB(); 
}

void SysInit(void)
{
    
    __disable_irq();
    if(aon2_ctrl_reg->pwr_ctrl_status0.f.boot_power_up == 0x1){
      aon1_ctrl_reg->aon1_reg3.f.reg_boot_pin_clr = 0x1;
      aon1_ctrl_reg->aon1_boot.r = ((0x1 << 0 ) | //BootConfig Enable
                                    (0x1 << 1 ) | //Pll Enable
                                    (0x1 << 2 ) | //Pll wait time or lock, 0:time, 1:lock
                                    (0x0 << 3 ) | //pll time, 110us/130us/150us/170us
                                    (80  << 5 ) | //pll div loop reg
                                    (0x1 << 13) | //hclk sel
                                    (4   << 14) | //hclk div denom
                                    (0x1 << 18) | //flash clk sel
                                    (2   << 19) | //flash div denom
                                    (0x1 << 23) | //flash 4line
                                    (0x2 << 24) | //flash sample delay
                                    (0x1 << 27) | //cache enable
                                    (0x0 << 28) | //wdt enable
                                    (7UL << 29)   //other, must be 0x7
                                   );
      NVIC_SystemReset();
    }
    fpu_init();
    SCB->VTOR = 0x2002000;
    __enable_irq();
}

void system_reset(void)
{
    NVIC_SystemReset();
}

