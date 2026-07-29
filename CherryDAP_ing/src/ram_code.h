#ifndef __RAM_CODE_H__
#define __RAM_CODE_H__

#if defined(__ARMCC_VERSION) || defined(__GNUC__)
#define RAM_CODE __attribute__((section(".ram_code"), noinline))
#else
#define RAM_CODE
#endif

#endif
