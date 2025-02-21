// ----------------------------------------------------------------------------
// Copyright Message                                                           
// ----------------------------------------------------------------------------
//                                                                             
// INGCHIPS confidential and proprietary.                                      
// COPYRIGHT (c) 2022 by INGCHIPS
//                                                                             
// All rights are reserved. Reproduction in whole or in part is                
// prohibited without the written consent of the copyright owner.              
//                                                                             
// ----------------------------------------------------------------------------
// Design Information                                                          
// ----------------------------------------------------------------------------
//                                                                             
// File         : uart_reg.h
//                                                                             
// Author       : INGCHIPS                                                     
//                                                                             
// Organisation : INGCHIPS                                                     
//                                                                             
// Date         : 2022/4/26 
//                                                                             
// Revision     : v0.1                                                         
//                                                                             
// Project      :                                                              
//                                                                             
// Description  :                                                              
//                                                                             
// ----------------------------------------------------------------------------
// Revision History                                                            
// ----------------------------------------------------------------------------
//                                                                             
// Date       Author          Revision  Change Description                     
// ========== =============== ========= =======================================
// 2022/4/26  INGCHIPS        v0.1      Create
//                                                                             
// ----------------------------------------------------------------------------
#ifndef UART_REG__H
#define UART_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DATA                             : 8 ;
        volatile uint32_t FE                               : 1 ;
        volatile uint32_t PE                               : 1 ;
        volatile uint32_t BE                               : 1 ;
        volatile uint32_t OE                               : 1 ;
        volatile uint32_t unused0                          : 4 ;
        volatile uint32_t unused1                          : 16;
    }f;
}UART_REG_UARTDR;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t FE                               : 1 ;
        volatile uint32_t PE                               : 1 ;
        volatile uint32_t BE                               : 1 ;
        volatile uint32_t OE                               : 1 ;
        volatile uint32_t Reserved                         : 4 ;
        volatile uint32_t unused2                          : 24;
    }f;
}UART_REG_UARTRSR_UARTECR;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t CTS                              : 1 ;
        volatile uint32_t DSR                              : 1 ;
        volatile uint32_t DCD                              : 1 ;
        volatile uint32_t BUSY                             : 1 ;
        volatile uint32_t RXFE                             : 1 ;
        volatile uint32_t TXFF                             : 1 ;
        volatile uint32_t RXFF                             : 1 ;
        volatile uint32_t TXFE                             : 1 ;
        volatile uint32_t RI                               : 1 ;
        volatile uint32_t unused3                          : 23;
    }f;
}UART_REG_UARTFR;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ILPDVSR                          : 8 ;
        volatile uint32_t unused4                          : 24;
    }f;
}UART_REG_UARTILPR;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t BAUD_DIVINT                      : 16;
        volatile uint32_t unused5                          : 16;
    }f;
}UART_REG_UARTIBRD;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t BAUD_DIVFRAC                     : 6 ;
        volatile uint32_t unused6                          : 26;
    }f;
}UART_REG_UARTFBRD;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t BRK                              : 1 ;
        volatile uint32_t PEN                              : 1 ;
        volatile uint32_t EPS                              : 1 ;
        volatile uint32_t STP2                             : 1 ;
        volatile uint32_t FEN                              : 1 ;
        volatile uint32_t WLEN                             : 2 ;
        volatile uint32_t SPS                              : 1 ;
        volatile uint32_t unused7                          : 24;
    }f;
}UART_REG_UARTLCR_H;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t UARTEN                           : 1 ;
        volatile uint32_t SIREN                            : 1 ;
        volatile uint32_t SIRLP                            : 1 ;
        volatile uint32_t rsvd0                            : 4 ;
        volatile uint32_t LBE                              : 1 ;
        volatile uint32_t TXE                              : 1 ;
        volatile uint32_t RXE                              : 1 ;
        volatile uint32_t DTR                              : 1 ;
        volatile uint32_t RTS                              : 1 ;
        volatile uint32_t Out1                             : 1 ;
        volatile uint32_t Out2                             : 1 ;
        volatile uint32_t RTSEn                            : 1 ;
        volatile uint32_t CTSEn                            : 1 ;
        volatile uint32_t unused8                          : 16;
    }f;
}UART_REG_UARTCR;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t TXIFLSEL                         : 8 ;
        volatile uint32_t RXIFLSEL                         : 8 ;
        volatile uint32_t unused9                          : 16;
    }f;
}UART_REG_UARTIFLS;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RIMIM                            : 1 ;
        volatile uint32_t CTSMIM                           : 1 ;
        volatile uint32_t DCDMIM                           : 1 ;
        volatile uint32_t DSRMIM                           : 1 ;
        volatile uint32_t RXIM                             : 1 ;
        volatile uint32_t TXIM                             : 1 ;
        volatile uint32_t RTIM                             : 1 ;
        volatile uint32_t FEIM                             : 1 ;
        volatile uint32_t PEIM                             : 1 ;
        volatile uint32_t BEIM                             : 1 ;
        volatile uint32_t OEIM                             : 1 ;
        volatile uint32_t unused10                         : 21;
    }f;
}UART_REG_UARTIMSC;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RIRMIS                           : 1 ;
        volatile uint32_t CTSRMIS                          : 1 ;
        volatile uint32_t DCDRMIS                          : 1 ;
        volatile uint32_t DSRRMIS                          : 1 ;
        volatile uint32_t RXRIS                            : 1 ;
        volatile uint32_t TXRIS                            : 1 ;
        volatile uint32_t RTRIS                            : 1 ;
        volatile uint32_t FERIS                            : 1 ;
        volatile uint32_t PERIS                            : 1 ;
        volatile uint32_t BERIS                            : 1 ;
        volatile uint32_t OERIS                            : 1 ;
        volatile uint32_t unused11                         : 21;
    }f;
}UART_REG_UARTRIS;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RIMMIS                           : 1 ;
        volatile uint32_t CTSMMIS                          : 1 ;
        volatile uint32_t DCDMMIS                          : 1 ;
        volatile uint32_t DSRMMIS                          : 1 ;
        volatile uint32_t RXMIS                            : 1 ;
        volatile uint32_t TXMIS                            : 1 ;
        volatile uint32_t RTMIS                            : 1 ;
        volatile uint32_t FEMIS                            : 1 ;
        volatile uint32_t PEMIS                            : 1 ;
        volatile uint32_t BEMIS                            : 1 ;
        volatile uint32_t OEMIS                            : 1 ;
        volatile uint32_t unused12                         : 21;
    }f;
}UART_REG_UARTMIS;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RIMIC                            : 1 ;
        volatile uint32_t CTSMIC                           : 1 ;
        volatile uint32_t DCDMIC                           : 1 ;
        volatile uint32_t DSRMIC                           : 1 ;
        volatile uint32_t RXIC                             : 1 ;
        volatile uint32_t TXIC                             : 1 ;
        volatile uint32_t RTIC                             : 1 ;
        volatile uint32_t FEIC                             : 1 ;
        volatile uint32_t PEIC                             : 1 ;
        volatile uint32_t BEIC                             : 1 ;
        volatile uint32_t OEIC                             : 1 ;
        volatile uint32_t unused13                         : 21;
    }f;
}UART_REG_UARTICR;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RXDMAE                           : 1 ;
        volatile uint32_t TXDMAE                           : 1 ;
        volatile uint32_t DMAONERR                         : 1 ;
        volatile uint32_t unused14                         : 29;
    }f;
}UART_REG_UARTDMACR;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t PartNumber0                      : 8 ;
        volatile uint32_t unused15                         : 24;
    }f;
}UART_REG_UARTPeriphID0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t PartNumber0                      : 4 ;
        volatile uint32_t Designer0                        : 4 ;
        volatile uint32_t unused16                         : 24;
    }f;
}UART_REG_UARTPeriphID1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Designer1                        : 4 ;
        volatile uint32_t Revision                         : 4 ;
        volatile uint32_t unused17                         : 24;
    }f;
}UART_REG_UARTPeriphID2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t Configuration                    : 8 ;
        volatile uint32_t unused18                         : 24;
    }f;
}UART_REG_UARTPeriphID3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t UARTPCellID0                     : 8 ;
        volatile uint32_t unused19                         : 24;
    }f;
}UART_REG_UARTPCellID0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t UARTPCellID1                     : 8 ;
        volatile uint32_t unused20                         : 24;
    }f;
}UART_REG_UARTPCellID1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t UARTPCellID2                     : 8 ;
        volatile uint32_t unused21                         : 24;
    }f;
}UART_REG_UARTPCellD2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t UARTPCellID3                     : 8 ;
        volatile uint32_t unused22                         : 24;
    }f;
}UART_REG_UARTPCellID3;

typedef struct
{
volatile UART_REG_UARTDR                                             UARTDR                        ; //0x0
volatile UART_REG_UARTRSR_UARTECR                                    UARTRSR_UARTECR               ; //0x4
volatile uint32_t                                                    null_reg0                     ; //0x8
volatile uint32_t                                                    null_reg1                     ; //0xC
volatile uint32_t                                                    null_reg2                     ; //0x10
volatile uint32_t                                                    null_reg3                     ; //0x14
volatile UART_REG_UARTFR                                             UARTFR                        ; //0x18
volatile uint32_t                                                    null_reg4                     ; //0x1C
volatile UART_REG_UARTILPR                                           UARTILPR                      ; //0x20
volatile UART_REG_UARTIBRD                                           UARTIBRD                      ; //0x24
volatile UART_REG_UARTFBRD                                           UARTFBRD                      ; //0x28
volatile UART_REG_UARTLCR_H                                          UARTLCR_H                     ; //0x2C
volatile UART_REG_UARTCR                                             UARTCR                        ; //0x30
volatile UART_REG_UARTIFLS                                           UARTIFLS                      ; //0x34
volatile UART_REG_UARTIMSC                                           UARTIMSC                      ; //0x38
volatile UART_REG_UARTRIS                                            UARTRIS                       ; //0x3C
volatile UART_REG_UARTMIS                                            UARTMIS                       ; //0x40
volatile UART_REG_UARTICR                                            UARTICR                       ; //0x44
volatile UART_REG_UARTDMACR                                          UARTDMACR                     ; //0x48
volatile uint32_t                                                    null_reg5                     ; //0x4C
volatile uint32_t                                                    null_reg6                     ; //0x50
volatile uint32_t                                                    null_reg7                     ; //0x54
volatile uint32_t                                                    null_reg8                     ; //0x58
volatile uint32_t                                                    null_reg9                     ; //0x5C
volatile uint32_t                                                    null_reg10                    ; //0x60
volatile uint32_t                                                    null_reg11                    ; //0x64
volatile uint32_t                                                    null_reg12                    ; //0x68
volatile uint32_t                                                    null_reg13                    ; //0x6C
volatile uint32_t                                                    null_reg14                    ; //0x70
volatile uint32_t                                                    null_reg15                    ; //0x74
volatile uint32_t                                                    null_reg16                    ; //0x78
volatile uint32_t                                                    null_reg17                    ; //0x7C
volatile uint32_t                                                    null_reg18                    ; //0x80
volatile uint32_t                                                    null_reg19                    ; //0x84
volatile uint32_t                                                    null_reg20                    ; //0x88
volatile uint32_t                                                    null_reg21                    ; //0x8C
volatile uint32_t                                                    null_reg22                    ; //0x90
volatile uint32_t                                                    null_reg23                    ; //0x94
volatile uint32_t                                                    null_reg24                    ; //0x98
volatile uint32_t                                                    null_reg25                    ; //0x9C
volatile uint32_t                                                    null_reg26                    ; //0xA0
volatile uint32_t                                                    null_reg27                    ; //0xA4
volatile uint32_t                                                    null_reg28                    ; //0xA8
volatile uint32_t                                                    null_reg29                    ; //0xAC
volatile uint32_t                                                    null_reg30                    ; //0xB0
volatile uint32_t                                                    null_reg31                    ; //0xB4
volatile uint32_t                                                    null_reg32                    ; //0xB8
volatile uint32_t                                                    null_reg33                    ; //0xBC
volatile uint32_t                                                    null_reg34                    ; //0xC0
volatile uint32_t                                                    null_reg35                    ; //0xC4
volatile uint32_t                                                    null_reg36                    ; //0xC8
volatile uint32_t                                                    null_reg37                    ; //0xCC
volatile uint32_t                                                    null_reg38                    ; //0xD0
volatile uint32_t                                                    null_reg39                    ; //0xD4
volatile uint32_t                                                    null_reg40                    ; //0xD8
volatile uint32_t                                                    null_reg41                    ; //0xDC
volatile uint32_t                                                    null_reg42                    ; //0xE0
volatile uint32_t                                                    null_reg43                    ; //0xE4
volatile uint32_t                                                    null_reg44                    ; //0xE8
volatile uint32_t                                                    null_reg45                    ; //0xEC
volatile uint32_t                                                    null_reg46                    ; //0xF0
volatile uint32_t                                                    null_reg47                    ; //0xF4
volatile uint32_t                                                    null_reg48                    ; //0xF8
volatile uint32_t                                                    null_reg49                    ; //0xFC
volatile uint32_t                                                    null_reg50                    ; //0x100
volatile uint32_t                                                    null_reg51                    ; //0x104
volatile uint32_t                                                    null_reg52                    ; //0x108
volatile uint32_t                                                    null_reg53                    ; //0x10C
volatile uint32_t                                                    null_reg54                    ; //0x110
volatile uint32_t                                                    null_reg55                    ; //0x114
volatile uint32_t                                                    null_reg56                    ; //0x118
volatile uint32_t                                                    null_reg57                    ; //0x11C
volatile uint32_t                                                    null_reg58                    ; //0x120
volatile uint32_t                                                    null_reg59                    ; //0x124
volatile uint32_t                                                    null_reg60                    ; //0x128
volatile uint32_t                                                    null_reg61                    ; //0x12C
volatile uint32_t                                                    null_reg62                    ; //0x130
volatile uint32_t                                                    null_reg63                    ; //0x134
volatile uint32_t                                                    null_reg64                    ; //0x138
volatile uint32_t                                                    null_reg65                    ; //0x13C
volatile uint32_t                                                    null_reg66                    ; //0x140
volatile uint32_t                                                    null_reg67                    ; //0x144
volatile uint32_t                                                    null_reg68                    ; //0x148
volatile uint32_t                                                    null_reg69                    ; //0x14C
volatile uint32_t                                                    null_reg70                    ; //0x150
volatile uint32_t                                                    null_reg71                    ; //0x154
volatile uint32_t                                                    null_reg72                    ; //0x158
volatile uint32_t                                                    null_reg73                    ; //0x15C
volatile uint32_t                                                    null_reg74                    ; //0x160
volatile uint32_t                                                    null_reg75                    ; //0x164
volatile uint32_t                                                    null_reg76                    ; //0x168
volatile uint32_t                                                    null_reg77                    ; //0x16C
volatile uint32_t                                                    null_reg78                    ; //0x170
volatile uint32_t                                                    null_reg79                    ; //0x174
volatile uint32_t                                                    null_reg80                    ; //0x178
volatile uint32_t                                                    null_reg81                    ; //0x17C
volatile uint32_t                                                    null_reg82                    ; //0x180
volatile uint32_t                                                    null_reg83                    ; //0x184
volatile uint32_t                                                    null_reg84                    ; //0x188
volatile uint32_t                                                    null_reg85                    ; //0x18C
volatile uint32_t                                                    null_reg86                    ; //0x190
volatile uint32_t                                                    null_reg87                    ; //0x194
volatile uint32_t                                                    null_reg88                    ; //0x198
volatile uint32_t                                                    null_reg89                    ; //0x19C
volatile uint32_t                                                    null_reg90                    ; //0x1A0
volatile uint32_t                                                    null_reg91                    ; //0x1A4
volatile uint32_t                                                    null_reg92                    ; //0x1A8
volatile uint32_t                                                    null_reg93                    ; //0x1AC
volatile uint32_t                                                    null_reg94                    ; //0x1B0
volatile uint32_t                                                    null_reg95                    ; //0x1B4
volatile uint32_t                                                    null_reg96                    ; //0x1B8
volatile uint32_t                                                    null_reg97                    ; //0x1BC
volatile uint32_t                                                    null_reg98                    ; //0x1C0
volatile uint32_t                                                    null_reg99                    ; //0x1C4
volatile uint32_t                                                    null_reg100                   ; //0x1C8
volatile uint32_t                                                    null_reg101                   ; //0x1CC
volatile uint32_t                                                    null_reg102                   ; //0x1D0
volatile uint32_t                                                    null_reg103                   ; //0x1D4
volatile uint32_t                                                    null_reg104                   ; //0x1D8
volatile uint32_t                                                    null_reg105                   ; //0x1DC
volatile uint32_t                                                    null_reg106                   ; //0x1E0
volatile uint32_t                                                    null_reg107                   ; //0x1E4
volatile uint32_t                                                    null_reg108                   ; //0x1E8
volatile uint32_t                                                    null_reg109                   ; //0x1EC
volatile uint32_t                                                    null_reg110                   ; //0x1F0
volatile uint32_t                                                    null_reg111                   ; //0x1F4
volatile uint32_t                                                    null_reg112                   ; //0x1F8
volatile uint32_t                                                    null_reg113                   ; //0x1FC
volatile uint32_t                                                    null_reg114                   ; //0x200
volatile uint32_t                                                    null_reg115                   ; //0x204
volatile uint32_t                                                    null_reg116                   ; //0x208
volatile uint32_t                                                    null_reg117                   ; //0x20C
volatile uint32_t                                                    null_reg118                   ; //0x210
volatile uint32_t                                                    null_reg119                   ; //0x214
volatile uint32_t                                                    null_reg120                   ; //0x218
volatile uint32_t                                                    null_reg121                   ; //0x21C
volatile uint32_t                                                    null_reg122                   ; //0x220
volatile uint32_t                                                    null_reg123                   ; //0x224
volatile uint32_t                                                    null_reg124                   ; //0x228
volatile uint32_t                                                    null_reg125                   ; //0x22C
volatile uint32_t                                                    null_reg126                   ; //0x230
volatile uint32_t                                                    null_reg127                   ; //0x234
volatile uint32_t                                                    null_reg128                   ; //0x238
volatile uint32_t                                                    null_reg129                   ; //0x23C
volatile uint32_t                                                    null_reg130                   ; //0x240
volatile uint32_t                                                    null_reg131                   ; //0x244
volatile uint32_t                                                    null_reg132                   ; //0x248
volatile uint32_t                                                    null_reg133                   ; //0x24C
volatile uint32_t                                                    null_reg134                   ; //0x250
volatile uint32_t                                                    null_reg135                   ; //0x254
volatile uint32_t                                                    null_reg136                   ; //0x258
volatile uint32_t                                                    null_reg137                   ; //0x25C
volatile uint32_t                                                    null_reg138                   ; //0x260
volatile uint32_t                                                    null_reg139                   ; //0x264
volatile uint32_t                                                    null_reg140                   ; //0x268
volatile uint32_t                                                    null_reg141                   ; //0x26C
volatile uint32_t                                                    null_reg142                   ; //0x270
volatile uint32_t                                                    null_reg143                   ; //0x274
volatile uint32_t                                                    null_reg144                   ; //0x278
volatile uint32_t                                                    null_reg145                   ; //0x27C
volatile uint32_t                                                    null_reg146                   ; //0x280
volatile uint32_t                                                    null_reg147                   ; //0x284
volatile uint32_t                                                    null_reg148                   ; //0x288
volatile uint32_t                                                    null_reg149                   ; //0x28C
volatile uint32_t                                                    null_reg150                   ; //0x290
volatile uint32_t                                                    null_reg151                   ; //0x294
volatile uint32_t                                                    null_reg152                   ; //0x298
volatile uint32_t                                                    null_reg153                   ; //0x29C
volatile uint32_t                                                    null_reg154                   ; //0x2A0
volatile uint32_t                                                    null_reg155                   ; //0x2A4
volatile uint32_t                                                    null_reg156                   ; //0x2A8
volatile uint32_t                                                    null_reg157                   ; //0x2AC
volatile uint32_t                                                    null_reg158                   ; //0x2B0
volatile uint32_t                                                    null_reg159                   ; //0x2B4
volatile uint32_t                                                    null_reg160                   ; //0x2B8
volatile uint32_t                                                    null_reg161                   ; //0x2BC
volatile uint32_t                                                    null_reg162                   ; //0x2C0
volatile uint32_t                                                    null_reg163                   ; //0x2C4
volatile uint32_t                                                    null_reg164                   ; //0x2C8
volatile uint32_t                                                    null_reg165                   ; //0x2CC
volatile uint32_t                                                    null_reg166                   ; //0x2D0
volatile uint32_t                                                    null_reg167                   ; //0x2D4
volatile uint32_t                                                    null_reg168                   ; //0x2D8
volatile uint32_t                                                    null_reg169                   ; //0x2DC
volatile uint32_t                                                    null_reg170                   ; //0x2E0
volatile uint32_t                                                    null_reg171                   ; //0x2E4
volatile uint32_t                                                    null_reg172                   ; //0x2E8
volatile uint32_t                                                    null_reg173                   ; //0x2EC
volatile uint32_t                                                    null_reg174                   ; //0x2F0
volatile uint32_t                                                    null_reg175                   ; //0x2F4
volatile uint32_t                                                    null_reg176                   ; //0x2F8
volatile uint32_t                                                    null_reg177                   ; //0x2FC
volatile uint32_t                                                    null_reg178                   ; //0x300
volatile uint32_t                                                    null_reg179                   ; //0x304
volatile uint32_t                                                    null_reg180                   ; //0x308
volatile uint32_t                                                    null_reg181                   ; //0x30C
volatile uint32_t                                                    null_reg182                   ; //0x310
volatile uint32_t                                                    null_reg183                   ; //0x314
volatile uint32_t                                                    null_reg184                   ; //0x318
volatile uint32_t                                                    null_reg185                   ; //0x31C
volatile uint32_t                                                    null_reg186                   ; //0x320
volatile uint32_t                                                    null_reg187                   ; //0x324
volatile uint32_t                                                    null_reg188                   ; //0x328
volatile uint32_t                                                    null_reg189                   ; //0x32C
volatile uint32_t                                                    null_reg190                   ; //0x330
volatile uint32_t                                                    null_reg191                   ; //0x334
volatile uint32_t                                                    null_reg192                   ; //0x338
volatile uint32_t                                                    null_reg193                   ; //0x33C
volatile uint32_t                                                    null_reg194                   ; //0x340
volatile uint32_t                                                    null_reg195                   ; //0x344
volatile uint32_t                                                    null_reg196                   ; //0x348
volatile uint32_t                                                    null_reg197                   ; //0x34C
volatile uint32_t                                                    null_reg198                   ; //0x350
volatile uint32_t                                                    null_reg199                   ; //0x354
volatile uint32_t                                                    null_reg200                   ; //0x358
volatile uint32_t                                                    null_reg201                   ; //0x35C
volatile uint32_t                                                    null_reg202                   ; //0x360
volatile uint32_t                                                    null_reg203                   ; //0x364
volatile uint32_t                                                    null_reg204                   ; //0x368
volatile uint32_t                                                    null_reg205                   ; //0x36C
volatile uint32_t                                                    null_reg206                   ; //0x370
volatile uint32_t                                                    null_reg207                   ; //0x374
volatile uint32_t                                                    null_reg208                   ; //0x378
volatile uint32_t                                                    null_reg209                   ; //0x37C
volatile uint32_t                                                    null_reg210                   ; //0x380
volatile uint32_t                                                    null_reg211                   ; //0x384
volatile uint32_t                                                    null_reg212                   ; //0x388
volatile uint32_t                                                    null_reg213                   ; //0x38C
volatile uint32_t                                                    null_reg214                   ; //0x390
volatile uint32_t                                                    null_reg215                   ; //0x394
volatile uint32_t                                                    null_reg216                   ; //0x398
volatile uint32_t                                                    null_reg217                   ; //0x39C
volatile uint32_t                                                    null_reg218                   ; //0x3A0
volatile uint32_t                                                    null_reg219                   ; //0x3A4
volatile uint32_t                                                    null_reg220                   ; //0x3A8
volatile uint32_t                                                    null_reg221                   ; //0x3AC
volatile uint32_t                                                    null_reg222                   ; //0x3B0
volatile uint32_t                                                    null_reg223                   ; //0x3B4
volatile uint32_t                                                    null_reg224                   ; //0x3B8
volatile uint32_t                                                    null_reg225                   ; //0x3BC
volatile uint32_t                                                    null_reg226                   ; //0x3C0
volatile uint32_t                                                    null_reg227                   ; //0x3C4
volatile uint32_t                                                    null_reg228                   ; //0x3C8
volatile uint32_t                                                    null_reg229                   ; //0x3CC
volatile uint32_t                                                    null_reg230                   ; //0x3D0
volatile uint32_t                                                    null_reg231                   ; //0x3D4
volatile uint32_t                                                    null_reg232                   ; //0x3D8
volatile uint32_t                                                    null_reg233                   ; //0x3DC
volatile uint32_t                                                    null_reg234                   ; //0x3E0
volatile uint32_t                                                    null_reg235                   ; //0x3E4
volatile uint32_t                                                    null_reg236                   ; //0x3E8
volatile uint32_t                                                    null_reg237                   ; //0x3EC
volatile uint32_t                                                    null_reg238                   ; //0x3F0
volatile uint32_t                                                    null_reg239                   ; //0x3F4
volatile uint32_t                                                    null_reg240                   ; //0x3F8
volatile uint32_t                                                    null_reg241                   ; //0x3FC
volatile uint32_t                                                    null_reg242                   ; //0x400
volatile uint32_t                                                    null_reg243                   ; //0x404
volatile uint32_t                                                    null_reg244                   ; //0x408
volatile uint32_t                                                    null_reg245                   ; //0x40C
volatile uint32_t                                                    null_reg246                   ; //0x410
volatile uint32_t                                                    null_reg247                   ; //0x414
volatile uint32_t                                                    null_reg248                   ; //0x418
volatile uint32_t                                                    null_reg249                   ; //0x41C
volatile uint32_t                                                    null_reg250                   ; //0x420
volatile uint32_t                                                    null_reg251                   ; //0x424
volatile uint32_t                                                    null_reg252                   ; //0x428
volatile uint32_t                                                    null_reg253                   ; //0x42C
volatile uint32_t                                                    null_reg254                   ; //0x430
volatile uint32_t                                                    null_reg255                   ; //0x434
volatile uint32_t                                                    null_reg256                   ; //0x438
volatile uint32_t                                                    null_reg257                   ; //0x43C
volatile uint32_t                                                    null_reg258                   ; //0x440
volatile uint32_t                                                    null_reg259                   ; //0x444
volatile uint32_t                                                    null_reg260                   ; //0x448
volatile uint32_t                                                    null_reg261                   ; //0x44C
volatile uint32_t                                                    null_reg262                   ; //0x450
volatile uint32_t                                                    null_reg263                   ; //0x454
volatile uint32_t                                                    null_reg264                   ; //0x458
volatile uint32_t                                                    null_reg265                   ; //0x45C
volatile uint32_t                                                    null_reg266                   ; //0x460
volatile uint32_t                                                    null_reg267                   ; //0x464
volatile uint32_t                                                    null_reg268                   ; //0x468
volatile uint32_t                                                    null_reg269                   ; //0x46C
volatile uint32_t                                                    null_reg270                   ; //0x470
volatile uint32_t                                                    null_reg271                   ; //0x474
volatile uint32_t                                                    null_reg272                   ; //0x478
volatile uint32_t                                                    null_reg273                   ; //0x47C
volatile uint32_t                                                    null_reg274                   ; //0x480
volatile uint32_t                                                    null_reg275                   ; //0x484
volatile uint32_t                                                    null_reg276                   ; //0x488
volatile uint32_t                                                    null_reg277                   ; //0x48C
volatile uint32_t                                                    null_reg278                   ; //0x490
volatile uint32_t                                                    null_reg279                   ; //0x494
volatile uint32_t                                                    null_reg280                   ; //0x498
volatile uint32_t                                                    null_reg281                   ; //0x49C
volatile uint32_t                                                    null_reg282                   ; //0x4A0
volatile uint32_t                                                    null_reg283                   ; //0x4A4
volatile uint32_t                                                    null_reg284                   ; //0x4A8
volatile uint32_t                                                    null_reg285                   ; //0x4AC
volatile uint32_t                                                    null_reg286                   ; //0x4B0
volatile uint32_t                                                    null_reg287                   ; //0x4B4
volatile uint32_t                                                    null_reg288                   ; //0x4B8
volatile uint32_t                                                    null_reg289                   ; //0x4BC
volatile uint32_t                                                    null_reg290                   ; //0x4C0
volatile uint32_t                                                    null_reg291                   ; //0x4C4
volatile uint32_t                                                    null_reg292                   ; //0x4C8
volatile uint32_t                                                    null_reg293                   ; //0x4CC
volatile uint32_t                                                    null_reg294                   ; //0x4D0
volatile uint32_t                                                    null_reg295                   ; //0x4D4
volatile uint32_t                                                    null_reg296                   ; //0x4D8
volatile uint32_t                                                    null_reg297                   ; //0x4DC
volatile uint32_t                                                    null_reg298                   ; //0x4E0
volatile uint32_t                                                    null_reg299                   ; //0x4E4
volatile uint32_t                                                    null_reg300                   ; //0x4E8
volatile uint32_t                                                    null_reg301                   ; //0x4EC
volatile uint32_t                                                    null_reg302                   ; //0x4F0
volatile uint32_t                                                    null_reg303                   ; //0x4F4
volatile uint32_t                                                    null_reg304                   ; //0x4F8
volatile uint32_t                                                    null_reg305                   ; //0x4FC
volatile uint32_t                                                    null_reg306                   ; //0x500
volatile uint32_t                                                    null_reg307                   ; //0x504
volatile uint32_t                                                    null_reg308                   ; //0x508
volatile uint32_t                                                    null_reg309                   ; //0x50C
volatile uint32_t                                                    null_reg310                   ; //0x510
volatile uint32_t                                                    null_reg311                   ; //0x514
volatile uint32_t                                                    null_reg312                   ; //0x518
volatile uint32_t                                                    null_reg313                   ; //0x51C
volatile uint32_t                                                    null_reg314                   ; //0x520
volatile uint32_t                                                    null_reg315                   ; //0x524
volatile uint32_t                                                    null_reg316                   ; //0x528
volatile uint32_t                                                    null_reg317                   ; //0x52C
volatile uint32_t                                                    null_reg318                   ; //0x530
volatile uint32_t                                                    null_reg319                   ; //0x534
volatile uint32_t                                                    null_reg320                   ; //0x538
volatile uint32_t                                                    null_reg321                   ; //0x53C
volatile uint32_t                                                    null_reg322                   ; //0x540
volatile uint32_t                                                    null_reg323                   ; //0x544
volatile uint32_t                                                    null_reg324                   ; //0x548
volatile uint32_t                                                    null_reg325                   ; //0x54C
volatile uint32_t                                                    null_reg326                   ; //0x550
volatile uint32_t                                                    null_reg327                   ; //0x554
volatile uint32_t                                                    null_reg328                   ; //0x558
volatile uint32_t                                                    null_reg329                   ; //0x55C
volatile uint32_t                                                    null_reg330                   ; //0x560
volatile uint32_t                                                    null_reg331                   ; //0x564
volatile uint32_t                                                    null_reg332                   ; //0x568
volatile uint32_t                                                    null_reg333                   ; //0x56C
volatile uint32_t                                                    null_reg334                   ; //0x570
volatile uint32_t                                                    null_reg335                   ; //0x574
volatile uint32_t                                                    null_reg336                   ; //0x578
volatile uint32_t                                                    null_reg337                   ; //0x57C
volatile uint32_t                                                    null_reg338                   ; //0x580
volatile uint32_t                                                    null_reg339                   ; //0x584
volatile uint32_t                                                    null_reg340                   ; //0x588
volatile uint32_t                                                    null_reg341                   ; //0x58C
volatile uint32_t                                                    null_reg342                   ; //0x590
volatile uint32_t                                                    null_reg343                   ; //0x594
volatile uint32_t                                                    null_reg344                   ; //0x598
volatile uint32_t                                                    null_reg345                   ; //0x59C
volatile uint32_t                                                    null_reg346                   ; //0x5A0
volatile uint32_t                                                    null_reg347                   ; //0x5A4
volatile uint32_t                                                    null_reg348                   ; //0x5A8
volatile uint32_t                                                    null_reg349                   ; //0x5AC
volatile uint32_t                                                    null_reg350                   ; //0x5B0
volatile uint32_t                                                    null_reg351                   ; //0x5B4
volatile uint32_t                                                    null_reg352                   ; //0x5B8
volatile uint32_t                                                    null_reg353                   ; //0x5BC
volatile uint32_t                                                    null_reg354                   ; //0x5C0
volatile uint32_t                                                    null_reg355                   ; //0x5C4
volatile uint32_t                                                    null_reg356                   ; //0x5C8
volatile uint32_t                                                    null_reg357                   ; //0x5CC
volatile uint32_t                                                    null_reg358                   ; //0x5D0
volatile uint32_t                                                    null_reg359                   ; //0x5D4
volatile uint32_t                                                    null_reg360                   ; //0x5D8
volatile uint32_t                                                    null_reg361                   ; //0x5DC
volatile uint32_t                                                    null_reg362                   ; //0x5E0
volatile uint32_t                                                    null_reg363                   ; //0x5E4
volatile uint32_t                                                    null_reg364                   ; //0x5E8
volatile uint32_t                                                    null_reg365                   ; //0x5EC
volatile uint32_t                                                    null_reg366                   ; //0x5F0
volatile uint32_t                                                    null_reg367                   ; //0x5F4
volatile uint32_t                                                    null_reg368                   ; //0x5F8
volatile uint32_t                                                    null_reg369                   ; //0x5FC
volatile uint32_t                                                    null_reg370                   ; //0x600
volatile uint32_t                                                    null_reg371                   ; //0x604
volatile uint32_t                                                    null_reg372                   ; //0x608
volatile uint32_t                                                    null_reg373                   ; //0x60C
volatile uint32_t                                                    null_reg374                   ; //0x610
volatile uint32_t                                                    null_reg375                   ; //0x614
volatile uint32_t                                                    null_reg376                   ; //0x618
volatile uint32_t                                                    null_reg377                   ; //0x61C
volatile uint32_t                                                    null_reg378                   ; //0x620
volatile uint32_t                                                    null_reg379                   ; //0x624
volatile uint32_t                                                    null_reg380                   ; //0x628
volatile uint32_t                                                    null_reg381                   ; //0x62C
volatile uint32_t                                                    null_reg382                   ; //0x630
volatile uint32_t                                                    null_reg383                   ; //0x634
volatile uint32_t                                                    null_reg384                   ; //0x638
volatile uint32_t                                                    null_reg385                   ; //0x63C
volatile uint32_t                                                    null_reg386                   ; //0x640
volatile uint32_t                                                    null_reg387                   ; //0x644
volatile uint32_t                                                    null_reg388                   ; //0x648
volatile uint32_t                                                    null_reg389                   ; //0x64C
volatile uint32_t                                                    null_reg390                   ; //0x650
volatile uint32_t                                                    null_reg391                   ; //0x654
volatile uint32_t                                                    null_reg392                   ; //0x658
volatile uint32_t                                                    null_reg393                   ; //0x65C
volatile uint32_t                                                    null_reg394                   ; //0x660
volatile uint32_t                                                    null_reg395                   ; //0x664
volatile uint32_t                                                    null_reg396                   ; //0x668
volatile uint32_t                                                    null_reg397                   ; //0x66C
volatile uint32_t                                                    null_reg398                   ; //0x670
volatile uint32_t                                                    null_reg399                   ; //0x674
volatile uint32_t                                                    null_reg400                   ; //0x678
volatile uint32_t                                                    null_reg401                   ; //0x67C
volatile uint32_t                                                    null_reg402                   ; //0x680
volatile uint32_t                                                    null_reg403                   ; //0x684
volatile uint32_t                                                    null_reg404                   ; //0x688
volatile uint32_t                                                    null_reg405                   ; //0x68C
volatile uint32_t                                                    null_reg406                   ; //0x690
volatile uint32_t                                                    null_reg407                   ; //0x694
volatile uint32_t                                                    null_reg408                   ; //0x698
volatile uint32_t                                                    null_reg409                   ; //0x69C
volatile uint32_t                                                    null_reg410                   ; //0x6A0
volatile uint32_t                                                    null_reg411                   ; //0x6A4
volatile uint32_t                                                    null_reg412                   ; //0x6A8
volatile uint32_t                                                    null_reg413                   ; //0x6AC
volatile uint32_t                                                    null_reg414                   ; //0x6B0
volatile uint32_t                                                    null_reg415                   ; //0x6B4
volatile uint32_t                                                    null_reg416                   ; //0x6B8
volatile uint32_t                                                    null_reg417                   ; //0x6BC
volatile uint32_t                                                    null_reg418                   ; //0x6C0
volatile uint32_t                                                    null_reg419                   ; //0x6C4
volatile uint32_t                                                    null_reg420                   ; //0x6C8
volatile uint32_t                                                    null_reg421                   ; //0x6CC
volatile uint32_t                                                    null_reg422                   ; //0x6D0
volatile uint32_t                                                    null_reg423                   ; //0x6D4
volatile uint32_t                                                    null_reg424                   ; //0x6D8
volatile uint32_t                                                    null_reg425                   ; //0x6DC
volatile uint32_t                                                    null_reg426                   ; //0x6E0
volatile uint32_t                                                    null_reg427                   ; //0x6E4
volatile uint32_t                                                    null_reg428                   ; //0x6E8
volatile uint32_t                                                    null_reg429                   ; //0x6EC
volatile uint32_t                                                    null_reg430                   ; //0x6F0
volatile uint32_t                                                    null_reg431                   ; //0x6F4
volatile uint32_t                                                    null_reg432                   ; //0x6F8
volatile uint32_t                                                    null_reg433                   ; //0x6FC
volatile uint32_t                                                    null_reg434                   ; //0x700
volatile uint32_t                                                    null_reg435                   ; //0x704
volatile uint32_t                                                    null_reg436                   ; //0x708
volatile uint32_t                                                    null_reg437                   ; //0x70C
volatile uint32_t                                                    null_reg438                   ; //0x710
volatile uint32_t                                                    null_reg439                   ; //0x714
volatile uint32_t                                                    null_reg440                   ; //0x718
volatile uint32_t                                                    null_reg441                   ; //0x71C
volatile uint32_t                                                    null_reg442                   ; //0x720
volatile uint32_t                                                    null_reg443                   ; //0x724
volatile uint32_t                                                    null_reg444                   ; //0x728
volatile uint32_t                                                    null_reg445                   ; //0x72C
volatile uint32_t                                                    null_reg446                   ; //0x730
volatile uint32_t                                                    null_reg447                   ; //0x734
volatile uint32_t                                                    null_reg448                   ; //0x738
volatile uint32_t                                                    null_reg449                   ; //0x73C
volatile uint32_t                                                    null_reg450                   ; //0x740
volatile uint32_t                                                    null_reg451                   ; //0x744
volatile uint32_t                                                    null_reg452                   ; //0x748
volatile uint32_t                                                    null_reg453                   ; //0x74C
volatile uint32_t                                                    null_reg454                   ; //0x750
volatile uint32_t                                                    null_reg455                   ; //0x754
volatile uint32_t                                                    null_reg456                   ; //0x758
volatile uint32_t                                                    null_reg457                   ; //0x75C
volatile uint32_t                                                    null_reg458                   ; //0x760
volatile uint32_t                                                    null_reg459                   ; //0x764
volatile uint32_t                                                    null_reg460                   ; //0x768
volatile uint32_t                                                    null_reg461                   ; //0x76C
volatile uint32_t                                                    null_reg462                   ; //0x770
volatile uint32_t                                                    null_reg463                   ; //0x774
volatile uint32_t                                                    null_reg464                   ; //0x778
volatile uint32_t                                                    null_reg465                   ; //0x77C
volatile uint32_t                                                    null_reg466                   ; //0x780
volatile uint32_t                                                    null_reg467                   ; //0x784
volatile uint32_t                                                    null_reg468                   ; //0x788
volatile uint32_t                                                    null_reg469                   ; //0x78C
volatile uint32_t                                                    null_reg470                   ; //0x790
volatile uint32_t                                                    null_reg471                   ; //0x794
volatile uint32_t                                                    null_reg472                   ; //0x798
volatile uint32_t                                                    null_reg473                   ; //0x79C
volatile uint32_t                                                    null_reg474                   ; //0x7A0
volatile uint32_t                                                    null_reg475                   ; //0x7A4
volatile uint32_t                                                    null_reg476                   ; //0x7A8
volatile uint32_t                                                    null_reg477                   ; //0x7AC
volatile uint32_t                                                    null_reg478                   ; //0x7B0
volatile uint32_t                                                    null_reg479                   ; //0x7B4
volatile uint32_t                                                    null_reg480                   ; //0x7B8
volatile uint32_t                                                    null_reg481                   ; //0x7BC
volatile uint32_t                                                    null_reg482                   ; //0x7C0
volatile uint32_t                                                    null_reg483                   ; //0x7C4
volatile uint32_t                                                    null_reg484                   ; //0x7C8
volatile uint32_t                                                    null_reg485                   ; //0x7CC
volatile uint32_t                                                    null_reg486                   ; //0x7D0
volatile uint32_t                                                    null_reg487                   ; //0x7D4
volatile uint32_t                                                    null_reg488                   ; //0x7D8
volatile uint32_t                                                    null_reg489                   ; //0x7DC
volatile uint32_t                                                    null_reg490                   ; //0x7E0
volatile uint32_t                                                    null_reg491                   ; //0x7E4
volatile uint32_t                                                    null_reg492                   ; //0x7E8
volatile uint32_t                                                    null_reg493                   ; //0x7EC
volatile uint32_t                                                    null_reg494                   ; //0x7F0
volatile uint32_t                                                    null_reg495                   ; //0x7F4
volatile uint32_t                                                    null_reg496                   ; //0x7F8
volatile uint32_t                                                    null_reg497                   ; //0x7FC
volatile uint32_t                                                    null_reg498                   ; //0x800
volatile uint32_t                                                    null_reg499                   ; //0x804
volatile uint32_t                                                    null_reg500                   ; //0x808
volatile uint32_t                                                    null_reg501                   ; //0x80C
volatile uint32_t                                                    null_reg502                   ; //0x810
volatile uint32_t                                                    null_reg503                   ; //0x814
volatile uint32_t                                                    null_reg504                   ; //0x818
volatile uint32_t                                                    null_reg505                   ; //0x81C
volatile uint32_t                                                    null_reg506                   ; //0x820
volatile uint32_t                                                    null_reg507                   ; //0x824
volatile uint32_t                                                    null_reg508                   ; //0x828
volatile uint32_t                                                    null_reg509                   ; //0x82C
volatile uint32_t                                                    null_reg510                   ; //0x830
volatile uint32_t                                                    null_reg511                   ; //0x834
volatile uint32_t                                                    null_reg512                   ; //0x838
volatile uint32_t                                                    null_reg513                   ; //0x83C
volatile uint32_t                                                    null_reg514                   ; //0x840
volatile uint32_t                                                    null_reg515                   ; //0x844
volatile uint32_t                                                    null_reg516                   ; //0x848
volatile uint32_t                                                    null_reg517                   ; //0x84C
volatile uint32_t                                                    null_reg518                   ; //0x850
volatile uint32_t                                                    null_reg519                   ; //0x854
volatile uint32_t                                                    null_reg520                   ; //0x858
volatile uint32_t                                                    null_reg521                   ; //0x85C
volatile uint32_t                                                    null_reg522                   ; //0x860
volatile uint32_t                                                    null_reg523                   ; //0x864
volatile uint32_t                                                    null_reg524                   ; //0x868
volatile uint32_t                                                    null_reg525                   ; //0x86C
volatile uint32_t                                                    null_reg526                   ; //0x870
volatile uint32_t                                                    null_reg527                   ; //0x874
volatile uint32_t                                                    null_reg528                   ; //0x878
volatile uint32_t                                                    null_reg529                   ; //0x87C
volatile uint32_t                                                    null_reg530                   ; //0x880
volatile uint32_t                                                    null_reg531                   ; //0x884
volatile uint32_t                                                    null_reg532                   ; //0x888
volatile uint32_t                                                    null_reg533                   ; //0x88C
volatile uint32_t                                                    null_reg534                   ; //0x890
volatile uint32_t                                                    null_reg535                   ; //0x894
volatile uint32_t                                                    null_reg536                   ; //0x898
volatile uint32_t                                                    null_reg537                   ; //0x89C
volatile uint32_t                                                    null_reg538                   ; //0x8A0
volatile uint32_t                                                    null_reg539                   ; //0x8A4
volatile uint32_t                                                    null_reg540                   ; //0x8A8
volatile uint32_t                                                    null_reg541                   ; //0x8AC
volatile uint32_t                                                    null_reg542                   ; //0x8B0
volatile uint32_t                                                    null_reg543                   ; //0x8B4
volatile uint32_t                                                    null_reg544                   ; //0x8B8
volatile uint32_t                                                    null_reg545                   ; //0x8BC
volatile uint32_t                                                    null_reg546                   ; //0x8C0
volatile uint32_t                                                    null_reg547                   ; //0x8C4
volatile uint32_t                                                    null_reg548                   ; //0x8C8
volatile uint32_t                                                    null_reg549                   ; //0x8CC
volatile uint32_t                                                    null_reg550                   ; //0x8D0
volatile uint32_t                                                    null_reg551                   ; //0x8D4
volatile uint32_t                                                    null_reg552                   ; //0x8D8
volatile uint32_t                                                    null_reg553                   ; //0x8DC
volatile uint32_t                                                    null_reg554                   ; //0x8E0
volatile uint32_t                                                    null_reg555                   ; //0x8E4
volatile uint32_t                                                    null_reg556                   ; //0x8E8
volatile uint32_t                                                    null_reg557                   ; //0x8EC
volatile uint32_t                                                    null_reg558                   ; //0x8F0
volatile uint32_t                                                    null_reg559                   ; //0x8F4
volatile uint32_t                                                    null_reg560                   ; //0x8F8
volatile uint32_t                                                    null_reg561                   ; //0x8FC
volatile uint32_t                                                    null_reg562                   ; //0x900
volatile uint32_t                                                    null_reg563                   ; //0x904
volatile uint32_t                                                    null_reg564                   ; //0x908
volatile uint32_t                                                    null_reg565                   ; //0x90C
volatile uint32_t                                                    null_reg566                   ; //0x910
volatile uint32_t                                                    null_reg567                   ; //0x914
volatile uint32_t                                                    null_reg568                   ; //0x918
volatile uint32_t                                                    null_reg569                   ; //0x91C
volatile uint32_t                                                    null_reg570                   ; //0x920
volatile uint32_t                                                    null_reg571                   ; //0x924
volatile uint32_t                                                    null_reg572                   ; //0x928
volatile uint32_t                                                    null_reg573                   ; //0x92C
volatile uint32_t                                                    null_reg574                   ; //0x930
volatile uint32_t                                                    null_reg575                   ; //0x934
volatile uint32_t                                                    null_reg576                   ; //0x938
volatile uint32_t                                                    null_reg577                   ; //0x93C
volatile uint32_t                                                    null_reg578                   ; //0x940
volatile uint32_t                                                    null_reg579                   ; //0x944
volatile uint32_t                                                    null_reg580                   ; //0x948
volatile uint32_t                                                    null_reg581                   ; //0x94C
volatile uint32_t                                                    null_reg582                   ; //0x950
volatile uint32_t                                                    null_reg583                   ; //0x954
volatile uint32_t                                                    null_reg584                   ; //0x958
volatile uint32_t                                                    null_reg585                   ; //0x95C
volatile uint32_t                                                    null_reg586                   ; //0x960
volatile uint32_t                                                    null_reg587                   ; //0x964
volatile uint32_t                                                    null_reg588                   ; //0x968
volatile uint32_t                                                    null_reg589                   ; //0x96C
volatile uint32_t                                                    null_reg590                   ; //0x970
volatile uint32_t                                                    null_reg591                   ; //0x974
volatile uint32_t                                                    null_reg592                   ; //0x978
volatile uint32_t                                                    null_reg593                   ; //0x97C
volatile uint32_t                                                    null_reg594                   ; //0x980
volatile uint32_t                                                    null_reg595                   ; //0x984
volatile uint32_t                                                    null_reg596                   ; //0x988
volatile uint32_t                                                    null_reg597                   ; //0x98C
volatile uint32_t                                                    null_reg598                   ; //0x990
volatile uint32_t                                                    null_reg599                   ; //0x994
volatile uint32_t                                                    null_reg600                   ; //0x998
volatile uint32_t                                                    null_reg601                   ; //0x99C
volatile uint32_t                                                    null_reg602                   ; //0x9A0
volatile uint32_t                                                    null_reg603                   ; //0x9A4
volatile uint32_t                                                    null_reg604                   ; //0x9A8
volatile uint32_t                                                    null_reg605                   ; //0x9AC
volatile uint32_t                                                    null_reg606                   ; //0x9B0
volatile uint32_t                                                    null_reg607                   ; //0x9B4
volatile uint32_t                                                    null_reg608                   ; //0x9B8
volatile uint32_t                                                    null_reg609                   ; //0x9BC
volatile uint32_t                                                    null_reg610                   ; //0x9C0
volatile uint32_t                                                    null_reg611                   ; //0x9C4
volatile uint32_t                                                    null_reg612                   ; //0x9C8
volatile uint32_t                                                    null_reg613                   ; //0x9CC
volatile uint32_t                                                    null_reg614                   ; //0x9D0
volatile uint32_t                                                    null_reg615                   ; //0x9D4
volatile uint32_t                                                    null_reg616                   ; //0x9D8
volatile uint32_t                                                    null_reg617                   ; //0x9DC
volatile uint32_t                                                    null_reg618                   ; //0x9E0
volatile uint32_t                                                    null_reg619                   ; //0x9E4
volatile uint32_t                                                    null_reg620                   ; //0x9E8
volatile uint32_t                                                    null_reg621                   ; //0x9EC
volatile uint32_t                                                    null_reg622                   ; //0x9F0
volatile uint32_t                                                    null_reg623                   ; //0x9F4
volatile uint32_t                                                    null_reg624                   ; //0x9F8
volatile uint32_t                                                    null_reg625                   ; //0x9FC
volatile uint32_t                                                    null_reg626                   ; //0xA00
volatile uint32_t                                                    null_reg627                   ; //0xA04
volatile uint32_t                                                    null_reg628                   ; //0xA08
volatile uint32_t                                                    null_reg629                   ; //0xA0C
volatile uint32_t                                                    null_reg630                   ; //0xA10
volatile uint32_t                                                    null_reg631                   ; //0xA14
volatile uint32_t                                                    null_reg632                   ; //0xA18
volatile uint32_t                                                    null_reg633                   ; //0xA1C
volatile uint32_t                                                    null_reg634                   ; //0xA20
volatile uint32_t                                                    null_reg635                   ; //0xA24
volatile uint32_t                                                    null_reg636                   ; //0xA28
volatile uint32_t                                                    null_reg637                   ; //0xA2C
volatile uint32_t                                                    null_reg638                   ; //0xA30
volatile uint32_t                                                    null_reg639                   ; //0xA34
volatile uint32_t                                                    null_reg640                   ; //0xA38
volatile uint32_t                                                    null_reg641                   ; //0xA3C
volatile uint32_t                                                    null_reg642                   ; //0xA40
volatile uint32_t                                                    null_reg643                   ; //0xA44
volatile uint32_t                                                    null_reg644                   ; //0xA48
volatile uint32_t                                                    null_reg645                   ; //0xA4C
volatile uint32_t                                                    null_reg646                   ; //0xA50
volatile uint32_t                                                    null_reg647                   ; //0xA54
volatile uint32_t                                                    null_reg648                   ; //0xA58
volatile uint32_t                                                    null_reg649                   ; //0xA5C
volatile uint32_t                                                    null_reg650                   ; //0xA60
volatile uint32_t                                                    null_reg651                   ; //0xA64
volatile uint32_t                                                    null_reg652                   ; //0xA68
volatile uint32_t                                                    null_reg653                   ; //0xA6C
volatile uint32_t                                                    null_reg654                   ; //0xA70
volatile uint32_t                                                    null_reg655                   ; //0xA74
volatile uint32_t                                                    null_reg656                   ; //0xA78
volatile uint32_t                                                    null_reg657                   ; //0xA7C
volatile uint32_t                                                    null_reg658                   ; //0xA80
volatile uint32_t                                                    null_reg659                   ; //0xA84
volatile uint32_t                                                    null_reg660                   ; //0xA88
volatile uint32_t                                                    null_reg661                   ; //0xA8C
volatile uint32_t                                                    null_reg662                   ; //0xA90
volatile uint32_t                                                    null_reg663                   ; //0xA94
volatile uint32_t                                                    null_reg664                   ; //0xA98
volatile uint32_t                                                    null_reg665                   ; //0xA9C
volatile uint32_t                                                    null_reg666                   ; //0xAA0
volatile uint32_t                                                    null_reg667                   ; //0xAA4
volatile uint32_t                                                    null_reg668                   ; //0xAA8
volatile uint32_t                                                    null_reg669                   ; //0xAAC
volatile uint32_t                                                    null_reg670                   ; //0xAB0
volatile uint32_t                                                    null_reg671                   ; //0xAB4
volatile uint32_t                                                    null_reg672                   ; //0xAB8
volatile uint32_t                                                    null_reg673                   ; //0xABC
volatile uint32_t                                                    null_reg674                   ; //0xAC0
volatile uint32_t                                                    null_reg675                   ; //0xAC4
volatile uint32_t                                                    null_reg676                   ; //0xAC8
volatile uint32_t                                                    null_reg677                   ; //0xACC
volatile uint32_t                                                    null_reg678                   ; //0xAD0
volatile uint32_t                                                    null_reg679                   ; //0xAD4
volatile uint32_t                                                    null_reg680                   ; //0xAD8
volatile uint32_t                                                    null_reg681                   ; //0xADC
volatile uint32_t                                                    null_reg682                   ; //0xAE0
volatile uint32_t                                                    null_reg683                   ; //0xAE4
volatile uint32_t                                                    null_reg684                   ; //0xAE8
volatile uint32_t                                                    null_reg685                   ; //0xAEC
volatile uint32_t                                                    null_reg686                   ; //0xAF0
volatile uint32_t                                                    null_reg687                   ; //0xAF4
volatile uint32_t                                                    null_reg688                   ; //0xAF8
volatile uint32_t                                                    null_reg689                   ; //0xAFC
volatile uint32_t                                                    null_reg690                   ; //0xB00
volatile uint32_t                                                    null_reg691                   ; //0xB04
volatile uint32_t                                                    null_reg692                   ; //0xB08
volatile uint32_t                                                    null_reg693                   ; //0xB0C
volatile uint32_t                                                    null_reg694                   ; //0xB10
volatile uint32_t                                                    null_reg695                   ; //0xB14
volatile uint32_t                                                    null_reg696                   ; //0xB18
volatile uint32_t                                                    null_reg697                   ; //0xB1C
volatile uint32_t                                                    null_reg698                   ; //0xB20
volatile uint32_t                                                    null_reg699                   ; //0xB24
volatile uint32_t                                                    null_reg700                   ; //0xB28
volatile uint32_t                                                    null_reg701                   ; //0xB2C
volatile uint32_t                                                    null_reg702                   ; //0xB30
volatile uint32_t                                                    null_reg703                   ; //0xB34
volatile uint32_t                                                    null_reg704                   ; //0xB38
volatile uint32_t                                                    null_reg705                   ; //0xB3C
volatile uint32_t                                                    null_reg706                   ; //0xB40
volatile uint32_t                                                    null_reg707                   ; //0xB44
volatile uint32_t                                                    null_reg708                   ; //0xB48
volatile uint32_t                                                    null_reg709                   ; //0xB4C
volatile uint32_t                                                    null_reg710                   ; //0xB50
volatile uint32_t                                                    null_reg711                   ; //0xB54
volatile uint32_t                                                    null_reg712                   ; //0xB58
volatile uint32_t                                                    null_reg713                   ; //0xB5C
volatile uint32_t                                                    null_reg714                   ; //0xB60
volatile uint32_t                                                    null_reg715                   ; //0xB64
volatile uint32_t                                                    null_reg716                   ; //0xB68
volatile uint32_t                                                    null_reg717                   ; //0xB6C
volatile uint32_t                                                    null_reg718                   ; //0xB70
volatile uint32_t                                                    null_reg719                   ; //0xB74
volatile uint32_t                                                    null_reg720                   ; //0xB78
volatile uint32_t                                                    null_reg721                   ; //0xB7C
volatile uint32_t                                                    null_reg722                   ; //0xB80
volatile uint32_t                                                    null_reg723                   ; //0xB84
volatile uint32_t                                                    null_reg724                   ; //0xB88
volatile uint32_t                                                    null_reg725                   ; //0xB8C
volatile uint32_t                                                    null_reg726                   ; //0xB90
volatile uint32_t                                                    null_reg727                   ; //0xB94
volatile uint32_t                                                    null_reg728                   ; //0xB98
volatile uint32_t                                                    null_reg729                   ; //0xB9C
volatile uint32_t                                                    null_reg730                   ; //0xBA0
volatile uint32_t                                                    null_reg731                   ; //0xBA4
volatile uint32_t                                                    null_reg732                   ; //0xBA8
volatile uint32_t                                                    null_reg733                   ; //0xBAC
volatile uint32_t                                                    null_reg734                   ; //0xBB0
volatile uint32_t                                                    null_reg735                   ; //0xBB4
volatile uint32_t                                                    null_reg736                   ; //0xBB8
volatile uint32_t                                                    null_reg737                   ; //0xBBC
volatile uint32_t                                                    null_reg738                   ; //0xBC0
volatile uint32_t                                                    null_reg739                   ; //0xBC4
volatile uint32_t                                                    null_reg740                   ; //0xBC8
volatile uint32_t                                                    null_reg741                   ; //0xBCC
volatile uint32_t                                                    null_reg742                   ; //0xBD0
volatile uint32_t                                                    null_reg743                   ; //0xBD4
volatile uint32_t                                                    null_reg744                   ; //0xBD8
volatile uint32_t                                                    null_reg745                   ; //0xBDC
volatile uint32_t                                                    null_reg746                   ; //0xBE0
volatile uint32_t                                                    null_reg747                   ; //0xBE4
volatile uint32_t                                                    null_reg748                   ; //0xBE8
volatile uint32_t                                                    null_reg749                   ; //0xBEC
volatile uint32_t                                                    null_reg750                   ; //0xBF0
volatile uint32_t                                                    null_reg751                   ; //0xBF4
volatile uint32_t                                                    null_reg752                   ; //0xBF8
volatile uint32_t                                                    null_reg753                   ; //0xBFC
volatile uint32_t                                                    null_reg754                   ; //0xC00
volatile uint32_t                                                    null_reg755                   ; //0xC04
volatile uint32_t                                                    null_reg756                   ; //0xC08
volatile uint32_t                                                    null_reg757                   ; //0xC0C
volatile uint32_t                                                    null_reg758                   ; //0xC10
volatile uint32_t                                                    null_reg759                   ; //0xC14
volatile uint32_t                                                    null_reg760                   ; //0xC18
volatile uint32_t                                                    null_reg761                   ; //0xC1C
volatile uint32_t                                                    null_reg762                   ; //0xC20
volatile uint32_t                                                    null_reg763                   ; //0xC24
volatile uint32_t                                                    null_reg764                   ; //0xC28
volatile uint32_t                                                    null_reg765                   ; //0xC2C
volatile uint32_t                                                    null_reg766                   ; //0xC30
volatile uint32_t                                                    null_reg767                   ; //0xC34
volatile uint32_t                                                    null_reg768                   ; //0xC38
volatile uint32_t                                                    null_reg769                   ; //0xC3C
volatile uint32_t                                                    null_reg770                   ; //0xC40
volatile uint32_t                                                    null_reg771                   ; //0xC44
volatile uint32_t                                                    null_reg772                   ; //0xC48
volatile uint32_t                                                    null_reg773                   ; //0xC4C
volatile uint32_t                                                    null_reg774                   ; //0xC50
volatile uint32_t                                                    null_reg775                   ; //0xC54
volatile uint32_t                                                    null_reg776                   ; //0xC58
volatile uint32_t                                                    null_reg777                   ; //0xC5C
volatile uint32_t                                                    null_reg778                   ; //0xC60
volatile uint32_t                                                    null_reg779                   ; //0xC64
volatile uint32_t                                                    null_reg780                   ; //0xC68
volatile uint32_t                                                    null_reg781                   ; //0xC6C
volatile uint32_t                                                    null_reg782                   ; //0xC70
volatile uint32_t                                                    null_reg783                   ; //0xC74
volatile uint32_t                                                    null_reg784                   ; //0xC78
volatile uint32_t                                                    null_reg785                   ; //0xC7C
volatile uint32_t                                                    null_reg786                   ; //0xC80
volatile uint32_t                                                    null_reg787                   ; //0xC84
volatile uint32_t                                                    null_reg788                   ; //0xC88
volatile uint32_t                                                    null_reg789                   ; //0xC8C
volatile uint32_t                                                    null_reg790                   ; //0xC90
volatile uint32_t                                                    null_reg791                   ; //0xC94
volatile uint32_t                                                    null_reg792                   ; //0xC98
volatile uint32_t                                                    null_reg793                   ; //0xC9C
volatile uint32_t                                                    null_reg794                   ; //0xCA0
volatile uint32_t                                                    null_reg795                   ; //0xCA4
volatile uint32_t                                                    null_reg796                   ; //0xCA8
volatile uint32_t                                                    null_reg797                   ; //0xCAC
volatile uint32_t                                                    null_reg798                   ; //0xCB0
volatile uint32_t                                                    null_reg799                   ; //0xCB4
volatile uint32_t                                                    null_reg800                   ; //0xCB8
volatile uint32_t                                                    null_reg801                   ; //0xCBC
volatile uint32_t                                                    null_reg802                   ; //0xCC0
volatile uint32_t                                                    null_reg803                   ; //0xCC4
volatile uint32_t                                                    null_reg804                   ; //0xCC8
volatile uint32_t                                                    null_reg805                   ; //0xCCC
volatile uint32_t                                                    null_reg806                   ; //0xCD0
volatile uint32_t                                                    null_reg807                   ; //0xCD4
volatile uint32_t                                                    null_reg808                   ; //0xCD8
volatile uint32_t                                                    null_reg809                   ; //0xCDC
volatile uint32_t                                                    null_reg810                   ; //0xCE0
volatile uint32_t                                                    null_reg811                   ; //0xCE4
volatile uint32_t                                                    null_reg812                   ; //0xCE8
volatile uint32_t                                                    null_reg813                   ; //0xCEC
volatile uint32_t                                                    null_reg814                   ; //0xCF0
volatile uint32_t                                                    null_reg815                   ; //0xCF4
volatile uint32_t                                                    null_reg816                   ; //0xCF8
volatile uint32_t                                                    null_reg817                   ; //0xCFC
volatile uint32_t                                                    null_reg818                   ; //0xD00
volatile uint32_t                                                    null_reg819                   ; //0xD04
volatile uint32_t                                                    null_reg820                   ; //0xD08
volatile uint32_t                                                    null_reg821                   ; //0xD0C
volatile uint32_t                                                    null_reg822                   ; //0xD10
volatile uint32_t                                                    null_reg823                   ; //0xD14
volatile uint32_t                                                    null_reg824                   ; //0xD18
volatile uint32_t                                                    null_reg825                   ; //0xD1C
volatile uint32_t                                                    null_reg826                   ; //0xD20
volatile uint32_t                                                    null_reg827                   ; //0xD24
volatile uint32_t                                                    null_reg828                   ; //0xD28
volatile uint32_t                                                    null_reg829                   ; //0xD2C
volatile uint32_t                                                    null_reg830                   ; //0xD30
volatile uint32_t                                                    null_reg831                   ; //0xD34
volatile uint32_t                                                    null_reg832                   ; //0xD38
volatile uint32_t                                                    null_reg833                   ; //0xD3C
volatile uint32_t                                                    null_reg834                   ; //0xD40
volatile uint32_t                                                    null_reg835                   ; //0xD44
volatile uint32_t                                                    null_reg836                   ; //0xD48
volatile uint32_t                                                    null_reg837                   ; //0xD4C
volatile uint32_t                                                    null_reg838                   ; //0xD50
volatile uint32_t                                                    null_reg839                   ; //0xD54
volatile uint32_t                                                    null_reg840                   ; //0xD58
volatile uint32_t                                                    null_reg841                   ; //0xD5C
volatile uint32_t                                                    null_reg842                   ; //0xD60
volatile uint32_t                                                    null_reg843                   ; //0xD64
volatile uint32_t                                                    null_reg844                   ; //0xD68
volatile uint32_t                                                    null_reg845                   ; //0xD6C
volatile uint32_t                                                    null_reg846                   ; //0xD70
volatile uint32_t                                                    null_reg847                   ; //0xD74
volatile uint32_t                                                    null_reg848                   ; //0xD78
volatile uint32_t                                                    null_reg849                   ; //0xD7C
volatile uint32_t                                                    null_reg850                   ; //0xD80
volatile uint32_t                                                    null_reg851                   ; //0xD84
volatile uint32_t                                                    null_reg852                   ; //0xD88
volatile uint32_t                                                    null_reg853                   ; //0xD8C
volatile uint32_t                                                    null_reg854                   ; //0xD90
volatile uint32_t                                                    null_reg855                   ; //0xD94
volatile uint32_t                                                    null_reg856                   ; //0xD98
volatile uint32_t                                                    null_reg857                   ; //0xD9C
volatile uint32_t                                                    null_reg858                   ; //0xDA0
volatile uint32_t                                                    null_reg859                   ; //0xDA4
volatile uint32_t                                                    null_reg860                   ; //0xDA8
volatile uint32_t                                                    null_reg861                   ; //0xDAC
volatile uint32_t                                                    null_reg862                   ; //0xDB0
volatile uint32_t                                                    null_reg863                   ; //0xDB4
volatile uint32_t                                                    null_reg864                   ; //0xDB8
volatile uint32_t                                                    null_reg865                   ; //0xDBC
volatile uint32_t                                                    null_reg866                   ; //0xDC0
volatile uint32_t                                                    null_reg867                   ; //0xDC4
volatile uint32_t                                                    null_reg868                   ; //0xDC8
volatile uint32_t                                                    null_reg869                   ; //0xDCC
volatile uint32_t                                                    null_reg870                   ; //0xDD0
volatile uint32_t                                                    null_reg871                   ; //0xDD4
volatile uint32_t                                                    null_reg872                   ; //0xDD8
volatile uint32_t                                                    null_reg873                   ; //0xDDC
volatile uint32_t                                                    null_reg874                   ; //0xDE0
volatile uint32_t                                                    null_reg875                   ; //0xDE4
volatile uint32_t                                                    null_reg876                   ; //0xDE8
volatile uint32_t                                                    null_reg877                   ; //0xDEC
volatile uint32_t                                                    null_reg878                   ; //0xDF0
volatile uint32_t                                                    null_reg879                   ; //0xDF4
volatile uint32_t                                                    null_reg880                   ; //0xDF8
volatile uint32_t                                                    null_reg881                   ; //0xDFC
volatile uint32_t                                                    null_reg882                   ; //0xE00
volatile uint32_t                                                    null_reg883                   ; //0xE04
volatile uint32_t                                                    null_reg884                   ; //0xE08
volatile uint32_t                                                    null_reg885                   ; //0xE0C
volatile uint32_t                                                    null_reg886                   ; //0xE10
volatile uint32_t                                                    null_reg887                   ; //0xE14
volatile uint32_t                                                    null_reg888                   ; //0xE18
volatile uint32_t                                                    null_reg889                   ; //0xE1C
volatile uint32_t                                                    null_reg890                   ; //0xE20
volatile uint32_t                                                    null_reg891                   ; //0xE24
volatile uint32_t                                                    null_reg892                   ; //0xE28
volatile uint32_t                                                    null_reg893                   ; //0xE2C
volatile uint32_t                                                    null_reg894                   ; //0xE30
volatile uint32_t                                                    null_reg895                   ; //0xE34
volatile uint32_t                                                    null_reg896                   ; //0xE38
volatile uint32_t                                                    null_reg897                   ; //0xE3C
volatile uint32_t                                                    null_reg898                   ; //0xE40
volatile uint32_t                                                    null_reg899                   ; //0xE44
volatile uint32_t                                                    null_reg900                   ; //0xE48
volatile uint32_t                                                    null_reg901                   ; //0xE4C
volatile uint32_t                                                    null_reg902                   ; //0xE50
volatile uint32_t                                                    null_reg903                   ; //0xE54
volatile uint32_t                                                    null_reg904                   ; //0xE58
volatile uint32_t                                                    null_reg905                   ; //0xE5C
volatile uint32_t                                                    null_reg906                   ; //0xE60
volatile uint32_t                                                    null_reg907                   ; //0xE64
volatile uint32_t                                                    null_reg908                   ; //0xE68
volatile uint32_t                                                    null_reg909                   ; //0xE6C
volatile uint32_t                                                    null_reg910                   ; //0xE70
volatile uint32_t                                                    null_reg911                   ; //0xE74
volatile uint32_t                                                    null_reg912                   ; //0xE78
volatile uint32_t                                                    null_reg913                   ; //0xE7C
volatile uint32_t                                                    null_reg914                   ; //0xE80
volatile uint32_t                                                    null_reg915                   ; //0xE84
volatile uint32_t                                                    null_reg916                   ; //0xE88
volatile uint32_t                                                    null_reg917                   ; //0xE8C
volatile uint32_t                                                    null_reg918                   ; //0xE90
volatile uint32_t                                                    null_reg919                   ; //0xE94
volatile uint32_t                                                    null_reg920                   ; //0xE98
volatile uint32_t                                                    null_reg921                   ; //0xE9C
volatile uint32_t                                                    null_reg922                   ; //0xEA0
volatile uint32_t                                                    null_reg923                   ; //0xEA4
volatile uint32_t                                                    null_reg924                   ; //0xEA8
volatile uint32_t                                                    null_reg925                   ; //0xEAC
volatile uint32_t                                                    null_reg926                   ; //0xEB0
volatile uint32_t                                                    null_reg927                   ; //0xEB4
volatile uint32_t                                                    null_reg928                   ; //0xEB8
volatile uint32_t                                                    null_reg929                   ; //0xEBC
volatile uint32_t                                                    null_reg930                   ; //0xEC0
volatile uint32_t                                                    null_reg931                   ; //0xEC4
volatile uint32_t                                                    null_reg932                   ; //0xEC8
volatile uint32_t                                                    null_reg933                   ; //0xECC
volatile uint32_t                                                    null_reg934                   ; //0xED0
volatile uint32_t                                                    null_reg935                   ; //0xED4
volatile uint32_t                                                    null_reg936                   ; //0xED8
volatile uint32_t                                                    null_reg937                   ; //0xEDC
volatile uint32_t                                                    null_reg938                   ; //0xEE0
volatile uint32_t                                                    null_reg939                   ; //0xEE4
volatile uint32_t                                                    null_reg940                   ; //0xEE8
volatile uint32_t                                                    null_reg941                   ; //0xEEC
volatile uint32_t                                                    null_reg942                   ; //0xEF0
volatile uint32_t                                                    null_reg943                   ; //0xEF4
volatile uint32_t                                                    null_reg944                   ; //0xEF8
volatile uint32_t                                                    null_reg945                   ; //0xEFC
volatile uint32_t                                                    null_reg946                   ; //0xF00
volatile uint32_t                                                    null_reg947                   ; //0xF04
volatile uint32_t                                                    null_reg948                   ; //0xF08
volatile uint32_t                                                    null_reg949                   ; //0xF0C
volatile uint32_t                                                    null_reg950                   ; //0xF10
volatile uint32_t                                                    null_reg951                   ; //0xF14
volatile uint32_t                                                    null_reg952                   ; //0xF18
volatile uint32_t                                                    null_reg953                   ; //0xF1C
volatile uint32_t                                                    null_reg954                   ; //0xF20
volatile uint32_t                                                    null_reg955                   ; //0xF24
volatile uint32_t                                                    null_reg956                   ; //0xF28
volatile uint32_t                                                    null_reg957                   ; //0xF2C
volatile uint32_t                                                    null_reg958                   ; //0xF30
volatile uint32_t                                                    null_reg959                   ; //0xF34
volatile uint32_t                                                    null_reg960                   ; //0xF38
volatile uint32_t                                                    null_reg961                   ; //0xF3C
volatile uint32_t                                                    null_reg962                   ; //0xF40
volatile uint32_t                                                    null_reg963                   ; //0xF44
volatile uint32_t                                                    null_reg964                   ; //0xF48
volatile uint32_t                                                    null_reg965                   ; //0xF4C
volatile uint32_t                                                    null_reg966                   ; //0xF50
volatile uint32_t                                                    null_reg967                   ; //0xF54
volatile uint32_t                                                    null_reg968                   ; //0xF58
volatile uint32_t                                                    null_reg969                   ; //0xF5C
volatile uint32_t                                                    null_reg970                   ; //0xF60
volatile uint32_t                                                    null_reg971                   ; //0xF64
volatile uint32_t                                                    null_reg972                   ; //0xF68
volatile uint32_t                                                    null_reg973                   ; //0xF6C
volatile uint32_t                                                    null_reg974                   ; //0xF70
volatile uint32_t                                                    null_reg975                   ; //0xF74
volatile uint32_t                                                    null_reg976                   ; //0xF78
volatile uint32_t                                                    null_reg977                   ; //0xF7C
volatile uint32_t                                                    null_reg978                   ; //0xF80
volatile uint32_t                                                    null_reg979                   ; //0xF84
volatile uint32_t                                                    null_reg980                   ; //0xF88
volatile uint32_t                                                    null_reg981                   ; //0xF8C
volatile uint32_t                                                    null_reg982                   ; //0xF90
volatile uint32_t                                                    null_reg983                   ; //0xF94
volatile uint32_t                                                    null_reg984                   ; //0xF98
volatile uint32_t                                                    null_reg985                   ; //0xF9C
volatile uint32_t                                                    null_reg986                   ; //0xFA0
volatile uint32_t                                                    null_reg987                   ; //0xFA4
volatile uint32_t                                                    null_reg988                   ; //0xFA8
volatile uint32_t                                                    null_reg989                   ; //0xFAC
volatile uint32_t                                                    null_reg990                   ; //0xFB0
volatile uint32_t                                                    null_reg991                   ; //0xFB4
volatile uint32_t                                                    null_reg992                   ; //0xFB8
volatile uint32_t                                                    null_reg993                   ; //0xFBC
volatile uint32_t                                                    null_reg994                   ; //0xFC0
volatile uint32_t                                                    null_reg995                   ; //0xFC4
volatile uint32_t                                                    null_reg996                   ; //0xFC8
volatile uint32_t                                                    null_reg997                   ; //0xFCC
volatile uint32_t                                                    null_reg998                   ; //0xFD0
volatile uint32_t                                                    null_reg999                   ; //0xFD4
volatile uint32_t                                                    null_reg1000                  ; //0xFD8
volatile uint32_t                                                    null_reg1001                  ; //0xFDC
volatile UART_REG_UARTPeriphID0                                      UARTPeriphID0                 ; //0xFE0
volatile UART_REG_UARTPeriphID1                                      UARTPeriphID1                 ; //0xFE4
volatile UART_REG_UARTPeriphID2                                      UARTPeriphID2                 ; //0xFE8
volatile UART_REG_UARTPeriphID3                                      UARTPeriphID3                 ; //0xFEC
volatile UART_REG_UARTPCellID0                                       UARTPCellID0                  ; //0xFF0
volatile UART_REG_UARTPCellID1                                       UARTPCellID1                  ; //0xFF4
volatile UART_REG_UARTPCellD2                                        UARTPCellD2                   ; //0xFF8
volatile UART_REG_UARTPCellID3                                       UARTPCellID3                  ; //0xFFC
}UART_REG_H;

#endif //UART_REG__H
