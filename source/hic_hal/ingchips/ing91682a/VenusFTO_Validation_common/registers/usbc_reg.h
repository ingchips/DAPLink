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
// File         : usbc_reg.h
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
#ifndef USBC_REG__H
#define USBC_REG__H

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t GOTGCTL_reserved_0               : 2 ;
        volatile uint32_t VbvalidOvEn                      : 1 ;
        volatile uint32_t VbvalidOvVal                     : 1 ;
        volatile uint32_t AvalidOvEn                       : 1 ;
        volatile uint32_t AvalidOvVal                      : 1 ;
        volatile uint32_t BvalidOvEn                       : 1 ;
        volatile uint32_t BvalidOvVal                      : 1 ;
        volatile uint32_t GOTGCTL_reserved_1               : 7 ;
        volatile uint32_t DbnceFltrBypass                  : 1 ;
        volatile uint32_t ConIDSts                         : 1 ;
        volatile uint32_t DbncTime                         : 1 ;
        volatile uint32_t ASesVld                          : 1 ;
        volatile uint32_t BSesVld                          : 1 ;
        volatile uint32_t OTGVer                           : 1 ;
        volatile uint32_t CurMod                           : 1 ;
        volatile uint32_t GOTGCTL_reserved_2               : 10;
    }f;
}usbc_REG_GOTGCTL;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t GOTGINT_reserved_0               : 2 ;
        volatile uint32_t SesEndDet                        : 1 ;
        volatile uint32_t GOTGINT_reserved_1               : 5 ;
        volatile uint32_t SesReqSucStsChng                 : 1 ;
        volatile uint32_t HstNegSucStsChng                 : 1 ;
        volatile uint32_t GOTGINT_reserved_2               : 7 ;
        volatile uint32_t HstNegDet                        : 1 ;
        volatile uint32_t ADevTOUTChg                      : 1 ;
        volatile uint32_t DbnceDone                        : 1 ;
        volatile uint32_t GOTGINT_reserved_3               : 12;
    }f;
}usbc_REG_GOTGINT;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t GlblIntrMsk                      : 1 ;
        volatile uint32_t HBstLen                          : 4 ;
        volatile uint32_t DMAEn                            : 1 ;
        volatile uint32_t GAHBCFG_reserved_0               : 1 ;
        volatile uint32_t NPTxFEmpLvl                      : 1 ;
        volatile uint32_t GAHBCFG_reserved_1               : 13;
        volatile uint32_t RemMemSupp                       : 1 ;
        volatile uint32_t NotiAllDmaWrit                   : 1 ;
        volatile uint32_t AHBSingle                        : 1 ;
        volatile uint32_t InvDescEndianess                 : 1 ;
        volatile uint32_t GAHBCFG_reserved_2               : 7 ;
    }f;
}usbc_REG_GAHBCFG;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t TOutCal                          : 3 ;
        volatile uint32_t PHYIf                            : 1 ;
        volatile uint32_t GUSBCFG_reserved_0               : 1 ;
        volatile uint32_t FSIntf                           : 1 ;
        volatile uint32_t PHYSel                           : 1 ;
        volatile uint32_t GUSBCFG_reserved_1               : 3 ;
        volatile uint32_t USBTrdTim                        : 4 ;
        volatile uint32_t GUSBCFG_reserved_2               : 8 ;
        volatile uint32_t TermSelDLPulse                   : 1 ;
        volatile uint32_t GUSBCFG_reserved_3               : 3 ;
        volatile uint32_t IC_USBCap                        : 1 ;
        volatile uint32_t GUSBCFG_reserved_4               : 1 ;
        volatile uint32_t TxEndDelay                       : 1 ;
        volatile uint32_t ForceHstMode                     : 1 ;
        volatile uint32_t ForceDevMode                     : 1 ;
        volatile uint32_t CorruptTxPkt                     : 1 ;
    }f;
}usbc_REG_GUSBCFG;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t CSftRst                          : 1 ;
        volatile uint32_t PIUFSSftRst                      : 1 ;
        volatile uint32_t FrmCntrRst                       : 1 ;
        volatile uint32_t GRSTCTL_reserved_0               : 1 ;
        volatile uint32_t RxFFlsh                          : 1 ;
        volatile uint32_t TxFFlsh                          : 1 ;
        volatile uint32_t TxFNum                           : 5 ;
        volatile uint32_t GRSTCTL_reserved_1               : 19;
        volatile uint32_t DMAReq                           : 1 ;
        volatile uint32_t AHBIdle                          : 1 ;
    }f;
}usbc_REG_GRSTCTL;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t CurMod                           : 1 ;
        volatile uint32_t ModeMis                          : 1 ;
        volatile uint32_t OTGInt                           : 1 ;
        volatile uint32_t Sof                              : 1 ;
        volatile uint32_t RxFLvl                           : 1 ;
        volatile uint32_t NPTxFEmp                         : 1 ;
        volatile uint32_t GINNakEff                        : 1 ;
        volatile uint32_t GOUTNakEff                       : 1 ;
        volatile uint32_t GINTSTS_reserved_0               : 2 ;
        volatile uint32_t ErlySusp                         : 1 ;
        volatile uint32_t USBSusp                          : 1 ;
        volatile uint32_t USBRst                           : 1 ;
        volatile uint32_t EnumDone                         : 1 ;
        volatile uint32_t ISOOutDrop                       : 1 ;
        volatile uint32_t EOPF                             : 1 ;
        volatile uint32_t GINTSTS_reserved_1               : 1 ;
        volatile uint32_t EPMis                            : 1 ;
        volatile uint32_t IEPInt                           : 1 ;
        volatile uint32_t OEPInt                           : 1 ;
        volatile uint32_t incompISOIN                      : 1 ;
        volatile uint32_t incomplP                         : 1 ;
        volatile uint32_t FetSusp                          : 1 ;
        volatile uint32_t ResetDet                         : 1 ;
        volatile uint32_t PrtInt                           : 1 ;
        volatile uint32_t HChInt                           : 1 ;
        volatile uint32_t GINTSTS_reserved_2               : 2 ;
        volatile uint32_t ConIDStsChng                     : 1 ;
        volatile uint32_t DisconnInt                       : 1 ;
        volatile uint32_t SessReqInt                       : 1 ;
        volatile uint32_t WkUpInt                          : 1 ;
    }f;
}usbc_REG_GINTSTS;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t GINTMSK_reserved_0               : 1 ;
        volatile uint32_t ModeMisMsk                       : 1 ;
        volatile uint32_t OTGIntMsk                        : 1 ;
        volatile uint32_t SofMsk                           : 1 ;
        volatile uint32_t RxFLvlMsk                        : 1 ;
        volatile uint32_t NPTxFEmpMsk                      : 1 ;
        volatile uint32_t GINNakEffMsk                     : 1 ;
        volatile uint32_t GOUTNakEffMsk                    : 1 ;
        volatile uint32_t GINTMSK_reserved_1               : 2 ;
        volatile uint32_t ErlySuspMsk                      : 1 ;
        volatile uint32_t USBSuspMsk                       : 1 ;
        volatile uint32_t USBRstMsk                        : 1 ;
        volatile uint32_t EnumDoneMsk                      : 1 ;
        volatile uint32_t ISOOutDropMsk                    : 1 ;
        volatile uint32_t EOPFMsk                          : 1 ;
        volatile uint32_t GINTMSK_reserved_2               : 1 ;
        volatile uint32_t EPMisMsk                         : 1 ;
        volatile uint32_t IEPIntMsk                        : 1 ;
        volatile uint32_t OEPIntMsk                        : 1 ;
        volatile uint32_t GINTMSK_reserved_3               : 1 ;
        volatile uint32_t incomplPMsK                      : 1 ;
        volatile uint32_t FetSuspMsk                       : 1 ;
        volatile uint32_t ResetDetMsk                      : 1 ;
        volatile uint32_t PrtIntMsk                        : 1 ;
        volatile uint32_t HChIntMsk                        : 1 ;
        volatile uint32_t GINTMSK_reserved_4               : 2 ;
        volatile uint32_t ConIDStsChngMsk                  : 1 ;
        volatile uint32_t DisconnIntMsk                    : 1 ;
        volatile uint32_t SessReqIntMsk                    : 1 ;
        volatile uint32_t WkUpIntMsk                       : 1 ;
    }f;
}usbc_REG_GINTMSK;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ChNum                            : 4 ;
        volatile uint32_t BCnt                             : 11;
        volatile uint32_t DPID                             : 2 ;
        volatile uint32_t PktSts                           : 4 ;
        volatile uint32_t FN                               : 4 ;
        volatile uint32_t GRXSTSR_reserved_0               : 7 ;
    }f;
}usbc_REG_GRXSTSR;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t ChNum                            : 4 ;
        volatile uint32_t BCnt                             : 11;
        volatile uint32_t DPID                             : 2 ;
        volatile uint32_t PktSts                           : 4 ;
        volatile uint32_t FN                               : 4 ;
        volatile uint32_t GRXSTSP_reserved_0               : 7 ;
    }f;
}usbc_REG_GRXSTSP;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RxFDep                           : 9 ;
        volatile uint32_t GRXFSIZ_reserved_0               : 23;
    }f;
}usbc_REG_GRXFSIZ;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t NPTXFStAddr                      : 9 ;
        volatile uint32_t GNPTXFSIZ_reserved_0             : 7 ;
        volatile uint32_t NPTXFDep                         : 9 ;
        volatile uint32_t GNPTXFSIZ_reserved_1             : 7 ;
    }f;
}usbc_REG_GNPTXFSIZ;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t NPTxFSpcAvail                    : 16;
        volatile uint32_t NPTxQSpcAvail                    : 8 ;
        volatile uint32_t NPTxQTop                         : 7 ;
        volatile uint32_t GNPTXSTS_reserved_0              : 1 ;
    }f;
}usbc_REG_GNPTXSTS;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t GPI                              : 16;
        volatile uint32_t GPO                              : 16;
    }f;
}usbc_REG_GGPIO;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t GUID                             : 32;
    }f;
}usbc_REG_GUID;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t SynopsysID                       : 32;
    }f;
}usbc_REG_GSNPSID;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t EpDir                            : 32;
    }f;
}usbc_REG_GHWCFG1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t OtgMode                          : 3 ;
        volatile uint32_t OtgArch                          : 2 ;
        volatile uint32_t SingPnt                          : 1 ;
        volatile uint32_t HSPhyType                        : 2 ;
        volatile uint32_t FSPhyType                        : 2 ;
        volatile uint32_t NumDevEps                        : 4 ;
        volatile uint32_t NumHstChnl                       : 4 ;
        volatile uint32_t PerioSupport                     : 1 ;
        volatile uint32_t DynFifoSizing                    : 1 ;
        volatile uint32_t MultiProcIntrpt                  : 1 ;
        volatile uint32_t GHWCFG2_reserved_0               : 1 ;
        volatile uint32_t NPTxQDepth                       : 2 ;
        volatile uint32_t PTxQDepth                        : 2 ;
        volatile uint32_t TknQDepth                        : 5 ;
        volatile uint32_t GHWCFG2_reserved_1               : 1 ;
    }f;
}usbc_REG_GHWCFG2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSizeWidth                    : 4 ;
        volatile uint32_t PktSizeWidth                     : 3 ;
        volatile uint32_t OtgEn                            : 1 ;
        volatile uint32_t I2CIntSel                        : 1 ;
        volatile uint32_t VndctlSupt                       : 1 ;
        volatile uint32_t OptFeature                       : 1 ;
        volatile uint32_t RstType                          : 1 ;
        volatile uint32_t ADPSupport                       : 1 ;
        volatile uint32_t HSICMode                         : 1 ;
        volatile uint32_t BCSupport                        : 1 ;
        volatile uint32_t LPMMode                          : 1 ;
        volatile uint32_t DfifoDepth                       : 16;
    }f;
}usbc_REG_GHWCFG3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t NumDevPerioEps                   : 4 ;
        volatile uint32_t PartialPwrDn                     : 1 ;
        volatile uint32_t AhbFreq                          : 1 ;
        volatile uint32_t Hibernation                      : 1 ;
        volatile uint32_t ExtendedHibernation              : 1 ;
        volatile uint32_t GHWCFG4_reserved_0               : 4 ;
        volatile uint32_t ACGSupt                          : 1 ;
        volatile uint32_t EnhancedLPMSupt                  : 1 ;
        volatile uint32_t PhyDataWidth                     : 2 ;
        volatile uint32_t NumCtlEps                        : 4 ;
        volatile uint32_t IddgFltr                         : 1 ;
        volatile uint32_t VBusValidFltr                    : 1 ;
        volatile uint32_t AValidFltr                       : 1 ;
        volatile uint32_t BValidFltr                       : 1 ;
        volatile uint32_t SessEndFltr                      : 1 ;
        volatile uint32_t DedFifoMode                      : 1 ;
        volatile uint32_t INEps                            : 4 ;
        volatile uint32_t DescDMAEnabled                   : 1 ;
        volatile uint32_t DescDMA                          : 1 ;
    }f;
}usbc_REG_GHWCFG4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t GDFIFOCfg                        : 16;
        volatile uint32_t EPInfoBaseAddr                   : 16;
    }f;
}usbc_REG_GDFIFOCFG;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t PTxFStAddr                       : 10;
        volatile uint32_t HPTXFSIZ_reserved_0              : 6 ;
        volatile uint32_t PTxFSize                         : 11;
        volatile uint32_t HPTXFSIZ_reserved_1              : 5 ;
    }f;
}usbc_REG_HPTXFSIZ;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t INEPnTxFStAddr                   : 9 ;
        volatile uint32_t DIEPTXF1_reserved_0              : 7 ;
        volatile uint32_t INEPnTxFDep                      : 6 ;
        volatile uint32_t DIEPTXF1_reserved_1              : 10;
    }f;
}usbc_REG_DIEPTXF1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t INEPnTxFStAddr                   : 9 ;
        volatile uint32_t DIEPTXF2_reserved_0              : 7 ;
        volatile uint32_t INEPnTxFDep                      : 6 ;
        volatile uint32_t DIEPTXF2_reserved_1              : 10;
    }f;
}usbc_REG_DIEPTXF2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t INEPnTxFStAddr                   : 9 ;
        volatile uint32_t DIEPTXF3_reserved_0              : 7 ;
        volatile uint32_t INEPnTxFDep                      : 6 ;
        volatile uint32_t DIEPTXF3_reserved_1              : 10;
    }f;
}usbc_REG_DIEPTXF3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t INEPnTxFStAddr                   : 9 ;
        volatile uint32_t DIEPTXF4_reserved_0              : 7 ;
        volatile uint32_t INEPnTxFDep                      : 6 ;
        volatile uint32_t DIEPTXF4_reserved_1              : 10;
    }f;
}usbc_REG_DIEPTXF4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t INEPnTxFStAddr                   : 9 ;
        volatile uint32_t DIEPTXF5_reserved_0              : 7 ;
        volatile uint32_t INEPnTxFDep                      : 6 ;
        volatile uint32_t DIEPTXF5_reserved_1              : 10;
    }f;
}usbc_REG_DIEPTXF5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t FSLSPclkSel                      : 2 ;
        volatile uint32_t FSLSSupp                         : 1 ;
        volatile uint32_t HCFG_reserved_0                  : 4 ;
        volatile uint32_t Ena32KHzS                        : 1 ;
        volatile uint32_t ResValid                         : 8 ;
        volatile uint32_t HCFG_reserved_1                  : 7 ;
        volatile uint32_t DescDMA                          : 1 ;
        volatile uint32_t FrListEn                         : 2 ;
        volatile uint32_t PerSchedEna                      : 1 ;
        volatile uint32_t HCFG_reserved_2                  : 4 ;
        volatile uint32_t ModeChTimEn                      : 1 ;
    }f;
}usbc_REG_HCFG;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t FrInt                            : 16;
        volatile uint32_t HFIRRldCtrl                      : 1 ;
        volatile uint32_t HFIR_reserved_0                  : 15;
    }f;
}usbc_REG_HFIR;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t FrNum                            : 16;
        volatile uint32_t FrRem                            : 16;
    }f;
}usbc_REG_HFNUM;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t HAINT                            : 12;
        volatile uint32_t HAINT_reserved_0                 : 20;
    }f;
}usbc_REG_HAINT;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t HAINTMsk                         : 12;
        volatile uint32_t HAINTMSK_reserved_0              : 20;
    }f;
}usbc_REG_HAINTMSK;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t HFLBAddr                         : 32;
    }f;
}usbc_REG_HFLBAddr;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t PrtConnSts                       : 1 ;
        volatile uint32_t PrtConnDet                       : 1 ;
        volatile uint32_t PrtEna                           : 1 ;
        volatile uint32_t PrtEnChng                        : 1 ;
        volatile uint32_t PrtOvrCurrAct                    : 1 ;
        volatile uint32_t PrtOvrCurrChng                   : 1 ;
        volatile uint32_t PrtRes                           : 1 ;
        volatile uint32_t PrtSusp                          : 1 ;
        volatile uint32_t PrtRst                           : 1 ;
        volatile uint32_t HPRT_reserved_0                  : 1 ;
        volatile uint32_t PrtLnSts                         : 2 ;
        volatile uint32_t PrtPwr                           : 1 ;
        volatile uint32_t PrtTstCtl                        : 4 ;
        volatile uint32_t PrtSpd                           : 2 ;
        volatile uint32_t HPRT_reserved_1                  : 13;
    }f;
}usbc_REG_HPRT;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t EPNum                            : 4 ;
        volatile uint32_t EPDir                            : 1 ;
        volatile uint32_t HCCHAR0_reserved_0               : 1 ;
        volatile uint32_t LSpdDev                          : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t EC                               : 2 ;
        volatile uint32_t DevAddr                          : 7 ;
        volatile uint32_t OddFrm                           : 1 ;
        volatile uint32_t ChDis                            : 1 ;
        volatile uint32_t ChEna                            : 1 ;
    }f;
}usbc_REG_HCCHAR0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t PrtAddr                          : 7 ;
        volatile uint32_t HubAddr                          : 7 ;
        volatile uint32_t XactPos                          : 2 ;
        volatile uint32_t CompSplt                         : 1 ;
        volatile uint32_t HCSPLT0_reserved_0               : 14;
        volatile uint32_t SpltEna                          : 1 ;
    }f;
}usbc_REG_HCSPLT0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t ChHltd                           : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t STALL                            : 1 ;
        volatile uint32_t NAK                              : 1 ;
        volatile uint32_t ACK                              : 1 ;
        volatile uint32_t NYET                             : 1 ;
        volatile uint32_t XactErr                          : 1 ;
        volatile uint32_t BblErr                           : 1 ;
        volatile uint32_t FrmOvrun                         : 1 ;
        volatile uint32_t DataTglErr                       : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t XCS_XACT_ERR                     : 1 ;
        volatile uint32_t DESC_LST_ROLLIntr                : 1 ;
        volatile uint32_t HCINT0_reserved_0                : 18;
    }f;
}usbc_REG_HCINT0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferComplMsk                     : 1 ;
        volatile uint32_t ChHltdMsk                        : 1 ;
        volatile uint32_t AHBErrMsk                        : 1 ;
        volatile uint32_t HCINTMSK0_reserved_0             : 8 ;
        volatile uint32_t BNAIntrMsk                       : 1 ;
        volatile uint32_t HCINTMSK0_reserved_1             : 1 ;
        volatile uint32_t DESC_LST_ROLLIntrMsk             : 1 ;
        volatile uint32_t HCINTMSK0_reserved_2             : 18;
    }f;
}usbc_REG_HCINTMSK0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t Pid                              : 2 ;
        volatile uint32_t DoPng                            : 1 ;
    }f;
}usbc_REG_HCTSIZ0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_HCDMA0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t HCDMAB                           : 32;
    }f;
}usbc_REG_HCDMAB0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t EPNum                            : 4 ;
        volatile uint32_t EPDir                            : 1 ;
        volatile uint32_t HCCHAR1_reserved_0               : 1 ;
        volatile uint32_t LSpdDev                          : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t EC                               : 2 ;
        volatile uint32_t DevAddr                          : 7 ;
        volatile uint32_t OddFrm                           : 1 ;
        volatile uint32_t ChDis                            : 1 ;
        volatile uint32_t ChEna                            : 1 ;
    }f;
}usbc_REG_HCCHAR1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t PrtAddr                          : 7 ;
        volatile uint32_t HubAddr                          : 7 ;
        volatile uint32_t XactPos                          : 2 ;
        volatile uint32_t CompSplt                         : 1 ;
        volatile uint32_t HCSPLT1_reserved_0               : 14;
        volatile uint32_t SpltEna                          : 1 ;
    }f;
}usbc_REG_HCSPLT1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t ChHltd                           : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t STALL                            : 1 ;
        volatile uint32_t NAK                              : 1 ;
        volatile uint32_t ACK                              : 1 ;
        volatile uint32_t NYET                             : 1 ;
        volatile uint32_t XactErr                          : 1 ;
        volatile uint32_t BblErr                           : 1 ;
        volatile uint32_t FrmOvrun                         : 1 ;
        volatile uint32_t DataTglErr                       : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t XCS_XACT_ERR                     : 1 ;
        volatile uint32_t DESC_LST_ROLLIntr                : 1 ;
        volatile uint32_t HCINT1_reserved_0                : 18;
    }f;
}usbc_REG_HCINT1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferComplMsk                     : 1 ;
        volatile uint32_t ChHltdMsk                        : 1 ;
        volatile uint32_t AHBErrMsk                        : 1 ;
        volatile uint32_t HCINTMSK1_reserved_0             : 8 ;
        volatile uint32_t BNAIntrMsk                       : 1 ;
        volatile uint32_t HCINTMSK1_reserved_1             : 1 ;
        volatile uint32_t DESC_LST_ROLLIntrMsk             : 1 ;
        volatile uint32_t HCINTMSK1_reserved_2             : 18;
    }f;
}usbc_REG_HCINTMSK1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t Pid                              : 2 ;
        volatile uint32_t DoPng                            : 1 ;
    }f;
}usbc_REG_HCTSIZ1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_HCDMA1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t HCDMAB                           : 32;
    }f;
}usbc_REG_HCDMAB1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t EPNum                            : 4 ;
        volatile uint32_t EPDir                            : 1 ;
        volatile uint32_t HCCHAR2_reserved_0               : 1 ;
        volatile uint32_t LSpdDev                          : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t EC                               : 2 ;
        volatile uint32_t DevAddr                          : 7 ;
        volatile uint32_t OddFrm                           : 1 ;
        volatile uint32_t ChDis                            : 1 ;
        volatile uint32_t ChEna                            : 1 ;
    }f;
}usbc_REG_HCCHAR2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t PrtAddr                          : 7 ;
        volatile uint32_t HubAddr                          : 7 ;
        volatile uint32_t XactPos                          : 2 ;
        volatile uint32_t CompSplt                         : 1 ;
        volatile uint32_t HCSPLT2_reserved_0               : 14;
        volatile uint32_t SpltEna                          : 1 ;
    }f;
}usbc_REG_HCSPLT2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t ChHltd                           : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t STALL                            : 1 ;
        volatile uint32_t NAK                              : 1 ;
        volatile uint32_t ACK                              : 1 ;
        volatile uint32_t NYET                             : 1 ;
        volatile uint32_t XactErr                          : 1 ;
        volatile uint32_t BblErr                           : 1 ;
        volatile uint32_t FrmOvrun                         : 1 ;
        volatile uint32_t DataTglErr                       : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t XCS_XACT_ERR                     : 1 ;
        volatile uint32_t DESC_LST_ROLLIntr                : 1 ;
        volatile uint32_t HCINT2_reserved_0                : 18;
    }f;
}usbc_REG_HCINT2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferComplMsk                     : 1 ;
        volatile uint32_t ChHltdMsk                        : 1 ;
        volatile uint32_t AHBErrMsk                        : 1 ;
        volatile uint32_t HCINTMSK2_reserved_0             : 8 ;
        volatile uint32_t BNAIntrMsk                       : 1 ;
        volatile uint32_t HCINTMSK2_reserved_1             : 1 ;
        volatile uint32_t DESC_LST_ROLLIntrMsk             : 1 ;
        volatile uint32_t HCINTMSK2_reserved_2             : 18;
    }f;
}usbc_REG_HCINTMSK2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t Pid                              : 2 ;
        volatile uint32_t DoPng                            : 1 ;
    }f;
}usbc_REG_HCTSIZ2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_HCDMA2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t HCDMAB                           : 32;
    }f;
}usbc_REG_HCDMAB2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t EPNum                            : 4 ;
        volatile uint32_t EPDir                            : 1 ;
        volatile uint32_t HCCHAR3_reserved_0               : 1 ;
        volatile uint32_t LSpdDev                          : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t EC                               : 2 ;
        volatile uint32_t DevAddr                          : 7 ;
        volatile uint32_t OddFrm                           : 1 ;
        volatile uint32_t ChDis                            : 1 ;
        volatile uint32_t ChEna                            : 1 ;
    }f;
}usbc_REG_HCCHAR3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t PrtAddr                          : 7 ;
        volatile uint32_t HubAddr                          : 7 ;
        volatile uint32_t XactPos                          : 2 ;
        volatile uint32_t CompSplt                         : 1 ;
        volatile uint32_t HCSPLT3_reserved_0               : 14;
        volatile uint32_t SpltEna                          : 1 ;
    }f;
}usbc_REG_HCSPLT3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t ChHltd                           : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t STALL                            : 1 ;
        volatile uint32_t NAK                              : 1 ;
        volatile uint32_t ACK                              : 1 ;
        volatile uint32_t NYET                             : 1 ;
        volatile uint32_t XactErr                          : 1 ;
        volatile uint32_t BblErr                           : 1 ;
        volatile uint32_t FrmOvrun                         : 1 ;
        volatile uint32_t DataTglErr                       : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t XCS_XACT_ERR                     : 1 ;
        volatile uint32_t DESC_LST_ROLLIntr                : 1 ;
        volatile uint32_t HCINT3_reserved_0                : 18;
    }f;
}usbc_REG_HCINT3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferComplMsk                     : 1 ;
        volatile uint32_t ChHltdMsk                        : 1 ;
        volatile uint32_t AHBErrMsk                        : 1 ;
        volatile uint32_t HCINTMSK3_reserved_0             : 8 ;
        volatile uint32_t BNAIntrMsk                       : 1 ;
        volatile uint32_t HCINTMSK3_reserved_1             : 1 ;
        volatile uint32_t DESC_LST_ROLLIntrMsk             : 1 ;
        volatile uint32_t HCINTMSK3_reserved_2             : 18;
    }f;
}usbc_REG_HCINTMSK3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t Pid                              : 2 ;
        volatile uint32_t DoPng                            : 1 ;
    }f;
}usbc_REG_HCTSIZ3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_HCDMA3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t HCDMAB                           : 32;
    }f;
}usbc_REG_HCDMAB3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t EPNum                            : 4 ;
        volatile uint32_t EPDir                            : 1 ;
        volatile uint32_t HCCHAR4_reserved_0               : 1 ;
        volatile uint32_t LSpdDev                          : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t EC                               : 2 ;
        volatile uint32_t DevAddr                          : 7 ;
        volatile uint32_t OddFrm                           : 1 ;
        volatile uint32_t ChDis                            : 1 ;
        volatile uint32_t ChEna                            : 1 ;
    }f;
}usbc_REG_HCCHAR4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t PrtAddr                          : 7 ;
        volatile uint32_t HubAddr                          : 7 ;
        volatile uint32_t XactPos                          : 2 ;
        volatile uint32_t CompSplt                         : 1 ;
        volatile uint32_t HCSPLT4_reserved_0               : 14;
        volatile uint32_t SpltEna                          : 1 ;
    }f;
}usbc_REG_HCSPLT4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t ChHltd                           : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t STALL                            : 1 ;
        volatile uint32_t NAK                              : 1 ;
        volatile uint32_t ACK                              : 1 ;
        volatile uint32_t NYET                             : 1 ;
        volatile uint32_t XactErr                          : 1 ;
        volatile uint32_t BblErr                           : 1 ;
        volatile uint32_t FrmOvrun                         : 1 ;
        volatile uint32_t DataTglErr                       : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t XCS_XACT_ERR                     : 1 ;
        volatile uint32_t DESC_LST_ROLLIntr                : 1 ;
        volatile uint32_t HCINT4_reserved_0                : 18;
    }f;
}usbc_REG_HCINT4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferComplMsk                     : 1 ;
        volatile uint32_t ChHltdMsk                        : 1 ;
        volatile uint32_t AHBErrMsk                        : 1 ;
        volatile uint32_t HCINTMSK4_reserved_0             : 8 ;
        volatile uint32_t BNAIntrMsk                       : 1 ;
        volatile uint32_t HCINTMSK4_reserved_1             : 1 ;
        volatile uint32_t DESC_LST_ROLLIntrMsk             : 1 ;
        volatile uint32_t HCINTMSK4_reserved_2             : 18;
    }f;
}usbc_REG_HCINTMSK4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t Pid                              : 2 ;
        volatile uint32_t DoPng                            : 1 ;
    }f;
}usbc_REG_HCTSIZ4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_HCDMA4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t HCDMAB                           : 32;
    }f;
}usbc_REG_HCDMAB4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t EPNum                            : 4 ;
        volatile uint32_t EPDir                            : 1 ;
        volatile uint32_t HCCHAR5_reserved_0               : 1 ;
        volatile uint32_t LSpdDev                          : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t EC                               : 2 ;
        volatile uint32_t DevAddr                          : 7 ;
        volatile uint32_t OddFrm                           : 1 ;
        volatile uint32_t ChDis                            : 1 ;
        volatile uint32_t ChEna                            : 1 ;
    }f;
}usbc_REG_HCCHAR5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t PrtAddr                          : 7 ;
        volatile uint32_t HubAddr                          : 7 ;
        volatile uint32_t XactPos                          : 2 ;
        volatile uint32_t CompSplt                         : 1 ;
        volatile uint32_t HCSPLT5_reserved_0               : 14;
        volatile uint32_t SpltEna                          : 1 ;
    }f;
}usbc_REG_HCSPLT5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t ChHltd                           : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t STALL                            : 1 ;
        volatile uint32_t NAK                              : 1 ;
        volatile uint32_t ACK                              : 1 ;
        volatile uint32_t NYET                             : 1 ;
        volatile uint32_t XactErr                          : 1 ;
        volatile uint32_t BblErr                           : 1 ;
        volatile uint32_t FrmOvrun                         : 1 ;
        volatile uint32_t DataTglErr                       : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t XCS_XACT_ERR                     : 1 ;
        volatile uint32_t DESC_LST_ROLLIntr                : 1 ;
        volatile uint32_t HCINT5_reserved_0                : 18;
    }f;
}usbc_REG_HCINT5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferComplMsk                     : 1 ;
        volatile uint32_t ChHltdMsk                        : 1 ;
        volatile uint32_t AHBErrMsk                        : 1 ;
        volatile uint32_t HCINTMSK5_reserved_0             : 8 ;
        volatile uint32_t BNAIntrMsk                       : 1 ;
        volatile uint32_t HCINTMSK5_reserved_1             : 1 ;
        volatile uint32_t DESC_LST_ROLLIntrMsk             : 1 ;
        volatile uint32_t HCINTMSK5_reserved_2             : 18;
    }f;
}usbc_REG_HCINTMSK5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t Pid                              : 2 ;
        volatile uint32_t DoPng                            : 1 ;
    }f;
}usbc_REG_HCTSIZ5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_HCDMA5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t HCDMAB                           : 32;
    }f;
}usbc_REG_HCDMAB5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t EPNum                            : 4 ;
        volatile uint32_t EPDir                            : 1 ;
        volatile uint32_t HCCHAR6_reserved_0               : 1 ;
        volatile uint32_t LSpdDev                          : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t EC                               : 2 ;
        volatile uint32_t DevAddr                          : 7 ;
        volatile uint32_t OddFrm                           : 1 ;
        volatile uint32_t ChDis                            : 1 ;
        volatile uint32_t ChEna                            : 1 ;
    }f;
}usbc_REG_HCCHAR6;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t PrtAddr                          : 7 ;
        volatile uint32_t HubAddr                          : 7 ;
        volatile uint32_t XactPos                          : 2 ;
        volatile uint32_t CompSplt                         : 1 ;
        volatile uint32_t HCSPLT6_reserved_0               : 14;
        volatile uint32_t SpltEna                          : 1 ;
    }f;
}usbc_REG_HCSPLT6;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t ChHltd                           : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t STALL                            : 1 ;
        volatile uint32_t NAK                              : 1 ;
        volatile uint32_t ACK                              : 1 ;
        volatile uint32_t NYET                             : 1 ;
        volatile uint32_t XactErr                          : 1 ;
        volatile uint32_t BblErr                           : 1 ;
        volatile uint32_t FrmOvrun                         : 1 ;
        volatile uint32_t DataTglErr                       : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t XCS_XACT_ERR                     : 1 ;
        volatile uint32_t DESC_LST_ROLLIntr                : 1 ;
        volatile uint32_t HCINT6_reserved_0                : 18;
    }f;
}usbc_REG_HCINT6;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferComplMsk                     : 1 ;
        volatile uint32_t ChHltdMsk                        : 1 ;
        volatile uint32_t AHBErrMsk                        : 1 ;
        volatile uint32_t HCINTMSK6_reserved_0             : 8 ;
        volatile uint32_t BNAIntrMsk                       : 1 ;
        volatile uint32_t HCINTMSK6_reserved_1             : 1 ;
        volatile uint32_t DESC_LST_ROLLIntrMsk             : 1 ;
        volatile uint32_t HCINTMSK6_reserved_2             : 18;
    }f;
}usbc_REG_HCINTMSK6;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t Pid                              : 2 ;
        volatile uint32_t DoPng                            : 1 ;
    }f;
}usbc_REG_HCTSIZ6;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_HCDMA6;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t HCDMAB                           : 32;
    }f;
}usbc_REG_HCDMAB6;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t EPNum                            : 4 ;
        volatile uint32_t EPDir                            : 1 ;
        volatile uint32_t HCCHAR7_reserved_0               : 1 ;
        volatile uint32_t LSpdDev                          : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t EC                               : 2 ;
        volatile uint32_t DevAddr                          : 7 ;
        volatile uint32_t OddFrm                           : 1 ;
        volatile uint32_t ChDis                            : 1 ;
        volatile uint32_t ChEna                            : 1 ;
    }f;
}usbc_REG_HCCHAR7;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t PrtAddr                          : 7 ;
        volatile uint32_t HubAddr                          : 7 ;
        volatile uint32_t XactPos                          : 2 ;
        volatile uint32_t CompSplt                         : 1 ;
        volatile uint32_t HCSPLT7_reserved_0               : 14;
        volatile uint32_t SpltEna                          : 1 ;
    }f;
}usbc_REG_HCSPLT7;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t ChHltd                           : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t STALL                            : 1 ;
        volatile uint32_t NAK                              : 1 ;
        volatile uint32_t ACK                              : 1 ;
        volatile uint32_t NYET                             : 1 ;
        volatile uint32_t XactErr                          : 1 ;
        volatile uint32_t BblErr                           : 1 ;
        volatile uint32_t FrmOvrun                         : 1 ;
        volatile uint32_t DataTglErr                       : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t XCS_XACT_ERR                     : 1 ;
        volatile uint32_t DESC_LST_ROLLIntr                : 1 ;
        volatile uint32_t HCINT7_reserved_0                : 18;
    }f;
}usbc_REG_HCINT7;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferComplMsk                     : 1 ;
        volatile uint32_t ChHltdMsk                        : 1 ;
        volatile uint32_t AHBErrMsk                        : 1 ;
        volatile uint32_t HCINTMSK7_reserved_0             : 8 ;
        volatile uint32_t BNAIntrMsk                       : 1 ;
        volatile uint32_t HCINTMSK7_reserved_1             : 1 ;
        volatile uint32_t DESC_LST_ROLLIntrMsk             : 1 ;
        volatile uint32_t HCINTMSK7_reserved_2             : 18;
    }f;
}usbc_REG_HCINTMSK7;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t Pid                              : 2 ;
        volatile uint32_t DoPng                            : 1 ;
    }f;
}usbc_REG_HCTSIZ7;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_HCDMA7;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t HCDMAB                           : 32;
    }f;
}usbc_REG_HCDMAB7;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t EPNum                            : 4 ;
        volatile uint32_t EPDir                            : 1 ;
        volatile uint32_t HCCHAR8_reserved_0               : 1 ;
        volatile uint32_t LSpdDev                          : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t EC                               : 2 ;
        volatile uint32_t DevAddr                          : 7 ;
        volatile uint32_t OddFrm                           : 1 ;
        volatile uint32_t ChDis                            : 1 ;
        volatile uint32_t ChEna                            : 1 ;
    }f;
}usbc_REG_HCCHAR8;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t PrtAddr                          : 7 ;
        volatile uint32_t HubAddr                          : 7 ;
        volatile uint32_t XactPos                          : 2 ;
        volatile uint32_t CompSplt                         : 1 ;
        volatile uint32_t HCSPLT8_reserved_0               : 14;
        volatile uint32_t SpltEna                          : 1 ;
    }f;
}usbc_REG_HCSPLT8;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t ChHltd                           : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t STALL                            : 1 ;
        volatile uint32_t NAK                              : 1 ;
        volatile uint32_t ACK                              : 1 ;
        volatile uint32_t NYET                             : 1 ;
        volatile uint32_t XactErr                          : 1 ;
        volatile uint32_t BblErr                           : 1 ;
        volatile uint32_t FrmOvrun                         : 1 ;
        volatile uint32_t DataTglErr                       : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t XCS_XACT_ERR                     : 1 ;
        volatile uint32_t DESC_LST_ROLLIntr                : 1 ;
        volatile uint32_t HCINT8_reserved_0                : 18;
    }f;
}usbc_REG_HCINT8;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferComplMsk                     : 1 ;
        volatile uint32_t ChHltdMsk                        : 1 ;
        volatile uint32_t AHBErrMsk                        : 1 ;
        volatile uint32_t HCINTMSK8_reserved_0             : 8 ;
        volatile uint32_t BNAIntrMsk                       : 1 ;
        volatile uint32_t HCINTMSK8_reserved_1             : 1 ;
        volatile uint32_t DESC_LST_ROLLIntrMsk             : 1 ;
        volatile uint32_t HCINTMSK8_reserved_2             : 18;
    }f;
}usbc_REG_HCINTMSK8;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t Pid                              : 2 ;
        volatile uint32_t DoPng                            : 1 ;
    }f;
}usbc_REG_HCTSIZ8;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_HCDMA8;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t HCDMAB                           : 32;
    }f;
}usbc_REG_HCDMAB8;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t EPNum                            : 4 ;
        volatile uint32_t EPDir                            : 1 ;
        volatile uint32_t HCCHAR9_reserved_0               : 1 ;
        volatile uint32_t LSpdDev                          : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t EC                               : 2 ;
        volatile uint32_t DevAddr                          : 7 ;
        volatile uint32_t OddFrm                           : 1 ;
        volatile uint32_t ChDis                            : 1 ;
        volatile uint32_t ChEna                            : 1 ;
    }f;
}usbc_REG_HCCHAR9;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t PrtAddr                          : 7 ;
        volatile uint32_t HubAddr                          : 7 ;
        volatile uint32_t XactPos                          : 2 ;
        volatile uint32_t CompSplt                         : 1 ;
        volatile uint32_t HCSPLT9_reserved_0               : 14;
        volatile uint32_t SpltEna                          : 1 ;
    }f;
}usbc_REG_HCSPLT9;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t ChHltd                           : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t STALL                            : 1 ;
        volatile uint32_t NAK                              : 1 ;
        volatile uint32_t ACK                              : 1 ;
        volatile uint32_t NYET                             : 1 ;
        volatile uint32_t XactErr                          : 1 ;
        volatile uint32_t BblErr                           : 1 ;
        volatile uint32_t FrmOvrun                         : 1 ;
        volatile uint32_t DataTglErr                       : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t XCS_XACT_ERR                     : 1 ;
        volatile uint32_t DESC_LST_ROLLIntr                : 1 ;
        volatile uint32_t HCINT9_reserved_0                : 18;
    }f;
}usbc_REG_HCINT9;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferComplMsk                     : 1 ;
        volatile uint32_t ChHltdMsk                        : 1 ;
        volatile uint32_t AHBErrMsk                        : 1 ;
        volatile uint32_t HCINTMSK9_reserved_0             : 8 ;
        volatile uint32_t BNAIntrMsk                       : 1 ;
        volatile uint32_t HCINTMSK9_reserved_1             : 1 ;
        volatile uint32_t DESC_LST_ROLLIntrMsk             : 1 ;
        volatile uint32_t HCINTMSK9_reserved_2             : 18;
    }f;
}usbc_REG_HCINTMSK9;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t Pid                              : 2 ;
        volatile uint32_t DoPng                            : 1 ;
    }f;
}usbc_REG_HCTSIZ9;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_HCDMA9;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t HCDMAB                           : 32;
    }f;
}usbc_REG_HCDMAB9;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t EPNum                            : 4 ;
        volatile uint32_t EPDir                            : 1 ;
        volatile uint32_t HCCHAR10_reserved_0              : 1 ;
        volatile uint32_t LSpdDev                          : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t EC                               : 2 ;
        volatile uint32_t DevAddr                          : 7 ;
        volatile uint32_t OddFrm                           : 1 ;
        volatile uint32_t ChDis                            : 1 ;
        volatile uint32_t ChEna                            : 1 ;
    }f;
}usbc_REG_HCCHAR10;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t PrtAddr                          : 7 ;
        volatile uint32_t HubAddr                          : 7 ;
        volatile uint32_t XactPos                          : 2 ;
        volatile uint32_t CompSplt                         : 1 ;
        volatile uint32_t HCSPLT10_reserved_0              : 14;
        volatile uint32_t SpltEna                          : 1 ;
    }f;
}usbc_REG_HCSPLT10;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t ChHltd                           : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t STALL                            : 1 ;
        volatile uint32_t NAK                              : 1 ;
        volatile uint32_t ACK                              : 1 ;
        volatile uint32_t NYET                             : 1 ;
        volatile uint32_t XactErr                          : 1 ;
        volatile uint32_t BblErr                           : 1 ;
        volatile uint32_t FrmOvrun                         : 1 ;
        volatile uint32_t DataTglErr                       : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t XCS_XACT_ERR                     : 1 ;
        volatile uint32_t DESC_LST_ROLLIntr                : 1 ;
        volatile uint32_t HCINT10_reserved_0               : 18;
    }f;
}usbc_REG_HCINT10;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferComplMsk                     : 1 ;
        volatile uint32_t ChHltdMsk                        : 1 ;
        volatile uint32_t AHBErrMsk                        : 1 ;
        volatile uint32_t HCINTMSK10_reserved_0            : 8 ;
        volatile uint32_t BNAIntrMsk                       : 1 ;
        volatile uint32_t HCINTMSK10_reserved_1            : 1 ;
        volatile uint32_t DESC_LST_ROLLIntrMsk             : 1 ;
        volatile uint32_t HCINTMSK10_reserved_2            : 18;
    }f;
}usbc_REG_HCINTMSK10;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t Pid                              : 2 ;
        volatile uint32_t DoPng                            : 1 ;
    }f;
}usbc_REG_HCTSIZ10;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_HCDMA10;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t HCDMAB                           : 32;
    }f;
}usbc_REG_HCDMAB10;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t EPNum                            : 4 ;
        volatile uint32_t EPDir                            : 1 ;
        volatile uint32_t HCCHAR11_reserved_0              : 1 ;
        volatile uint32_t LSpdDev                          : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t EC                               : 2 ;
        volatile uint32_t DevAddr                          : 7 ;
        volatile uint32_t OddFrm                           : 1 ;
        volatile uint32_t ChDis                            : 1 ;
        volatile uint32_t ChEna                            : 1 ;
    }f;
}usbc_REG_HCCHAR11;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t PrtAddr                          : 7 ;
        volatile uint32_t HubAddr                          : 7 ;
        volatile uint32_t XactPos                          : 2 ;
        volatile uint32_t CompSplt                         : 1 ;
        volatile uint32_t HCSPLT11_reserved_0              : 14;
        volatile uint32_t SpltEna                          : 1 ;
    }f;
}usbc_REG_HCSPLT11;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t ChHltd                           : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t STALL                            : 1 ;
        volatile uint32_t NAK                              : 1 ;
        volatile uint32_t ACK                              : 1 ;
        volatile uint32_t NYET                             : 1 ;
        volatile uint32_t XactErr                          : 1 ;
        volatile uint32_t BblErr                           : 1 ;
        volatile uint32_t FrmOvrun                         : 1 ;
        volatile uint32_t DataTglErr                       : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t XCS_XACT_ERR                     : 1 ;
        volatile uint32_t DESC_LST_ROLLIntr                : 1 ;
        volatile uint32_t HCINT11_reserved_0               : 18;
    }f;
}usbc_REG_HCINT11;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferComplMsk                     : 1 ;
        volatile uint32_t ChHltdMsk                        : 1 ;
        volatile uint32_t AHBErrMsk                        : 1 ;
        volatile uint32_t HCINTMSK11_reserved_0            : 8 ;
        volatile uint32_t BNAIntrMsk                       : 1 ;
        volatile uint32_t HCINTMSK11_reserved_1            : 1 ;
        volatile uint32_t DESC_LST_ROLLIntrMsk             : 1 ;
        volatile uint32_t HCINTMSK11_reserved_2            : 18;
    }f;
}usbc_REG_HCINTMSK11;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t Pid                              : 2 ;
        volatile uint32_t DoPng                            : 1 ;
    }f;
}usbc_REG_HCTSIZ11;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_HCDMA11;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t HCDMAB                           : 32;
    }f;
}usbc_REG_HCDMAB11;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DevSpd                           : 2 ;
        volatile uint32_t NZStsOUTHShk                     : 1 ;
        volatile uint32_t Ena32KHzSusp                     : 1 ;
        volatile uint32_t DevAddr                          : 7 ;
        volatile uint32_t PerFrInt                         : 2 ;
        volatile uint32_t EnDevOutNak                      : 1 ;
        volatile uint32_t XCVRDLY                          : 1 ;
        volatile uint32_t ErraticIntMsk                    : 1 ;
        volatile uint32_t DCFG_reserved_0                  : 7 ;
        volatile uint32_t DescDMA                          : 1 ;
        volatile uint32_t PerSchIntvl                      : 2 ;
        volatile uint32_t ResValid                         : 6 ;
    }f;
}usbc_REG_DCFG;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t RmtWkUpSig                       : 1 ;
        volatile uint32_t SftDiscon                        : 1 ;
        volatile uint32_t GNPINNakSts                      : 1 ;
        volatile uint32_t GOUTNakSts                       : 1 ;
        volatile uint32_t TstCtl                           : 3 ;
        volatile uint32_t SGNPInNak                        : 1 ;
        volatile uint32_t CGNPInNak                        : 1 ;
        volatile uint32_t SGOUTNak                         : 1 ;
        volatile uint32_t CGOUTNak                         : 1 ;
        volatile uint32_t PWROnPrgDone                     : 1 ;
        volatile uint32_t DCTL_reserved_0                  : 1 ;
        volatile uint32_t GMC                              : 2 ;
        volatile uint32_t IgnrFrmNum                       : 1 ;
        volatile uint32_t NakOnBble                        : 1 ;
        volatile uint32_t EnContOnBNA                      : 1 ;
        volatile uint32_t DCTL_reserved_1                  : 14;
    }f;
}usbc_REG_DCTL;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t SuspSts                          : 1 ;
        volatile uint32_t EnumSpd                          : 2 ;
        volatile uint32_t ErrticErr                        : 1 ;
        volatile uint32_t DSTS_reserved_0                  : 4 ;
        volatile uint32_t SOFFN                            : 14;
        volatile uint32_t DevLnSts                         : 2 ;
        volatile uint32_t DSTS_reserved_1                  : 8 ;
    }f;
}usbc_REG_DSTS;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferComplMsk                     : 1 ;
        volatile uint32_t EPDisbldMsk                      : 1 ;
        volatile uint32_t AHBErrMsk                        : 1 ;
        volatile uint32_t TimeOUTMsk                       : 1 ;
        volatile uint32_t INTknTXFEmpMsk                   : 1 ;
        volatile uint32_t INTknEPMisMsk                    : 1 ;
        volatile uint32_t INEPNakEffMsk                    : 1 ;
        volatile uint32_t DIEPMSK_reserved_0               : 1 ;
        volatile uint32_t TxfifoUndrnMsk                   : 1 ;
        volatile uint32_t BNAInIntrMsk                     : 1 ;
        volatile uint32_t DIEPMSK_reserved_1               : 3 ;
        volatile uint32_t NAKMsk                           : 1 ;
        volatile uint32_t DIEPMSK_reserved_2               : 18;
    }f;
}usbc_REG_DIEPMSK;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferComplMsk                     : 1 ;
        volatile uint32_t EPDisbldMsk                      : 1 ;
        volatile uint32_t AHBErrMsk                        : 1 ;
        volatile uint32_t SetUPMsk                         : 1 ;
        volatile uint32_t OUTTknEPdisMsk                   : 1 ;
        volatile uint32_t StsPhseRcvdMsk                   : 1 ;
        volatile uint32_t Back2BackSETup                   : 1 ;
        volatile uint32_t DOEPMSK_reserved_0               : 1 ;
        volatile uint32_t OutPktErrMsk                     : 1 ;
        volatile uint32_t BnaOutIntrMsk                    : 1 ;
        volatile uint32_t DOEPMSK_reserved_1               : 2 ;
        volatile uint32_t BbleErrMsk                       : 1 ;
        volatile uint32_t NAKMsk                           : 1 ;
        volatile uint32_t NYETMsk                          : 1 ;
        volatile uint32_t DOEPMSK_reserved_2               : 17;
    }f;
}usbc_REG_DOEPMSK;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t InEpInt0                         : 1 ;
        volatile uint32_t InEpInt1                         : 1 ;
        volatile uint32_t InEpInt2                         : 1 ;
        volatile uint32_t InEpInt3                         : 1 ;
        volatile uint32_t InEpInt4                         : 1 ;
        volatile uint32_t InEpInt5                         : 1 ;
        volatile uint32_t DAINT_reserved_0                 : 10;
        volatile uint32_t OutEPInt0                        : 1 ;
        volatile uint32_t OutEPInt1                        : 1 ;
        volatile uint32_t OutEPInt2                        : 1 ;
        volatile uint32_t OutEPInt3                        : 1 ;
        volatile uint32_t OutEPInt4                        : 1 ;
        volatile uint32_t OutEPInt5                        : 1 ;
        volatile uint32_t DAINT_reserved_1                 : 10;
    }f;
}usbc_REG_DAINT;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t InEpMsk0                         : 1 ;
        volatile uint32_t InEpMsk1                         : 1 ;
        volatile uint32_t InEpMsk2                         : 1 ;
        volatile uint32_t InEpMsk3                         : 1 ;
        volatile uint32_t InEpMsk4                         : 1 ;
        volatile uint32_t InEpMsk5                         : 1 ;
        volatile uint32_t DAINTMSK_reserved_0              : 10;
        volatile uint32_t OutEPMsk0                        : 1 ;
        volatile uint32_t OutEPMsk1                        : 1 ;
        volatile uint32_t OutEPMsk2                        : 1 ;
        volatile uint32_t OutEPMsk3                        : 1 ;
        volatile uint32_t OutEPMsk4                        : 1 ;
        volatile uint32_t OutEPMsk5                        : 1 ;
        volatile uint32_t DAINTMSK_reserved_1              : 10;
    }f;
}usbc_REG_DAINTMSK;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DVBUSDis                         : 16;
        volatile uint32_t DVBUSDIS_reserved_0              : 16;
    }f;
}usbc_REG_DVBUSDIS;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DVBUSPulse                       : 12;
        volatile uint32_t DVBUSPULSE_reserved_0            : 20;
    }f;
}usbc_REG_DVBUSPULSE;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t NonISOThrEn                      : 1 ;
        volatile uint32_t ISOThrEn                         : 1 ;
        volatile uint32_t TxThrLen                         : 9 ;
        volatile uint32_t AHBThrRatio                      : 2 ;
        volatile uint32_t DTHRCTL_reserved_0               : 3 ;
        volatile uint32_t RxThrEn                          : 1 ;
        volatile uint32_t RxThrLen                         : 9 ;
        volatile uint32_t DTHRCTL_reserved_1               : 1 ;
        volatile uint32_t ArbPrkEn                         : 1 ;
        volatile uint32_t DTHRCTL_reserved_2               : 4 ;
    }f;
}usbc_REG_DTHRCTL;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t InEpTxfEmpMsk                    : 16;
        volatile uint32_t DIEPEMPMSK_reserved_0            : 16;
    }f;
}usbc_REG_DIEPEMPMSK;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DIEPCTL0_MPS                     : 2 ;
        volatile uint32_t DIEPCTL0_reserved_0              : 9 ;
        volatile uint32_t NEXTEP                           : 4 ;
        volatile uint32_t USBActEP                         : 1 ;
        volatile uint32_t DIEPCTL0_reserved_1              : 1 ;
        volatile uint32_t NAKSts                           : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t DIEPCTL0_reserved_2              : 1 ;
        volatile uint32_t Stall                            : 1 ;
        volatile uint32_t TxFNum                           : 4 ;
        volatile uint32_t CNAK                             : 1 ;
        volatile uint32_t SNAK                             : 1 ;
        volatile uint32_t DIEPCTL0_reserved_3              : 2 ;
        volatile uint32_t EPDis                            : 1 ;
        volatile uint32_t EPEna                            : 1 ;
    }f;
}usbc_REG_DIEPCTL0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t EPDisbld                         : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t TimeOUT                          : 1 ;
        volatile uint32_t INTknTXFEmp                      : 1 ;
        volatile uint32_t INTknEPMis                       : 1 ;
        volatile uint32_t INEPNakEff                       : 1 ;
        volatile uint32_t TxFEmp                           : 1 ;
        volatile uint32_t TxfifoUndrn                      : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t DIEPINT0_reserved_0              : 1 ;
        volatile uint32_t PktDrpSts                        : 1 ;
        volatile uint32_t BbleErr                          : 1 ;
        volatile uint32_t NAKIntrpt                        : 1 ;
        volatile uint32_t NYETIntrpt                       : 1 ;
        volatile uint32_t DIEPINT0_reserved_1              : 17;
    }f;
}usbc_REG_DIEPINT0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 7 ;
        volatile uint32_t DIEPTSIZ0_reserved_0             : 12;
        volatile uint32_t PktCnt                           : 2 ;
        volatile uint32_t DIEPTSIZ0_reserved_1             : 11;
    }f;
}usbc_REG_DIEPTSIZ0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_DIEPDMA0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t INEPTxFSpcAvail                  : 16;
        volatile uint32_t DTXFSTS0_reserved_0              : 16;
    }f;
}usbc_REG_DTXFSTS0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMABufferAddr                    : 32;
    }f;
}usbc_REG_DIEPDMAB0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t DIEPCTL1_reserved_0              : 4 ;
        volatile uint32_t USBActEP                         : 1 ;
        volatile uint32_t DPID                             : 1 ;
        volatile uint32_t NAKSts                           : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t DIEPCTL1_reserved_1              : 1 ;
        volatile uint32_t Stall                            : 1 ;
        volatile uint32_t TxFNum                           : 4 ;
        volatile uint32_t CNAK                             : 1 ;
        volatile uint32_t SNAK                             : 1 ;
        volatile uint32_t SetD0PID                         : 1 ;
        volatile uint32_t SetD1PID                         : 1 ;
        volatile uint32_t EPDis                            : 1 ;
        volatile uint32_t EPEna                            : 1 ;
    }f;
}usbc_REG_DIEPCTL1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t EPDisbld                         : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t TimeOUT                          : 1 ;
        volatile uint32_t INTknTXFEmp                      : 1 ;
        volatile uint32_t INTknEPMis                       : 1 ;
        volatile uint32_t INEPNakEff                       : 1 ;
        volatile uint32_t TxFEmp                           : 1 ;
        volatile uint32_t TxfifoUndrn                      : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t DIEPINT1_reserved_0              : 1 ;
        volatile uint32_t PktDrpSts                        : 1 ;
        volatile uint32_t BbleErr                          : 1 ;
        volatile uint32_t NAKIntrpt                        : 1 ;
        volatile uint32_t NYETIntrpt                       : 1 ;
        volatile uint32_t DIEPINT1_reserved_1              : 17;
    }f;
}usbc_REG_DIEPINT1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t MC                               : 2 ;
        volatile uint32_t DIEPTSIZ1_reserved_0             : 1 ;
    }f;
}usbc_REG_DIEPTSIZ1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_DIEPDMA1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t INEPTxFSpcAvail                  : 16;
        volatile uint32_t DTXFSTS1_reserved_0              : 16;
    }f;
}usbc_REG_DTXFSTS1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMABufferAddr                    : 32;
    }f;
}usbc_REG_DIEPDMAB1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t DIEPCTL2_reserved_0              : 4 ;
        volatile uint32_t USBActEP                         : 1 ;
        volatile uint32_t DPID                             : 1 ;
        volatile uint32_t NAKSts                           : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t DIEPCTL2_reserved_1              : 1 ;
        volatile uint32_t Stall                            : 1 ;
        volatile uint32_t TxFNum                           : 4 ;
        volatile uint32_t CNAK                             : 1 ;
        volatile uint32_t SNAK                             : 1 ;
        volatile uint32_t SetD0PID                         : 1 ;
        volatile uint32_t SetD1PID                         : 1 ;
        volatile uint32_t EPDis                            : 1 ;
        volatile uint32_t EPEna                            : 1 ;
    }f;
}usbc_REG_DIEPCTL2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t EPDisbld                         : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t TimeOUT                          : 1 ;
        volatile uint32_t INTknTXFEmp                      : 1 ;
        volatile uint32_t INTknEPMis                       : 1 ;
        volatile uint32_t INEPNakEff                       : 1 ;
        volatile uint32_t TxFEmp                           : 1 ;
        volatile uint32_t TxfifoUndrn                      : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t DIEPINT2_reserved_0              : 1 ;
        volatile uint32_t PktDrpSts                        : 1 ;
        volatile uint32_t BbleErr                          : 1 ;
        volatile uint32_t NAKIntrpt                        : 1 ;
        volatile uint32_t NYETIntrpt                       : 1 ;
        volatile uint32_t DIEPINT2_reserved_1              : 17;
    }f;
}usbc_REG_DIEPINT2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t MC                               : 2 ;
        volatile uint32_t DIEPTSIZ2_reserved_0             : 1 ;
    }f;
}usbc_REG_DIEPTSIZ2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_DIEPDMA2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t INEPTxFSpcAvail                  : 16;
        volatile uint32_t DTXFSTS2_reserved_0              : 16;
    }f;
}usbc_REG_DTXFSTS2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMABufferAddr                    : 32;
    }f;
}usbc_REG_DIEPDMAB2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t DIEPCTL3_reserved_0              : 4 ;
        volatile uint32_t USBActEP                         : 1 ;
        volatile uint32_t DPID                             : 1 ;
        volatile uint32_t NAKSts                           : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t DIEPCTL3_reserved_1              : 1 ;
        volatile uint32_t Stall                            : 1 ;
        volatile uint32_t TxFNum                           : 4 ;
        volatile uint32_t CNAK                             : 1 ;
        volatile uint32_t SNAK                             : 1 ;
        volatile uint32_t SetD0PID                         : 1 ;
        volatile uint32_t SetD1PID                         : 1 ;
        volatile uint32_t EPDis                            : 1 ;
        volatile uint32_t EPEna                            : 1 ;
    }f;
}usbc_REG_DIEPCTL3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t EPDisbld                         : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t TimeOUT                          : 1 ;
        volatile uint32_t INTknTXFEmp                      : 1 ;
        volatile uint32_t INTknEPMis                       : 1 ;
        volatile uint32_t INEPNakEff                       : 1 ;
        volatile uint32_t TxFEmp                           : 1 ;
        volatile uint32_t TxfifoUndrn                      : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t DIEPINT3_reserved_0              : 1 ;
        volatile uint32_t PktDrpSts                        : 1 ;
        volatile uint32_t BbleErr                          : 1 ;
        volatile uint32_t NAKIntrpt                        : 1 ;
        volatile uint32_t NYETIntrpt                       : 1 ;
        volatile uint32_t DIEPINT3_reserved_1              : 17;
    }f;
}usbc_REG_DIEPINT3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t MC                               : 2 ;
        volatile uint32_t DIEPTSIZ3_reserved_0             : 1 ;
    }f;
}usbc_REG_DIEPTSIZ3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_DIEPDMA3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t INEPTxFSpcAvail                  : 16;
        volatile uint32_t DTXFSTS3_reserved_0              : 16;
    }f;
}usbc_REG_DTXFSTS3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMABufferAddr                    : 32;
    }f;
}usbc_REG_DIEPDMAB3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t DIEPCTL4_reserved_0              : 4 ;
        volatile uint32_t USBActEP                         : 1 ;
        volatile uint32_t DPID                             : 1 ;
        volatile uint32_t NAKSts                           : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t DIEPCTL4_reserved_1              : 1 ;
        volatile uint32_t Stall                            : 1 ;
        volatile uint32_t TxFNum                           : 4 ;
        volatile uint32_t CNAK                             : 1 ;
        volatile uint32_t SNAK                             : 1 ;
        volatile uint32_t SetD0PID                         : 1 ;
        volatile uint32_t SetD1PID                         : 1 ;
        volatile uint32_t EPDis                            : 1 ;
        volatile uint32_t EPEna                            : 1 ;
    }f;
}usbc_REG_DIEPCTL4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t EPDisbld                         : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t TimeOUT                          : 1 ;
        volatile uint32_t INTknTXFEmp                      : 1 ;
        volatile uint32_t INTknEPMis                       : 1 ;
        volatile uint32_t INEPNakEff                       : 1 ;
        volatile uint32_t TxFEmp                           : 1 ;
        volatile uint32_t TxfifoUndrn                      : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t DIEPINT4_reserved_0              : 1 ;
        volatile uint32_t PktDrpSts                        : 1 ;
        volatile uint32_t BbleErr                          : 1 ;
        volatile uint32_t NAKIntrpt                        : 1 ;
        volatile uint32_t NYETIntrpt                       : 1 ;
        volatile uint32_t DIEPINT4_reserved_1              : 17;
    }f;
}usbc_REG_DIEPINT4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t MC                               : 2 ;
        volatile uint32_t DIEPTSIZ4_reserved_0             : 1 ;
    }f;
}usbc_REG_DIEPTSIZ4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_DIEPDMA4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t INEPTxFSpcAvail                  : 16;
        volatile uint32_t DTXFSTS4_reserved_0              : 16;
    }f;
}usbc_REG_DTXFSTS4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMABufferAddr                    : 32;
    }f;
}usbc_REG_DIEPDMAB4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t DIEPCTL5_reserved_0              : 4 ;
        volatile uint32_t USBActEP                         : 1 ;
        volatile uint32_t DPID                             : 1 ;
        volatile uint32_t NAKSts                           : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t DIEPCTL5_reserved_1              : 1 ;
        volatile uint32_t Stall                            : 1 ;
        volatile uint32_t TxFNum                           : 4 ;
        volatile uint32_t CNAK                             : 1 ;
        volatile uint32_t SNAK                             : 1 ;
        volatile uint32_t SetD0PID                         : 1 ;
        volatile uint32_t SetD1PID                         : 1 ;
        volatile uint32_t EPDis                            : 1 ;
        volatile uint32_t EPEna                            : 1 ;
    }f;
}usbc_REG_DIEPCTL5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t EPDisbld                         : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t TimeOUT                          : 1 ;
        volatile uint32_t INTknTXFEmp                      : 1 ;
        volatile uint32_t INTknEPMis                       : 1 ;
        volatile uint32_t INEPNakEff                       : 1 ;
        volatile uint32_t TxFEmp                           : 1 ;
        volatile uint32_t TxfifoUndrn                      : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t DIEPINT5_reserved_0              : 1 ;
        volatile uint32_t PktDrpSts                        : 1 ;
        volatile uint32_t BbleErr                          : 1 ;
        volatile uint32_t NAKIntrpt                        : 1 ;
        volatile uint32_t NYETIntrpt                       : 1 ;
        volatile uint32_t DIEPINT5_reserved_1              : 17;
    }f;
}usbc_REG_DIEPINT5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t MC                               : 2 ;
        volatile uint32_t DIEPTSIZ5_reserved_0             : 1 ;
    }f;
}usbc_REG_DIEPTSIZ5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_DIEPDMA5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t INEPTxFSpcAvail                  : 16;
        volatile uint32_t DTXFSTS5_reserved_0              : 16;
    }f;
}usbc_REG_DTXFSTS5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMABufferAddr                    : 32;
    }f;
}usbc_REG_DIEPDMAB5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 2 ;
        volatile uint32_t DOEPCTL0_reserved_0              : 13;
        volatile uint32_t USBActEP                         : 1 ;
        volatile uint32_t DOEPCTL0_reserved_1              : 1 ;
        volatile uint32_t NAKSts                           : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t Snp                              : 1 ;
        volatile uint32_t Stall                            : 1 ;
        volatile uint32_t DOEPCTL0_reserved_2              : 4 ;
        volatile uint32_t CNAK                             : 1 ;
        volatile uint32_t SNAK                             : 1 ;
        volatile uint32_t DOEPCTL0_reserved_3              : 2 ;
        volatile uint32_t EPDis                            : 1 ;
        volatile uint32_t EPEna                            : 1 ;
    }f;
}usbc_REG_DOEPCTL0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t EPDisbld                         : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t SetUp                            : 1 ;
        volatile uint32_t OUTTknEPdis                      : 1 ;
        volatile uint32_t StsPhseRcvd                      : 1 ;
        volatile uint32_t Back2BackSETup                   : 1 ;
        volatile uint32_t DOEPINT0_reserved_0              : 1 ;
        volatile uint32_t OutPktErr                        : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t DOEPINT0_reserved_1              : 1 ;
        volatile uint32_t PktDrpSts                        : 1 ;
        volatile uint32_t BbleErr                          : 1 ;
        volatile uint32_t NAKIntrpt                        : 1 ;
        volatile uint32_t NYETIntrpt                       : 1 ;
        volatile uint32_t StupPktRcvd                      : 1 ;
        volatile uint32_t DOEPINT0_reserved_2              : 16;
    }f;
}usbc_REG_DOEPINT0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 7 ;
        volatile uint32_t DOEPTSIZ0_reserved_0             : 12;
        volatile uint32_t PktCnt                           : 1 ;
        volatile uint32_t DOEPTSIZ0_reserved_1             : 9 ;
        volatile uint32_t SUPCnt                           : 2 ;
        volatile uint32_t DOEPTSIZ0_reserved_2             : 1 ;
    }f;
}usbc_REG_DOEPTSIZ0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_DOEPDMA0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMABufferAddr                    : 32;
    }f;
}usbc_REG_DOEPDMAB0;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t DOEPCTL1_reserved_0              : 4 ;
        volatile uint32_t USBActEP                         : 1 ;
        volatile uint32_t DPID                             : 1 ;
        volatile uint32_t NAKSts                           : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t Snp                              : 1 ;
        volatile uint32_t Stall                            : 1 ;
        volatile uint32_t DOEPCTL1_reserved_1              : 4 ;
        volatile uint32_t CNAK                             : 1 ;
        volatile uint32_t SNAK                             : 1 ;
        volatile uint32_t SetD0PID                         : 1 ;
        volatile uint32_t SetD1PID                         : 1 ;
        volatile uint32_t EPDis                            : 1 ;
        volatile uint32_t EPEna                            : 1 ;
    }f;
}usbc_REG_DOEPCTL1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t EPDisbld                         : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t SetUp                            : 1 ;
        volatile uint32_t OUTTknEPdis                      : 1 ;
        volatile uint32_t StsPhseRcvd                      : 1 ;
        volatile uint32_t Back2BackSETup                   : 1 ;
        volatile uint32_t DOEPINT1_reserved_0              : 1 ;
        volatile uint32_t OutPktErr                        : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t DOEPINT1_reserved_1              : 1 ;
        volatile uint32_t PktDrpSts                        : 1 ;
        volatile uint32_t BbleErr                          : 1 ;
        volatile uint32_t NAKIntrpt                        : 1 ;
        volatile uint32_t NYETIntrpt                       : 1 ;
        volatile uint32_t StupPktRcvd                      : 1 ;
        volatile uint32_t DOEPINT1_reserved_2              : 16;
    }f;
}usbc_REG_DOEPINT1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t RxDPID                           : 2 ;
        volatile uint32_t DOEPTSIZ1_reserved_0             : 1 ;
    }f;
}usbc_REG_DOEPTSIZ1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_DOEPDMA1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMABufferAddr                    : 32;
    }f;
}usbc_REG_DOEPDMAB1;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t DOEPCTL2_reserved_0              : 4 ;
        volatile uint32_t USBActEP                         : 1 ;
        volatile uint32_t DPID                             : 1 ;
        volatile uint32_t NAKSts                           : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t Snp                              : 1 ;
        volatile uint32_t Stall                            : 1 ;
        volatile uint32_t DOEPCTL2_reserved_1              : 4 ;
        volatile uint32_t CNAK                             : 1 ;
        volatile uint32_t SNAK                             : 1 ;
        volatile uint32_t SetD0PID                         : 1 ;
        volatile uint32_t SetD1PID                         : 1 ;
        volatile uint32_t EPDis                            : 1 ;
        volatile uint32_t EPEna                            : 1 ;
    }f;
}usbc_REG_DOEPCTL2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t EPDisbld                         : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t SetUp                            : 1 ;
        volatile uint32_t OUTTknEPdis                      : 1 ;
        volatile uint32_t StsPhseRcvd                      : 1 ;
        volatile uint32_t Back2BackSETup                   : 1 ;
        volatile uint32_t DOEPINT2_reserved_0              : 1 ;
        volatile uint32_t OutPktErr                        : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t DOEPINT2_reserved_1              : 1 ;
        volatile uint32_t PktDrpSts                        : 1 ;
        volatile uint32_t BbleErr                          : 1 ;
        volatile uint32_t NAKIntrpt                        : 1 ;
        volatile uint32_t NYETIntrpt                       : 1 ;
        volatile uint32_t StupPktRcvd                      : 1 ;
        volatile uint32_t DOEPINT2_reserved_2              : 16;
    }f;
}usbc_REG_DOEPINT2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t RxDPID                           : 2 ;
        volatile uint32_t DOEPTSIZ2_reserved_0             : 1 ;
    }f;
}usbc_REG_DOEPTSIZ2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_DOEPDMA2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMABufferAddr                    : 32;
    }f;
}usbc_REG_DOEPDMAB2;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t DOEPCTL3_reserved_0              : 4 ;
        volatile uint32_t USBActEP                         : 1 ;
        volatile uint32_t DPID                             : 1 ;
        volatile uint32_t NAKSts                           : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t Snp                              : 1 ;
        volatile uint32_t Stall                            : 1 ;
        volatile uint32_t DOEPCTL3_reserved_1              : 4 ;
        volatile uint32_t CNAK                             : 1 ;
        volatile uint32_t SNAK                             : 1 ;
        volatile uint32_t SetD0PID                         : 1 ;
        volatile uint32_t SetD1PID                         : 1 ;
        volatile uint32_t EPDis                            : 1 ;
        volatile uint32_t EPEna                            : 1 ;
    }f;
}usbc_REG_DOEPCTL3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t EPDisbld                         : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t SetUp                            : 1 ;
        volatile uint32_t OUTTknEPdis                      : 1 ;
        volatile uint32_t StsPhseRcvd                      : 1 ;
        volatile uint32_t Back2BackSETup                   : 1 ;
        volatile uint32_t DOEPINT3_reserved_0              : 1 ;
        volatile uint32_t OutPktErr                        : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t DOEPINT3_reserved_1              : 1 ;
        volatile uint32_t PktDrpSts                        : 1 ;
        volatile uint32_t BbleErr                          : 1 ;
        volatile uint32_t NAKIntrpt                        : 1 ;
        volatile uint32_t NYETIntrpt                       : 1 ;
        volatile uint32_t StupPktRcvd                      : 1 ;
        volatile uint32_t DOEPINT3_reserved_2              : 16;
    }f;
}usbc_REG_DOEPINT3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t RxDPID                           : 2 ;
        volatile uint32_t DOEPTSIZ3_reserved_0             : 1 ;
    }f;
}usbc_REG_DOEPTSIZ3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_DOEPDMA3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMABufferAddr                    : 32;
    }f;
}usbc_REG_DOEPDMAB3;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t DOEPCTL4_reserved_0              : 4 ;
        volatile uint32_t USBActEP                         : 1 ;
        volatile uint32_t DPID                             : 1 ;
        volatile uint32_t NAKSts                           : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t Snp                              : 1 ;
        volatile uint32_t Stall                            : 1 ;
        volatile uint32_t DOEPCTL4_reserved_1              : 4 ;
        volatile uint32_t CNAK                             : 1 ;
        volatile uint32_t SNAK                             : 1 ;
        volatile uint32_t SetD0PID                         : 1 ;
        volatile uint32_t SetD1PID                         : 1 ;
        volatile uint32_t EPDis                            : 1 ;
        volatile uint32_t EPEna                            : 1 ;
    }f;
}usbc_REG_DOEPCTL4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t EPDisbld                         : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t SetUp                            : 1 ;
        volatile uint32_t OUTTknEPdis                      : 1 ;
        volatile uint32_t StsPhseRcvd                      : 1 ;
        volatile uint32_t Back2BackSETup                   : 1 ;
        volatile uint32_t DOEPINT4_reserved_0              : 1 ;
        volatile uint32_t OutPktErr                        : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t DOEPINT4_reserved_1              : 1 ;
        volatile uint32_t PktDrpSts                        : 1 ;
        volatile uint32_t BbleErr                          : 1 ;
        volatile uint32_t NAKIntrpt                        : 1 ;
        volatile uint32_t NYETIntrpt                       : 1 ;
        volatile uint32_t StupPktRcvd                      : 1 ;
        volatile uint32_t DOEPINT4_reserved_2              : 16;
    }f;
}usbc_REG_DOEPINT4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t RxDPID                           : 2 ;
        volatile uint32_t DOEPTSIZ4_reserved_0             : 1 ;
    }f;
}usbc_REG_DOEPTSIZ4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_DOEPDMA4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMABufferAddr                    : 32;
    }f;
}usbc_REG_DOEPDMAB4;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t MPS                              : 11;
        volatile uint32_t DOEPCTL5_reserved_0              : 4 ;
        volatile uint32_t USBActEP                         : 1 ;
        volatile uint32_t DPID                             : 1 ;
        volatile uint32_t NAKSts                           : 1 ;
        volatile uint32_t EPType                           : 2 ;
        volatile uint32_t Snp                              : 1 ;
        volatile uint32_t Stall                            : 1 ;
        volatile uint32_t DOEPCTL5_reserved_1              : 4 ;
        volatile uint32_t CNAK                             : 1 ;
        volatile uint32_t SNAK                             : 1 ;
        volatile uint32_t SetD0PID                         : 1 ;
        volatile uint32_t SetD1PID                         : 1 ;
        volatile uint32_t EPDis                            : 1 ;
        volatile uint32_t EPEna                            : 1 ;
    }f;
}usbc_REG_DOEPCTL5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferCompl                        : 1 ;
        volatile uint32_t EPDisbld                         : 1 ;
        volatile uint32_t AHBErr                           : 1 ;
        volatile uint32_t SetUp                            : 1 ;
        volatile uint32_t OUTTknEPdis                      : 1 ;
        volatile uint32_t StsPhseRcvd                      : 1 ;
        volatile uint32_t Back2BackSETup                   : 1 ;
        volatile uint32_t DOEPINT5_reserved_0              : 1 ;
        volatile uint32_t OutPktErr                        : 1 ;
        volatile uint32_t BNAIntr                          : 1 ;
        volatile uint32_t DOEPINT5_reserved_1              : 1 ;
        volatile uint32_t PktDrpSts                        : 1 ;
        volatile uint32_t BbleErr                          : 1 ;
        volatile uint32_t NAKIntrpt                        : 1 ;
        volatile uint32_t NYETIntrpt                       : 1 ;
        volatile uint32_t StupPktRcvd                      : 1 ;
        volatile uint32_t DOEPINT5_reserved_2              : 16;
    }f;
}usbc_REG_DOEPINT5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t XferSize                         : 19;
        volatile uint32_t PktCnt                           : 10;
        volatile uint32_t RxDPID                           : 2 ;
        volatile uint32_t DOEPTSIZ5_reserved_0             : 1 ;
    }f;
}usbc_REG_DOEPTSIZ5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMAAddr                          : 32;
    }f;
}usbc_REG_DOEPDMA5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t DMABufferAddr                    : 32;
    }f;
}usbc_REG_DOEPDMAB5;

typedef union
{
    volatile uint32_t r;
    struct
    {
        volatile uint32_t StopPclk                         : 1 ;
        volatile uint32_t PCGCCTL_reserved_0               : 2 ;
        volatile uint32_t RstPdwnModule                    : 1 ;
        volatile uint32_t PCGCCTL_reserved_1               : 2 ;
        volatile uint32_t PhySleep                         : 1 ;
        volatile uint32_t L1Suspended                      : 1 ;
        volatile uint32_t PCGCCTL_reserved_2               : 24;
    }f;
}usbc_REG_PCGCCTL;

typedef struct
{
volatile usbc_REG_GOTGCTL                                            GOTGCTL                       ; //0x0
volatile usbc_REG_GOTGINT                                            GOTGINT                       ; //0x4
volatile usbc_REG_GAHBCFG                                            GAHBCFG                       ; //0x8
volatile usbc_REG_GUSBCFG                                            GUSBCFG                       ; //0xC
volatile usbc_REG_GRSTCTL                                            GRSTCTL                       ; //0x10
volatile usbc_REG_GINTSTS                                            GINTSTS                       ; //0x14
volatile usbc_REG_GINTMSK                                            GINTMSK                       ; //0x18
volatile usbc_REG_GRXSTSR                                            GRXSTSR                       ; //0x1C
volatile usbc_REG_GRXSTSP                                            GRXSTSP                       ; //0x20
volatile usbc_REG_GRXFSIZ                                            GRXFSIZ                       ; //0x24
volatile usbc_REG_GNPTXFSIZ                                          GNPTXFSIZ                     ; //0x28
volatile usbc_REG_GNPTXSTS                                           GNPTXSTS                      ; //0x2C
volatile uint32_t                                                    null_reg0                     ; //0x30
volatile uint32_t                                                    null_reg1                     ; //0x34
volatile usbc_REG_GGPIO                                              GGPIO                         ; //0x38
volatile usbc_REG_GUID                                               GUID                          ; //0x3C
volatile usbc_REG_GSNPSID                                            GSNPSID                       ; //0x40
volatile usbc_REG_GHWCFG1                                            GHWCFG1                       ; //0x44
volatile usbc_REG_GHWCFG2                                            GHWCFG2                       ; //0x48
volatile usbc_REG_GHWCFG3                                            GHWCFG3                       ; //0x4C
volatile usbc_REG_GHWCFG4                                            GHWCFG4                       ; //0x50
volatile uint32_t                                                    null_reg2                     ; //0x54
volatile uint32_t                                                    null_reg3                     ; //0x58
volatile usbc_REG_GDFIFOCFG                                          GDFIFOCFG                     ; //0x5C
volatile uint32_t                                                    null_reg4                     ; //0x60
volatile uint32_t                                                    null_reg5                     ; //0x64
volatile uint32_t                                                    null_reg6                     ; //0x68
volatile uint32_t                                                    null_reg7                     ; //0x6C
volatile uint32_t                                                    null_reg8                     ; //0x70
volatile uint32_t                                                    null_reg9                     ; //0x74
volatile uint32_t                                                    null_reg10                    ; //0x78
volatile uint32_t                                                    null_reg11                    ; //0x7C
volatile uint32_t                                                    null_reg12                    ; //0x80
volatile uint32_t                                                    null_reg13                    ; //0x84
volatile uint32_t                                                    null_reg14                    ; //0x88
volatile uint32_t                                                    null_reg15                    ; //0x8C
volatile uint32_t                                                    null_reg16                    ; //0x90
volatile uint32_t                                                    null_reg17                    ; //0x94
volatile uint32_t                                                    null_reg18                    ; //0x98
volatile uint32_t                                                    null_reg19                    ; //0x9C
volatile uint32_t                                                    null_reg20                    ; //0xA0
volatile uint32_t                                                    null_reg21                    ; //0xA4
volatile uint32_t                                                    null_reg22                    ; //0xA8
volatile uint32_t                                                    null_reg23                    ; //0xAC
volatile uint32_t                                                    null_reg24                    ; //0xB0
volatile uint32_t                                                    null_reg25                    ; //0xB4
volatile uint32_t                                                    null_reg26                    ; //0xB8
volatile uint32_t                                                    null_reg27                    ; //0xBC
volatile uint32_t                                                    null_reg28                    ; //0xC0
volatile uint32_t                                                    null_reg29                    ; //0xC4
volatile uint32_t                                                    null_reg30                    ; //0xC8
volatile uint32_t                                                    null_reg31                    ; //0xCC
volatile uint32_t                                                    null_reg32                    ; //0xD0
volatile uint32_t                                                    null_reg33                    ; //0xD4
volatile uint32_t                                                    null_reg34                    ; //0xD8
volatile uint32_t                                                    null_reg35                    ; //0xDC
volatile uint32_t                                                    null_reg36                    ; //0xE0
volatile uint32_t                                                    null_reg37                    ; //0xE4
volatile uint32_t                                                    null_reg38                    ; //0xE8
volatile uint32_t                                                    null_reg39                    ; //0xEC
volatile uint32_t                                                    null_reg40                    ; //0xF0
volatile uint32_t                                                    null_reg41                    ; //0xF4
volatile uint32_t                                                    null_reg42                    ; //0xF8
volatile uint32_t                                                    null_reg43                    ; //0xFC
volatile usbc_REG_HPTXFSIZ                                           HPTXFSIZ                      ; //0x100
volatile usbc_REG_DIEPTXF1                                           DIEPTXF1                      ; //0x104
volatile usbc_REG_DIEPTXF2                                           DIEPTXF2                      ; //0x108
volatile usbc_REG_DIEPTXF3                                           DIEPTXF3                      ; //0x10C
volatile usbc_REG_DIEPTXF4                                           DIEPTXF4                      ; //0x110
volatile usbc_REG_DIEPTXF5                                           DIEPTXF5                      ; //0x114
volatile uint32_t                                                    null_reg44                    ; //0x118
volatile uint32_t                                                    null_reg45                    ; //0x11C
volatile uint32_t                                                    null_reg46                    ; //0x120
volatile uint32_t                                                    null_reg47                    ; //0x124
volatile uint32_t                                                    null_reg48                    ; //0x128
volatile uint32_t                                                    null_reg49                    ; //0x12C
volatile uint32_t                                                    null_reg50                    ; //0x130
volatile uint32_t                                                    null_reg51                    ; //0x134
volatile uint32_t                                                    null_reg52                    ; //0x138
volatile uint32_t                                                    null_reg53                    ; //0x13C
volatile uint32_t                                                    null_reg54                    ; //0x140
volatile uint32_t                                                    null_reg55                    ; //0x144
volatile uint32_t                                                    null_reg56                    ; //0x148
volatile uint32_t                                                    null_reg57                    ; //0x14C
volatile uint32_t                                                    null_reg58                    ; //0x150
volatile uint32_t                                                    null_reg59                    ; //0x154
volatile uint32_t                                                    null_reg60                    ; //0x158
volatile uint32_t                                                    null_reg61                    ; //0x15C
volatile uint32_t                                                    null_reg62                    ; //0x160
volatile uint32_t                                                    null_reg63                    ; //0x164
volatile uint32_t                                                    null_reg64                    ; //0x168
volatile uint32_t                                                    null_reg65                    ; //0x16C
volatile uint32_t                                                    null_reg66                    ; //0x170
volatile uint32_t                                                    null_reg67                    ; //0x174
volatile uint32_t                                                    null_reg68                    ; //0x178
volatile uint32_t                                                    null_reg69                    ; //0x17C
volatile uint32_t                                                    null_reg70                    ; //0x180
volatile uint32_t                                                    null_reg71                    ; //0x184
volatile uint32_t                                                    null_reg72                    ; //0x188
volatile uint32_t                                                    null_reg73                    ; //0x18C
volatile uint32_t                                                    null_reg74                    ; //0x190
volatile uint32_t                                                    null_reg75                    ; //0x194
volatile uint32_t                                                    null_reg76                    ; //0x198
volatile uint32_t                                                    null_reg77                    ; //0x19C
volatile uint32_t                                                    null_reg78                    ; //0x1A0
volatile uint32_t                                                    null_reg79                    ; //0x1A4
volatile uint32_t                                                    null_reg80                    ; //0x1A8
volatile uint32_t                                                    null_reg81                    ; //0x1AC
volatile uint32_t                                                    null_reg82                    ; //0x1B0
volatile uint32_t                                                    null_reg83                    ; //0x1B4
volatile uint32_t                                                    null_reg84                    ; //0x1B8
volatile uint32_t                                                    null_reg85                    ; //0x1BC
volatile uint32_t                                                    null_reg86                    ; //0x1C0
volatile uint32_t                                                    null_reg87                    ; //0x1C4
volatile uint32_t                                                    null_reg88                    ; //0x1C8
volatile uint32_t                                                    null_reg89                    ; //0x1CC
volatile uint32_t                                                    null_reg90                    ; //0x1D0
volatile uint32_t                                                    null_reg91                    ; //0x1D4
volatile uint32_t                                                    null_reg92                    ; //0x1D8
volatile uint32_t                                                    null_reg93                    ; //0x1DC
volatile uint32_t                                                    null_reg94                    ; //0x1E0
volatile uint32_t                                                    null_reg95                    ; //0x1E4
volatile uint32_t                                                    null_reg96                    ; //0x1E8
volatile uint32_t                                                    null_reg97                    ; //0x1EC
volatile uint32_t                                                    null_reg98                    ; //0x1F0
volatile uint32_t                                                    null_reg99                    ; //0x1F4
volatile uint32_t                                                    null_reg100                   ; //0x1F8
volatile uint32_t                                                    null_reg101                   ; //0x1FC
volatile uint32_t                                                    null_reg102                   ; //0x200
volatile uint32_t                                                    null_reg103                   ; //0x204
volatile uint32_t                                                    null_reg104                   ; //0x208
volatile uint32_t                                                    null_reg105                   ; //0x20C
volatile uint32_t                                                    null_reg106                   ; //0x210
volatile uint32_t                                                    null_reg107                   ; //0x214
volatile uint32_t                                                    null_reg108                   ; //0x218
volatile uint32_t                                                    null_reg109                   ; //0x21C
volatile uint32_t                                                    null_reg110                   ; //0x220
volatile uint32_t                                                    null_reg111                   ; //0x224
volatile uint32_t                                                    null_reg112                   ; //0x228
volatile uint32_t                                                    null_reg113                   ; //0x22C
volatile uint32_t                                                    null_reg114                   ; //0x230
volatile uint32_t                                                    null_reg115                   ; //0x234
volatile uint32_t                                                    null_reg116                   ; //0x238
volatile uint32_t                                                    null_reg117                   ; //0x23C
volatile uint32_t                                                    null_reg118                   ; //0x240
volatile uint32_t                                                    null_reg119                   ; //0x244
volatile uint32_t                                                    null_reg120                   ; //0x248
volatile uint32_t                                                    null_reg121                   ; //0x24C
volatile uint32_t                                                    null_reg122                   ; //0x250
volatile uint32_t                                                    null_reg123                   ; //0x254
volatile uint32_t                                                    null_reg124                   ; //0x258
volatile uint32_t                                                    null_reg125                   ; //0x25C
volatile uint32_t                                                    null_reg126                   ; //0x260
volatile uint32_t                                                    null_reg127                   ; //0x264
volatile uint32_t                                                    null_reg128                   ; //0x268
volatile uint32_t                                                    null_reg129                   ; //0x26C
volatile uint32_t                                                    null_reg130                   ; //0x270
volatile uint32_t                                                    null_reg131                   ; //0x274
volatile uint32_t                                                    null_reg132                   ; //0x278
volatile uint32_t                                                    null_reg133                   ; //0x27C
volatile uint32_t                                                    null_reg134                   ; //0x280
volatile uint32_t                                                    null_reg135                   ; //0x284
volatile uint32_t                                                    null_reg136                   ; //0x288
volatile uint32_t                                                    null_reg137                   ; //0x28C
volatile uint32_t                                                    null_reg138                   ; //0x290
volatile uint32_t                                                    null_reg139                   ; //0x294
volatile uint32_t                                                    null_reg140                   ; //0x298
volatile uint32_t                                                    null_reg141                   ; //0x29C
volatile uint32_t                                                    null_reg142                   ; //0x2A0
volatile uint32_t                                                    null_reg143                   ; //0x2A4
volatile uint32_t                                                    null_reg144                   ; //0x2A8
volatile uint32_t                                                    null_reg145                   ; //0x2AC
volatile uint32_t                                                    null_reg146                   ; //0x2B0
volatile uint32_t                                                    null_reg147                   ; //0x2B4
volatile uint32_t                                                    null_reg148                   ; //0x2B8
volatile uint32_t                                                    null_reg149                   ; //0x2BC
volatile uint32_t                                                    null_reg150                   ; //0x2C0
volatile uint32_t                                                    null_reg151                   ; //0x2C4
volatile uint32_t                                                    null_reg152                   ; //0x2C8
volatile uint32_t                                                    null_reg153                   ; //0x2CC
volatile uint32_t                                                    null_reg154                   ; //0x2D0
volatile uint32_t                                                    null_reg155                   ; //0x2D4
volatile uint32_t                                                    null_reg156                   ; //0x2D8
volatile uint32_t                                                    null_reg157                   ; //0x2DC
volatile uint32_t                                                    null_reg158                   ; //0x2E0
volatile uint32_t                                                    null_reg159                   ; //0x2E4
volatile uint32_t                                                    null_reg160                   ; //0x2E8
volatile uint32_t                                                    null_reg161                   ; //0x2EC
volatile uint32_t                                                    null_reg162                   ; //0x2F0
volatile uint32_t                                                    null_reg163                   ; //0x2F4
volatile uint32_t                                                    null_reg164                   ; //0x2F8
volatile uint32_t                                                    null_reg165                   ; //0x2FC
volatile uint32_t                                                    null_reg166                   ; //0x300
volatile uint32_t                                                    null_reg167                   ; //0x304
volatile uint32_t                                                    null_reg168                   ; //0x308
volatile uint32_t                                                    null_reg169                   ; //0x30C
volatile uint32_t                                                    null_reg170                   ; //0x310
volatile uint32_t                                                    null_reg171                   ; //0x314
volatile uint32_t                                                    null_reg172                   ; //0x318
volatile uint32_t                                                    null_reg173                   ; //0x31C
volatile uint32_t                                                    null_reg174                   ; //0x320
volatile uint32_t                                                    null_reg175                   ; //0x324
volatile uint32_t                                                    null_reg176                   ; //0x328
volatile uint32_t                                                    null_reg177                   ; //0x32C
volatile uint32_t                                                    null_reg178                   ; //0x330
volatile uint32_t                                                    null_reg179                   ; //0x334
volatile uint32_t                                                    null_reg180                   ; //0x338
volatile uint32_t                                                    null_reg181                   ; //0x33C
volatile uint32_t                                                    null_reg182                   ; //0x340
volatile uint32_t                                                    null_reg183                   ; //0x344
volatile uint32_t                                                    null_reg184                   ; //0x348
volatile uint32_t                                                    null_reg185                   ; //0x34C
volatile uint32_t                                                    null_reg186                   ; //0x350
volatile uint32_t                                                    null_reg187                   ; //0x354
volatile uint32_t                                                    null_reg188                   ; //0x358
volatile uint32_t                                                    null_reg189                   ; //0x35C
volatile uint32_t                                                    null_reg190                   ; //0x360
volatile uint32_t                                                    null_reg191                   ; //0x364
volatile uint32_t                                                    null_reg192                   ; //0x368
volatile uint32_t                                                    null_reg193                   ; //0x36C
volatile uint32_t                                                    null_reg194                   ; //0x370
volatile uint32_t                                                    null_reg195                   ; //0x374
volatile uint32_t                                                    null_reg196                   ; //0x378
volatile uint32_t                                                    null_reg197                   ; //0x37C
volatile uint32_t                                                    null_reg198                   ; //0x380
volatile uint32_t                                                    null_reg199                   ; //0x384
volatile uint32_t                                                    null_reg200                   ; //0x388
volatile uint32_t                                                    null_reg201                   ; //0x38C
volatile uint32_t                                                    null_reg202                   ; //0x390
volatile uint32_t                                                    null_reg203                   ; //0x394
volatile uint32_t                                                    null_reg204                   ; //0x398
volatile uint32_t                                                    null_reg205                   ; //0x39C
volatile uint32_t                                                    null_reg206                   ; //0x3A0
volatile uint32_t                                                    null_reg207                   ; //0x3A4
volatile uint32_t                                                    null_reg208                   ; //0x3A8
volatile uint32_t                                                    null_reg209                   ; //0x3AC
volatile uint32_t                                                    null_reg210                   ; //0x3B0
volatile uint32_t                                                    null_reg211                   ; //0x3B4
volatile uint32_t                                                    null_reg212                   ; //0x3B8
volatile uint32_t                                                    null_reg213                   ; //0x3BC
volatile uint32_t                                                    null_reg214                   ; //0x3C0
volatile uint32_t                                                    null_reg215                   ; //0x3C4
volatile uint32_t                                                    null_reg216                   ; //0x3C8
volatile uint32_t                                                    null_reg217                   ; //0x3CC
volatile uint32_t                                                    null_reg218                   ; //0x3D0
volatile uint32_t                                                    null_reg219                   ; //0x3D4
volatile uint32_t                                                    null_reg220                   ; //0x3D8
volatile uint32_t                                                    null_reg221                   ; //0x3DC
volatile uint32_t                                                    null_reg222                   ; //0x3E0
volatile uint32_t                                                    null_reg223                   ; //0x3E4
volatile uint32_t                                                    null_reg224                   ; //0x3E8
volatile uint32_t                                                    null_reg225                   ; //0x3EC
volatile uint32_t                                                    null_reg226                   ; //0x3F0
volatile uint32_t                                                    null_reg227                   ; //0x3F4
volatile uint32_t                                                    null_reg228                   ; //0x3F8
volatile uint32_t                                                    null_reg229                   ; //0x3FC
volatile usbc_REG_HCFG                                               HCFG                          ; //0x400
volatile usbc_REG_HFIR                                               HFIR                          ; //0x404
volatile usbc_REG_HFNUM                                              HFNUM                         ; //0x408
volatile uint32_t                                                    null_reg230                   ; //0x40C
volatile uint32_t                                                    null_reg231                   ; //0x410
volatile usbc_REG_HAINT                                              HAINT                         ; //0x414
volatile usbc_REG_HAINTMSK                                           HAINTMSK                      ; //0x418
volatile usbc_REG_HFLBAddr                                           HFLBAddr                      ; //0x41C
volatile uint32_t                                                    null_reg232                   ; //0x420
volatile uint32_t                                                    null_reg233                   ; //0x424
volatile uint32_t                                                    null_reg234                   ; //0x428
volatile uint32_t                                                    null_reg235                   ; //0x42C
volatile uint32_t                                                    null_reg236                   ; //0x430
volatile uint32_t                                                    null_reg237                   ; //0x434
volatile uint32_t                                                    null_reg238                   ; //0x438
volatile uint32_t                                                    null_reg239                   ; //0x43C
volatile usbc_REG_HPRT                                               HPRT                          ; //0x440
volatile uint32_t                                                    null_reg240                   ; //0x444
volatile uint32_t                                                    null_reg241                   ; //0x448
volatile uint32_t                                                    null_reg242                   ; //0x44C
volatile uint32_t                                                    null_reg243                   ; //0x450
volatile uint32_t                                                    null_reg244                   ; //0x454
volatile uint32_t                                                    null_reg245                   ; //0x458
volatile uint32_t                                                    null_reg246                   ; //0x45C
volatile uint32_t                                                    null_reg247                   ; //0x460
volatile uint32_t                                                    null_reg248                   ; //0x464
volatile uint32_t                                                    null_reg249                   ; //0x468
volatile uint32_t                                                    null_reg250                   ; //0x46C
volatile uint32_t                                                    null_reg251                   ; //0x470
volatile uint32_t                                                    null_reg252                   ; //0x474
volatile uint32_t                                                    null_reg253                   ; //0x478
volatile uint32_t                                                    null_reg254                   ; //0x47C
volatile uint32_t                                                    null_reg255                   ; //0x480
volatile uint32_t                                                    null_reg256                   ; //0x484
volatile uint32_t                                                    null_reg257                   ; //0x488
volatile uint32_t                                                    null_reg258                   ; //0x48C
volatile uint32_t                                                    null_reg259                   ; //0x490
volatile uint32_t                                                    null_reg260                   ; //0x494
volatile uint32_t                                                    null_reg261                   ; //0x498
volatile uint32_t                                                    null_reg262                   ; //0x49C
volatile uint32_t                                                    null_reg263                   ; //0x4A0
volatile uint32_t                                                    null_reg264                   ; //0x4A4
volatile uint32_t                                                    null_reg265                   ; //0x4A8
volatile uint32_t                                                    null_reg266                   ; //0x4AC
volatile uint32_t                                                    null_reg267                   ; //0x4B0
volatile uint32_t                                                    null_reg268                   ; //0x4B4
volatile uint32_t                                                    null_reg269                   ; //0x4B8
volatile uint32_t                                                    null_reg270                   ; //0x4BC
volatile uint32_t                                                    null_reg271                   ; //0x4C0
volatile uint32_t                                                    null_reg272                   ; //0x4C4
volatile uint32_t                                                    null_reg273                   ; //0x4C8
volatile uint32_t                                                    null_reg274                   ; //0x4CC
volatile uint32_t                                                    null_reg275                   ; //0x4D0
volatile uint32_t                                                    null_reg276                   ; //0x4D4
volatile uint32_t                                                    null_reg277                   ; //0x4D8
volatile uint32_t                                                    null_reg278                   ; //0x4DC
volatile uint32_t                                                    null_reg279                   ; //0x4E0
volatile uint32_t                                                    null_reg280                   ; //0x4E4
volatile uint32_t                                                    null_reg281                   ; //0x4E8
volatile uint32_t                                                    null_reg282                   ; //0x4EC
volatile uint32_t                                                    null_reg283                   ; //0x4F0
volatile uint32_t                                                    null_reg284                   ; //0x4F4
volatile uint32_t                                                    null_reg285                   ; //0x4F8
volatile uint32_t                                                    null_reg286                   ; //0x4FC
volatile usbc_REG_HCCHAR0                                            HCCHAR0                       ; //0x500
volatile usbc_REG_HCSPLT0                                            HCSPLT0                       ; //0x504
volatile usbc_REG_HCINT0                                             HCINT0                        ; //0x508
volatile usbc_REG_HCINTMSK0                                          HCINTMSK0                     ; //0x50C
volatile usbc_REG_HCTSIZ0                                            HCTSIZ0                       ; //0x510
volatile usbc_REG_HCDMA0                                             HCDMA0                        ; //0x514
volatile uint32_t                                                    null_reg287                   ; //0x518
volatile usbc_REG_HCDMAB0                                            HCDMAB0                       ; //0x51C
volatile usbc_REG_HCCHAR1                                            HCCHAR1                       ; //0x520
volatile usbc_REG_HCSPLT1                                            HCSPLT1                       ; //0x524
volatile usbc_REG_HCINT1                                             HCINT1                        ; //0x528
volatile usbc_REG_HCINTMSK1                                          HCINTMSK1                     ; //0x52C
volatile usbc_REG_HCTSIZ1                                            HCTSIZ1                       ; //0x530
volatile usbc_REG_HCDMA1                                             HCDMA1                        ; //0x534
volatile uint32_t                                                    null_reg288                   ; //0x538
volatile usbc_REG_HCDMAB1                                            HCDMAB1                       ; //0x53C
volatile usbc_REG_HCCHAR2                                            HCCHAR2                       ; //0x540
volatile usbc_REG_HCSPLT2                                            HCSPLT2                       ; //0x544
volatile usbc_REG_HCINT2                                             HCINT2                        ; //0x548
volatile usbc_REG_HCINTMSK2                                          HCINTMSK2                     ; //0x54C
volatile usbc_REG_HCTSIZ2                                            HCTSIZ2                       ; //0x550
volatile usbc_REG_HCDMA2                                             HCDMA2                        ; //0x554
volatile uint32_t                                                    null_reg289                   ; //0x558
volatile usbc_REG_HCDMAB2                                            HCDMAB2                       ; //0x55C
volatile usbc_REG_HCCHAR3                                            HCCHAR3                       ; //0x560
volatile usbc_REG_HCSPLT3                                            HCSPLT3                       ; //0x564
volatile usbc_REG_HCINT3                                             HCINT3                        ; //0x568
volatile usbc_REG_HCINTMSK3                                          HCINTMSK3                     ; //0x56C
volatile usbc_REG_HCTSIZ3                                            HCTSIZ3                       ; //0x570
volatile usbc_REG_HCDMA3                                             HCDMA3                        ; //0x574
volatile uint32_t                                                    null_reg290                   ; //0x578
volatile usbc_REG_HCDMAB3                                            HCDMAB3                       ; //0x57C
volatile usbc_REG_HCCHAR4                                            HCCHAR4                       ; //0x580
volatile usbc_REG_HCSPLT4                                            HCSPLT4                       ; //0x584
volatile usbc_REG_HCINT4                                             HCINT4                        ; //0x588
volatile usbc_REG_HCINTMSK4                                          HCINTMSK4                     ; //0x58C
volatile usbc_REG_HCTSIZ4                                            HCTSIZ4                       ; //0x590
volatile usbc_REG_HCDMA4                                             HCDMA4                        ; //0x594
volatile uint32_t                                                    null_reg291                   ; //0x598
volatile usbc_REG_HCDMAB4                                            HCDMAB4                       ; //0x59C
volatile usbc_REG_HCCHAR5                                            HCCHAR5                       ; //0x5A0
volatile usbc_REG_HCSPLT5                                            HCSPLT5                       ; //0x5A4
volatile usbc_REG_HCINT5                                             HCINT5                        ; //0x5A8
volatile usbc_REG_HCINTMSK5                                          HCINTMSK5                     ; //0x5AC
volatile usbc_REG_HCTSIZ5                                            HCTSIZ5                       ; //0x5B0
volatile usbc_REG_HCDMA5                                             HCDMA5                        ; //0x5B4
volatile uint32_t                                                    null_reg292                   ; //0x5B8
volatile usbc_REG_HCDMAB5                                            HCDMAB5                       ; //0x5BC
volatile usbc_REG_HCCHAR6                                            HCCHAR6                       ; //0x5C0
volatile usbc_REG_HCSPLT6                                            HCSPLT6                       ; //0x5C4
volatile usbc_REG_HCINT6                                             HCINT6                        ; //0x5C8
volatile usbc_REG_HCINTMSK6                                          HCINTMSK6                     ; //0x5CC
volatile usbc_REG_HCTSIZ6                                            HCTSIZ6                       ; //0x5D0
volatile usbc_REG_HCDMA6                                             HCDMA6                        ; //0x5D4
volatile uint32_t                                                    null_reg293                   ; //0x5D8
volatile usbc_REG_HCDMAB6                                            HCDMAB6                       ; //0x5DC
volatile usbc_REG_HCCHAR7                                            HCCHAR7                       ; //0x5E0
volatile usbc_REG_HCSPLT7                                            HCSPLT7                       ; //0x5E4
volatile usbc_REG_HCINT7                                             HCINT7                        ; //0x5E8
volatile usbc_REG_HCINTMSK7                                          HCINTMSK7                     ; //0x5EC
volatile usbc_REG_HCTSIZ7                                            HCTSIZ7                       ; //0x5F0
volatile usbc_REG_HCDMA7                                             HCDMA7                        ; //0x5F4
volatile uint32_t                                                    null_reg294                   ; //0x5F8
volatile usbc_REG_HCDMAB7                                            HCDMAB7                       ; //0x5FC
volatile usbc_REG_HCCHAR8                                            HCCHAR8                       ; //0x600
volatile usbc_REG_HCSPLT8                                            HCSPLT8                       ; //0x604
volatile usbc_REG_HCINT8                                             HCINT8                        ; //0x608
volatile usbc_REG_HCINTMSK8                                          HCINTMSK8                     ; //0x60C
volatile usbc_REG_HCTSIZ8                                            HCTSIZ8                       ; //0x610
volatile usbc_REG_HCDMA8                                             HCDMA8                        ; //0x614
volatile uint32_t                                                    null_reg295                   ; //0x618
volatile usbc_REG_HCDMAB8                                            HCDMAB8                       ; //0x61C
volatile usbc_REG_HCCHAR9                                            HCCHAR9                       ; //0x620
volatile usbc_REG_HCSPLT9                                            HCSPLT9                       ; //0x624
volatile usbc_REG_HCINT9                                             HCINT9                        ; //0x628
volatile usbc_REG_HCINTMSK9                                          HCINTMSK9                     ; //0x62C
volatile usbc_REG_HCTSIZ9                                            HCTSIZ9                       ; //0x630
volatile usbc_REG_HCDMA9                                             HCDMA9                        ; //0x634
volatile uint32_t                                                    null_reg296                   ; //0x638
volatile usbc_REG_HCDMAB9                                            HCDMAB9                       ; //0x63C
volatile usbc_REG_HCCHAR10                                           HCCHAR10                      ; //0x640
volatile usbc_REG_HCSPLT10                                           HCSPLT10                      ; //0x644
volatile usbc_REG_HCINT10                                            HCINT10                       ; //0x648
volatile usbc_REG_HCINTMSK10                                         HCINTMSK10                    ; //0x64C
volatile usbc_REG_HCTSIZ10                                           HCTSIZ10                      ; //0x650
volatile usbc_REG_HCDMA10                                            HCDMA10                       ; //0x654
volatile uint32_t                                                    null_reg297                   ; //0x658
volatile usbc_REG_HCDMAB10                                           HCDMAB10                      ; //0x65C
volatile usbc_REG_HCCHAR11                                           HCCHAR11                      ; //0x660
volatile usbc_REG_HCSPLT11                                           HCSPLT11                      ; //0x664
volatile usbc_REG_HCINT11                                            HCINT11                       ; //0x668
volatile usbc_REG_HCINTMSK11                                         HCINTMSK11                    ; //0x66C
volatile usbc_REG_HCTSIZ11                                           HCTSIZ11                      ; //0x670
volatile usbc_REG_HCDMA11                                            HCDMA11                       ; //0x674
volatile uint32_t                                                    null_reg298                   ; //0x678
volatile usbc_REG_HCDMAB11                                           HCDMAB11                      ; //0x67C
volatile uint32_t                                                    null_reg299                   ; //0x680
volatile uint32_t                                                    null_reg300                   ; //0x684
volatile uint32_t                                                    null_reg301                   ; //0x688
volatile uint32_t                                                    null_reg302                   ; //0x68C
volatile uint32_t                                                    null_reg303                   ; //0x690
volatile uint32_t                                                    null_reg304                   ; //0x694
volatile uint32_t                                                    null_reg305                   ; //0x698
volatile uint32_t                                                    null_reg306                   ; //0x69C
volatile uint32_t                                                    null_reg307                   ; //0x6A0
volatile uint32_t                                                    null_reg308                   ; //0x6A4
volatile uint32_t                                                    null_reg309                   ; //0x6A8
volatile uint32_t                                                    null_reg310                   ; //0x6AC
volatile uint32_t                                                    null_reg311                   ; //0x6B0
volatile uint32_t                                                    null_reg312                   ; //0x6B4
volatile uint32_t                                                    null_reg313                   ; //0x6B8
volatile uint32_t                                                    null_reg314                   ; //0x6BC
volatile uint32_t                                                    null_reg315                   ; //0x6C0
volatile uint32_t                                                    null_reg316                   ; //0x6C4
volatile uint32_t                                                    null_reg317                   ; //0x6C8
volatile uint32_t                                                    null_reg318                   ; //0x6CC
volatile uint32_t                                                    null_reg319                   ; //0x6D0
volatile uint32_t                                                    null_reg320                   ; //0x6D4
volatile uint32_t                                                    null_reg321                   ; //0x6D8
volatile uint32_t                                                    null_reg322                   ; //0x6DC
volatile uint32_t                                                    null_reg323                   ; //0x6E0
volatile uint32_t                                                    null_reg324                   ; //0x6E4
volatile uint32_t                                                    null_reg325                   ; //0x6E8
volatile uint32_t                                                    null_reg326                   ; //0x6EC
volatile uint32_t                                                    null_reg327                   ; //0x6F0
volatile uint32_t                                                    null_reg328                   ; //0x6F4
volatile uint32_t                                                    null_reg329                   ; //0x6F8
volatile uint32_t                                                    null_reg330                   ; //0x6FC
volatile uint32_t                                                    null_reg331                   ; //0x700
volatile uint32_t                                                    null_reg332                   ; //0x704
volatile uint32_t                                                    null_reg333                   ; //0x708
volatile uint32_t                                                    null_reg334                   ; //0x70C
volatile uint32_t                                                    null_reg335                   ; //0x710
volatile uint32_t                                                    null_reg336                   ; //0x714
volatile uint32_t                                                    null_reg337                   ; //0x718
volatile uint32_t                                                    null_reg338                   ; //0x71C
volatile uint32_t                                                    null_reg339                   ; //0x720
volatile uint32_t                                                    null_reg340                   ; //0x724
volatile uint32_t                                                    null_reg341                   ; //0x728
volatile uint32_t                                                    null_reg342                   ; //0x72C
volatile uint32_t                                                    null_reg343                   ; //0x730
volatile uint32_t                                                    null_reg344                   ; //0x734
volatile uint32_t                                                    null_reg345                   ; //0x738
volatile uint32_t                                                    null_reg346                   ; //0x73C
volatile uint32_t                                                    null_reg347                   ; //0x740
volatile uint32_t                                                    null_reg348                   ; //0x744
volatile uint32_t                                                    null_reg349                   ; //0x748
volatile uint32_t                                                    null_reg350                   ; //0x74C
volatile uint32_t                                                    null_reg351                   ; //0x750
volatile uint32_t                                                    null_reg352                   ; //0x754
volatile uint32_t                                                    null_reg353                   ; //0x758
volatile uint32_t                                                    null_reg354                   ; //0x75C
volatile uint32_t                                                    null_reg355                   ; //0x760
volatile uint32_t                                                    null_reg356                   ; //0x764
volatile uint32_t                                                    null_reg357                   ; //0x768
volatile uint32_t                                                    null_reg358                   ; //0x76C
volatile uint32_t                                                    null_reg359                   ; //0x770
volatile uint32_t                                                    null_reg360                   ; //0x774
volatile uint32_t                                                    null_reg361                   ; //0x778
volatile uint32_t                                                    null_reg362                   ; //0x77C
volatile uint32_t                                                    null_reg363                   ; //0x780
volatile uint32_t                                                    null_reg364                   ; //0x784
volatile uint32_t                                                    null_reg365                   ; //0x788
volatile uint32_t                                                    null_reg366                   ; //0x78C
volatile uint32_t                                                    null_reg367                   ; //0x790
volatile uint32_t                                                    null_reg368                   ; //0x794
volatile uint32_t                                                    null_reg369                   ; //0x798
volatile uint32_t                                                    null_reg370                   ; //0x79C
volatile uint32_t                                                    null_reg371                   ; //0x7A0
volatile uint32_t                                                    null_reg372                   ; //0x7A4
volatile uint32_t                                                    null_reg373                   ; //0x7A8
volatile uint32_t                                                    null_reg374                   ; //0x7AC
volatile uint32_t                                                    null_reg375                   ; //0x7B0
volatile uint32_t                                                    null_reg376                   ; //0x7B4
volatile uint32_t                                                    null_reg377                   ; //0x7B8
volatile uint32_t                                                    null_reg378                   ; //0x7BC
volatile uint32_t                                                    null_reg379                   ; //0x7C0
volatile uint32_t                                                    null_reg380                   ; //0x7C4
volatile uint32_t                                                    null_reg381                   ; //0x7C8
volatile uint32_t                                                    null_reg382                   ; //0x7CC
volatile uint32_t                                                    null_reg383                   ; //0x7D0
volatile uint32_t                                                    null_reg384                   ; //0x7D4
volatile uint32_t                                                    null_reg385                   ; //0x7D8
volatile uint32_t                                                    null_reg386                   ; //0x7DC
volatile uint32_t                                                    null_reg387                   ; //0x7E0
volatile uint32_t                                                    null_reg388                   ; //0x7E4
volatile uint32_t                                                    null_reg389                   ; //0x7E8
volatile uint32_t                                                    null_reg390                   ; //0x7EC
volatile uint32_t                                                    null_reg391                   ; //0x7F0
volatile uint32_t                                                    null_reg392                   ; //0x7F4
volatile uint32_t                                                    null_reg393                   ; //0x7F8
volatile uint32_t                                                    null_reg394                   ; //0x7FC
volatile usbc_REG_DCFG                                               DCFG                          ; //0x800
volatile usbc_REG_DCTL                                               DCTL                          ; //0x804
volatile usbc_REG_DSTS                                               DSTS                          ; //0x808
volatile uint32_t                                                    null_reg395                   ; //0x80C
volatile usbc_REG_DIEPMSK                                            DIEPMSK                       ; //0x810
volatile usbc_REG_DOEPMSK                                            DOEPMSK                       ; //0x814
volatile usbc_REG_DAINT                                              DAINT                         ; //0x818
volatile usbc_REG_DAINTMSK                                           DAINTMSK                      ; //0x81C
volatile uint32_t                                                    null_reg396                   ; //0x820
volatile uint32_t                                                    null_reg397                   ; //0x824
volatile usbc_REG_DVBUSDIS                                           DVBUSDIS                      ; //0x828
volatile usbc_REG_DVBUSPULSE                                         DVBUSPULSE                    ; //0x82C
volatile usbc_REG_DTHRCTL                                            DTHRCTL                       ; //0x830
volatile usbc_REG_DIEPEMPMSK                                         DIEPEMPMSK                    ; //0x834
volatile uint32_t                                                    null_reg398                   ; //0x838
volatile uint32_t                                                    null_reg399                   ; //0x83C
volatile uint32_t                                                    null_reg400                   ; //0x840
volatile uint32_t                                                    null_reg401                   ; //0x844
volatile uint32_t                                                    null_reg402                   ; //0x848
volatile uint32_t                                                    null_reg403                   ; //0x84C
volatile uint32_t                                                    null_reg404                   ; //0x850
volatile uint32_t                                                    null_reg405                   ; //0x854
volatile uint32_t                                                    null_reg406                   ; //0x858
volatile uint32_t                                                    null_reg407                   ; //0x85C
volatile uint32_t                                                    null_reg408                   ; //0x860
volatile uint32_t                                                    null_reg409                   ; //0x864
volatile uint32_t                                                    null_reg410                   ; //0x868
volatile uint32_t                                                    null_reg411                   ; //0x86C
volatile uint32_t                                                    null_reg412                   ; //0x870
volatile uint32_t                                                    null_reg413                   ; //0x874
volatile uint32_t                                                    null_reg414                   ; //0x878
volatile uint32_t                                                    null_reg415                   ; //0x87C
volatile uint32_t                                                    null_reg416                   ; //0x880
volatile uint32_t                                                    null_reg417                   ; //0x884
volatile uint32_t                                                    null_reg418                   ; //0x888
volatile uint32_t                                                    null_reg419                   ; //0x88C
volatile uint32_t                                                    null_reg420                   ; //0x890
volatile uint32_t                                                    null_reg421                   ; //0x894
volatile uint32_t                                                    null_reg422                   ; //0x898
volatile uint32_t                                                    null_reg423                   ; //0x89C
volatile uint32_t                                                    null_reg424                   ; //0x8A0
volatile uint32_t                                                    null_reg425                   ; //0x8A4
volatile uint32_t                                                    null_reg426                   ; //0x8A8
volatile uint32_t                                                    null_reg427                   ; //0x8AC
volatile uint32_t                                                    null_reg428                   ; //0x8B0
volatile uint32_t                                                    null_reg429                   ; //0x8B4
volatile uint32_t                                                    null_reg430                   ; //0x8B8
volatile uint32_t                                                    null_reg431                   ; //0x8BC
volatile uint32_t                                                    null_reg432                   ; //0x8C0
volatile uint32_t                                                    null_reg433                   ; //0x8C4
volatile uint32_t                                                    null_reg434                   ; //0x8C8
volatile uint32_t                                                    null_reg435                   ; //0x8CC
volatile uint32_t                                                    null_reg436                   ; //0x8D0
volatile uint32_t                                                    null_reg437                   ; //0x8D4
volatile uint32_t                                                    null_reg438                   ; //0x8D8
volatile uint32_t                                                    null_reg439                   ; //0x8DC
volatile uint32_t                                                    null_reg440                   ; //0x8E0
volatile uint32_t                                                    null_reg441                   ; //0x8E4
volatile uint32_t                                                    null_reg442                   ; //0x8E8
volatile uint32_t                                                    null_reg443                   ; //0x8EC
volatile uint32_t                                                    null_reg444                   ; //0x8F0
volatile uint32_t                                                    null_reg445                   ; //0x8F4
volatile uint32_t                                                    null_reg446                   ; //0x8F8
volatile uint32_t                                                    null_reg447                   ; //0x8FC
volatile usbc_REG_DIEPCTL0                                           DIEPCTL0                      ; //0x900
volatile uint32_t                                                    null_reg448                   ; //0x904
volatile usbc_REG_DIEPINT0                                           DIEPINT0                      ; //0x908
volatile uint32_t                                                    null_reg449                   ; //0x90C
volatile usbc_REG_DIEPTSIZ0                                          DIEPTSIZ0                     ; //0x910
volatile usbc_REG_DIEPDMA0                                           DIEPDMA0                      ; //0x914
volatile usbc_REG_DTXFSTS0                                           DTXFSTS0                      ; //0x918
volatile usbc_REG_DIEPDMAB0                                          DIEPDMAB0                     ; //0x91C
volatile usbc_REG_DIEPCTL1                                           DIEPCTL1                      ; //0x920
volatile uint32_t                                                    null_reg450                   ; //0x924
volatile usbc_REG_DIEPINT1                                           DIEPINT1                      ; //0x928
volatile uint32_t                                                    null_reg451                   ; //0x92C
volatile usbc_REG_DIEPTSIZ1                                          DIEPTSIZ1                     ; //0x930
volatile usbc_REG_DIEPDMA1                                           DIEPDMA1                      ; //0x934
volatile usbc_REG_DTXFSTS1                                           DTXFSTS1                      ; //0x938
volatile usbc_REG_DIEPDMAB1                                          DIEPDMAB1                     ; //0x93C
volatile usbc_REG_DIEPCTL2                                           DIEPCTL2                      ; //0x940
volatile uint32_t                                                    null_reg452                   ; //0x944
volatile usbc_REG_DIEPINT2                                           DIEPINT2                      ; //0x948
volatile uint32_t                                                    null_reg453                   ; //0x94C
volatile usbc_REG_DIEPTSIZ2                                          DIEPTSIZ2                     ; //0x950
volatile usbc_REG_DIEPDMA2                                           DIEPDMA2                      ; //0x954
volatile usbc_REG_DTXFSTS2                                           DTXFSTS2                      ; //0x958
volatile usbc_REG_DIEPDMAB2                                          DIEPDMAB2                     ; //0x95C
volatile usbc_REG_DIEPCTL3                                           DIEPCTL3                      ; //0x960
volatile uint32_t                                                    null_reg454                   ; //0x964
volatile usbc_REG_DIEPINT3                                           DIEPINT3                      ; //0x968
volatile uint32_t                                                    null_reg455                   ; //0x96C
volatile usbc_REG_DIEPTSIZ3                                          DIEPTSIZ3                     ; //0x970
volatile usbc_REG_DIEPDMA3                                           DIEPDMA3                      ; //0x974
volatile usbc_REG_DTXFSTS3                                           DTXFSTS3                      ; //0x978
volatile usbc_REG_DIEPDMAB3                                          DIEPDMAB3                     ; //0x97C
volatile usbc_REG_DIEPCTL4                                           DIEPCTL4                      ; //0x980
volatile uint32_t                                                    null_reg456                   ; //0x984
volatile usbc_REG_DIEPINT4                                           DIEPINT4                      ; //0x988
volatile uint32_t                                                    null_reg457                   ; //0x98C
volatile usbc_REG_DIEPTSIZ4                                          DIEPTSIZ4                     ; //0x990
volatile usbc_REG_DIEPDMA4                                           DIEPDMA4                      ; //0x994
volatile usbc_REG_DTXFSTS4                                           DTXFSTS4                      ; //0x998
volatile usbc_REG_DIEPDMAB4                                          DIEPDMAB4                     ; //0x99C
volatile usbc_REG_DIEPCTL5                                           DIEPCTL5                      ; //0x9A0
volatile uint32_t                                                    null_reg458                   ; //0x9A4
volatile usbc_REG_DIEPINT5                                           DIEPINT5                      ; //0x9A8
volatile uint32_t                                                    null_reg459                   ; //0x9AC
volatile usbc_REG_DIEPTSIZ5                                          DIEPTSIZ5                     ; //0x9B0
volatile usbc_REG_DIEPDMA5                                           DIEPDMA5                      ; //0x9B4
volatile usbc_REG_DTXFSTS5                                           DTXFSTS5                      ; //0x9B8
volatile usbc_REG_DIEPDMAB5                                          DIEPDMAB5                     ; //0x9BC
volatile uint32_t                                                    null_reg460                   ; //0x9C0
volatile uint32_t                                                    null_reg461                   ; //0x9C4
volatile uint32_t                                                    null_reg462                   ; //0x9C8
volatile uint32_t                                                    null_reg463                   ; //0x9CC
volatile uint32_t                                                    null_reg464                   ; //0x9D0
volatile uint32_t                                                    null_reg465                   ; //0x9D4
volatile uint32_t                                                    null_reg466                   ; //0x9D8
volatile uint32_t                                                    null_reg467                   ; //0x9DC
volatile uint32_t                                                    null_reg468                   ; //0x9E0
volatile uint32_t                                                    null_reg469                   ; //0x9E4
volatile uint32_t                                                    null_reg470                   ; //0x9E8
volatile uint32_t                                                    null_reg471                   ; //0x9EC
volatile uint32_t                                                    null_reg472                   ; //0x9F0
volatile uint32_t                                                    null_reg473                   ; //0x9F4
volatile uint32_t                                                    null_reg474                   ; //0x9F8
volatile uint32_t                                                    null_reg475                   ; //0x9FC
volatile uint32_t                                                    null_reg476                   ; //0xA00
volatile uint32_t                                                    null_reg477                   ; //0xA04
volatile uint32_t                                                    null_reg478                   ; //0xA08
volatile uint32_t                                                    null_reg479                   ; //0xA0C
volatile uint32_t                                                    null_reg480                   ; //0xA10
volatile uint32_t                                                    null_reg481                   ; //0xA14
volatile uint32_t                                                    null_reg482                   ; //0xA18
volatile uint32_t                                                    null_reg483                   ; //0xA1C
volatile uint32_t                                                    null_reg484                   ; //0xA20
volatile uint32_t                                                    null_reg485                   ; //0xA24
volatile uint32_t                                                    null_reg486                   ; //0xA28
volatile uint32_t                                                    null_reg487                   ; //0xA2C
volatile uint32_t                                                    null_reg488                   ; //0xA30
volatile uint32_t                                                    null_reg489                   ; //0xA34
volatile uint32_t                                                    null_reg490                   ; //0xA38
volatile uint32_t                                                    null_reg491                   ; //0xA3C
volatile uint32_t                                                    null_reg492                   ; //0xA40
volatile uint32_t                                                    null_reg493                   ; //0xA44
volatile uint32_t                                                    null_reg494                   ; //0xA48
volatile uint32_t                                                    null_reg495                   ; //0xA4C
volatile uint32_t                                                    null_reg496                   ; //0xA50
volatile uint32_t                                                    null_reg497                   ; //0xA54
volatile uint32_t                                                    null_reg498                   ; //0xA58
volatile uint32_t                                                    null_reg499                   ; //0xA5C
volatile uint32_t                                                    null_reg500                   ; //0xA60
volatile uint32_t                                                    null_reg501                   ; //0xA64
volatile uint32_t                                                    null_reg502                   ; //0xA68
volatile uint32_t                                                    null_reg503                   ; //0xA6C
volatile uint32_t                                                    null_reg504                   ; //0xA70
volatile uint32_t                                                    null_reg505                   ; //0xA74
volatile uint32_t                                                    null_reg506                   ; //0xA78
volatile uint32_t                                                    null_reg507                   ; //0xA7C
volatile uint32_t                                                    null_reg508                   ; //0xA80
volatile uint32_t                                                    null_reg509                   ; //0xA84
volatile uint32_t                                                    null_reg510                   ; //0xA88
volatile uint32_t                                                    null_reg511                   ; //0xA8C
volatile uint32_t                                                    null_reg512                   ; //0xA90
volatile uint32_t                                                    null_reg513                   ; //0xA94
volatile uint32_t                                                    null_reg514                   ; //0xA98
volatile uint32_t                                                    null_reg515                   ; //0xA9C
volatile uint32_t                                                    null_reg516                   ; //0xAA0
volatile uint32_t                                                    null_reg517                   ; //0xAA4
volatile uint32_t                                                    null_reg518                   ; //0xAA8
volatile uint32_t                                                    null_reg519                   ; //0xAAC
volatile uint32_t                                                    null_reg520                   ; //0xAB0
volatile uint32_t                                                    null_reg521                   ; //0xAB4
volatile uint32_t                                                    null_reg522                   ; //0xAB8
volatile uint32_t                                                    null_reg523                   ; //0xABC
volatile uint32_t                                                    null_reg524                   ; //0xAC0
volatile uint32_t                                                    null_reg525                   ; //0xAC4
volatile uint32_t                                                    null_reg526                   ; //0xAC8
volatile uint32_t                                                    null_reg527                   ; //0xACC
volatile uint32_t                                                    null_reg528                   ; //0xAD0
volatile uint32_t                                                    null_reg529                   ; //0xAD4
volatile uint32_t                                                    null_reg530                   ; //0xAD8
volatile uint32_t                                                    null_reg531                   ; //0xADC
volatile uint32_t                                                    null_reg532                   ; //0xAE0
volatile uint32_t                                                    null_reg533                   ; //0xAE4
volatile uint32_t                                                    null_reg534                   ; //0xAE8
volatile uint32_t                                                    null_reg535                   ; //0xAEC
volatile uint32_t                                                    null_reg536                   ; //0xAF0
volatile uint32_t                                                    null_reg537                   ; //0xAF4
volatile uint32_t                                                    null_reg538                   ; //0xAF8
volatile uint32_t                                                    null_reg539                   ; //0xAFC
volatile usbc_REG_DOEPCTL0                                           DOEPCTL0                      ; //0xB00
volatile uint32_t                                                    null_reg540                   ; //0xB04
volatile usbc_REG_DOEPINT0                                           DOEPINT0                      ; //0xB08
volatile uint32_t                                                    null_reg541                   ; //0xB0C
volatile usbc_REG_DOEPTSIZ0                                          DOEPTSIZ0                     ; //0xB10
volatile usbc_REG_DOEPDMA0                                           DOEPDMA0                      ; //0xB14
volatile uint32_t                                                    null_reg542                   ; //0xB18
volatile usbc_REG_DOEPDMAB0                                          DOEPDMAB0                     ; //0xB1C
volatile usbc_REG_DOEPCTL1                                           DOEPCTL1                      ; //0xB20
volatile uint32_t                                                    null_reg543                   ; //0xB24
volatile usbc_REG_DOEPINT1                                           DOEPINT1                      ; //0xB28
volatile uint32_t                                                    null_reg544                   ; //0xB2C
volatile usbc_REG_DOEPTSIZ1                                          DOEPTSIZ1                     ; //0xB30
volatile usbc_REG_DOEPDMA1                                           DOEPDMA1                      ; //0xB34
volatile uint32_t                                                    null_reg545                   ; //0xB38
volatile usbc_REG_DOEPDMAB1                                          DOEPDMAB1                     ; //0xB3C
volatile usbc_REG_DOEPCTL2                                           DOEPCTL2                      ; //0xB40
volatile uint32_t                                                    null_reg546                   ; //0xB44
volatile usbc_REG_DOEPINT2                                           DOEPINT2                      ; //0xB48
volatile uint32_t                                                    null_reg547                   ; //0xB4C
volatile usbc_REG_DOEPTSIZ2                                          DOEPTSIZ2                     ; //0xB50
volatile usbc_REG_DOEPDMA2                                           DOEPDMA2                      ; //0xB54
volatile uint32_t                                                    null_reg548                   ; //0xB58
volatile usbc_REG_DOEPDMAB2                                          DOEPDMAB2                     ; //0xB5C
volatile usbc_REG_DOEPCTL3                                           DOEPCTL3                      ; //0xB60
volatile uint32_t                                                    null_reg549                   ; //0xB64
volatile usbc_REG_DOEPINT3                                           DOEPINT3                      ; //0xB68
volatile uint32_t                                                    null_reg550                   ; //0xB6C
volatile usbc_REG_DOEPTSIZ3                                          DOEPTSIZ3                     ; //0xB70
volatile usbc_REG_DOEPDMA3                                           DOEPDMA3                      ; //0xB74
volatile uint32_t                                                    null_reg551                   ; //0xB78
volatile usbc_REG_DOEPDMAB3                                          DOEPDMAB3                     ; //0xB7C
volatile usbc_REG_DOEPCTL4                                           DOEPCTL4                      ; //0xB80
volatile uint32_t                                                    null_reg552                   ; //0xB84
volatile usbc_REG_DOEPINT4                                           DOEPINT4                      ; //0xB88
volatile uint32_t                                                    null_reg553                   ; //0xB8C
volatile usbc_REG_DOEPTSIZ4                                          DOEPTSIZ4                     ; //0xB90
volatile usbc_REG_DOEPDMA4                                           DOEPDMA4                      ; //0xB94
volatile uint32_t                                                    null_reg554                   ; //0xB98
volatile usbc_REG_DOEPDMAB4                                          DOEPDMAB4                     ; //0xB9C
volatile usbc_REG_DOEPCTL5                                           DOEPCTL5                      ; //0xBA0
volatile uint32_t                                                    null_reg555                   ; //0xBA4
volatile usbc_REG_DOEPINT5                                           DOEPINT5                      ; //0xBA8
volatile uint32_t                                                    null_reg556                   ; //0xBAC
volatile usbc_REG_DOEPTSIZ5                                          DOEPTSIZ5                     ; //0xBB0
volatile usbc_REG_DOEPDMA5                                           DOEPDMA5                      ; //0xBB4
volatile uint32_t                                                    null_reg557                   ; //0xBB8
volatile usbc_REG_DOEPDMAB5                                          DOEPDMAB5                     ; //0xBBC
volatile uint32_t                                                    null_reg558                   ; //0xBC0
volatile uint32_t                                                    null_reg559                   ; //0xBC4
volatile uint32_t                                                    null_reg560                   ; //0xBC8
volatile uint32_t                                                    null_reg561                   ; //0xBCC
volatile uint32_t                                                    null_reg562                   ; //0xBD0
volatile uint32_t                                                    null_reg563                   ; //0xBD4
volatile uint32_t                                                    null_reg564                   ; //0xBD8
volatile uint32_t                                                    null_reg565                   ; //0xBDC
volatile uint32_t                                                    null_reg566                   ; //0xBE0
volatile uint32_t                                                    null_reg567                   ; //0xBE4
volatile uint32_t                                                    null_reg568                   ; //0xBE8
volatile uint32_t                                                    null_reg569                   ; //0xBEC
volatile uint32_t                                                    null_reg570                   ; //0xBF0
volatile uint32_t                                                    null_reg571                   ; //0xBF4
volatile uint32_t                                                    null_reg572                   ; //0xBF8
volatile uint32_t                                                    null_reg573                   ; //0xBFC
volatile uint32_t                                                    null_reg574                   ; //0xC00
volatile uint32_t                                                    null_reg575                   ; //0xC04
volatile uint32_t                                                    null_reg576                   ; //0xC08
volatile uint32_t                                                    null_reg577                   ; //0xC0C
volatile uint32_t                                                    null_reg578                   ; //0xC10
volatile uint32_t                                                    null_reg579                   ; //0xC14
volatile uint32_t                                                    null_reg580                   ; //0xC18
volatile uint32_t                                                    null_reg581                   ; //0xC1C
volatile uint32_t                                                    null_reg582                   ; //0xC20
volatile uint32_t                                                    null_reg583                   ; //0xC24
volatile uint32_t                                                    null_reg584                   ; //0xC28
volatile uint32_t                                                    null_reg585                   ; //0xC2C
volatile uint32_t                                                    null_reg586                   ; //0xC30
volatile uint32_t                                                    null_reg587                   ; //0xC34
volatile uint32_t                                                    null_reg588                   ; //0xC38
volatile uint32_t                                                    null_reg589                   ; //0xC3C
volatile uint32_t                                                    null_reg590                   ; //0xC40
volatile uint32_t                                                    null_reg591                   ; //0xC44
volatile uint32_t                                                    null_reg592                   ; //0xC48
volatile uint32_t                                                    null_reg593                   ; //0xC4C
volatile uint32_t                                                    null_reg594                   ; //0xC50
volatile uint32_t                                                    null_reg595                   ; //0xC54
volatile uint32_t                                                    null_reg596                   ; //0xC58
volatile uint32_t                                                    null_reg597                   ; //0xC5C
volatile uint32_t                                                    null_reg598                   ; //0xC60
volatile uint32_t                                                    null_reg599                   ; //0xC64
volatile uint32_t                                                    null_reg600                   ; //0xC68
volatile uint32_t                                                    null_reg601                   ; //0xC6C
volatile uint32_t                                                    null_reg602                   ; //0xC70
volatile uint32_t                                                    null_reg603                   ; //0xC74
volatile uint32_t                                                    null_reg604                   ; //0xC78
volatile uint32_t                                                    null_reg605                   ; //0xC7C
volatile uint32_t                                                    null_reg606                   ; //0xC80
volatile uint32_t                                                    null_reg607                   ; //0xC84
volatile uint32_t                                                    null_reg608                   ; //0xC88
volatile uint32_t                                                    null_reg609                   ; //0xC8C
volatile uint32_t                                                    null_reg610                   ; //0xC90
volatile uint32_t                                                    null_reg611                   ; //0xC94
volatile uint32_t                                                    null_reg612                   ; //0xC98
volatile uint32_t                                                    null_reg613                   ; //0xC9C
volatile uint32_t                                                    null_reg614                   ; //0xCA0
volatile uint32_t                                                    null_reg615                   ; //0xCA4
volatile uint32_t                                                    null_reg616                   ; //0xCA8
volatile uint32_t                                                    null_reg617                   ; //0xCAC
volatile uint32_t                                                    null_reg618                   ; //0xCB0
volatile uint32_t                                                    null_reg619                   ; //0xCB4
volatile uint32_t                                                    null_reg620                   ; //0xCB8
volatile uint32_t                                                    null_reg621                   ; //0xCBC
volatile uint32_t                                                    null_reg622                   ; //0xCC0
volatile uint32_t                                                    null_reg623                   ; //0xCC4
volatile uint32_t                                                    null_reg624                   ; //0xCC8
volatile uint32_t                                                    null_reg625                   ; //0xCCC
volatile uint32_t                                                    null_reg626                   ; //0xCD0
volatile uint32_t                                                    null_reg627                   ; //0xCD4
volatile uint32_t                                                    null_reg628                   ; //0xCD8
volatile uint32_t                                                    null_reg629                   ; //0xCDC
volatile uint32_t                                                    null_reg630                   ; //0xCE0
volatile uint32_t                                                    null_reg631                   ; //0xCE4
volatile uint32_t                                                    null_reg632                   ; //0xCE8
volatile uint32_t                                                    null_reg633                   ; //0xCEC
volatile uint32_t                                                    null_reg634                   ; //0xCF0
volatile uint32_t                                                    null_reg635                   ; //0xCF4
volatile uint32_t                                                    null_reg636                   ; //0xCF8
volatile uint32_t                                                    null_reg637                   ; //0xCFC
volatile uint32_t                                                    null_reg638                   ; //0xD00
volatile uint32_t                                                    null_reg639                   ; //0xD04
volatile uint32_t                                                    null_reg640                   ; //0xD08
volatile uint32_t                                                    null_reg641                   ; //0xD0C
volatile uint32_t                                                    null_reg642                   ; //0xD10
volatile uint32_t                                                    null_reg643                   ; //0xD14
volatile uint32_t                                                    null_reg644                   ; //0xD18
volatile uint32_t                                                    null_reg645                   ; //0xD1C
volatile uint32_t                                                    null_reg646                   ; //0xD20
volatile uint32_t                                                    null_reg647                   ; //0xD24
volatile uint32_t                                                    null_reg648                   ; //0xD28
volatile uint32_t                                                    null_reg649                   ; //0xD2C
volatile uint32_t                                                    null_reg650                   ; //0xD30
volatile uint32_t                                                    null_reg651                   ; //0xD34
volatile uint32_t                                                    null_reg652                   ; //0xD38
volatile uint32_t                                                    null_reg653                   ; //0xD3C
volatile uint32_t                                                    null_reg654                   ; //0xD40
volatile uint32_t                                                    null_reg655                   ; //0xD44
volatile uint32_t                                                    null_reg656                   ; //0xD48
volatile uint32_t                                                    null_reg657                   ; //0xD4C
volatile uint32_t                                                    null_reg658                   ; //0xD50
volatile uint32_t                                                    null_reg659                   ; //0xD54
volatile uint32_t                                                    null_reg660                   ; //0xD58
volatile uint32_t                                                    null_reg661                   ; //0xD5C
volatile uint32_t                                                    null_reg662                   ; //0xD60
volatile uint32_t                                                    null_reg663                   ; //0xD64
volatile uint32_t                                                    null_reg664                   ; //0xD68
volatile uint32_t                                                    null_reg665                   ; //0xD6C
volatile uint32_t                                                    null_reg666                   ; //0xD70
volatile uint32_t                                                    null_reg667                   ; //0xD74
volatile uint32_t                                                    null_reg668                   ; //0xD78
volatile uint32_t                                                    null_reg669                   ; //0xD7C
volatile uint32_t                                                    null_reg670                   ; //0xD80
volatile uint32_t                                                    null_reg671                   ; //0xD84
volatile uint32_t                                                    null_reg672                   ; //0xD88
volatile uint32_t                                                    null_reg673                   ; //0xD8C
volatile uint32_t                                                    null_reg674                   ; //0xD90
volatile uint32_t                                                    null_reg675                   ; //0xD94
volatile uint32_t                                                    null_reg676                   ; //0xD98
volatile uint32_t                                                    null_reg677                   ; //0xD9C
volatile uint32_t                                                    null_reg678                   ; //0xDA0
volatile uint32_t                                                    null_reg679                   ; //0xDA4
volatile uint32_t                                                    null_reg680                   ; //0xDA8
volatile uint32_t                                                    null_reg681                   ; //0xDAC
volatile uint32_t                                                    null_reg682                   ; //0xDB0
volatile uint32_t                                                    null_reg683                   ; //0xDB4
volatile uint32_t                                                    null_reg684                   ; //0xDB8
volatile uint32_t                                                    null_reg685                   ; //0xDBC
volatile uint32_t                                                    null_reg686                   ; //0xDC0
volatile uint32_t                                                    null_reg687                   ; //0xDC4
volatile uint32_t                                                    null_reg688                   ; //0xDC8
volatile uint32_t                                                    null_reg689                   ; //0xDCC
volatile uint32_t                                                    null_reg690                   ; //0xDD0
volatile uint32_t                                                    null_reg691                   ; //0xDD4
volatile uint32_t                                                    null_reg692                   ; //0xDD8
volatile uint32_t                                                    null_reg693                   ; //0xDDC
volatile uint32_t                                                    null_reg694                   ; //0xDE0
volatile uint32_t                                                    null_reg695                   ; //0xDE4
volatile uint32_t                                                    null_reg696                   ; //0xDE8
volatile uint32_t                                                    null_reg697                   ; //0xDEC
volatile uint32_t                                                    null_reg698                   ; //0xDF0
volatile uint32_t                                                    null_reg699                   ; //0xDF4
volatile uint32_t                                                    null_reg700                   ; //0xDF8
volatile uint32_t                                                    null_reg701                   ; //0xDFC
volatile usbc_REG_PCGCCTL                                            PCGCCTL                       ; //0xE00
}usbc_REG_H;

#endif //usbc_REG__H
