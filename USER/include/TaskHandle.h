#ifndef __TASKHANDLE_H
#define __TASKHANDLE_H
extern unsigned char  g_ucRecivePLCData[RCV_MAX_NUM];
extern ScmCSE7759TypeDef ScmCSE7759;
extern ScmControllerTypeDef ScmController;
extern unsigned char g_ucUART1ReceiveFlag;
extern unsigned char g_ucReceiveDataLen;

extern unsigned char g_ucRelayOnTimeCNT;
extern unsigned char g_ucMOC3063OffTimeCNT;
extern unsigned char g_ucRelayOnFlag;
extern unsigned char g_ucMOC3063OffFlag;
extern unsigned char g_ucCSE7759Flag;
extern unsigned char g_ucNodeGroupNUM;
extern unsigned int  g_uiNTC_AD_Vaule ;
extern unsigned int  g_uiFB_AD_Vaule;
extern unsigned char g_ucADCTimeCNT;
extern unsigned char g_ucDimVaule;
extern unsigned int  g_uiControllerStatus;
extern unsigned char g_ucControllerParameter[];
//extern unsigned char g_ucReadADVauleCNT;
//extern unsigned char g_ucNTCFlag ;
extern signed char g_cControllerTemperature;
extern unsigned char g_ucNodeGroupNum;
void FunctionOperate(void);
void CSE7759Calculate(void);
void STM8S_ADC1_Convert(void);
void CheckReceiveData(void);

unsigned char  FactorySetting(unsigned char ucCommand);
void ASE7759Correction(void);
void ASE7759Init(unsigned char ucStatus);
void LED_Flash(unsigned char ucType);
//void NTC_AD_Vaule(void);
//void CSE7759Calculate(void);
#endif