/**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  MJX Application Team
  * @version V2.1.0
  * @date    16-May-2014
  * @brief   Main program body
  ******************************************************************************
  */ 
/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "main.h"
#include "System_Initialize.h"
#include "TemperatureNTC.h"
#include "TaskHandle.h"
#include "PID.h"
/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
unsigned int bubbleSort(unsigned int array[],unsigned char size );
/************** Private variables ****************/
unsigned char g_ucControllerParameter[23]={0x00}; //�������Ĳ���
unsigned char g_ucADCTimeCNT = 0;      //AD����ʱ����
unsigned char g_ucCSE7759Flag = 0;     
unsigned char g_ucNodeGroupNum = 0x00; //Ĭ�����
signed char g_cControllerTemperature;  //�������¶�
unsigned int g_uiNTC_AD_Vaule = 0;     //NTC AD����ֵ
unsigned int g_uiFB_AD_Vaule = 0;      //0-10V������ѹ AD����ֵ
unsigned char g_ucDimVaule = 0xc8;     //����ֵ �ϵ�Ĭ�����10v
unsigned int g_uiControllerStatus = 0; //��������״̬λ
unsigned char g_ucRecivePLCData[RCV_MAX_NUM]={0x00};  //���ڽ��ջ���
unsigned char g_ucReceiveTimeCnt = RCV_MAX_TIME;
unsigned char g_ucReceiveDataLen = 0;
unsigned char g_ucUART1ReceiveFlag = 0;               //���ڽ������־
unsigned char g_ucPowerCalculateComplete = 1;         //0�����ʼ���δ��ɣ� 1�����ʼ������

unsigned int g_uiCorrectionStatus = 0x00;
volatile unsigned char g_ucRelayOperateRequest = 0;  //�̵�����������λ��1�����󿪼̵�����2������ؼ̵���
#ifndef TEST_ON_OFF
  #error "Please define TEST_ON_OFF!"
#elif TEST_ON_OFF == 1
  volatile unsigned int g_uiOnOffInterval;  //���ص�ʱ��������λms
  volatile unsigned char g_ucOnOffFlag = 1;     //���͹صı�־��1������2���أ�3������ɣ�4�������
#endif
//char buffer[4];
ScmCSE7759TypeDef ScmCSE7759;
ScmControllerTypeDef ScmController;
#ifndef SEECRET_EN 
    #error "Please define SEECRET_EN!"
#elif   SEECRET_EN == 1
  unsigned char g_ucUniqueDeviceId[12] = {0};
  unsigned char g_ucUniqueDeviceIdTemp[12] = {0};
  volatile unsigned short int g_uiDelayTime = 0;
  unsigned char g_ucLisenceEn = 0;
void ReadUniqueToEeprom(unsigned char *p);
unsigned char Compare(unsigned char *puc1, unsigned char *puc2);
void GetUniqueID(unsigned char *p);
#endif
//���ܼ���ʹ�õı���
volatile unsigned int  g_uiEnergySaveEn = 0;     //���ܱ���ʹ��λ,0:�����棻���㣺����
volatile unsigned long ulEnergyCurrent = 0;
volatile unsigned long ulEnergyTotalPluse = 0;   //�������ܵ���������
#if ENERGY_MODE == 1
volatile unsigned int  g_uiEnergyCnt = 0;        //���ܼ�����
#elif ENERGY_MODE == 2
volatile unsigned int  g_uiEnergy_100mWs = 0;         //����������
volatile unsigned long g_uliEnergyTotal_100mWh = 0;    //������������
#endif
#ifndef S0_10V_CALIBRATE_EN
  #error "Please define S0_10V_CALIBRATE_EN!"
#elif S0_10V_CALIBRATE_EN == 1
unsigned int g_ui0_10V_Param;   //unit:0.001
#endif
volatile unsigned char g_ucEnergyClearFlag = 0;  //���������־��0��û�����㣻1�������ξ�������ܣ�ֹͣ������ܲ���
void TurnOffOutput(void);
/* Private functions ---------------------------------------------------------*/
int main(void)
{   
  unsigned int uiTemp;
  /************��ʼ��***************/
   /*!<Set High speed internal clock 16M */ 
    //CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
    ClkSet();
   /* Confige GPIO */
    GPIO_Config();
    //MOC3063_OFF();
    RELAY_OFF();
    TIM1_Config();
    TIM2_Config();
    TIM4_Config();
    ADC1_Config();
    UART1_Config();
    
    //������ʱ������
    TIM1->CR1 |= 0x01;	
    TIM2->CR1 |= 0x01;
    TIM4->CR1 |= 0x01;
    ADC1->CR1 |= 0x01;    //����ADCģ��
    Delay_ms(2000);       //��ʱ�ȴ�ADCģ���ȶ� 1s
    IncPIDInit();
    g_uiCorrectionStatus = FLASH_ReadByte(COR_STA_ADD)&0x38;
    ASE7759Init(g_uiCorrectionStatus); //ϵ����ʼ��         
    Delay_ms(50);       
    ADC1->CR1 |= 0x01;  //����ADCת��      
    Set_PWM_Output(200);
    g_ucNodeGroupNum = FLASH_ReadByte(NodeGroupAddress);//��ȡ���
    WDG_Config();         // ���ÿ��Ź�
    __enable_interrupt(); // ��ȫ���ж�
    UART1_IT_Set(UART1_IT_RXNE_OR, 1);
    Delay_ms(30); 
    g_ucRelayOperateRequest = 1;//�ϵ�Ĭ�Ͽ���
#if SEECRET_EN == 1
    GetUniqueID(g_ucUniqueDeviceId);
    ReadUniqueToEeprom(g_ucUniqueDeviceIdTemp);
    if(Compare(g_ucUniqueDeviceId,g_ucUniqueDeviceIdTemp)){
       g_ucLisenceEn = 1;
    }
#endif
    //WriteGroupToFlash(NodeGroupAddress,0x01);
/* Infinite loop */   
  while (1)
  {  
#if SEECRET_EN == 1
    if(g_uiDelayTime >= SEECRET_DELAY_TIME && g_ucLisenceEn == 0){
       TurnOffOutput();											
    }
#endif
    CheckReceiveData(); // 1
    ASE7759Correction();// 2
#if TEST_ON_OFF == 1
    if(g_ucOnOffFlag == 3){//������ʱ��ɣ�׼���ػ�
       g_ucUART1ReceiveFlag = 2;
       g_ucRecivePLCData[0] = 0x11;
       g_ucRecivePLCData[1] = 0x00;
       g_ucRecivePLCData[2] = 0x11;
       g_ucOnOffFlag = 2;
    }else if(g_ucOnOffFlag == 4){//�ػ���ʱ��ɣ�׼������
        g_ucUART1ReceiveFlag = 2;
        g_ucRecivePLCData[0] = 0x11;
        g_ucRecivePLCData[1] = 0xC8;
        g_ucRecivePLCData[2] = 0xD9;
        g_ucOnOffFlag = 1;
    }
#endif
    FunctionOperate();  // 3 ���밴˳��
    IWDG->KR =0xAA;     //ι�� 
    CSE7759Calculate();
    STM8S_ADC1_Convert();   
    IWDG->KR =0xAA; //ι��   
    //�ж��Ƿ��д洢���ܵ�����
#if ENERGY_MODE == 1
    if (g_uiEnergySaveEn != 0 && (g_ucCSE7759Flag & POWER_BIT)&& (g_ucEnergyClearFlag==0)){
      ulEnergyCurrent = ReadEnergyParam();
      uiTemp = g_uiEnergySaveEn;
      ulEnergyCurrent += 1;
      SaveEnergy(ulEnergyCurrent);
      g_uiEnergySaveEn = 0;
    }
    if (g_ucPowerCalculateComplete != 0 && (g_ucCSE7759Flag & POWER_BIT)){//��
        g_ucCSE7759Flag &= ~POWER_BIT;
        g_ucPowerCalculateComplete = 0;
    }
#elif ENERGY_MODE == 2
    if ((g_uiEnergySaveEn != 0) && (g_ucEnergyClearFlag==0)){
      ulEnergyCurrent = ReadEnergyParam();
      uiTemp = g_uiEnergySaveEn/3600;    //��100mWsת��Ϊ100mWh
      ulEnergyCurrent += uiTemp;
      SaveEnergy(ulEnergyCurrent);
      g_uiEnergySaveEn = 0;
    }
#endif
    IWDG->KR =0xAA; //ι�� 
    
  }  
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/




