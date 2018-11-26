#include "stm8s.h"
#include "main.h"    
#include "System_Initialize.h"
#include "TaskHandle.h"
#include "TemperatureNTC.h"
#include "PID.h"

#define  POWER_BUFFER_LONG    5
void AckFunction(void);
//static unsigned short int bubbleSort(unsigned short int array[],unsigned char size );
static void ControllerStatus(unsigned char ucMode);
//static unsigned char s_ucComStatus = 0x01;     // A:0x01�̵���  B:0x02 BAT200
static unsigned char fg_ucProVolFlag = 0;      // ��ѹ������־��
unsigned char ucCommandOnFlag = 1;              // �����־��������������غͱ����أ������ʱ��1
int iTempPid = 0;
unsigned char fg_ucLowPowerFlag = 0;
extern volatile unsigned char g_ucRelayOperateRequest;
extern unsigned char g_ucPowerCalculateComplete;
typedef union{
  float fData ;
  long  lData ;
}UcmTypeDef;      //intercept

UcmTypeDef ePowerCheck;  //
UcmTypeDef eVoltageCheck;
UcmTypeDef eCurrentCheck;

static unsigned int fg_uiPowerTimeCntCheck;
static unsigned int fg_uiPowerPluseCntCheck;
static unsigned int fg_uiVoltageTimeCntCheck;
static unsigned int fg_uiVoltagePluseCntCheck;
static unsigned int fg_uiCurrentTimeCntCheck;
static unsigned int fg_uiCurrentPluseCntCheck;

static unsigned int uiPowerTimeCntBuffer[POWER_BUFFER_LONG];
static unsigned int uiPowerPluseCntBuffer[POWER_BUFFER_LONG];

extern unsigned int g_uiCorrectionStatus;
#ifndef SEECRET_EN
    #error "Please define SEECRET_EN!"
#elif   SEECRET_EN == 1
  extern unsigned char g_ucUniqueDeviceId[];
  extern unsigned char g_ucUniqueDeviceIdTemp[];
  void GetUniqueID(unsigned char *p);
  void WriteUniqueToEeprom(unsigned char *p);
#endif
//���ܼ���
extern volatile unsigned char g_uiEnergySaveEn;
extern volatile unsigned long ulEnergyTotalPluse;
#ifndef ENERGY_MODE
   #error "Please define ENERGY_MODE!"
#elif ENERGY_MODE == 1
   extern volatile unsigned int  g_uiEnergyCnt;
#elif ENERGY_MODE == 2
   extern volatile unsigned int g_uiEnergy_100mWs;
#endif
extern volatile unsigned long ulEnergyCurrent;
#ifndef S0_10V_CALIBRATE_EN
  #error "Please define S0_10V_CALIBRATE_EN!"
#elif S0_10V_CALIBRATE_EN == 1
extern unsigned int g_ui0_10V_Param;   //unit:0.001
#endif
extern volatile unsigned char g_ucEnergyClearFlag;
/*******************************************************************************
* Function Name : TurnOffOutput.
* Description   : �رտɿع�ͼ̵���.
*******************************************************************************/
void ReciveBufferClear(void)
{   unsigned char ucCnt;
    for(ucCnt=0;ucCnt<RCV_MAX_NUM;ucCnt++){
        g_ucRecivePLCData[ucCnt] = 0;
    }
}
/*******************************************************************************
* Function Name : TurnOffOutput.
* Description   : �رտɿع�ͼ̵���.
*******************************************************************************/
void TurnOffOutput(void)
{															
  RELAY_OFF();
  ucCommandOnFlag = 0;
  fg_ucProVolFlag = 0;  //��ѹ������־����
  //g_uiControllerStatus &= ~OP;//�����ʱ�־ ��0
}

/*******************************************************************************
* Function Name : TurnOnOutput.
* Description   : �򿪿ɿع�ͼ̵���.
*******************************************************************************/
void TurnOnOutput(void) //�����
{
  RELAY_ON();
  ucCommandOnFlag = 1;
  fg_ucProVolFlag = 0;
}
/*******************************************************************************
* Function Name : ProTurnOffOutput.
* Description   : ���������رտɿع�ͼ̵���.
*******************************************************************************/
void ProTurnOffOutput(void)
{														
  g_ucRelayOperateRequest = 0x02;
  //g_uiControllerStatus &= ~OP;//�����ʱ�־ ��0
}

/*******************************************************************************
* Function Name : ProTurnOnOutput.
* Description   : �������� �򿪿ɿع�ͼ̵���.
*******************************************************************************/
void ProTurnOnOutput(void)
{
  g_ucRelayOperateRequest = 0x01;
  //g_uiControllerStatus &= ~OP;
}
/*******************************************************************************
* Function Name : CheckReceiveData.
* Description   : ������һ֡���ݺ���к�У��.
*******************************************************************************/
void CheckReceiveData(void)
{
  unsigned char ucDataSum;
  unsigned char i;
 
  if(g_ucUART1ReceiveFlag == 2){
    g_ucUART1ReceiveFlag = 1;
    if(g_ucRecivePLCData[0] == CONFIG_CMD || g_ucRecivePLCData[0] == 0X30 || g_ucRecivePLCData[0] == 0X31){  //CONFIG_CMD == 0xA1
      return;
    }
    ucDataSum = 0;
    for(i=0;i<g_ucReceiveDataLen-1;i++){
      ucDataSum += g_ucRecivePLCData[i];
    }
    if(ucDataSum != g_ucRecivePLCData[g_ucReceiveDataLen-1]){
      //������ݻ���
      ReciveBufferClear();
      g_ucUART1ReceiveFlag = 0;
    } 
  }
}

/*******************************************************************************
* Function Name : ReadParameter.
* Description   : ������4BytesתΪ������.
*******************************************************************************/
long ReadParameter(unsigned char ucFlag)
{
  long lTemp1  = 0;
  unsigned char i;
  if(ucFlag == GRA){
    for(i = 0;i < 4;i++){
      lTemp1 <<= 8;
      lTemp1 |= g_ucRecivePLCData[i+2];        
    }   
  }else if(ucFlag == INT){
    for(i=0;i<4;i++){
      lTemp1 <<= 8;
      lTemp1 |= g_ucRecivePLCData[i+6];         
    }    
  }
  return  lTemp1;
}
/*******************************************************************************
* Function Name : ReadParameterVIP.
* Description   : ������4BytesתΪ������.
ucFlag: GRA:б��  INT:�ؾ�
ucVip��0�����ʣ�1����ѹ�� 2������
*******************************************************************************/
long ReadParameterVIP(unsigned char ucFlag, unsigned char ucVip)
{
  long lTemp1  = 0;
  unsigned char i;
  for(i = 0;i < 4;i++){
    lTemp1 <<= 8;
    lTemp1 |= g_ucRecivePLCData[i + 2 + ucVip*4];        
  }   
  return  lTemp1;
}

/*******************************************************************************
* Function Name : AckA1Com.
* Description   : ��Ӧ��������.
ucParm:���Ӳ���:  0xff:�����ĸò���
*******************************************************************************/
void AckA1Com(unsigned char ucCom, unsigned char ucParm)
{
  unsigned char ucAckTable[10];
  ucAckTable[0] = 0x8A;
  ucAckTable[1] = ucCom; //��Ӧ����
  switch(ucCom){
  case RECOVER_CMD:     //�ָ�����      
    ucAckTable[2] = g_ucRecivePLCData[2]; //�ָ���Ŀ
    ucAckTable[3] = 0x01;
    ucAckTable[4] = ucAckTable[0]+ucAckTable[1]+ucAckTable[2]+ucAckTable[3];
    UART1_SendString(ucAckTable,5);
    break;
  case CONFIG_VIP:        //Ӧ�� У�����빦�� 05
    if (ucParm == 0x03){
      ucAckTable[2] = 0x03;
    }else{
      ucAckTable[2] = 0x01;
    }
    ucAckTable[3] = ucAckTable[0]+ucAckTable[1]+ucAckTable[2];
    UART1_SendString(ucAckTable,4);
    break;
#if SEECRET_EN == 1
  case CONFIG_LICENSE:
    ucAckTable[2] = (unsigned char)SOFT_VERSION;
    ucAckTable[3] = (unsigned char)(SOFT_VERSION>>8);
    ucAckTable[4] = HARDWARE_VERSION;
    ucAckTable[5] = DEVICE_CODE;
    ucAckTable[6] = ucParm;
    ucAckTable[7] = ucAckTable[0]+ucAckTable[1]+ucAckTable[2]+ucAckTable[3]+ucAckTable[4]+ucAckTable[5]+ucAckTable[6];
    UART1_SendString(ucAckTable,8);
    break;
#endif
  case CLREA_ENERGY:
    ucAckTable[2] = DEVICE_CODE;
    ucAckTable[3] = ucParm;  
    ucAckTable[4] = ucAckTable[0]+ucAckTable[1]+ucAckTable[2]+ucAckTable[3];
    UART1_SendString(ucAckTable,5);
    break;
  default :
    ucAckTable[1] = g_ucRecivePLCData[1];
    ucAckTable[2] = 0x04;
    ucAckTable[3] = ucAckTable[0]+ucAckTable[1]+ucAckTable[2];
    UART1_SendString(ucAckTable,4);
    break;
  }  
}
/*******************************************************************************
* Function Name : AckA2Com.
* Description   : ��Ӧ��ȡ����.
*******************************************************************************/
void AckA2Com(unsigned char ucCom) //��Ӧ��ȡ����
{
   unsigned char ucAckTable[27];
   unsigned char i;
   //long lTempFlashData = 0;
   ucAckTable[0] = 0x8B;
   ucAckTable[1] = ucCom;
   switch(ucCom){
   case 0x01:      //��ȡ״̬
     i =  FLASH_ReadByte(COR_STA_ADD)&0x38;
     ucAckTable[2] = 0;
     ucAckTable[3] = i;
     ucAckTable[4] = ucAckTable[0] + ucAckTable[1] + ucAckTable[2] + ucAckTable[3]; 
     UART1_SendString(ucAckTable,5);
     break;
   default:
     ucAckTable[2] = 0x04;
     ucAckTable[3] = ucAckTable[0]+ucAckTable[1]+ucAckTable[2];
     UART1_SendString(ucAckTable,4);     
   }            
}
/*******************************************************************************
* Function Name : ASE7759Correction.
* Description   : �������õ�ֵ����Ӧ.
*******************************************************************************/
void ASE7759Correction(void)
{
  unsigned char ucCom;
  unsigned char ucTemp;   //
  unsigned char i;
  unsigned long ulTemp;
  float fTemp;
  unsigned char ucCnt;
  unsigned int  uiTemp;
  UcmTypeDef eTemp;
  static unsigned char s_ucSaveFlag;
  if(g_ucUART1ReceiveFlag == 1){
    if(g_ucRecivePLCData[0] == CONFIG_CMD){   // ����Ϊ����  CONFIG_CMD = A1
      ucTemp = CONFIG_CMD;
      for(i=1;i<g_ucReceiveDataLen-2;i++){    // ���У��
        ucTemp ^= g_ucRecivePLCData[i];     
      }
      if(ucTemp!= g_ucRecivePLCData[g_ucReceiveDataLen-2]){ // �ж����У��
        return;
      }
      ucTemp = g_ucRecivePLCData[0];        // �����׸��ֽ�
      for(i=1;i<g_ucReceiveDataLen-2;i++){  // ��У��
        ucTemp +=g_ucRecivePLCData[i];     
      }
      if(ucTemp!= g_ucRecivePLCData[g_ucReceiveDataLen-1]){ // �жϺ�У��
        return;
      }                
      ucCom = g_ucRecivePLCData[1]; // ��ȡ������
      switch(ucCom){
      case RECOVER_CMD:            // �ָ�Ĭ������
        if(g_ucReceiveDataLen != 5){
          break;
        }
        if(FactorySetting(g_ucRecivePLCData[2]) == SUCCESS){
          WriteDataToFlash(COR_STA_ADD,(unsigned char)g_uiCorrectionStatus);
          AckA1Com(g_ucRecivePLCData[1],0xff);
        }            
        break;
      case CONFIG_VIP:
        if(g_ucReceiveDataLen != 18){
          break;
        } 
        //�жϲ���
        eTemp.lData = ReadParameterVIP(GRA,0);
        fTemp = eTemp.fData * ScmCSE7759.uiPowerTimeCnt / ScmCSE7759.uiPowerPulseCnt;
        if (fTemp < PT_DEFAULT_L || fTemp > PT_DEFAULT_H){//�жϹ���У׼��������
           AckA1Com(CONFIG_VIP, 0x03);
           break;
        }
        eTemp.lData = ReadParameterVIP(GRA,1);
        fTemp = eTemp.fData * ScmCSE7759.uiVoltageTimeCnt / ScmCSE7759.uiVoltagePulseCnt;
        if (fTemp < VT_DEFAULT_L || fTemp > VT_DEFAULT_H){//�жϵ�ѹУ׼��������
           AckA1Com(CONFIG_VIP, 0x03);
           break;
        }
        eTemp.lData = ReadParameterVIP(GRA,2);
        fTemp = eTemp.fData * ScmCSE7759.uiCurrentTimeCnt / ScmCSE7759.uiCurrentPulseCnt;
        if (fTemp < AT_DEFAULT_L || fTemp > AT_DEFAULT_H){//�жϵ���У׼��������
           AckA1Com(CONFIG_VIP, 0x03);
           break;
        }
#if S0_10V_CALIBRATE_EN == 1
        /* 0~10VУ�� */
        uiTemp = ((unsigned int)(g_ucRecivePLCData[14]) << 8) + g_ucRecivePLCData[15];
        g_ui0_10V_Param = (unsigned int)(g_ui0_10V_Param * ((float)uiTemp/1000.0));
        Set0_10vOutput(g_ucDimVaule);
        //�洢0~10VУ��ֵ
        WriteDataToFlash(S0_10V_CHECK_ADR, (unsigned char)(g_ui0_10V_Param>>8));
        WriteDataToFlash(S0_10V_CHECK_ADR + 1, (unsigned char)g_ui0_10V_Param);
        WriteDataToFlash(HSI_10V_FLAG_ADR, HSI_10V_FLAG);
#endif  
        //�洢У��ֵ
        ePowerCheck.lData = ReadParameterVIP(GRA,0);
        eVoltageCheck.lData = ReadParameterVIP(GRA,1);
        eCurrentCheck.lData = ReadParameterVIP(GRA,2);
        
        WriteLongDataToFlash(POW_GRA_ADD,ePowerCheck.lData);
        for(ucCnt = 0,uiTemp = 0;ucCnt<POWER_BUFFER_LONG;ucCnt++){
          uiTemp += uiPowerTimeCntBuffer[ucCnt];
        }
        fg_uiPowerTimeCntCheck = uiTemp/POWER_BUFFER_LONG;
        for(ucCnt = 0,uiTemp = 0;ucCnt<POWER_BUFFER_LONG;ucCnt++){
          uiTemp += uiPowerPluseCntBuffer[ucCnt];
        }
        fg_uiPowerPluseCntCheck = uiTemp/POWER_BUFFER_LONG;
        ulTemp = fg_uiPowerTimeCntCheck; 
        ulTemp <<= 16;
        ulTemp |= fg_uiPowerPluseCntCheck; 
        WriteLongDataToFlash(POW_INT_ADD,ulTemp); 
        
        WriteLongDataToFlash(VOL_GRA_ADD,eVoltageCheck.lData);
        fg_uiVoltageTimeCntCheck = ScmCSE7759.uiVoltageTimeCnt;
        fg_uiVoltagePluseCntCheck = ScmCSE7759.uiVoltagePulseCnt;
        ulTemp = fg_uiVoltageTimeCntCheck;
        ulTemp <<= 16;
        ulTemp |= fg_uiVoltagePluseCntCheck; 
        WriteLongDataToFlash(VOL_INT_ADD,ulTemp); 
      
        WriteLongDataToFlash(CUR_GRA_ADD,eCurrentCheck.lData);
        fg_uiCurrentTimeCntCheck = ScmCSE7759.uiCurrentTimeCnt;
        fg_uiCurrentPluseCntCheck = ScmCSE7759.uiCurrentPulseCnt; 
        ulTemp = fg_uiCurrentTimeCntCheck;
        ulTemp <<= 16;
        ulTemp |= fg_uiCurrentPluseCntCheck; 
        WriteLongDataToFlash(CUR_INT_ADD,ulTemp); 
        s_ucSaveFlag = 0; 
        s_ucSaveFlag |= ICA;
        s_ucSaveFlag |= IVA;
        s_ucSaveFlag |= IPA;       
        g_uiCorrectionStatus |= s_ucSaveFlag;
        WriteDataToFlash(COR_STA_ADD,(unsigned char)g_uiCorrectionStatus);// ��У��״̬д��EEPROM  
        AckA1Com(CONFIG_VIP, 0xff);//AckA1Com(g_ucRecivePLCData[1]);
        break;
#if SEECRET_EN == 1
      case CONFIG_LICENSE:
        if (g_ucRecivePLCData[2] == DEVICE_CODE){//�ж��豸���Ƿ����
          GetUniqueID(g_ucUniqueDeviceId);
          WriteUniqueToEeprom(g_ucUniqueDeviceId);
          AckA1Com(CONFIG_LICENSE,0x02);
        }else{
          AckA1Com(CONFIG_LICENSE,0x01);
        }
        break;
#endif
      case CLREA_ENERGY:
        if (g_ucRecivePLCData[2] == DEVICE_CODE){//�ж��豸���Ƿ����
      #if ENERGY_MODE == 1
          g_uiEnergyCnt = 0;
      #elif ENERGY_MODE == 2
          g_uiEnergy_100mWs = 0;
      #endif
          g_uiEnergySaveEn = 0;
          ulEnergyCurrent = 0;
          g_ucEnergyClearFlag = 1; //ֹͣ��������
          SaveEnergy(ulEnergyCurrent);
          AckA1Com(CLREA_ENERGY,0x02);
        }else{
          AckA1Com(CLREA_ENERGY,0x01);
        }
        break;
      default: AckA1Com(NO_OPTION,0xff);
        break;
      } 
      //������ݻ���
      ReciveBufferClear();
      g_ucUART1ReceiveFlag = 0;
    }else if(g_ucRecivePLCData[0] == READ_CMD){ 
      if(g_ucReceiveDataLen == 3){
         AckA2Com(g_ucRecivePLCData[1]);
      }
      //������ݻ���
      ReciveBufferClear();
      g_ucUART1ReceiveFlag = 0;
    }
  }
}
/*
  UcmTypeDef ePowerGradient;  //б��
  UcmTypeDef ePowerIntercept; //�ؾ�
  #define VT  0.2167
  #define AT  0.00167
  #define PT  0.603
*/
/*******************************************************************************
* Function Name : ASE7759Init.
* Description   : 7759�ϵ��ʼ�����ϵ��.
*******************************************************************************/      
void ASE7759Init(unsigned char ucStatus)
{
    unsigned long ulTemp;
    if((ucStatus & 0x38) != 0X38 ){
        FactorySetting(RECOVER_ALL);
        return;
    }
    // ����
    ePowerCheck.lData = ReadLongDataFromFlash(POW_GRA_ADD);
    ulTemp = ReadLongDataFromFlash(POW_INT_ADD);
    fg_uiPowerTimeCntCheck = (ulTemp >>16);
    fg_uiPowerPluseCntCheck = (unsigned short int)ulTemp;
    // ��ѹ
    eVoltageCheck.lData = ReadLongDataFromFlash(VOL_GRA_ADD);
    ulTemp = ReadLongDataFromFlash(VOL_INT_ADD);
    fg_uiVoltageTimeCntCheck = (ulTemp >>16);
    fg_uiVoltagePluseCntCheck = (unsigned short int)ulTemp;
    // ����
    eCurrentCheck.lData = ReadLongDataFromFlash(CUR_GRA_ADD);
    ulTemp = ReadLongDataFromFlash(CUR_INT_ADD);
    fg_uiCurrentTimeCntCheck = (ulTemp >>16);
    fg_uiCurrentPluseCntCheck = (unsigned short int)ulTemp;
}

/*******************************************************************************
* Function Name : CheckUartRecive.
* Description   : У����յ�������.У������������3���ֽ�
* return��0��ʧ��     1���ɹ�
*******************************************************************************/
static unsigned char CheckUartRecive(void)
{
    if (3 != g_ucReceiveDataLen || (unsigned char)(g_ucRecivePLCData[0] + g_ucRecivePLCData[1]) != g_ucRecivePLCData[2]){
       return(0);
    }else{
       return(1);
    }
}
/*******************************************************************************
* Function Name : DimFun.
* Description   : ����.
* input�� ucDimValue��the dim value range from 0 to 200
*******************************************************************************/
static void DimFun(unsigned char ucDimValue)
{
  if (ucDimValue <= 200){
    g_ucDimVaule = ucDimValue; //����ֵ
    if((g_uiControllerStatus & ALL_PRO) == 0){
      if(g_ucDimVaule==0x00){ //�ص�
        TurnOffOutput();											
      }else {		
        TurnOnOutput();    
      }
      Set_PWM_Output(g_ucDimVaule);            
      Set0_10vOutput(g_ucDimVaule);
    }
  }
}
/*******************************************************************************
* Function Name : FunctionOperate.
* Description   : ���ݲ�ͬ������в���.
*******************************************************************************/
void FunctionOperate(void)
{
  //static unsigned char s_ucComStatus = 0x01;// A:0x01�̵���  B:0x02 BAT200
  unsigned char ucCommand = 0x00;
#if PROTOCOL == GRT_V14
  unsigned char ucGroupNum = 0x00;
  unsigned char ucTempVaule = 0x00; 
#endif

  unsigned char ucSendEnFlag = 0;//���ͻظ��ı�־��0�������ͣ� 1������
#if PROTOCOL == XJP_V01
  static unsigned char s_ucDimEn = 1; 
#endif
  if(g_ucUART1ReceiveFlag == 1){
    ucCommand = g_ucRecivePLCData[0];
    switch(ucCommand){ 
#ifndef PROTOCOL
    #error "Please define PROTOCOL!"
#elif PROTOCOL == GRT_V14
    /**********���� 0x11��0x12��0x15*********/
    case 0x11:
      g_ucDimVaule = g_ucRecivePLCData[1]; //��ȡ����ֵ
      if((g_uiControllerStatus & ALL_PRO) == 0){
          if(g_ucDimVaule==0x00){ //�ص�
            TurnOffOutput();											
          }else {		
            TurnOnOutput();    
          }
          Set_PWM_Output(g_ucDimVaule);            
          Set0_10vOutput(g_ucDimVaule);
      }            
      break;		           
    case 0x12:
      ucGroupNum = g_ucRecivePLCData[1]; //�����յ������
      ucTempVaule = g_ucRecivePLCData[2]; //11 13 14 ����
      g_ucDimVaule = g_ucRecivePLCData[3]; //����ֵ
      if((ucGroupNum == g_ucNodeGroupNum)||(ucGroupNum == 0x00)){  //�ж��յ�������뱾������Ƿ�һ��
        if((g_uiControllerStatus & ALL_PRO) == 0){
            if(g_ucDimVaule==0x00){ //�ص�
              TurnOffOutput();											
            }else {		
              TurnOnOutput();    
            }
            Set_PWM_Output(g_ucDimVaule);            
            Set0_10vOutput(g_ucDimVaule);
        }
      }
      break;	        	           						
    case 0x15:
      ucGroupNum = g_ucRecivePLCData[1];
      g_ucNodeGroupNum = ucGroupNum;                       //�������
      WriteGroupToFlash(NodeGroupAddress,g_ucNodeGroupNum);//������ŵ�fash                
      break;
#elif PROTOCOL == XJP_V01
    case 0x12: 
    case 0x13:
      if (CheckUartRecive() == 0){
        break;
      }
      if (ucCommand == 0x13){
        ucSendEnFlag = 1;
      }
      if (s_ucDimEn == 1){
        DimFun(g_ucRecivePLCData[1]);
      }
      break;
    case 0x14:
    case 0x15:
      if(CheckUartRecive() == 0){
        break;
      }
      ucSendEnFlag = 1;
      if(ucCommand == 0x14){
        s_ucDimEn = 0;
      }else{
        s_ucDimEn = 1;
      }
      DimFun(g_ucRecivePLCData[1]);
      break;
    case 0x16:
      if(CheckUartRecive() == 0 || g_ucRecivePLCData[1] != 0X01){
        break;
      }
      ucSendEnFlag = 1;
      break;
    case 0X17:
      if(CheckUartRecive() == 0){//У�鲻ͨ��
        break;
      }
      ucSendEnFlag = 1;
      if(s_ucDimEn == 1){
        DimFun(g_ucRecivePLCData[1]);
      }
      break;
    case 0X30: //��㲥
    case 0X31: //���ò�ѯ��
      if((unsigned char)(g_ucRecivePLCData[0] + g_ucRecivePLCData[1] + g_ucRecivePLCData[2]) != g_ucRecivePLCData[4] ||
         (unsigned char)(g_ucRecivePLCData[0] ^ g_ucRecivePLCData[1] ^ g_ucRecivePLCData[2]) != g_ucRecivePLCData[3] ||
           g_ucReceiveDataLen != 5){
         break;
      }
      if (ucCommand == 0x30){
          if (g_ucRecivePLCData[1] == 0 || g_ucNodeGroupNum == g_ucRecivePLCData[1]){
             DimFun(g_ucRecivePLCData[2]);
          }
      }else{
          ucSendEnFlag = 1;
          if (g_ucRecivePLCData[1] == 0){
              WriteGroupToFlash(NodeGroupAddress,g_ucRecivePLCData[2]);//������ŵ�fash
          }
      }
      break;
       /* TEST */
    case 0X23:   //23 00 23
      ucSendEnFlag = 1;
      break;
    default:break;
#endif
   }
   if(ucSendEnFlag == 1){
    AckFunction();
   }
    //������ݻ���
    ReciveBufferClear();
    g_ucUART1ReceiveFlag = 0;
  }
  
}
/*******************************************************************************
* Function Name : AckFunction.
* Description   : Ӧ����.
*******************************************************************************/
void AckFunction(void)
{
  unsigned char ucTempdata = 0;
#if PROTOCOL == GRT_V14
  unsigned char ucTempdata2 = 0;
#endif
  unsigned char ucn = 0;
  unsigned char ucSendTotal;
  ucTempdata = g_ucRecivePLCData[0];
  switch(ucTempdata)
  {
#if PROTOCOL == GRT_V14
    case 0x11:UART1_SendChar(0x80);
      UART1_SendChar(ucTempdata2);		          
      UART1_SendChar(0x80+ucTempdata2);
      break;
    case 0x15:UART1_SendChar(0x85);
      UART1_SendChar(ucTempdata2);
      UART1_SendChar(0x85+ucTempdata2);
      break;
    case 0x01:ControllerStatus();
      for(ucn=0;ucn<20;ucn++){
        UART1_SendChar(g_ucControllerParameter[ucn]);
      }
      break;
#elif PROTOCOL == XJP_V01
    case 0x13:
      if (200 < g_ucRecivePLCData[1] && g_ucRecivePLCData[1] < 0xFF){ //����ֵ��201~ 254֮�䣬���ظ���
          break;
      }
      if (g_ucRecivePLCData[1] == 0xFF){
          ControllerStatus(18);
          ucSendTotal = 18;
      }else{
          ControllerStatus(17);
          ucSendTotal = 17;
      }
      for(ucn=0;ucn<ucSendTotal;ucn++){
          UART1_SendChar(g_ucControllerParameter[ucn]);
      }
      break;
    case 0x14:
      UART1_SendChar(0x94);
      UART1_SendChar(g_ucDimVaule);
      UART1_SendChar(0x94 + g_ucDimVaule);
      break;
    case 0x15:
      UART1_SendChar(0x95);
      UART1_SendChar(g_ucDimVaule);
      UART1_SendChar(0x95 + g_ucDimVaule);
      break; 
    case 0x16: //��ȡ��Դ״̬
      g_ucControllerParameter[0] = 0x96;
      g_ucControllerParameter[1] = (unsigned char)SOFT_VERSION;
      g_ucControllerParameter[2] = (unsigned char)(SOFT_VERSION>>8);
      g_ucControllerParameter[3] = (unsigned char)HARDWARE_VERSION;
      g_ucControllerParameter[4] = DEVICE_CODE;
      g_ucControllerParameter[5] = ReadLongDataFromFlash(COR_STA_ADD);
      g_ucControllerParameter[6] = 0;
      for(ucn=0;ucn<6;ucn++){
        g_ucControllerParameter[6] += g_ucControllerParameter[ucn];
      }
      for(ucn=0;ucn<7;ucn++){
        UART1_SendChar(g_ucControllerParameter[ucn]);
      }
      break;
    case 0x17:
      ControllerStatus(22);
      ucSendTotal = 22;
      for(ucn=0;ucn<ucSendTotal;ucn++){
          UART1_SendChar(g_ucControllerParameter[ucn]);
      }
      break;
    case 0X31: //���ò�ѯ��
      UART1_SendChar(0xB1);
      g_ucNodeGroupNum = FLASH_ReadByte(NodeGroupAddress);//��ȡ���
      UART1_SendChar(g_ucNodeGroupNum);
      UART1_SendChar(0XB1 + g_ucNodeGroupNum);
      break;
  #if SEECRET_EN == 1
     /* TEST */
     case 0X23:   //23 00 23
      for(ucn=0;ucn<12;ucn++){
          UART1_SendChar(g_ucUniqueDeviceId[ucn]);
      }
      break;
  #endif
#endif
  }	
}
/*******************************************************************************
* Function Name :FactorySetting 
* Description   : 
*******************************************************************************/
unsigned char FactorySetting(unsigned char ucCommand)
{
  switch(ucCommand){
  case RECOVER_ALL:
    ePowerCheck.fData = PT;
    eVoltageCheck.fData = VT;
    eCurrentCheck.fData = AT; 
    
    fg_uiPowerTimeCntCheck = TIME_CNT_DEFAULT;
    fg_uiPowerPluseCntCheck = PLUSE_CNT_DEFAULT;
    // ��ѹ
    fg_uiVoltageTimeCntCheck = TIME_CNT_DEFAULT;
    fg_uiVoltagePluseCntCheck = PLUSE_CNT_DEFAULT;
    // ����
    fg_uiCurrentTimeCntCheck = TIME_CNT_DEFAULT;
    fg_uiCurrentPluseCntCheck = PLUSE_CNT_DEFAULT;
    
    g_uiCorrectionStatus = 0x00;
#if S0_10V_CALIBRATE_EN == 1
    g_ui0_10V_Param = 1000;
#endif
    break;
  default:
    return FAIL;  
  }
  return SUCCESS;
}
/*******************************************************************************
* Function Name : OverPowerPro.
* Description   : ����Ƿ�أ�����״̬.
*******************************************************************************/
void OverPowerPro(void)
{
  static unsigned char s_ucProPowerCnt = 0;
#ifndef POW_PRO
  #error "Please define POW_PRO!"
#elif POW_PRO == 1
  if(ScmController.uiPower > PRO_H_POWER || ScmController.uiInputCurrent > OVER_CURRENT){ //���빦�ʹ���,�����������������ֹ����ʱ�����
    s_ucProPowerCnt++;
    if(s_ucProPowerCnt > 2){  
        s_ucProPowerCnt = 0;
        ProTurnOffOutput();    //Ƿѹ �ص�
        Set_PWM_Output(0);     //PWM duty = 0
        sPtr->SetPoint = 0;   //0-10V ->0V
        g_uiControllerStatus |= OP;
        g_uiControllerStatus &= ~UP;                
    }    
  }else{
    //g_uiControllerStatus &= ~OP;
    if(RELAY_STATUS){
      if(ScmController.uiPower < UNDER_POWER){      
        g_uiControllerStatus |= UP;//
      }else{
        g_uiControllerStatus &= ~UP;
      }
    }else{
      g_uiControllerStatus &= ~UP;
    }        
    s_ucProPowerCnt = 0;
  }   
#endif

  //���ü̵���״̬ 
#ifndef  PROTOCOL  
  #error "Please define PROTOCOL!"
#elif    PROTOCOL == GRT_V14
  if(RELAY_STATUS){
    g_uiControllerStatus |= RS;
  }else{ 
    g_uiControllerStatus &= ~RS;
  }
#endif
    //����Ƿ��������״̬  
}
/*******************************************************************************
* Function Name : VoltagePro.
* Description   : .
*******************************************************************************/
void VoltagePro(void)
{
  static unsigned char s_ucProHighVolCnt = 0;
  static unsigned char s_ucNormalVolCnt = 0;
  static unsigned char s_ucProLowVolCnt = 0;
#ifndef VOL_PRO  //main.h
  #error "Please define VOL_PRO!"
#elif VOL_PRO == 1
  if(ScmController.uiInputVoltage > PRO_H_VOLTAGE){  //�����ѹ����Ͽ����      
    s_ucNormalVolCnt = 0;       //������ѹ��������
    s_ucProHighVolCnt++;
    if(s_ucProHighVolCnt > 2){
      s_ucProHighVolCnt = 0;
      ProTurnOffOutput();       //��ѹ�ص�
      Set_PWM_Output(0);        //PWM duty = 0
      sPtr->SetPoint = 0;       //0-10V ->0V      
      fg_ucProVolFlag = 1;      //��ѹ�����ص� ��� �� 1
      g_uiControllerStatus |= OV;
      g_uiControllerStatus &= ~UV;                             
    }					
  }else if(ScmController.uiInputVoltage < PRO_L_VOLTAGE){       //�����ѹ���ͶϿ����   
    s_ucNormalVolCnt = 0;   //������ѹ�������� 
    s_ucProLowVolCnt++;
    if(s_ucProLowVolCnt > 2){    
      s_ucProLowVolCnt = 0;   
      ProTurnOffOutput();    //Ƿѹ �ص�
      Set_PWM_Output(0);     //PWM duty = 0
      sPtr->SetPoint = 0;    //0-10V ->0V      
      fg_ucProVolFlag = 1;   //��ѹ�����ص� ��� �� 1
      g_uiControllerStatus |= UV;//
      g_uiControllerStatus &= ~OV;                    
    }        
  }else{
    s_ucProLowVolCnt = 0;
    s_ucProHighVolCnt = 0;
    if(fg_ucProVolFlag == 0){
      g_uiControllerStatus &= ~UV;
      g_uiControllerStatus &= ~OV;        
    } 
  }
  //��ѹ�ָ�����
  if(fg_ucProVolFlag == 1){   //�����������͹���Ƿѹ����״̬
    if((ScmController.uiInputVoltage<RECO_H_VOLTAGE)&&(ScmController.uiInputVoltage>RECO_L_VOLTAGE)){    
        s_ucNormalVolCnt++;             
        if(s_ucNormalVolCnt >= 3){
          s_ucNormalVolCnt = 0;
          g_uiControllerStatus &= ~UV;
          g_uiControllerStatus &= ~OV;
          if(ucCommandOnFlag == 1 && ((g_uiControllerStatus & ALL_PRO) == 0)){
            if(!(RELAY_STATUS)){    //�̵����ǹص�
              ProTurnOnOutput();    //�ָ����
              Set_PWM_Output(g_ucDimVaule);
              Set0_10vOutput(g_ucDimVaule);
              fg_ucProVolFlag = 0;              
            }                    
          }       
       }     
    }        
  }
  //�¶ȱ���
  //���ʹ��߱���
  
#endif  
    //����Ƿѹ����ѹ״̬
}
/*******************************************************************************
* Function Name : .
* Description   : .
*******************************************************************************/
void CSE7759Calculate(void)
{    
#define   TRANSITION_POWER     100   //���빦�ʵ�����ת�۵㹦�ʣ���λ��0.1W
  static float fVoltage = 1,fCurrent = 1,fPower = 1;
  static unsigned char s_ucPcnt = 0;
  if(g_ucCSE7759Flag & POWER_BIT){  
    uiPowerTimeCntBuffer[s_ucPcnt] = ScmCSE7759.uiPowerTimeCnt;
    uiPowerPluseCntBuffer[s_ucPcnt] = ScmCSE7759.uiPowerPulseCnt;
    if(++s_ucPcnt >= POWER_BUFFER_LONG){
      s_ucPcnt = 0;
    }
    fPower = (ePowerCheck.fData * fg_uiPowerTimeCntCheck * ScmCSE7759.uiPowerPulseCnt) / ((float)fg_uiPowerPluseCntCheck * ScmCSE7759.uiPowerTimeCnt);
    ScmController.uiPower = (unsigned int)(fPower * 10);
    OverPowerPro();
    ScmController.ucPowerFactory = (unsigned char)((fPower*100)/(fCurrent * fVoltage)); 
    if(ScmController.ucPowerFactory > 100){
      ScmController.ucPowerFactory = 100;
    } 
#if ENERGY_MODE == 1
    g_ucPowerCalculateComplete = 1;  
#elif ENERGY_MODE == 2
    g_ucCSE7759Flag &= ~POWER_BIT;
#endif
  }   
  if(g_ucCSE7759Flag & VOL_BIT){
    fVoltage = (eVoltageCheck.fData * fg_uiVoltageTimeCntCheck * ScmCSE7759.uiVoltagePulseCnt) / ((float)fg_uiVoltagePluseCntCheck * ScmCSE7759.uiVoltageTimeCnt);
    ScmController.uiInputVoltage = (unsigned int)(fVoltage * 10);
    VoltagePro();    
    g_ucCSE7759Flag &= ~VOL_BIT;    
  }else if(g_ucCSE7759Flag & CUR_BIT){
    fCurrent = (eCurrentCheck.fData * fg_uiCurrentTimeCntCheck * ScmCSE7759.uiCurrentPulseCnt) / ((float)fg_uiCurrentPluseCntCheck * ScmCSE7759.uiCurrentTimeCnt);
    ScmController.uiInputCurrent = (unsigned int)(fCurrent * 1000);
    if(RELAY_STATUS){
      if(ScmController.uiInputCurrent < UNDER_CURRENT){
        g_uiControllerStatus |= UC;
      }else if(ScmController.uiInputCurrent > UNDER_CURRENT + 2){
        g_uiControllerStatus &= ~UC;
      }
    }else{ 
      g_uiControllerStatus &= ~UC;
    }             //UNDER_CURRENT
    g_ucCSE7759Flag &= ~CUR_BIT;  
  }
}  
/*******************************************************************************
* Function Name : STM8S_ADC1_Convert.
* Description   : ��ʱ������ms.
*******************************************************************************/
void STM8S_ADC1_Convert(void)
{
  unsigned char ucChTemp = 0;
  unsigned int uiADVauleH = 0;
  unsigned char ucADVauleL = 0;
  //static unsigned short int s_uiNTC_AD[12]= {0x00};
  static unsigned int  s_uiNtcAdValueSum = 0;
  static unsigned char s_ucNTC_CNT = 0;
  if(ADC1->CSR & 0x80/*ADC1_GetFlagStatus(ADC1_FLAG_EOC)*/){
    ucChTemp = ADC1->CSR&0x0f;  //��ȡ��ǰ��ͨ��
      ucADVauleL = ADC1->DRL;    //�ȶ�ȡ�Ͱ�λ
      uiADVauleH = ADC1->DRH;    //���ȡ�߰�λ
      if(ucChTemp == NTC_CHANNLE){     
        s_uiNtcAdValueSum += ((uiADVauleH<<8)|ucADVauleL);
        s_ucNTC_CNT++;
        ADC1->CSR = FB_CHANNLE;
        if(s_ucNTC_CNT >= 16){                          //�������12��
          s_ucNTC_CNT = 0;
          g_uiNTC_AD_Vaule = s_uiNtcAdValueSum>>4;     // s_uiNtcAdValueSum / 16
          s_uiNtcAdValueSum = 0;
          g_cControllerTemperature = TemperatureCalculate(g_uiNTC_AD_Vaule); //�����¶�
         /****************�¶ȱ�������*****************/ 
   #ifndef TEM_PRO   //main.h 
       #error "Please define TEM_PRO!"
   #elif TEM_PRO == 1      
         if(ucCommandOnFlag == 1){ //�����������������  fg_ucProVolFlag       
           if(g_cControllerTemperature > OVER_TEMP){
                g_uiControllerStatus |= OT;
                if(RELAY_STATUS){         //����̵����ǿ�ͨ��
                    ProTurnOffOutput();     //�ض����
                    Set_PWM_Output(0);      //PWM duty = 0
                    sPtr->SetPoint = 0;     //0-10V ->0V 
                }                                                    
            }else if(g_cControllerTemperature < RECO_TEMP){ //�¶��������
                  g_uiControllerStatus &= ~OT;
                  if(!(RELAY_STATUS)&&((g_uiControllerStatus & ALL_PRO) == 0)){  //��� ���ڹ��±��� �̵����պ�״̬
                    ProTurnOnOutput();  //�ָ����
                    Set_PWM_Output(g_ucDimVaule);
                    Set0_10vOutput(g_ucDimVaule);
                  }                         
            }
          }
          //���±�����־
         if(g_cControllerTemperature < UNDER_TEMP){
            g_uiControllerStatus |= LT;
         }else{
            g_uiControllerStatus &= ~LT;
         }
     #endif    
        }
      }else if(ucChTemp == FB_CHANNLE){                        
        g_uiFB_AD_Vaule = (uiADVauleH<<8)|ucADVauleL;
        ADC1->CSR = NTC_CHANNLE;
        iTempPid += IncPIDCalc(g_uiFB_AD_Vaule);
        if(iTempPid > PWM_TCNT0_10V){
          iTempPid = PWM_TCNT0_10V;
        }else if(iTempPid < 0){
          iTempPid = 0;
        }
        Set_0_10V_Output(PWM_TCNT0_10V - iTempPid);  //Set_0_10V_Output(iTempPid);          	    
      }
      ADC1->CR1 |= 0x01;//ADC1_StartConversion(); 
    }
}

/*******************************************************************************
* Function Name : .
* Description   : .
*******************************************************************************/
/*static unsigned short int bubbleSort(unsigned short int array[],unsigned char size )//��������д�С��������  
{  
  unsigned short int uiADTemp = 0;
  unsigned char i,j;
    
  for(i=0;i<size-1;i++){        
    for(j=0;j<size-1;j++){           
      if(array[j]>array[j+1]){               
        uiADTemp = array[j];  
        array[j] = array[j+1];  
        array[j+1] = uiADTemp;  
      }  
    }  
  }    
   uiADTemp = 0; //��0
   for(i=2;i<10;i++){      //ȡ�м�8�������
      uiADTemp += array[i];
   }
   uiADTemp = uiADTemp>>3; //��8������ƽ��
   return uiADTemp ;
}*/
/*******************************************************************************
* Function Name : ControllerStatus.
* Description   : ������״̬��־λ.
*******************************************************************************/
static void ControllerStatus(unsigned char ucMode)
{
  unsigned char ucCnt = 0;
  unsigned char ucSum = 0;
  unsigned long ulEnergyCurrent_100mWh;
  unsigned long ulTemp;
#if ENERGY_MODE == 1
  unsigned int  uiEnergyCntTemp;
  double   dTemp;
#elif ENERGY_MODE == 2
  unsigned int  uiTemp;
#endif
#ifndef PROTOCOL
  #error "Please define PROTOCOL!"
#endif
#if PROTOCOL == GRT_V14
  g_ucControllerParameter[0] = 0x81;
  g_ucControllerParameter[1] = 0x73; //�豸��	
  g_ucControllerParameter[2] = g_cControllerTemperature;
  g_ucControllerParameter[3] = ScmController.uiInputVoltage >> 8;
  g_ucControllerParameter[4] = ScmController.uiInputVoltage & 0xff;
  g_ucControllerParameter[5] = ScmController.uiInputCurrent >> 8;
  g_ucControllerParameter[6] = ScmController.uiInputCurrent & 0xff;
  g_ucControllerParameter[7] = 0x00;
  g_ucControllerParameter[8] = 0x00;
  g_ucControllerParameter[9] = ScmController.uiPower >> 8;
  g_ucControllerParameter[10] = ScmController.uiPower & 0xff;
  g_ucControllerParameter[11] = 0x00;
  g_ucControllerParameter[12] = 0x00;
  g_ucControllerParameter[13] = ScmController.ucPowerFactory;	
  g_ucControllerParameter[14] = 0x0;
  if(RELAY_STATUS){
    g_ucControllerParameter[15] = g_ucDimVaule;
  }else{
    g_ucControllerParameter[15] = 0x00;
  }  
  g_ucControllerParameter[16] = 0x00;
  g_ucControllerParameter[17] = g_uiControllerStatus >> 8;
  g_ucControllerParameter[18] = g_uiControllerStatus & 0xff;
  ucSum = 0;
  for(ucCnt=0;ucCnt<19;ucCnt++){
    ucSum += g_ucControllerParameter[ucCnt];
  }	
  g_ucControllerParameter[19] = ucSum;	
#elif PROTOCOL == XJP_V01
  g_ucControllerParameter[0] = 0x93;
  g_ucControllerParameter[1] = g_cControllerTemperature; 
  if(RELAY_STATUS){
      g_ucControllerParameter[2] = ScmController.uiInputVoltage & 0xff;
      g_ucControllerParameter[3] = ScmController.uiInputVoltage >> 8;
      g_ucControllerParameter[4] = ScmController.uiInputCurrent & 0xff;
      g_ucControllerParameter[5] = ScmController.uiInputCurrent >> 8;
  }else{
      g_ucControllerParameter[2] = 0;
      g_ucControllerParameter[3] = 0;
      g_ucControllerParameter[4] = 0;
      g_ucControllerParameter[5] = 0;
  }
  g_ucControllerParameter[6] = ScmController.uiInputVoltage & 0xff;//ScmController.uiInputCurrent & 0xff;
  g_ucControllerParameter[7] = ScmController.uiInputVoltage >> 8;
  g_ucControllerParameter[8] = ScmController.uiInputCurrent & 0xff;
  g_ucControllerParameter[9] = ScmController.uiInputCurrent >> 8;//ScmController.uiPower >> 8;
  g_ucControllerParameter[10] = ScmController.uiPower & 0xff;
  g_ucControllerParameter[11] = ScmController.uiPower >> 8;
  g_ucControllerParameter[12] = ScmController.ucPowerFactory;
  g_ucControllerParameter[13] = g_uiControllerStatus & 0xff;	
  g_ucControllerParameter[14] = g_uiControllerStatus >> 8;
  if(RELAY_STATUS){
    g_ucControllerParameter[15] = g_ucDimVaule;
  }else{
    g_ucControllerParameter[15] = 0x00;
  }  
  ucSum = 0;
  if (ucMode == 17){
    for(ucCnt=1;ucCnt<16;ucCnt++){
    ucSum += g_ucControllerParameter[ucCnt];
    }	
    g_ucControllerParameter[16] = ucSum;
  }else if(ucMode == 18){
    g_ucControllerParameter[16] = g_ucNodeGroupNum;
    for(ucCnt=1;ucCnt<17;ucCnt++){
    ucSum += g_ucControllerParameter[ucCnt];
    }	
    g_ucControllerParameter[17] = ucSum;
  }else if(ucMode == 22){
    g_ucControllerParameter[0] = 0x97;
    g_ucControllerParameter[16] = 0;  //�������������ն�ֵ
    ulTemp = ReadEnergyParam();
#if ENERGY_MODE == 1
    if (g_uiEnergySaveEn != 0){ 
      ulTemp += g_uiEnergySaveEn;
    }
    uiEnergyCntTemp = g_uiEnergyCnt;
    //(ePowerCheck.fData * fg_uiPowerTimeCntCheck * ScmCSE7759.uiPowerPulseCnt) / ((float)fg_uiPowerPluseCntCheck * ScmCSE7759.uiPowerTimeCnt)
    //1�����������ĵ��ܣ�У����ģ� = ePowerCheck.fData * (fg_uiPowerTimeCntCheck/1000) / ScmCSE7759.uiPowerTimeCnt  ��λWs�����룩
    dTemp = ePowerCheck.fData * (fg_uiPowerTimeCntCheck/1000.0) / fg_uiPowerPluseCntCheck;
    ulEnergyCurrent_100mWh = (unsigned long)(dTemp * ENERGY_SAVE_CNT * ulTemp/360.0 + dTemp * uiEnergyCntTemp/360.0);  //������ܵ��ܣ���ת��Ϊ0.1Wh
    //���ܣ���λ��ǰ��
#elif ENERGY_MODE == 2
    if (g_uiEnergySaveEn != 0){
      uiTemp = g_uiEnergy_100mWs;
      ulEnergyCurrent_100mWh = ulTemp + (g_uiEnergySaveEn + uiTemp)/3600;
    }
    ulEnergyCurrent_100mWh = ulTemp + g_uiEnergy_100mWs/3600;
#endif
    g_ucControllerParameter[17] = (unsigned char)ulEnergyCurrent_100mWh;
    g_ucControllerParameter[18] = (unsigned char)(ulEnergyCurrent_100mWh >> 8);
    g_ucControllerParameter[19] = (unsigned char)(ulEnergyCurrent_100mWh >> 16);
    g_ucControllerParameter[20] = (unsigned char)(ulEnergyCurrent_100mWh >> 24);
    for(ucCnt=1;ucCnt<21;ucCnt++){
        ucSum += g_ucControllerParameter[ucCnt];
    }	
    g_ucControllerParameter[21] = ucSum;
  }
#endif 
}

#if   SEECRET_EN == 1
//��ȡID
#define     ID_BaseAddress         (0x4865)
void GetUniqueID(unsigned char *p)
{
  unsigned char i;
  unsigned char *pIDStart=(unsigned char *)(ID_BaseAddress);    
  for(i=0;i!=12;i++){*p++=*pIDStart++;}
}
//д12�ֽڵ�UID��EEPROM
void WriteUniqueToEeprom(unsigned char *p)
{
  unsigned char ucCnt;
  /*Define FLASH programming time*/
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
  /* Unlock Data memory */
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  for(ucCnt = 0; ucCnt < 12; ucCnt++){
      FLASH_ProgramByte((UidSaveAdr+ucCnt), *(p+ucCnt));
  }
  FLASH_Lock(FLASH_MEMTYPE_DATA);
}
//��EEPROM��ȡ12���ֽڵ�����
void ReadUniqueToEeprom(unsigned char *p)
{
  unsigned char ucCnt;
  for(ucCnt = 0;ucCnt<12;ucCnt++){
      *(p + ucCnt) = FLASH_ReadByte(UidSaveAdr+ucCnt);
  }
}
//�Ƚ�����12�ֽڵ������Ƿ���ͬ
//�ǣ�����1���񣺷���0
unsigned char Compare(unsigned char *puc1, unsigned char *puc2)
{   unsigned char ucCnt;
    for(ucCnt=0;ucCnt<12;ucCnt++){
      if(*(puc1 + ucCnt) != *(puc2 + ucCnt)){
        return(0);
      }else{
        continue;
      }
    }
    return(1);
}
#endif








