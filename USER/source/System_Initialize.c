#include "System_Initialize.h"
#include "stm8s.h"
#include "main.h"
unsigned int pwm_duty  = 50;

/*******************************************************************************
* Function Name : GPIO_Config.
* Description   : STM8 ʱ������.
*******************************************************************************/
void UART1_IT_Set(unsigned short int UART1_IT, char NewState)
{
    uint8_t uartreg = 0, itpos = 0x00;
    /* Get the UART1 register index */
    uartreg = (uint8_t)((uint16_t)UART1_IT >> 0x08);
    /* Get the UART1 IT index */
    itpos = (uint8_t)((uint8_t)1 << (uint8_t)((uint8_t)UART1_IT & (uint8_t)0x0F));

    if (NewState != 0)
    {
        /**< Enable the Interrupt bits according to UART1_IT mask */
        if (uartreg == 0x01)
        {
            UART1->CR1 |= itpos;
        }
        else if (uartreg == 0x02)
        {
            UART1->CR2 |= itpos;
        }
        else
        {
            UART1->CR4 |= itpos;
        }
    }
    else
    {
        /**< Disable the interrupt bits according to UART1_IT mask */
        if (uartreg == 0x01)
        {
            UART1->CR1 &= (uint8_t)(~itpos);
        }
        else if (uartreg == 0x02)
        {
            UART1->CR2 &= (uint8_t)(~itpos);
        }
        else
        {
            UART1->CR4 &= (uint8_t)(~itpos);
        }
    }

}
/*******************************************************************************
* Function Name : ClkSet.
* Description   : STM8 ʱ������.
*******************************************************************************/
void ClkSet(void)
{
#ifndef SYS_CLOCK
  #error "Please define SYS_CLOCK!"
#elif SYS_CLOCK == 1
  CLK->CKDIVR &= (uint8_t)(~CLK_CKDIVR_HSIDIV); /* Clear High speed internal clock prescaler */
  CLK->CKDIVR |= (uint8_t)CLK_PRESCALER_HSIDIV1;         /* Set High speed internal clock prescaler */
#elif SYS_CLOCK == 2
  CLK->CKDIVR |= CLK_PRESCALER_CPUDIV1;               // CPU ʱ�ӷ�Ƶ 1��CPUʱ�� = �ⲿʱ��(�����ⲿ����Ƶ��)
  CLK->ECKR |= CLK_ECKR_HSEEN;                        // �����ⲿ������������
  while(!(CLK->ECKR & CLK_ECKR_HSERDY));              // �ȴ��ⲿ��������׼����
  CLK->SWCR |= CLK_SWCR_SWEN;                         // ʹ���л�
  CLK->SWR = CLK_SOURCE_HSE;                          // ѡ��оƬ�ⲿ�ĸ�������Ϊ��ʱ��
  while(!(CLK->SWCR&CLK_SWCR_SWIF));                  // �ȴ��л��ɹ�
  CLK->SWCR &= ~(CLK_SWCR_SWEN|CLK_SWCR_SWIF);       // ����л���־
#endif
}
/*******************************************************************************
* Function Name : GPIO_Config.
* Description   : STM8 IO������.
*******************************************************************************/
void GPIO_Config(void)
{
  /* ��PA1/PA2/PA3���ó�������� */
  GPIOA->DDR = 0x0E;
  GPIOA->CR1 = 0x0E;
  GPIOA->CR2 = 0x00;
  /* ��PB4/PB5���ó��������*/
  GPIOB->DDR = 0x30;
  GPIOB->CR1 = 0x30;
  GPIOB->CR2 = 0x00;
  /* ��PC3/PC4����Ϊ�������룻PC5/PC6����Ϊ���������PC7����Ϊ���룬���ж� */
  GPIOC->DDR = 0x60;	 
  GPIOC->CR1 = 0x78;	// PC3��PC4����������
  GPIOC->CR2 = 0x80;
  EXTI->CR1 = 0x10;    //����PC�ڵĶ˿��������ز����ж�
  //����PC���жϼ���Ϊ3(HIGH)�������ж�Ϊ2(LOW)��
  ITC->ISPR2 = 0x0c;     
  ITC->ISPR4 = 0x00;     
  ITC->ISPR5 = 0x00;    
  /* ��PD5ΪUART_TX,PD6ΪUART_RX���ó���������, */
  /* PD4������� PD3/PD2ΪAIN���ó��������� */
  GPIOD->DDR = 0x30;
  GPIOD->CR1 = 0x30;
  GPIOD->CR2 = 0x00;	
  //ADC_TDRL = 0x0c; // �ر�ʩ����
}
/*******************************************************************************
* Function Name : WDG_Config.
* Description   : ���Ź�����.
*******************************************************************************/
void WDG_Config(void)
{
    IWDG->KR = 0xCC; //����IWDG
    IWDG->KR = 0x55; //������ܱ�����IWDG_PR��IWDG_RLR�Ĵ����Ĳ���.
    IWDG->PR = 0x06; //Ԥ��Ƶϵ�� 256
    IWDG->RLR= 0xFF; //���Ź���������װ����ֵ�����Ź�������Լ1.02s
    IWDG->KR = 0xAA; //ι��
}
/*******************************************************************************
* Function Name : UART1_Config.
* Description   : ����1���ã�9600 �жϽ���.
*******************************************************************************/
void UART1_Config(void)
{
    unsigned int baudBRR = 0;
    baudBRR = FMASTER / BAUD_RATE;
    UART1->CR1 = 0X00; // 1����ʼλ��8������λ������żУ��
    UART1->CR2 = 0x00; 
    UART1->CR3 = 0x00; // 1��ֹͣλ��SCK���ű���ֹ   
    UART1->SR = 0;// ״̬�Ĵ���
    // ���ò����ʣ�����ע�����¼��㣺
    // (1) ������дBRR2
    // (2) BRR1��ŵ��Ƿ�Ƶϵ���ĵ�11λ����4λ��
    // (3) BRR2��ŵ��Ƿ�Ƶϵ���ĵ�15λ����12λ���͵�3λ����0 λ
    UART1->BRR2 = (unsigned char)(((baudBRR >> 8) & 0x00f0) \
                                               | (baudBRR & 0x000f));   
    UART1->BRR1 = (unsigned char)(baudBRR >> 4);
    // ���ƼĴ���2
    UART1->CR2 = 0x2C; 
}
/*******************************************************************************
* Function Name : ADC1_Config.
* Description   : ADC����.
*******************************************************************************/
void ADC1_Config(void)
{
    /* ���ơ�״̬�Ĵ��� */
    //ADC1->CSR = 0x24;// ʹ��ת�������жϣ�ɨ�跶ΧAIN3
    ADC1->CSR = 0x04;
    ADC1->CR1 = 0x70;  // fADC=fMASTER/18��δ����ADC,����ɨ��ģʽ
    ADC1->CR2 = 0x08;  // �Ҷ��룬����ģʽ
    ADC1->CR3 = 0x80;  // ���ݻ��湦��
    /* ʩ���ش�����ֹ�Ĵ��� */
    ADC1->TDRL = 0x18; // �ر�AN3~AN4ʩ����
}

/*******************************************************************************
* Function Name : UART1_SendChar.
* Description   : ����һ����һ���ַ�.
*******************************************************************************/
void UART1_SendChar(unsigned char ch)
{			
    UART1->DR = ch ;// ��Ҫ���͵��ַ��͵����ݼĴ���
    while((UART1->SR & 0x80) == 0);// ������ͼĴ�����Ϊ�գ���ȴ�
}
/*******************************************************************************
* Function Name : UART1_SendString.
* Description   : ����һ�����ַ���.
*******************************************************************************/
void UART1_SendString(unsigned char* Data,unsigned int len)
{
    unsigned int i=0;
    for(;i<len;i++)
      UART1_SendChar(Data[i]);
}
/*******************************************************************************
* Function Name : UART1_SendChar.
* Description   : ����һ����һ���ַ�.
*******************************************************************************/
void TIM1_Config(void)
{
    /* �¼������Ĵ��� */
    //TIM1->EGR = 0x21;   // ������������¼�	//��ʼ����                	
    //TIM1->EGR |= 0x18;  // �������CCIE��CCINE��CCiP��CCiNP��OCIMλ//��ʼ���� 
    /* ���ƼĴ��� */
    TIM1->CR1 = 0x80;     //TIM1_ARR��Ԥװ�ػ��������壬���ض���ģʽ�����������ϼ�������ֹ������������
    TIM1->CR2 = 0x00;	
    TIM1->SMCR = 0x00;/* ��ģʽ���ƼĴ��� */	
    TIM1->ETR = 0x00; /* �ⲿ�����Ĵ��� */	
    TIM1->IER = 0x19; /* ��ʱ���ж�ʹ�ܼĴ��� */	 // ������3/4�жϺ͸����жϡ�	
    TIM1->SR1 = 0x00; /* ״̬�Ĵ���1 */	
    TIM1->SR2 = 0x00; /* ״̬�Ĵ���2 */
    /******* �Ƚ�ģʽ�Ĵ��� *******/
    TIM1->CCMR1 = 0x68;	// PWMģʽ1
    TIM1->CCMR2 = 0x01;
    TIM1->CCMR3 = 0xF1;	// CH3 ����Ϊ����IC3ӳ����TI3FP3�ϣ�
                        // ����Ƶ��fSAMPLING=fMASTER��N=2��
                        // ��Ԥ��Ƶ��
    TIM1->CCMR4 = 0xF1;	// CH4 ����Ϊ����IC4ӳ����TI4FP4�� 0x12
                        // ����Ƶ��fSAMPLING=fMASTER��N=2��
                        // ��Ԥ��Ƶ��
    /******* �Ƚ����ʹ�ܼĴ��� *******/
    TIM1->CCER1 = 0x01;	// ʹ�� PWM TIM1_CH1 ��� �ߵ�ƽ��Ч
    TIM1->CCER2 = 0x11;	// ��׽������TI3F(T14F))�ĸߵ�ƽ�������أ�
                        // ����ʹ��
    /******* ��������ֵ *******/
    TIM1->CNTRH;
    TIM1->CNTRL;
    /******* ʱ��Ԥ��Ƶ��1~65535���� *******/
    /******* fck_cnt = fck_psc/(PSCR[15:0] + 1) *******/
    TIM1->PSCRH = 0;
    TIM1->PSCRL = 0;
    /******* �Զ���װ�ؼĴ��� *******/
    TIM1->ARRH = (unsigned char)(PWM_TCNT0_10V >> 8);// ȷ��PWM���Ƶ�� 2K
    TIM1->ARRL = (unsigned char)(PWM_TCNT0_10V & 0xFF);
    /******* �ظ������Ĵ��� *******/
    TIM1->RCR = 0;
    /******* �ȽϼĴ�����ֵ *******/
    TIM1->CCR1H = (unsigned char)(pwm_duty >> 8);//set ռ�ձ�
    TIM1->CCR1L = (unsigned char)(pwm_duty&0xFF);
    /******* ɲ���Ĵ��� *******/
    TIM1->BKR = 0x80;	//��ʹ��
    /******* �����Ĵ��� *******/
    TIM1->DTR;
    /******* �������״̬�Ĵ��� *******/
    TIM1->OISR;
    // TIM1->CR1 |= 0x01;//zj ����������
    //ʹ�������������ͨ��
    TIM1_ITConfig(TIM1_IT_CC3, ENABLE); 
    TIM1_ITConfig(TIM1_IT_CC4, ENABLE); 
}
/*******************************************************************************
* Function Name : Set_0_10V_Output.
* Description   : ����0-10V���.
*******************************************************************************/ 

void Set_0_10V_Output(unsigned int uiPulseWide)
{
 /* unsigned int uiTemp = 0;
  uiTemp = ucPulseWide*40;
  TIM1->CCR1H = uiTemp>>8;
  TIM1->CCR1L = uiTemp&0xff;*/
  uiPulseWide = PWM_TCNT0_10V - uiPulseWide;
  TIM1->CCR1H = uiPulseWide >> 8;     //CCR1 8000 100%
  TIM1->CCR1L = uiPulseWide & 0xff;
}
    
/***********************************************************/
/* �������ܣ� ����TIM2ģ��                                 */
/* ��������� ��                                           */
/* ��������� ��                                           */
/* ����ֵ�� ��                                             */
/* ��ע��  PWM�����ź����                                 */
/***********************************************************/
void TIM2_Config(void)
{
    TIM2->CR1 = 0x00;	// �����������жϣ�ʹ�ܼ�����
    TIM2->IER = 0x01;	// ��������жϡ��˴��ø����жϼ�ʱ
    TIM2->SR1 = 0;
    TIM2->EGR = 0;
    TIM2->CCMR1 = 0x68;	// PWMģʽ1������TIMx_CCR1�Ĵ�����Ԥװ�ع���
    TIM2->CCER1 = 0x03;	// ����--OC1�ź��������Ӧ���������
    TIM2->PSCR  = 0x00;	// Fck_cnt = Fck_psc/2^(psc[3:0]) = Fck_psc/1
    TIM2->ARRH  = (unsigned char)(PWM_TCNT>> 8);// ����PWM�����Ƶ�� 400Hz
    TIM2->ARRL  = (unsigned char)PWM_TCNT; 
    TIM2->CCR1H = 3;	// ����ռ�ձ�
    TIM2->CCR1L = 244;      
}
/*******************************************************************************
* Function Name : Set_PWM_Output.
* Description   : ����PWM�����ucPulseWide 0-200.
*******************************************************************************/
void Set_PWM_Output(unsigned char ucPulseWide)
{
    unsigned int uiTemp = 0;
    uiTemp = (unsigned int)(ucPulseWide / 200.0 * PWM_TCNT) + 1;   
    TIM2->CCR1H = uiTemp>>8;	// ����ռ�ձ�
    TIM2->CCR1L = uiTemp&0xff; 
}
/***********************************************************/
/* �������ܣ� ����TIM4ģ��                                 */
/* ��������� ��                                           */
/* ��������� ��                                           */
/* ����ֵ�� ��                                             */
/* ��ע��   ��                                             */
/***********************************************************/
void TIM4_Config(void)
{
    // ���ƼĴ���1
    TIM4->CR1 = 0x80;	// TIM4_ARR�Ĵ���ͨ������Ԥװ��
    // �ж�ʹ�ܼĴ���
    TIM4->IER = 0x00;	// �ر��ж�
    // ״̬�Ĵ���
    TIM4->SR1 = 0;
    // �¼������Ĵ���
    TIM4->EGR = 0;
    // Ԥ��Ƶ�Ĵ���
    // fCK_CNT = fCK_PSC/2^(PSC[2��0]) = fMASTER / 2^6
    // fCK_CNT = 16MHz / 64 = 250kHz
    TIM4->PSCR = 0x06;
    // �Զ���װ�ؼĴ���
    // ��ʱʱ�� = ARR / fCK_CNT = 125 / 125k = 1ms
    //TIM4->ARR = 125;
    TIM4->ARR = FMASTER/64/1000; //1ms
}
/*******************************************************************************
* Function Name : Delay_ms.
* Description   : ��ʱ������ms.
*******************************************************************************/
void Delay_ms(unsigned int ms)
{ 
    volatile unsigned int i,j;
    for(j=0;j<ms;j++){   
      for(i=0;i<0x1ff;i++);
      
    }
   // IWDG->PR = IWDG_Prescaler_4;
   // IWDG->RLR = 0x00;
}

/*******************************************************************************
* Function Name : WriteGroupToFlash.
* Description   : WriteGroupToFlash.
* Input         : .
* Output        : .
* Return        : .
*******************************************************************************/
void WriteGroupToFlash(unsigned int StartAddress,unsigned char ucData)
{
    WriteDataToFlash(StartAddress, ucData);
}


void WriteDataToFlash(unsigned int StartAddress,unsigned char ucData)
{
    /*Define FLASH programming time*/
    FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
    /* Unlock Data memory */
    FLASH_Unlock(FLASH_MEMTYPE_DATA);
    FLASH_ProgramByte(StartAddress, ucData);
    FLASH_Lock(FLASH_MEMTYPE_DATA);  
}

void WriteLongDataToFlash(unsigned int StartAddress,long lData)
{
  unsigned char i;
  unsigned char ucTempData;
  ucTempData = lData&0xff;
  /*Define FLASH programming time*/
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
    /* Unlock Data memory */
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  for(i=0;i<4;i++){
    FLASH_ProgramByte(StartAddress+i, ucTempData); //��λ�ڵ͵�ַ
    lData = lData>>8;     
    ucTempData = lData&0xff;
  }    
  FLASH_Lock(FLASH_MEMTYPE_DATA);  
}

long ReadLongDataFromFlash(unsigned int StartAddress)
{
  long lTempData = 0;
  
  lTempData = FLASH_ReadByte(StartAddress+3);//��λ�ڸߵ�ַ �ȶ��ߵ�ַλ
  lTempData = lTempData<<8;
  lTempData += FLASH_ReadByte(StartAddress+2);
  lTempData = lTempData<<8;
  lTempData += FLASH_ReadByte(StartAddress+1);
  lTempData = lTempData<<8;
  lTempData += FLASH_ReadByte(StartAddress); 
  return lTempData;
}

//���ܱ����ȡ����
/*******************************************************************************
* Function Name : ReadEnergyGroup.
* Description   : Read one Group Energy param from eeprom.
* Input         : uiAddress: Start adress of the energy group.
* Output        : none.
* Return        : Energy param.
*******************************************************************************/
static ScmEnergyGroup ReadEnergyGroup(unsigned int uiAddress)
{
  unsigned char ucCnt;
  ScmEnergyGroup sEnergyParam;
  unsigned char *pucTemp;
  pucTemp = (unsigned char*)(&sEnergyParam);
  for(ucCnt=0;ucCnt<ENERGY_GROUP_LONG;ucCnt++){
    *(pucTemp + ucCnt) = FLASH_ReadByte(uiAddress + ucCnt);
  }
  return sEnergyParam;
}
/*******************************************************************************
* Function Name : ReadEnergyParamAndAdress.
* Description   : Read valable Energy param from eeprom.
* Input         : puiAddress: The adress can saved param in.(cooperate with Save energy)
* Output        : none.
* Return        : Energy param.
*******************************************************************************/
static ScmEnergyGroup ReadEnergyParamAndAdress(unsigned int *puiAddress)
{
  unsigned char ucCnt;
  unsigned int  uiAdressTemp;
  ScmEnergyGroup sEnergyParam,sEnergyParamLast;
  sEnergyParamLast.ucGroupNumber = 0;
  sEnergyParamLast.ulEnergy_wh = 0;
  uiAdressTemp = ENERGY_SAVE_ADR_START;
  for(ucCnt=0;ucCnt<ENERGY_GROUP_TOTAL;ucCnt++){
    sEnergyParam = ReadEnergyGroup(uiAdressTemp);
   //�ж�eeprom������û�б��������
    if(ucCnt==0){
      if((sEnergyParam.ucGroupNumber & 0x80) == 0){ 
        sEnergyParamLast.ucGroupNumber = 0;  
        sEnergyParamLast.ulEnergy_wh = 0;
        *puiAddress = ENERGY_SAVE_ADR_START;
        break;
      }  
    }
    //�����һ��������Ч����һ��������Ч����ô��һ�����ݾ���������Ч���ݡ�������δ������һ�ֵ������
    if((sEnergyParamLast.ucGroupNumber & 0x80) && (sEnergyParam.ucGroupNumber & 0x80)==0){
       *puiAddress = uiAdressTemp;
       break;
    }
    //�ҵ�eeprom����������ݣ�����Ч���ݡ�
    if((sEnergyParamLast.ucGroupNumber & 0x80) && (sEnergyParam.ucGroupNumber & 0x80)){
      if(((sEnergyParamLast.ucGroupNumber + 1) & 0x7f) != (sEnergyParam.ucGroupNumber & 0x7f)){
        *puiAddress = uiAdressTemp;
        break;
      }
    }
    //������һ�������ַ 
    if(ucCnt == ENERGY_GROUP_TOTAL-1){         
      uiAdressTemp = ENERGY_SAVE_ADR_START;
    }else{
      uiAdressTemp += ENERGY_GROUP_LONG;
    }
    *puiAddress = uiAdressTemp;
    sEnergyParamLast = sEnergyParam;
  }
  return sEnergyParamLast;
}

/*******************************************************************************
* Function Name : ReadEnergyParam.
* Description   : Read valable Energy param from eeprom.
* Input         : none.
* Output        : none.
* Return        : Energy param.
*******************************************************************************/
unsigned long ReadEnergyParam(void)
{
  unsigned int  uiEepromAdress;
  ScmEnergyGroup  sEnergyTemp;
  sEnergyTemp = ReadEnergyParamAndAdress(&uiEepromAdress);
//sEnergyTemp = ReadEnergyGroup(ENERGY_SAVE_ADR_START);
  return sEnergyTemp.ulEnergy_wh;
//return uiEepromAdress;
}
/*******************************************************************************
* Function Name : SaveEnergy.
* Description   : Save energy param to eeprom.
* Input         : ulEnergy_wh: The energy param which needs to save.
* Output        : none.
* Return        : none.
*******************************************************************************/
void SaveEnergy(unsigned long ulEnergy_wh)
{
  unsigned char i;
  unsigned int  uiEepromAdress;
  ScmEnergyGroup  sEnergyTemp;
  sEnergyTemp = ReadEnergyParamAndAdress(&uiEepromAdress);//ȡ�ÿ��Ա����eeprom��ַ�ͱ�������
  sEnergyTemp.ucGroupNumber++;        //��ż�1
  sEnergyTemp.ucGroupNumber |= 0X80;  //������λ��1
  sEnergyTemp.ulEnergy_wh = ulEnergy_wh;
  /*Define FLASH programming time*/
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
    /* Unlock Data memory */
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  FLASH->IAPSR &= ~0x04;
  for(i=0;i<ENERGY_GROUP_LONG;i++){
    FLASH_ProgramByte(uiEepromAdress+i, *((unsigned char*)(&sEnergyTemp) + i)); //��λ�ڵ͵�ַ 
    while((FLASH->IAPSR & 0x04) == 0);
    LED_Toggle();
    FLASH->IAPSR &= ~0x04;
  }  
  FLASH_Lock(FLASH_MEMTYPE_DATA);

 
}


