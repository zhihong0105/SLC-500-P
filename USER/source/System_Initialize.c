#include "System_Initialize.h"
#include "stm8s.h"
#include "main.h"
unsigned int pwm_duty  = 50;

/*******************************************************************************
* Function Name : GPIO_Config.
* Description   : STM8 时钟配置.
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
* Description   : STM8 时钟配置.
*******************************************************************************/
void ClkSet(void)
{
#ifndef SYS_CLOCK
  #error "Please define SYS_CLOCK!"
#elif SYS_CLOCK == 1
  CLK->CKDIVR &= (uint8_t)(~CLK_CKDIVR_HSIDIV); /* Clear High speed internal clock prescaler */
  CLK->CKDIVR |= (uint8_t)CLK_PRESCALER_HSIDIV1;         /* Set High speed internal clock prescaler */
#elif SYS_CLOCK == 2
  CLK->CKDIVR |= CLK_PRESCALER_CPUDIV1;               // CPU 时钟分频 1，CPU时钟 = 外部时钟(即是外部晶振频率)
  CLK->ECKR |= CLK_ECKR_HSEEN;                        // 允许外部高速振荡器工作
  while(!(CLK->ECKR & CLK_ECKR_HSERDY));              // 等待外部高速振荡器准备好
  CLK->SWCR |= CLK_SWCR_SWEN;                         // 使能切换
  CLK->SWR = CLK_SOURCE_HSE;                          // 选择芯片外部的高速振荡器为主时钟
  while(!(CLK->SWCR&CLK_SWCR_SWIF));                  // 等待切换成功
  CLK->SWCR &= ~(CLK_SWCR_SWEN|CLK_SWCR_SWIF);       // 清除切换标志
#endif
}
/*******************************************************************************
* Function Name : GPIO_Config.
* Description   : STM8 IO口配置.
*******************************************************************************/
void GPIO_Config(void)
{
  /* 将PA1/PA2/PA3设置成推挽输出 */
  GPIOA->DDR = 0x0E;
  GPIOA->CR1 = 0x0E;
  GPIOA->CR2 = 0x00;
  /* 将PB4/PB5设置成推挽输出*/
  GPIOB->DDR = 0x30;
  GPIOB->CR1 = 0x30;
  GPIOB->CR2 = 0x00;
  /* 将PC3/PC4设置为上拉输入；PC5/PC6设置为推挽输出；PC7设置为输入，带中断 */
  GPIOC->DDR = 0x60;	 
  GPIOC->CR1 = 0x78;	// PC3、PC4带上拉输入
  GPIOC->CR2 = 0x80;
  EXTI->CR1 = 0x10;    //设置PC口的端口在上升沿产生中断
  //设置PC口中断级别为3(HIGH)，其他中断为2(LOW)；
  ITC->ISPR2 = 0x0c;     
  ITC->ISPR4 = 0x00;     
  ITC->ISPR5 = 0x00;    
  /* 将PD5为UART_TX,PD6为UART_RX设置成上拉输入, */
  /* PD4推挽输出 PD3/PD2为AIN设置成悬浮输入 */
  GPIOD->DDR = 0x30;
  GPIOD->CR1 = 0x30;
  GPIOD->CR2 = 0x00;	
  //ADC_TDRL = 0x0c; // 关闭施密特
}
/*******************************************************************************
* Function Name : WDG_Config.
* Description   : 看门狗配置.
*******************************************************************************/
void WDG_Config(void)
{
    IWDG->KR = 0xCC; //启动IWDG
    IWDG->KR = 0x55; //允许对受保护的IWDG_PR和IWDG_RLR寄存器的操作.
    IWDG->PR = 0x06; //预分频系数 256
    IWDG->RLR= 0xFF; //看门狗计数器重装载数值，看门狗的周期约1.02s
    IWDG->KR = 0xAA; //喂狗
}
/*******************************************************************************
* Function Name : UART1_Config.
* Description   : 串口1配置，9600 中断接收.
*******************************************************************************/
void UART1_Config(void)
{
    unsigned int baudBRR = 0;
    baudBRR = FMASTER / BAUD_RATE;
    UART1->CR1 = 0X00; // 1个起始位，8个数据位，无奇偶校验
    UART1->CR2 = 0x00; 
    UART1->CR3 = 0x00; // 1个停止位，SCK引脚被禁止   
    UART1->SR = 0;// 状态寄存器
    // 设置波特率，必须注意以下几点：
    // (1) 必须先写BRR2
    // (2) BRR1存放的是分频系数的第11位到第4位，
    // (3) BRR2存放的是分频系数的第15位到第12位，和第3位到第0 位
    UART1->BRR2 = (unsigned char)(((baudBRR >> 8) & 0x00f0) \
                                               | (baudBRR & 0x000f));   
    UART1->BRR1 = (unsigned char)(baudBRR >> 4);
    // 控制寄存器2
    UART1->CR2 = 0x2C; 
}
/*******************************************************************************
* Function Name : ADC1_Config.
* Description   : ADC配置.
*******************************************************************************/
void ADC1_Config(void)
{
    /* 控制、状态寄存器 */
    //ADC1->CSR = 0x24;// 使能转换结束中断，扫描范围AIN3
    ADC1->CSR = 0x04;
    ADC1->CR1 = 0x70;  // fADC=fMASTER/18，未启动ADC,单次扫描模式
    ADC1->CR2 = 0x08;  // 右对齐，单次模式
    ADC1->CR3 = 0x80;  // 数据缓存功能
    /* 施密特触发禁止寄存器 */
    ADC1->TDRL = 0x18; // 关闭AN3~AN4施密特
}

/*******************************************************************************
* Function Name : UART1_SendChar.
* Description   : 串口一发送一个字符.
*******************************************************************************/
void UART1_SendChar(unsigned char ch)
{			
    UART1->DR = ch ;// 将要发送的字符送的数据寄存器
    while((UART1->SR & 0x80) == 0);// 如果发送寄存器不为空，则等待
}
/*******************************************************************************
* Function Name : UART1_SendString.
* Description   : 串口一发送字符串.
*******************************************************************************/
void UART1_SendString(unsigned char* Data,unsigned int len)
{
    unsigned int i=0;
    for(;i<len;i++)
      UART1_SendChar(Data[i]);
}
/*******************************************************************************
* Function Name : UART1_SendChar.
* Description   : 串口一发送一个字符.
*******************************************************************************/
void TIM1_Config(void)
{
    /* 事件产生寄存器 */
    //TIM1->EGR = 0x21;   // 允许产生更新事件	//开始不用                	
    //TIM1->EGR |= 0x18;  // 允许更新CCIE、CCINE、CCiP，CCiNP，OCIM位//开始不用 
    /* 控制寄存器 */
    TIM1->CR1 = 0x80;     //TIM1_ARR由预装载缓冲器缓冲，边沿对齐模式，计数器向上计数；禁止计数器计数；
    TIM1->CR2 = 0x00;	
    TIM1->SMCR = 0x00;/* 从模式控制寄存器 */	
    TIM1->ETR = 0x00; /* 外部触发寄存器 */	
    TIM1->IER = 0x19; /* 定时器中断使能寄存器 */	 // 允许捕获3/4中断和更新中断。	
    TIM1->SR1 = 0x00; /* 状态寄存器1 */	
    TIM1->SR2 = 0x00; /* 状态寄存器2 */
    /******* 比较模式寄存器 *******/
    TIM1->CCMR1 = 0x68;	// PWM模式1
    TIM1->CCMR2 = 0x01;
    TIM1->CCMR3 = 0xF1;	// CH3 配置为输入IC3映射在TI3FP3上，
                        // 采样频率fSAMPLING=fMASTER，N=2，
                        // 无预分频器
    TIM1->CCMR4 = 0xF1;	// CH4 配置为输入IC4映射在TI4FP4上 0x12
                        // 采样频率fSAMPLING=fMASTER，N=2，
                        // 无预分频器
    /******* 比较输出使能寄存器 *******/
    TIM1->CCER1 = 0x01;	// 使能 PWM TIM1_CH1 输出 高电平有效
    TIM1->CCER2 = 0x11;	// 捕捉发生在TI3F(T14F))的高电平或上升沿；
                        // 捕获使能
    /******* 计数器的值 *******/
    TIM1->CNTRH;
    TIM1->CNTRL;
    /******* 时钟预分频，1~65535任意 *******/
    /******* fck_cnt = fck_psc/(PSCR[15:0] + 1) *******/
    TIM1->PSCRH = 0;
    TIM1->PSCRL = 0;
    /******* 自动重装载寄存器 *******/
    TIM1->ARRH = (unsigned char)(PWM_TCNT0_10V >> 8);// 确定PWM输出频率 2K
    TIM1->ARRL = (unsigned char)(PWM_TCNT0_10V & 0xFF);
    /******* 重复计数寄存器 *******/
    TIM1->RCR = 0;
    /******* 比较寄存器的值 *******/
    TIM1->CCR1H = (unsigned char)(pwm_duty >> 8);//set 占空比
    TIM1->CCR1L = (unsigned char)(pwm_duty&0xFF);
    /******* 刹车寄存器 *******/
    TIM1->BKR = 0x80;	//主使能
    /******* 死区寄存器 *******/
    TIM1->DTR;
    /******* 输出空闲状态寄存器 *******/
    TIM1->OISR;
    // TIM1->CR1 |= 0x01;//zj 启动计数器
    //使能输入计量测试通道
    TIM1_ITConfig(TIM1_IT_CC3, ENABLE); 
    TIM1_ITConfig(TIM1_IT_CC4, ENABLE); 
}
/*******************************************************************************
* Function Name : Set_0_10V_Output.
* Description   : 设置0-10V输出.
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
/* 函数功能： 配置TIM2模块                                 */
/* 输入参数： 无                                           */
/* 输出参数： 无                                           */
/* 返回值： 无                                             */
/* 备注：  PWM调光信号输出                                 */
/***********************************************************/
void TIM2_Config(void)
{
    TIM2->CR1 = 0x00;	// 溢出允许产生中断，使能计数器
    TIM2->IER = 0x01;	// 允许更新中断。此处用更新中断计时
    TIM2->SR1 = 0;
    TIM2->EGR = 0;
    TIM2->CCMR1 = 0x68;	// PWM模式1，开启TIMx_CCR1寄存器的预装载功能
    TIM2->CCER1 = 0x03;	// 开启--OC1信号输出到对应的输出引脚
    TIM2->PSCR  = 0x00;	// Fck_cnt = Fck_psc/2^(psc[3:0]) = Fck_psc/1
    TIM2->ARRH  = (unsigned char)(PWM_TCNT>> 8);// 设置PWM输出的频率 400Hz
    TIM2->ARRL  = (unsigned char)PWM_TCNT; 
    TIM2->CCR1H = 3;	// 设置占空比
    TIM2->CCR1L = 244;      
}
/*******************************************************************************
* Function Name : Set_PWM_Output.
* Description   : 设置PWM输出，ucPulseWide 0-200.
*******************************************************************************/
void Set_PWM_Output(unsigned char ucPulseWide)
{
    unsigned int uiTemp = 0;
    uiTemp = (unsigned int)(ucPulseWide / 200.0 * PWM_TCNT) + 1;   
    TIM2->CCR1H = uiTemp>>8;	// 设置占空比
    TIM2->CCR1L = uiTemp&0xff; 
}
/***********************************************************/
/* 函数功能： 配置TIM4模块                                 */
/* 输入参数： 无                                           */
/* 输出参数： 无                                           */
/* 返回值： 无                                             */
/* 备注：   无                                             */
/***********************************************************/
void TIM4_Config(void)
{
    // 控制寄存器1
    TIM4->CR1 = 0x80;	// TIM4_ARR寄存器通过缓冲预装载
    // 中断使能寄存器
    TIM4->IER = 0x00;	// 关闭中断
    // 状态寄存器
    TIM4->SR1 = 0;
    // 事件产生寄存器
    TIM4->EGR = 0;
    // 预分频寄存器
    // fCK_CNT = fCK_PSC/2^(PSC[2：0]) = fMASTER / 2^6
    // fCK_CNT = 16MHz / 64 = 250kHz
    TIM4->PSCR = 0x06;
    // 自动重装载寄存器
    // 定时时间 = ARR / fCK_CNT = 125 / 125k = 1ms
    //TIM4->ARR = 125;
    TIM4->ARR = FMASTER/64/1000; //1ms
}
/*******************************************************************************
* Function Name : Delay_ms.
* Description   : 延时函数，ms.
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
    FLASH_ProgramByte(StartAddress+i, ucTempData); //低位在低地址
    lData = lData>>8;     
    ucTempData = lData&0xff;
  }    
  FLASH_Lock(FLASH_MEMTYPE_DATA);  
}

long ReadLongDataFromFlash(unsigned int StartAddress)
{
  long lTempData = 0;
  
  lTempData = FLASH_ReadByte(StartAddress+3);//高位在高地址 先读高地址位
  lTempData = lTempData<<8;
  lTempData += FLASH_ReadByte(StartAddress+2);
  lTempData = lTempData<<8;
  lTempData += FLASH_ReadByte(StartAddress+1);
  lTempData = lTempData<<8;
  lTempData += FLASH_ReadByte(StartAddress); 
  return lTempData;
}

//电能保存读取函数
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
   //判断eeprom里面有没有保存过数据
    if(ucCnt==0){
      if((sEnergyParam.ucGroupNumber & 0x80) == 0){ 
        sEnergyParamLast.ucGroupNumber = 0;  
        sEnergyParamLast.ulEnergy_wh = 0;
        *puiAddress = ENERGY_SAVE_ADR_START;
        break;
      }  
    }
    //如果上一个数据有效，这一个数据无效，那么上一个数据就是最终有效数据。（用于未保存完一轮的情况）
    if((sEnergyParamLast.ucGroupNumber & 0x80) && (sEnergyParam.ucGroupNumber & 0x80)==0){
       *puiAddress = uiAdressTemp;
       break;
    }
    //找到eeprom最后存入的数据，即有效数据。
    if((sEnergyParamLast.ucGroupNumber & 0x80) && (sEnergyParam.ucGroupNumber & 0x80)){
      if(((sEnergyParamLast.ucGroupNumber + 1) & 0x7f) != (sEnergyParam.ucGroupNumber & 0x7f)){
        *puiAddress = uiAdressTemp;
        break;
      }
    }
    //计算下一个保存地址 
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
  sEnergyTemp = ReadEnergyParamAndAdress(&uiEepromAdress);//取得可以保存的eeprom地址和保存的序号
  sEnergyTemp.ucGroupNumber++;        //组号加1
  sEnergyTemp.ucGroupNumber |= 0X80;  //组号最高位置1
  sEnergyTemp.ulEnergy_wh = ulEnergy_wh;
  /*Define FLASH programming time*/
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
    /* Unlock Data memory */
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  FLASH->IAPSR &= ~0x04;
  for(i=0;i<ENERGY_GROUP_LONG;i++){
    FLASH_ProgramByte(uiEepromAdress+i, *((unsigned char*)(&sEnergyTemp) + i)); //低位在低地址 
    while((FLASH->IAPSR & 0x04) == 0);
    LED_Toggle();
    FLASH->IAPSR &= ~0x04;
  }  
  FLASH_Lock(FLASH_MEMTYPE_DATA);

 
}


