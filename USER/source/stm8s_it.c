/**
  ******************************************************************************
  * @file     stm8s_it.c
  * @author   MJX Application Team
  * @version  V2.1.0
  * @date     26-MAY-2014
  * @brief    Main Interrupt Service Routines.
  *           This file provides template for all peripherals interrupt service 
  *           routine.
  ******************************************************************************
  */ 
/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_it.h"
#include "System_Initialize.h"
#include "main.h" 
#include "TaskHandle.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
#define RELAY_OPERATE_DELAY_ON     6
#define RELAY_OPERATE_DELAY_OFF    6
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern ScmCSE7759TypeDef ScmCSE7759;
static unsigned short int fg_uiPowerCC4Cnt = 0,fg_uiVolCurCC3Cnt = 0;
extern unsigned char g_ucReceiveTimeCnt;
extern unsigned char g_ucReceiveDataLen ;
extern volatile unsigned char g_ucRelayOperateRequest;
static unsigned char fg_ucRelayOperateDelay = 0;

static unsigned char fg_uc7759PowerState = 0;  //7759测量的辅助标志
static unsigned char fg_uc7759VIState = 0;  //7759测量的辅助标志
static unsigned short int fg_uiPowerTimeCnt = 0;
static unsigned short int fg_uiVITimeCnt = 0;
#ifndef TEST_ON_OFF
  #error "Please define TEST_ON_OFF!" 
#elif TEST_ON_OFF == 1
  extern volatile unsigned int g_uiOnOffInterval;  //开关的时间间隔，单位ms
  extern volatile unsigned char g_ucOnOffFlag;
#endif
#ifndef SEECRET_EN
    #error "Please define SEECRET_EN!"
#elif   SEECRET_EN == 1
  extern volatile unsigned short int g_uiDelayTime;
#endif

//电能计量
extern volatile unsigned char g_uiEnergySaveEn;  
extern volatile unsigned long ulEnergyTotalPluse;
#if ENERGY_MODE == 1
extern volatile unsigned int  g_uiEnergyCnt;
#elif ENERGY_MODE == 2
extern volatile unsigned int  g_uiEnergy_100mWs;         //电能数量。
extern volatile unsigned long g_uiEnergyTotal_100mWh;    //电能总数量。
#endif
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

#ifdef _COSMIC_
 
/**
  * @brief Dummy Interrupt routine
  * @par Parameters:
  * None
  * @retval
  * None
*/
INTERRUPT_HANDLER(NonHandledInterrupt, 25)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#endif /*_COSMIC_*/


/**
  * @brief Auto Wake Up Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(AWU_IRQHandler, 1)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief SPI Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(SPI_IRQHandler, 10)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}


// 替代TIM4 1ms interrupt
static void Function1ms(void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */ // 1ms interrupt
    static unsigned short int s_uiDelayTimeCnt = 0;
    static unsigned short int s_uiWaitPowerTimeCnt, s_uiWaitVITimeCnt;
#if ENERGY_MODE == 2
    static unsigned short int s_uiOneSecondCnt = 0;
    unsigned short int uiTemp;
#endif
 PB5_Toggle();
    //功率测量
    if((g_ucCSE7759Flag & POWER_BIT) == 0){
      switch(fg_uc7759PowerState){
        case 0:   
          fg_uiPowerTimeCnt = 0;
          s_uiWaitPowerTimeCnt = 0;
          fg_uc7759PowerState = 1; //等待脉冲边沿
          break;
        case 2://计时器计时
        case 3:
          fg_uiPowerTimeCnt++;
          if (fg_uiPowerTimeCnt >= 1000){
            fg_uc7759PowerState = 4;
          }
          break;
        case 4://计时超过1s或者进入测量单周期模式，继续计时
          fg_uiPowerTimeCnt++;
        case 1:
          s_uiWaitPowerTimeCnt++;
          if (s_uiWaitPowerTimeCnt > 6000){ //整个测量过程必须在6s内完成，否则强制结束
              fg_uc7759PowerState = 5;
              fg_uiPowerCC4Cnt = 1;
              fg_uiPowerTimeCnt = 6000;
          }
          break;
        case 5://计时结束，计算周期
          ScmCSE7759.uiPowerTimeCnt = fg_uiPowerTimeCnt;
          ScmCSE7759.uiPowerPulseCnt = fg_uiPowerCC4Cnt;
          g_ucCSE7759Flag |= POWER_BIT;
          fg_uc7759PowerState = 0; 
          break;
        default:
          break;
      }
    }
    //电压电流测量
    if ((g_ucCSE7759Flag & (VOL_BIT | CUR_BIT)) == 0){
      switch(fg_uc7759VIState){
        case 0:
          fg_uiVITimeCnt = 0;
          s_uiWaitVITimeCnt = 0;
          fg_uc7759VIState = 1; //等待脉冲边沿
          break;
        case 2://计时器计时
        case 3:
          fg_uiVITimeCnt++;
          if (fg_uiVITimeCnt >= 1000){
            fg_uc7759VIState = 4;
          }
          break;
        case 4://计时超过1s或者进入测量单周期模式，继续计时
          fg_uiVITimeCnt++;
        case 1:
          s_uiWaitVITimeCnt++;
          if (s_uiWaitVITimeCnt > 6000){ //整个测量过程必须在6s内完成，否则强制结束
              fg_uc7759VIState = 5;
              fg_uiVolCurCC3Cnt = 1;
              fg_uiVITimeCnt = 6000;
          }
          break;
        case 5://计时结束，计算周期
          if(GPIOB->ODR & 0x10){  //电压档
            ScmCSE7759.uiVoltageTimeCnt = fg_uiVITimeCnt;//(unsigned long)fg_uiVITimeCnt * 1000 / fg_uiVolCurCC3Cnt;
            ScmCSE7759.uiVoltagePulseCnt = fg_uiVolCurCC3Cnt;
            g_ucCSE7759Flag |= VOL_BIT;
            CURRENT_SET();//切换到电流档
          }else{//电流档
            ScmCSE7759.uiCurrentTimeCnt = fg_uiVITimeCnt;//(unsigned long)fg_uiVITimeCnt * 1000 / fg_uiVolCurCC3Cnt;
            ScmCSE7759.uiCurrentPulseCnt = fg_uiVolCurCC3Cnt;
            g_ucCSE7759Flag |= CUR_BIT;
            VOLTAGE_SET();//切换到电压档
          }
          s_uiDelayTimeCnt = 0;
          fg_uc7759VIState = 6;
          break;
        case 6: //切换通道延时
          if(s_uiDelayTimeCnt < DELAY_MAX_TIME){
             s_uiDelayTimeCnt++;
          }else{
             fg_uc7759VIState = 0;
          }
          break;
        default:
          break;
      } 
    }
    /* 继电器延时开启 */
    if (fg_ucRelayOperateDelay > 0){
        fg_ucRelayOperateDelay--;
        if(fg_ucRelayOperateDelay == 0){
            switch(g_ucRelayOperateRequest){
                case 0x01: //开继电器
                  RELAY_ON();
                  g_ucRelayOperateRequest = 0;
                  break;
                case 0x02: //关继电器
                  RELAY_OFF();
                  g_ucRelayOperateRequest = 0;
                  break;
                default:
                  g_ucRelayOperateRequest = 0;
                  break;
           }
        }
    }
#if TEST_ON_OFF == 1
    if (g_ucOnOffFlag == 1){
        if (g_uiOnOffInterval < TEST_ON_TIME){
            g_uiOnOffInterval++;
        }else{
            g_ucOnOffFlag = 3;
            g_uiOnOffInterval = 0;
        }
    }else if(g_ucOnOffFlag == 2){
        if (g_uiOnOffInterval < TEST_OFF_TIME){
            g_uiOnOffInterval++;
        }else{
            g_ucOnOffFlag = 4;
            g_uiOnOffInterval = 0;
        }
    }
#endif
#if ENERGY_MODE == 2
    if(++s_uiOneSecondCnt >= 1000){
       s_uiOneSecondCnt = 0;
       g_uiEnergy_100mWs += ScmController.uiPower;
       if(g_uiEnergy_100mWs >= ENERGY_SAVE_WS){
          uiTemp = g_uiEnergy_100mWs;
          g_uiEnergy_100mWs %= ENERGY_SAVE_WS;
          g_uiEnergySaveEn = uiTemp - g_uiEnergy_100mWs;  //需要保存的电能
       }
    }
#endif
}
/**
  * @brief Timer1 Update/Overflow/Trigger/Break Interrupt routine.
  * @param  None
  * @retval None
  */

INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
    static unsigned char s_ucCnt = 0;
    //更新中断
    if(TIM1->SR1 & 0x01){  //0.5ms中断一次
      TIM1->SR1 &= ~0x01;
      if(++s_ucCnt >=2){
        s_ucCnt = 0;
        Function1ms();
      }
    }
}
/**
  * 函数功能： 定时器1捕获、比较中断服务程序  
  * @brief Timer1 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM1_CAP_COM_IRQHandler, 12)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
    if((TIM1->SR1 & 0x08)){  //TIM1_CH3  电压或电流 
      //清除中断标志位
      TIM1->SR1 &= 0xF7;	// 清除CC3IF
      TIM1->SR2 &= 0xf7;	// 清除CC3OF 
      switch(fg_uc7759VIState){
        case 1: //首个脉冲，触发计时器计时
          fg_uiVolCurCC3Cnt = 0;
          fg_uc7759VIState = 2;
          break;
        case 2: //第二个脉冲，判断信号的周期是大于100ms还是小于100ms。
          fg_uiVolCurCC3Cnt++;  
          if (fg_uiVITimeCnt > 100){ //周期大于100ms，进入测量单个周期模式
            fg_uc7759VIState = 5; //停止计时
          }else{ //周期小于100ms，则进入测量多个脉冲模式
            fg_uc7759VIState = 3;
          }
          break;
        case 3:
          fg_uiVolCurCC3Cnt++;
          break;
        case 4:
          fg_uiVolCurCC3Cnt++;  
          fg_uc7759VIState = 5; //停止计时
          break;
      }
    }
    if((TIM1->SR1 & 0x10)){  //TIM1_CH4    功率    
      switch(fg_uc7759PowerState){
        case 1: //首个脉冲，触发计时器计时
          fg_uiPowerCC4Cnt = 0;
          fg_uc7759PowerState = 2;
          break;
        case 2: //第二个脉冲，判断信号的周期是大于100ms还是小于100ms。
          fg_uiPowerCC4Cnt++;  
          if (fg_uiPowerTimeCnt > 100){ //周期大于100ms，进入测量单个周期模式
            fg_uc7759PowerState = 5; //停止计时
          }else{ //周期小于100ms，则进入测量多个脉冲模式
            fg_uc7759PowerState = 3;
          }
          break;
        case 3:
          fg_uiPowerCC4Cnt++;
          break;
        case 4:
          fg_uiPowerCC4Cnt++;  
          fg_uc7759PowerState = 5; //停止计时
          break;
      }
      //电能计量
      ulEnergyTotalPluse++;
#ifndef ENERGY_MODE
  #error "Please define ENERGY_MODE!"
#elif ENERGY_MODE == 1
      g_uiEnergyCnt++;
      if(g_uiEnergySaveEn == 0){
        if (g_uiEnergyCnt >= ENERGY_SAVE_CNT){
          //g_uiEnergySaveEn = g_uiEnergyCnt / ENERGY_SAVE_CNT;
          //g_uiEnergyCnt = g_uiEnergyCnt % ENERGY_SAVE_CNT;
          g_uiEnergySaveEn = 1;
          g_uiEnergyCnt = 0;
        }
      }
#endif
      TIM1->SR1 &= 0xEF;	// 清除CC4IF
      TIM1->SR2 &= 0xEF;	// 清除CC4OF 
    }
}

#ifdef STM8S903
/**
  * @brief Timer5 Update/Overflow/Break/Trigger Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM5_UPD_OVF_BRK_TRG_IRQHandler, 13)
 {
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
 }
 
/**
  * @brief Timer5 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM5_CAP_COM_IRQHandler, 14)
 {
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
 }

#else /*STM8S208, STM8S207, STM8S105 or STM8S103 or STM8AF62Ax or STM8AF52Ax or STM8AF626x */
/**
* 函数功能:定时器2溢出中断服务程序
  * @brief Timer2 Update/Overflow/Break Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13)
 { //中断溢出时间2.5ms  
   TIM2->SR1 &= 0xFE;// 清中断标志位 2.5ms
    
   if(g_ucADCTimeCNT < ADC_MAX_TIME){
     g_ucADCTimeCNT++;
   } 
   if(g_ucReceiveTimeCnt < RCV_MAX_TIME){
     g_ucReceiveTimeCnt++;
     if(g_ucReceiveTimeCnt >= RCV_MAX_TIME){
       g_ucUART1ReceiveFlag = 2;//接收成功标志
     }
   }
#if SEECRET_EN == 1
   if(g_uiDelayTime < SEECRET_DELAY_TIME){
      g_uiDelayTime++;
   }
#endif
 }            

/**
  * @brief Timer2 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM2_CAP_COM_IRQHandler, 14)
 {
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
 }
#endif /*STM8S903*/

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S103) || \
    defined(STM8S003) ||  defined (STM8AF62Ax) || defined (STM8AF52Ax) || defined (STM8S903)
/**
  * @brief UART1 TX Interrupt routine.
  * @param  None
  * @retval None
  */
 //  unsigned char c;    
 INTERRUPT_HANDLER(UART1_TX_IRQHandler, 17)
 {
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
  // c++;
 }

/**
  * @brief UART1 RX Interrupt routine.
  * @param  None
  * @retval None
  */

 INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18)
 {
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */	       
  unsigned char ucTempReceiveData = 0;
  static unsigned char s_ucReceiveDataCNT = 0;
  if(UART1->SR & 0x20/*UART1_GetITStatus(UART1_IT_RXNE) != RESET*/){ 
    ucTempReceiveData = UART1->DR;
    if(g_ucUART1ReceiveFlag == 0){
        if(g_ucReceiveTimeCnt >= RCV_MAX_TIME){
          s_ucReceiveDataCNT = 0;  
          g_ucReceiveTimeCnt = 0;
        }   
        //g_ucReceiveTimeCnt = 0;
        if(s_ucReceiveDataCNT < RCV_MAX_NUM){ //限制接收长度
          g_ucRecivePLCData[s_ucReceiveDataCNT++] = ucTempReceiveData;
          g_ucReceiveDataLen = s_ucReceiveDataCNT;
        }
        //g_ucRecivePLCData[s_ucReceiveDataCNT] = '\0';
    }
  }
}   

#endif /*STM8S208 or STM8S207 or STM8S103 or STM8S903 or STM8AF62Ax or STM8AF52Ax */

/**
  * @brief I2C Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(I2C_IRQHandler, 19)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
/**
  * @brief ADC2 interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(ADC2_IRQHandler, 22)
 {
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
 }
#else /*STM8S105, STM8S103 or STM8S903 or STM8AF626x */
/**
  * @brief ADC1 interrupt routine.
  * @par Parameters:
  * None
  * @retval 
  * None
  */
 INTERRUPT_HANDLER(ADC1_IRQHandler, 22)
 {

   /*   unsigned int Temp= 0;       
    static unsigned int index = 0;
   ADC1->CSR = 0x24; 
    AD_results[index] = ADC1->DRL;//先读低再读高 
    Temp = ADC1->DRH;
    AD_results[index]= (Temp<<8)|AD_results[index];
    index = (index+1)%Num_of_Results; // Increment results index, modulo
    if(index==0)
    { 
      AD_Flag=1;
      ADC1->CR1 &= ~0x01;// turn off ADC转换
    }
    else
    {
      ADC1->CSR &= 0x7f;// 清ADC中断标志 
      ADC1->CR1 |= 0x01;// 启动ADC转换
    } */   
 }

#endif /*STM8S208 or STM8S207 or STM8AF52Ax or STM8AF62Ax */

#ifdef STM8S903
/**
  * @brief Timer6 Update/Overflow/Trigger Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM6_UPD_OVF_TRG_IRQHandler, 23)
 {
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
 }
#else /*STM8S208, STM8S207, STM8S105 or STM8S103 or STM8AF52Ax or STM8AF62Ax or STM8AF626x */
/**
  * @brief Timer4 Update/Overflow Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */ // 1ms interrupt
    TIM4->SR1 = 0x00;
}        
    
#endif /*STM8S903*/

/**
  * @brief Eeprom EEC Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EEPROM_EEC_IRQHandler, 24)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5)
{
  if(g_ucRelayOperateRequest != 0){
     if(g_ucRelayOperateRequest == 1){
        fg_ucRelayOperateDelay = RELAY_OPERATE_DELAY_ON;
     }else if(g_ucRelayOperateRequest == 2){
        fg_ucRelayOperateDelay = RELAY_OPERATE_DELAY_OFF;
     }else{
        g_ucRelayOperateRequest = 0;
     }
  }
}
                  
/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/