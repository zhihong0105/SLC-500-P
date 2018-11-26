#ifndef __main_H
#define __main_H

#define SOFT_VERSION         0X0105
#define HARDWARE_VERSION     0X0C
#define DEVICE_CODE          0XC9

#define SEECRET_EN           0       /* 0：关闭；1：使能 */
#if  SEECRET_EN == 1
#define  SEECRET_DELAY_TIME  (400 * 60)  /* unit:  2.5ms */
#endif

#define S0_10V_CALIBRATE_EN  1      /* 通过治具校准0~10V输出使能位  0：关闭； 1：使能 */

#define ENERGY_MODE          1      /* 1:记脉冲 2：功率积分 */
#if  ENERGY_MODE == 2
  #define ENERGY_SAVE_Wh     0.5    /* 保存电能的阈值，单位：Wh */
  #define ENERGY_SAVE_WS     ((unsigned int)(ENERGY_SAVE_Wh * 3600 * 10))  /* 保存电能的阈值，单位：100mWs */
#endif
//operate time max 20ms
//release time max 10ms
#define RELAY_ON_MAX_TIME    10  //10*2.5ms  + 20ms 
#define MOC3063_OFF_MAX_TIME 20 //*2.5ms   - 10ms

#define RCV_MAX_NUM          30   //串口最长接收字节数 
#define RCV_MAX_TIME         4    //4*2.5ms = 10ms 接收定时 超10ms为一帧
#define ADC_MAX_TIME         1
 
#define POWER_BIT            0x01
#define VOL_BIT              0x02
#define CUR_BIT              0x04


#define POWER_MAX_TIME       1000
#define VOL_MAX_TIME         1000
#define CUR_MAX_TIME         1000
#define DELAY_MAX_TIME       500
 //1byte
 //2bytes
#define TEM_PRO              1  //0:关闭 1:使能
#define VOL_PRO              1  //0:关闭 1:使能
#define POW_PRO              1  //0:关闭 1:使能

 //EEPROM ADD
#define POW_GRA_ADD          0x4000       //4 bytes 
#define POW_INT_ADD          0x4004       //4 bytes

#define VOL_GRA_ADD          0x4008       //4 bytes
#define VOL_INT_ADD          0x400C       //4 bytes

#define CUR_GRA_ADD          0x4010       //4 bytes
#define CUR_INT_ADD          0x4014       //4 bytes

#define COR_STA_ADD          0x4018       //1 byte
#define NodeGroupAddress     0x4019       //1 byte
#define UidSaveAdr           0x401A       //12 bytes
#define S0_10V_CHECK_ADR     0x4026       //2 bytes
#define CLK_HSI_CHECK_ADR    0x4028       //1 byte
#define HSI_10V_FLAG_ADR     0x4029       //1 byte

#define HSI_10V_FLAG         ((unsigned char)(0x56))
//电能计量保存
typedef struct
{
  unsigned char ucGroupNumber; //bit7: 0:没写入数据，1：已写入数据； bit6~bit0: 组号：0~（ENERGY_GROUP_TOTAL -1）
  unsigned long ulEnergy_wh;
}ScmEnergyGroup;
#define ENERGY_SAVE_ADR_START   0x402A
#define ENERGY_GROUP_TOTAL      119    //电能保存，一共119组，每组5个字节
#define ENERGY_SAVE_ADR_END     (ENERGY_SAVE_ADR_START + ENERGY_GROUP_TOTAL*5)
#define ENERGY_GROUP_LONG       sizeof(ScmEnergyGroup)

#define ENERGY_SAVE_POWER       0.5   //达到该功率开始保存功率数据，unit:wh
#define ENERGY_SAVE_CNT         ((unsigned int)(ENERGY_SAVE_POWER*3600/PT))   //计量ENERGY_SAVE_POWER电能所用的脉冲数量   3099

#define NTC_CHANNLE             0x04
#define FB_CHANNLE              0x03
#define NODE_NUM_01

//#define VT  0.2167
//#define AT  0.00167
//#define PT  0.603
#define VT                0.2015          //频率为1Hz时对应的电压 V
#define AT                0.001596        //频率为1Hz时对应的电流 A
#define PT                0.5808          //频率为1Hz时对应的功率 W
#define TIME_CNT_DEFAULT   1000
#define PLUSE_CNT_DEFAULT  1
#define VT_DEFAULT_H     ((VT * TIME_CNT_DEFAULT / PLUSE_CNT_DEFAULT)*1.3)
#define VT_DEFAULT_L     ((VT * TIME_CNT_DEFAULT / PLUSE_CNT_DEFAULT)*0.7)
#define AT_DEFAULT_H     ((AT * TIME_CNT_DEFAULT / PLUSE_CNT_DEFAULT)*1.3)    
#define AT_DEFAULT_L     ((AT * TIME_CNT_DEFAULT / PLUSE_CNT_DEFAULT)*0.7)
#define PT_DEFAULT_H     ((PT * TIME_CNT_DEFAULT / PLUSE_CNT_DEFAULT)*1.3)    
#define PT_DEFAULT_L     ((PT * TIME_CNT_DEFAULT / PLUSE_CNT_DEFAULT)*0.7)

#define GRA             0x01  //斜率
#define INT             0x02  //截距

#define IPA             0x08  //BIT3 校正输入功率 状态位 BIT3
#define IVA             0x10  //BIT4 校正输入电压 状态位 BIT4
#define ICA             0x20  //BIT5 校正输入电流 状态位 BIT5

#define RECOVER_ALL     0x00
#define RECOVER_POW     0x05
#define RECOVER_VOL     0x06
#define RECOVER_CUR     0x07

#define CONFIG_CMD      0xA1   //Data[0]
#define READ_CMD        0xA2

#define RECOVER_CMD     0x01  //Data[1]
#define CONFIG_VIP      0x05
#define CONFIG_LICENSE  0X06
#define CLREA_ENERGY    0x07
#define HIS_0_10V       0X08 


#define NO_OPTION       0xB1

#define ACK_RCO_FAIL    0xB2


#define SUCCESS         0x01
#define FAIL            0x00

/* 该宏定义用于替代 FLASH_Unlock(FLASH_MEMTYPE_DATA)函数，其中的x无意义 */
#define FLASH_Unlock(x) FLASH->DUKR = 0xAE; FLASH->DUKR = 0x56/* Warning: keys are reversed on data memory !!! */
/* 该宏定义用于替代 FLASH_Lock(FLASH_MEMTYPE_DATA)函数，其中的x无意义 */
#define FLASH_Lock(x)   FLASH->IAPSR &= 0XF7;  
/* 该宏定义用于替代 FLASH_ReadByte()函数 */
#define FLASH_ReadByte(x)  (*(PointerAttr uint8_t *) (uint16_t)(x)) 
/* 该宏定义用于替代 FLASH_SetProgrammingTime()函数 */
#define FLASH_SetProgrammingTime(x)  FLASH->CR1 &= (uint8_t)(~FLASH_CR1_FIX);FLASH->CR1 |= (uint8_t)(x)
/* 该宏定义用于替代 FLASH_ProgramByte()函数 */
#define FLASH_ProgramByte(x,y)  *(PointerAttr uint8_t*) (uint16_t)(x) = (y);
/* 该宏定义用于替代 TIM1_ITConfig()函数 */
#define TIM1_ITConfig(x,y)   if((y)!=0){TIM1->IER |= (uint8_t)(x);}else {TIM1->IER &= (uint8_t)(~(uint8_t)(x));}
#endif