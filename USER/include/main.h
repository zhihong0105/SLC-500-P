#ifndef __main_H
#define __main_H

#define SOFT_VERSION         0X0105
#define HARDWARE_VERSION     0X0C
#define DEVICE_CODE          0XC9

#define SEECRET_EN           0       /* 0���رգ�1��ʹ�� */
#if  SEECRET_EN == 1
#define  SEECRET_DELAY_TIME  (400 * 60)  /* unit:  2.5ms */
#endif

#define S0_10V_CALIBRATE_EN  1      /* ͨ���ξ�У׼0~10V���ʹ��λ  0���رգ� 1��ʹ�� */

#define ENERGY_MODE          1      /* 1:������ 2�����ʻ��� */
#if  ENERGY_MODE == 2
  #define ENERGY_SAVE_Wh     0.5    /* ������ܵ���ֵ����λ��Wh */
  #define ENERGY_SAVE_WS     ((unsigned int)(ENERGY_SAVE_Wh * 3600 * 10))  /* ������ܵ���ֵ����λ��100mWs */
#endif
//operate time max 20ms
//release time max 10ms
#define RELAY_ON_MAX_TIME    10  //10*2.5ms  + 20ms 
#define MOC3063_OFF_MAX_TIME 20 //*2.5ms   - 10ms

#define RCV_MAX_NUM          30   //����������ֽ��� 
#define RCV_MAX_TIME         4    //4*2.5ms = 10ms ���ն�ʱ ��10msΪһ֡
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
#define TEM_PRO              1  //0:�ر� 1:ʹ��
#define VOL_PRO              1  //0:�ر� 1:ʹ��
#define POW_PRO              1  //0:�ر� 1:ʹ��

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
//���ܼ�������
typedef struct
{
  unsigned char ucGroupNumber; //bit7: 0:ûд�����ݣ�1����д�����ݣ� bit6~bit0: ��ţ�0~��ENERGY_GROUP_TOTAL -1��
  unsigned long ulEnergy_wh;
}ScmEnergyGroup;
#define ENERGY_SAVE_ADR_START   0x402A
#define ENERGY_GROUP_TOTAL      119    //���ܱ��棬һ��119�飬ÿ��5���ֽ�
#define ENERGY_SAVE_ADR_END     (ENERGY_SAVE_ADR_START + ENERGY_GROUP_TOTAL*5)
#define ENERGY_GROUP_LONG       sizeof(ScmEnergyGroup)

#define ENERGY_SAVE_POWER       0.5   //�ﵽ�ù��ʿ�ʼ���湦�����ݣ�unit:wh
#define ENERGY_SAVE_CNT         ((unsigned int)(ENERGY_SAVE_POWER*3600/PT))   //����ENERGY_SAVE_POWER�������õ���������   3099

#define NTC_CHANNLE             0x04
#define FB_CHANNLE              0x03
#define NODE_NUM_01

//#define VT  0.2167
//#define AT  0.00167
//#define PT  0.603
#define VT                0.2015          //Ƶ��Ϊ1Hzʱ��Ӧ�ĵ�ѹ V
#define AT                0.001596        //Ƶ��Ϊ1Hzʱ��Ӧ�ĵ��� A
#define PT                0.5808          //Ƶ��Ϊ1Hzʱ��Ӧ�Ĺ��� W
#define TIME_CNT_DEFAULT   1000
#define PLUSE_CNT_DEFAULT  1
#define VT_DEFAULT_H     ((VT * TIME_CNT_DEFAULT / PLUSE_CNT_DEFAULT)*1.3)
#define VT_DEFAULT_L     ((VT * TIME_CNT_DEFAULT / PLUSE_CNT_DEFAULT)*0.7)
#define AT_DEFAULT_H     ((AT * TIME_CNT_DEFAULT / PLUSE_CNT_DEFAULT)*1.3)    
#define AT_DEFAULT_L     ((AT * TIME_CNT_DEFAULT / PLUSE_CNT_DEFAULT)*0.7)
#define PT_DEFAULT_H     ((PT * TIME_CNT_DEFAULT / PLUSE_CNT_DEFAULT)*1.3)    
#define PT_DEFAULT_L     ((PT * TIME_CNT_DEFAULT / PLUSE_CNT_DEFAULT)*0.7)

#define GRA             0x01  //б��
#define INT             0x02  //�ؾ�

#define IPA             0x08  //BIT3 У�����빦�� ״̬λ BIT3
#define IVA             0x10  //BIT4 У�������ѹ ״̬λ BIT4
#define ICA             0x20  //BIT5 У��������� ״̬λ BIT5

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

/* �ú궨��������� FLASH_Unlock(FLASH_MEMTYPE_DATA)���������е�x������ */
#define FLASH_Unlock(x) FLASH->DUKR = 0xAE; FLASH->DUKR = 0x56/* Warning: keys are reversed on data memory !!! */
/* �ú궨��������� FLASH_Lock(FLASH_MEMTYPE_DATA)���������е�x������ */
#define FLASH_Lock(x)   FLASH->IAPSR &= 0XF7;  
/* �ú궨��������� FLASH_ReadByte()���� */
#define FLASH_ReadByte(x)  (*(PointerAttr uint8_t *) (uint16_t)(x)) 
/* �ú궨��������� FLASH_SetProgrammingTime()���� */
#define FLASH_SetProgrammingTime(x)  FLASH->CR1 &= (uint8_t)(~FLASH_CR1_FIX);FLASH->CR1 |= (uint8_t)(x)
/* �ú궨��������� FLASH_ProgramByte()���� */
#define FLASH_ProgramByte(x,y)  *(PointerAttr uint8_t*) (uint16_t)(x) = (y);
/* �ú궨��������� TIM1_ITConfig()���� */
#define TIM1_ITConfig(x,y)   if((y)!=0){TIM1->IER |= (uint8_t)(x);}else {TIM1->IER &= (uint8_t)(~(uint8_t)(x));}
#endif