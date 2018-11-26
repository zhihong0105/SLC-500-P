#ifndef __System_Initialize_H
#define __System_Initialize_H

//Э��ѡ��
#define PROTOCOL        XJP_V01
//Э�鶨��
#define GRT_V14         1
#define XJP_V01         2
//����ʹ��
#define TEST_ON_OFF     0   /* �������Զ����أ������俪�����ܣ�0���رգ�1��ʹ�� */
#if TEST_ON_OFF == 1
#define TEST_ON_TIME    3000 //unit: ms 
#define TEST_OFF_TIME   3000 //unit: ms 
#endif
/*************** Private macro ***************/
//#define TRUE 			1
//#define FALSE			0
//#define NULL			0
//************** оƬ�������� ********************//
#define VCC 		5		  // VCC 
#define SYS_CLOCK       2                 //ϵͳʱ��ѡ�� 1��HSI  2��HSE
// 1��HSI;2��HSE
  #if SYS_CLOCK == 1
    #define FMASTER	16000000	  // ��Ƶ��16MHz
  #elif SYS_CLOCK == 2
    #define FMASTER	10000000	  // ��Ƶ��12MHz
  #endif
#define BAUD_RATE	115200//9600
#define DIM_COM		0x11		  // ���������ʶ
#define PWM_FRE		400	  // PWM ����Ƶ��
#define PWM_FRE_0_10V	2000	  // 0~10V����PWMƵ��
#define PWM_TCNT	(FMASTER / PWM_FRE)    	 // ����Ƶ��40 000
#define PWM_TCNT0_10V   (FMASTER / PWM_FRE_0_10V)  // ����Ƶ��8 000

#define SEL_ON()	GPIOB->ODR = GPIOB->ODR | 0x10 //PB4 CSE7759 ѡ���
#define SEL_OFF()	GPIOB->ODR = GPIOB->ODR & 0xEF
#define SWITCH_ON()	GPIOB->ODR = GPIOB->ODR & 0xDF //PB5 
#define SWITCH_OFF()	GPIOB->ODR = GPIOB->ODR | 0x20
#define MOC3063_ON()	GPIOC->ODR = GPIOC->ODR | 0x80 //PC7
#define MOC3063_OFF()	GPIOC->ODR = GPIOC->ODR & 0x7F
#define RELAY_ON()	GPIOD->ODR = GPIOD->ODR | 0x10 //PD4
#define RELAY_OFF()	GPIOD->ODR = GPIOD->ODR & 0xEF
#define RELAY_STATUS    GPIOD->ODR&0x10

#define LED_ON()        GPIOA->ODR = GPIOA->ODR | 0x08  //PA3
#define LED_OFF()       GPIOA->ODR = GPIOA->ODR & (~0x08)
#define LED_Toggle()    GPIOA->ODR ^= 0X08;//GPIO_WriteReverse(GPIOA,GPIO_PIN_2)//PA2
#define PB5_ON()        GPIOB->ODR = GPIOB->ODR | 0x20  //PB5
#define PB5_OFF()       GPIOB->ODR = GPIOB->ODR & (~0x20)
#define PB5_Toggle()    GPIOB->ODR ^= 0X20;

#define CURRENT_SET()   GPIOB->ODR = GPIOB->ODR & 0xEF //PB4 = 0�� CSE7759 ѡ���
#define VOLTAGE_SET()   GPIOB->ODR = GPIOB->ODR | 0x10 //PB4 = 1�� CSE7759 ѡ���
#define READ_CSE_SET()  (GPIOB->ODR & 0x10)
#define VOLTAGE         0x10
//*************** ����������ֵ ********************//
//#define OVER_VOLTAGE	2770	// ��ѹ��ѹΪ320V����λ0.1V
//#define UNDER_VOLTAGE	1650	// Ƿѹ��ѹΪ160V����λ0.1V 

#define PRO_H_VOLTAGE	2850  // ��ѹ������ѹΪ285V����λ0.1V
#define RECO_H_VOLTAGE	2800  // ��ѹ�ָ���ѹΪ280V����λ0.1V

#define PRO_L_VOLTAGE	950//1550  // Ƿѹ������ѹΪ155V����λ0.1V
#define RECO_L_VOLTAGE	1000//1600  // Ƿѹ�ָ���ѹΪ160V����λ0.1V

//#define PRO_H_VOLTAGE	0xffff  // ��ѹ������ѹΪ280V����λ0.1V -------------------
//#define PRO_L_VOLTAGE	0x00    // ��ѹ������ѹΪ160V����λ0.1V ---------------------

#define OVER_CURRENT	4000  	// ������Ϊ4A����λ1mA	
#define UNDER_CURRENT	20	    // Ƿ����Ϊ0.1A����λ1mA

#define UNDER_POWER	30	  // 3W	//������״̬�£�ֻ�����ϲ��ػ�
#define PRO_H_POWER	5200	// �����ʱ���ֵ520W����λ0.1W
//#define PRO_H_POWER	0xffff	// �����ʱ���ֵ600W����λ0.1W----------------------

//#define DERATE_TEMP1	85	  // derate temperature is 75��,unit 1 degree
//#define DERATE_TEMP2	75	  // derate temperature is 50��,unit 1 degree

#define OVER_TEMP	95	  // ���¶�ֵ95�ȣ���λ1��
#define RECO_TEMP       90    // �ָ������¶� 90 

#define UNDER_TEMP	-25	  // ���¶�ֵ-25�ȣ���λ1��		//ֻ�����ϲ��ػ�

//*************** ����״ֵ̬ *********************//
#if PROTOCOL == GRT_V14
    #define RS  0x0001	// 0�̵���״̬
    #define UC	0x0002	// 1�����������
    #define OC	0x0004	// 2�����������
    #define UV	0x0008	// 3�����ѹ����
    #define OV	0x0010	// 4�����ѹ����
    #define UP	0x0020	// 5���빦�ʹ���
    #define OP	0x0040	// 6���빦�ʹ���
    #define RI	0x0080	// 7�̵���
#elif PROTOCOL == XJP_V01
    #define UV  0x0001	// y
    #define OV	0x0002	// y
    #define OP	0x0004	// y
    #define /*OL*/ UP	0x0008	// n
    #define SC	0x0010	// n
    #define US	0x0020	// n
    #define LT	0x0040	// n
    #define OT	0x0080	// n
    /* �����־λ�������� */
    #define UC	0x0000
    #define RS  0x0000    
    #define ALL_PRO      (UV | OV | OP | SC | US | OT)  //��ػ��ı���
#endif
/************** Private variables ****************/

typedef struct{     
	  unsigned short int uiCurrentTimeCnt;    
	  unsigned short int uiVoltageTimeCnt;   
    unsigned short int uiPowerTimeCnt;  
    unsigned short int uiPowerPulseCnt;
	  unsigned short int uiCurrentPulseCnt;    
	  unsigned short int uiVoltagePulseCnt; 
}ScmCSE7759TypeDef;

typedef struct{
	  unsigned short int uiInputVoltage;    
	  unsigned short int uiInputCurrent;    
	  unsigned short int uiPower;          
	  unsigned char       ucPowerFactory;
}ScmControllerTypeDef;


//��������
void WDG_Config(void);
void UART1_Config(void);
void UART1_SendChar(unsigned char ch);
void UART1_SendString(unsigned char* Data,unsigned int len);
void ADC1_Config(void);
void UART1_IT_Set(unsigned short int UART1_IT, char NewState);
void ClkSet(void);
void GPIO_Config(void);
void TIM1_Config(void);
void TIM2_Config(void);
void TIM4_Config(void);
void Delay_ms(unsigned int ms);
void Set_PWM_Output(unsigned char ucPulseWide);
//void Set_0_10V_Output(unsigned char ucPulseWide);
void Set_0_10V_Output(unsigned int uiPulseWide);
void WriteGroupToFlash(unsigned int StartAddress,unsigned char ucData);
void WriteDataToFlash(unsigned int StartAddress,unsigned char ucData);
void WriteLongDataToFlash(unsigned int StartAddress,long lData);
long ReadLongDataFromFlash(unsigned int StartAddress);
//���ܱ���
unsigned long ReadEnergyParam(void);
void SaveEnergy(unsigned long ulEnergy_wh);
#endif 