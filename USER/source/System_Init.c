#include "System_Initialize.h"
#include "stm8s.h"

void WDG_Config(void)
{
  IWDG->KR =0xCC; //����IWDG
  IWDG->KR =0x55; //������ܱ�����IWDG_PR��IWDG_RLR�Ĵ����Ĳ���.
  IWDG->PR =0x05; //Ԥ��Ƶϵ�� 128
  IWDG->RLR=0xFF; //���Ź���������װ����ֵ�����Ź�������Լ510ms
  IWDG->KR =0xAA; //ι��
}