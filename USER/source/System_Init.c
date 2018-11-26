#include "System_Initialize.h"
#include "stm8s.h"

void WDG_Config(void)
{
  IWDG->KR =0xCC; //启动IWDG
  IWDG->KR =0x55; //允许对受保护的IWDG_PR和IWDG_RLR寄存器的操作.
  IWDG->PR =0x05; //预分频系数 128
  IWDG->RLR=0xFF; //看门狗计数器重装载数值，看门狗的周期约510ms
  IWDG->KR =0xAA; //喂狗
}