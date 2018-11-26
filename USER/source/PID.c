
#include "PID.h"
#include "main.h"
#ifndef S0_10V_CALIBRATE_EN
  #error "Please define S0_10V_CALIBRATE_EN!"
#elif S0_10V_CALIBRATE_EN == 1
extern unsigned int g_ui0_10V_Param;   //unit:0.001
#endif
PID sPID;
PID *sPtr = &sPID;

/*************初始化参数*************/
void IncPIDInit(void)
{
  sPtr->SumError = 0;
  sPtr->LastError = 0;
  sPtr->PrevError = 0;
  
  sPtr->Proportion = 1;  // 比例
  sPtr->Integral = 0.2;    // 积分
  sPtr->Derivative = 0.1;  // 微分
  //sPtr->SetPoint = 625;  // 目标 625 对应为10V   上电阻 8.2K  下 3.6K
  sPtr->SetPoint = 723;    // 目标 625 对应为10V   上电阻 15K 下 8.2K //723.8
  //sPtr->SetPoint = 711;  //对应 5.00V
}
/*************增量式*************/
int IncPIDCalc(int NextPiont)
{
  register int iError,iIncPid;         //E[k]//E[k-1]//E[k-2]
  iError = sPtr->SetPoint - NextPiont;
  iIncPid = (int)(sPtr->Proportion * iError - sPtr->Integral * sPtr->LastError + sPtr->Derivative * sPtr->PrevError);           
         
  //存储误差，用下次计算
  sPtr->PrevError = sPtr->LastError;
  sPtr->LastError = iError;
  
  return (iIncPid);
}

void Set0_10vOutput(unsigned char ucDim) 
{
#if S0_10V_CALIBRATE_EN == 1
  sPtr->SetPoint = (int)(ucDim * 3.61931 * g_ui0_10V_Param / 1000.0);//基准 5.00
#else
  sPtr->SetPoint = (int)(ucDim * 3.61931);//基准 5.00
#endif
 }
  
/*
PS:
sPtr->SetPoint 为目标要设定的值
函数入口       为当前测量的电压


*/

/*************位置式*************/
/*
unsigned int LocPIDCalc(int NextPoint)
{
  register int iError,dError;
  
  iError = sPtr->SetPoint - NextPiont;
  sPtr->SumError += iError;
  dError = iError - sPtr->LastError;
  sPtr->LastError = iError;
  return(sPtr->Proportion * iError \
         + sPtr->Integral*sPtr->SumError\
         + sPtr->Derivative*dError);
}
*/

