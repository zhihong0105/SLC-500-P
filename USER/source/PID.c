
#include "PID.h"
#include "main.h"
#ifndef S0_10V_CALIBRATE_EN
  #error "Please define S0_10V_CALIBRATE_EN!"
#elif S0_10V_CALIBRATE_EN == 1
extern unsigned int g_ui0_10V_Param;   //unit:0.001
#endif
PID sPID;
PID *sPtr = &sPID;

/*************��ʼ������*************/
void IncPIDInit(void)
{
  sPtr->SumError = 0;
  sPtr->LastError = 0;
  sPtr->PrevError = 0;
  
  sPtr->Proportion = 1;  // ����
  sPtr->Integral = 0.2;    // ����
  sPtr->Derivative = 0.1;  // ΢��
  //sPtr->SetPoint = 625;  // Ŀ�� 625 ��ӦΪ10V   �ϵ��� 8.2K  �� 3.6K
  sPtr->SetPoint = 723;    // Ŀ�� 625 ��ӦΪ10V   �ϵ��� 15K �� 8.2K //723.8
  //sPtr->SetPoint = 711;  //��Ӧ 5.00V
}
/*************����ʽ*************/
int IncPIDCalc(int NextPiont)
{
  register int iError,iIncPid;         //E[k]//E[k-1]//E[k-2]
  iError = sPtr->SetPoint - NextPiont;
  iIncPid = (int)(sPtr->Proportion * iError - sPtr->Integral * sPtr->LastError + sPtr->Derivative * sPtr->PrevError);           
         
  //�洢�����´μ���
  sPtr->PrevError = sPtr->LastError;
  sPtr->LastError = iError;
  
  return (iIncPid);
}

void Set0_10vOutput(unsigned char ucDim) 
{
#if S0_10V_CALIBRATE_EN == 1
  sPtr->SetPoint = (int)(ucDim * 3.61931 * g_ui0_10V_Param / 1000.0);//��׼ 5.00
#else
  sPtr->SetPoint = (int)(ucDim * 3.61931);//��׼ 5.00
#endif
 }
  
/*
PS:
sPtr->SetPoint ΪĿ��Ҫ�趨��ֵ
�������       Ϊ��ǰ�����ĵ�ѹ


*/

/*************λ��ʽ*************/
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

