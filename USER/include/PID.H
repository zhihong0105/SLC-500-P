#ifndef __PID_H
#define __PID_H
typedef struct {
  int SetPoint;  //设定目标
  long SumError; //误差累积

  double Proportion; //比例常数
  double Integral;   //积分常数
  double Derivative; //微分常数

  int LastError;  //Error[-1]
  int PrevError;  //Error[-2]
}PID;

void IncPIDInit(void);
int IncPIDCalc(int NextPiont);
void Set0_10vOutput(unsigned char ucDim) ;
extern PID *sPtr;
 #endif
