#ifndef __PID_H
#define __PID_H
typedef struct {
  int SetPoint;  //�趨Ŀ��
  long SumError; //����ۻ�

  double Proportion; //��������
  double Integral;   //���ֳ���
  double Derivative; //΢�ֳ���

  int LastError;  //Error[-1]
  int PrevError;  //Error[-2]
}PID;

void IncPIDInit(void);
int IncPIDCalc(int NextPiont);
void Set0_10vOutput(unsigned char ucDim) ;
extern PID *sPtr;
 #endif