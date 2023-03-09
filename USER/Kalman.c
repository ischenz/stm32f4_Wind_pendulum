#include "Kalman.h"
//static double p_last = 0;
//static double x_last = 0;

#define P_Q 0.05
#define M_R 5.05

/*
        Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
        R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��
        ����p�ĳ�ֵ�������ȡ�����ǲ���Ϊ0��Ϊ0�Ļ��������˲�������Ϊ�Ѿ��������˲����ˣ�
q,r��ֵ��Ҫ�����Գ����������˾���(������¶ȼ��ж��ƣ��Լ���ĳ������ж�ǿ)
r���������˲����������ʵ�����ߵ�����̶ȣ�rԽСԽ�ӽ���
q�������˲��������ƽ���̶ȣ�qԽСԽƽ����
*/

//static double KalmanFilter(const double ResrcData)
//{
// 
//    double R = M_R;
//    double Q = P_Q;
// 
//    double x_mid = x_last;
//    double x_now;
// 
//    double p_mid ;
//    double p_now;
// 
//    double kg;
// 
//    //����p_last ���� kalmanFilter_A ��pֱ��ȡ0
//    x_mid=x_last;                       //x_last=x(k-1|k-1),x_mid=x(k|k-1)
//    p_mid=p_last+Q;                     //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
// 
//    /*
//      *  �������˲��������Ҫ��ʽ
//      */
//    kg=p_mid/(p_mid+R);                 //Ϊkalman filter��R Ϊ����
//    x_now=x_mid+kg*(ResrcData-x_mid);   //���Ƴ�������ֵ
//    p_now=(1-kg)*p_mid;                 //����ֵ��Ӧ��covariance
//    p_last = p_now;                     //����covariance ֵ
//    x_last = x_now;                     //����ϵͳ״ֵ̬
// 
//    return x_now;
//}
 
float kalmanFilter_A(float inData)
{
  static float prevData=0;
  //����p�ĳ�ֵ�������ȡ�����ǲ���Ϊ0��Ϊ0�Ļ��������˲�������Ϊ�Ѿ��������˲����ˣ�
  static float p=0.01, q=P_Q, r=M_R, kGain=0;
    p = p+q;
    kGain = p/(p+r);
 
    inData = prevData+(kGain*(inData-prevData));
    p = (1-kGain)*p;
 
    prevData = inData;
 
    return inData;
}
