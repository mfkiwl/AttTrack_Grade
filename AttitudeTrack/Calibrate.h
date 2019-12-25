#ifndef __CALIBRATE_H__
#define __CALIBRATE_H__
#include "ComFunc.h"
#include "DataStruct.h"
#include "config.h"
#include <vector>
using namespace std;
//˫λ�÷��궨��װ�������궨�Ӽ���ƫ��
//��װ�����ò�Ҫ����30deg
class Cal_Install_Error
{
public:
	vector<double> ax1, ay1, az1, ax2, ay2, az2,ax3,ay3,az3;  //��̬��������
	double max1, may1, maz1, max2, may2, maz2,max3,may3,maz3;
	double sax1, say1, saz1, sax2, say2, saz2,sax3,say3,saz3;
	vector<double> head1, rotat1;
	double std_head, std_rotat;
	double mean_head, mean_rotat;
	double accx_bias, accy_bias, accz_bias;
	int winlen;  
	int winlen1;
	double blade_installangle[2]; //����ˮƽ��װ����
	double mainfall_installangle[2];//����ˮƽ��װ����
	double installyaw;
	double installrotation;
public:
	void Cal_Install_Error_init(void);
	/*˫λ�÷��궨*/
	// option =1��2��3 1--��һλ�ò��� 2--�ڶ�λ�ò��� 3--���㰲װ����
	int Acc_Bias_2pos_cal(AttTrackData atdata, int option);
	/*˫���߰�װ���궨*/
	int InstallErr_Rotation(double rotation);
	void Comp_InstallErr_Acc(double acc[3], double installroll, double installpitch);
	void Comp_Acc_bias(AttTrackData* atdata,double acc_bias[3]);
};
#endif

