#ifndef __CALIBRATE_H__
#define __CALIBRATE_H__
#include "ComFunc.h"
#include "DataStruct.h"
#include "config.h"
#include <vector>
using namespace std;
//双位置法标定安装误差（出厂标定加计零偏误差）
//安装误差最好不要超过30deg
class Cal_Install_Error
{
public:
	vector<double> ax1, ay1, az1, ax2, ay2, az2,ax3,ay3,az3;  //静态数据向量
	double max1, may1, maz1, max2, may2, maz2,max3,may3,maz3;
	double sax1, say1, saz1, sax2, say2, saz2,sax3,say3,saz3;
	vector<double> head1, rotat1;
	double std_head, std_rotat;
	double mean_head, mean_rotat;
	double accx_bias, accy_bias, accz_bias;
	int winlen;  
	int winlen1;
	double blade_installangle[2]; //铲刀水平安装误差角
	double mainfall_installangle[2];//车体水平安装误差角
	double installyaw;
	double installrotation;
public:
	void Cal_Install_Error_init(void);
	/*双位置法标定*/
	// option =1，2，3 1--第一位置采样 2--第二位置采样 3--计算安装误差角
	int Acc_Bias_2pos_cal(AttTrackData atdata, int option);
	/*双天线安装误差标定*/
	int InstallErr_Rotation(double rotation);
	void Comp_InstallErr_Acc(double acc[3], double installroll, double installpitch);
	void Comp_Acc_bias(AttTrackData* atdata,double acc_bias[3]);
};
#endif

