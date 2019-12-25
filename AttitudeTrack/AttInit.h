#ifndef __ATTINIT_H__
#define __ATTINIT_H__
/***********阈值需要实际确定********************/
#include <vector>
#include <algorithm>
#include "ComFunc.h"
#include "config.h"
#include "DataStruct.h"
#define Heading_wlen 50
#define IMU_wlen 1000
using namespace std;
class AttInit
{
public:
	vector<double> vax1;
	vector<double> vay1;
	vector<double> vaz1;
	vector<double> vgx1;
	vector<double> vgy1;
	vector<double> vgz1;
	vector<double> vax2;
	vector<double> vay2;
	vector<double> vaz2;
	vector<double> vgx2;
	vector<double> vgy2;
	vector<double> vgz2;
	double bias_gx1, bias_gy1, bias_gz1;
	double std_gx1, std_gy1, std_gz1;
	double std_ax1, std_ay1, std_az1;
	double bias_gx2, bias_gy2, bias_gz2;
	double std_gx2, std_gy2, std_gz2;
	double std_ax2, std_ay2, std_az2;
	vector<double> vgpsyaw;  //0-360
	vector<double> vgpsroll;
	double mean_gyaw, std_gyaw;
	double mean_groll, std_groll;
	double roll1,pitch1,roll2,pitch2;
	double cross_slope1,tilt_slope1,tilt_slope2;  //水平偏转角，竖直偏转
	int bfinshinit;  //初始化成功标志，以及错误状态标志
	int binit_gyaw;
	int binit_groll;
	int binit_att_balde;
	int binit_att_mainfall;
	int binit_gbias_balde;
	int binit_gbias_mainfall;
public:
	AttInit();
	~AttInit();
	int process(AttTrackData iatd, AttTrackCfg_t* confg);
};
#endif
