#ifndef __ELEVATIONESTIMATE_H__
#define __ELEVATIONESTIMATE_H__
#include "AttTrack_lib.h"
#include "kalmanfilter.h"
#include "DataStruct.h"
#include "mem.h"
#define Ki 0.95
#define Kp 0.001
#define Alt_WIN 50
using namespace std;
class ElevationEstimate
{
public:
	vector<double> vgpsalt;
	vector<double> vgpspitch;
	//平滑滤波器容器
	vector<double> queue_gnss_alt;
	vector<double> queue_gnss_ver;
	double  alt;        //海拔高 m  向上为正
	double  vu;         //高程方向速度 m/s  向上为正
	double  accalt;     //高程方向加速度  m/s2
	double  accbias;    //估计的零偏误差 m/s2
	double  anzpre;     //导航系下Z轴加计
	double  accpre[3];  
	double pre_alt;
	double kf_alt;
	double kf_vu;
	double tmp_vu;
	double tmp_alt;
	double Acc_Alt;
	double Gnss_Alt;
	double mean_Alt;
	double mean_ver;
	KF kfelv;           //标准kalman滤波器模型
	int binitkf;
	int bcompfilter;
public:
	ElevationEstimate();
	~ElevationEstimate();
	int initstate(double gnssalt, double gnssvu);
	int Alt_Complement_Filter(double acc[3], double dt, double roll, double pitch, double gnssalt, double gnssvn, int gpsstate, int bgnssupdata, int bstatic);
	int update_kf(AttTrackData& iatd, double att[3],int bstatic);
	int Gnss_alt_Err(AttTrackData& iatd);
	int Gnss_Amooth(AttTrackData& iatd, int option);
};
//载体系三维向量转换至当地水平坐标系，NED,前右下
void Conver_b2lever(double roll, double pitch, double var_b[3], double var_lever[3]);
#endif
