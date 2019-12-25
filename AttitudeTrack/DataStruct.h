#ifndef __DATASTRUCT_H__
#define __DATASTRUCT_H__
#include "ComFunc.h"
#include "AttTrack_lib.h"
#include "config.h"
//定义导航系为NED，载体系为FRD
//#define I80S_ADIS16460
//#define 
#define IS203
#define GNSS_UPDATE_TIME 50  //GNSS采样间隔 50ms 20hz
#define IMU_UPDATE_TIME 5   //5ms 200hz
#include <time.h>
const static double gpst0[] = { 1980, 1, 6, 0, 0, 0 }; /* gps time reference */
const static double gst0[] = { 1999, 8, 22, 0, 0, 0 }; /* galileo system time reference */
const static double bdt0[] = { 2006, 1, 1, 0, 0, 0 }; /* beidou time reference */
const static double leaps[][7] = { /* leap seconds {y,m,d,h,m,s,utc-gpst,...} */
	{ 2017, 1, 1, 0, 0, 0, -18 },
	{ 2015, 7, 1, 0, 0, 0, -17 },
	{ 2012, 7, 1, 0, 0, 0, -16 },
	{ 2009, 1, 1, 0, 0, 0, -15 },
	{ 2006, 1, 1, 0, 0, 0, -14 },
	{ 1999, 1, 1, 0, 0, 0, -13 },
	{ 1997, 7, 1, 0, 0, 0, -12 },
	{ 1996, 1, 1, 0, 0, 0, -11 },
	{ 1994, 7, 1, 0, 0, 0, -10 },
	{ 1993, 7, 1, 0, 0, 0, -9 },
	{ 1992, 7, 1, 0, 0, 0, -8 },
	{ 1991, 1, 1, 0, 0, 0, -7 },
	{ 1990, 1, 1, 0, 0, 0, -6 },
	{ 1988, 1, 1, 0, 0, 0, -5 },
	{ 1985, 7, 1, 0, 0, 0, -4 },
	{ 1983, 7, 1, 0, 0, 0, -3 },
	{ 1982, 7, 1, 0, 0, 0, -2 },
	{ 1981, 7, 1, 0, 0, 0, -1 }
};
typedef struct {        /* time struct */
	time_t time;        /* time (s) expressed by standard time_t 从1970.01.01 0秒到现在的秒数 long int*/
	double sec;         /* fraction of second under 1 s */
} gtime_t;
double time2gpst(gtime_t t, int *week);
gtime_t epoch2time(const double *ep);
void time2epoch(gtime_t t, double *ep);
gtime_t gpst2utc(gtime_t t);
double timediff(gtime_t t1, gtime_t t2);
gtime_t timeadd(gtime_t t, double sec);
class AttTrackData
{
public:
	double imutime_balde;      //铲刀IMU数据时间戳，秒
	double utc_time;
	double gyo_balde[3];       //铲刀XYZ陀螺角速度，rad/s，前右下
	double acc_balde[3];       //铲刀XYZ加计加速度，m/s2,前右下	
	int bbaldeimu_update;      //铲刀IMU数据更新标志

	double imutime_mainfall;      //车架IMU数据时间戳，秒
	double integ_pitch;			  //车体陀螺积分俯仰角
	double gyo_mainfall[3];       //车架XYZ陀螺角速度，rad/s，前右下
	double acc_mainfall[3];       //车架XYZ加计加速度，m/s2,前右下
	int bmainfallimu_update;      //车架IMU数据更新标志

	double rotation_balde;     //铲刀相对旋转角度，rad/s

	bool gnss_alt_err;	       //gnss高程异常标志
	double gnsstime;           //GPS时间戳
	double gnsstime_pre;	   //GPS前一刻时间戳
	int gnss_lost_num;		   //GPS中断点数
	double BLH_ant[3];         //天线相位中心位置，纬度，经度，高程（rad,rad,m）
	double Mean_Gnss_Alt;	   //前后两高程均值
	double Gnss_Alt_Pre;
	double speed;              //gnss地速
	double speed_ver;          //gnss竖直速度
	int gstate_pos;            //GNSS定位解状态
	double yaw_2ant;           //双天线航向，deg,北偏东
	double roll_2ant;          //双天线横滚角，deg
	double pitch_2ant;	       //双天线俯仰角，deg
	double gyaw_noise;         //双天线航向标准差
	double groll_noise;        //双天线横滚角标准差
	int gstate_2ant;           //双天线航向解状态
	int bgnssupdate;           //gnss解更新

	double ep[6];			//utc时间
	int week;				//gnss周
	double weektime_sec;	//gnss
	double imu_time;		//IMU时间戳
	
	int balde_install_falg;
	int body_install_falg;
	int rotation_install_flag;
public:
	void Init();
	void Reset();
	AttTrackData& operator=(const AttTrackData& atdata);
	void Convert_baldeimu();
	void Convert_mainfallimu();
};
void decode(ATdata_t* atd, AttTrackData* atdata, AttTrackCfg_t* cfg);
void Data_Check(AttTrackData* atdata);//数据异常值检测，是否丢点
void Gnss_time_decode(AttTrackData* atdata);
#endif


