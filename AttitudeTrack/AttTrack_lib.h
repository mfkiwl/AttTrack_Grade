#ifndef __ATTTRACK_LIB_H__
#define __ATTTRACK_LIB_H__
#include "stdio.h"
// 铲刀IMU惯性传感器观测数据
struct Balde_IMUdata
{
	double imutimetarget;   //IMU输出的时间戳，秒/毫秒，100Hz，计算采样时间间隔
	double accx;            //加速度计X轴输出   g
	double accy;            //        Y         g
	double accz;            //        Z         g
	double gyox;            //陀螺X轴输出，     rad/s
	double gyoy;            //    Y             rad/s
	double gyoz;            //    Z             rad/s
};
typedef struct Balde_IMUdata Balde_IMUdata_t;
// 车体IMU惯性传感器观测数据
struct Body_IMUdata
{
	double imutimetarget;   //IMU输出的时间戳，秒/毫秒，100Hz，计算采样时间间隔
	double accx;            //加速度计X轴输出   g
	double accy;            //        Y         g
	double accz;            //        Z         g
	double gyox;            //陀螺X轴输出，     rad/s
	double gyoy;            //    Y             rad/s
	double gyoz;            //    Z             rad/s
};
typedef struct Body_IMUdata Body_IMUdata_t;
//MC100 GNSS观测数据
struct GNSSdata
{
	double gnsstimetarget;   //GNSS观测的时间戳，周内秒，10Hz
	double lat;              //纬度, WGS84       rad
	double lon;              //经度，WGS84       rad
	double alt;              //高程，WGS84        m
	double gnssyaw;          //双天线航向，北偏东(-180 - 180)      rad
	double noise_gyaw;       //双天线航向观测噪声 rad
	double gnssroll;         //双天线横滚角       rad
	double noise_groll;      //双天线横滚角观测噪声 rad
	double speed;            //地速              m/s
	double speed_ver;        //高程速度          m/s
	int state_pos;           //GNSS位置解状态
	int state_yaw;           //GNSS航向解状态
};
typedef struct GNSSdata GNSSdata_t;
//平地机姿态跟踪观测数据
struct ATdata
{
	Balde_IMUdata_t imu_balde;    //铲刀IMU观测数据，测量铲刀的水平姿态角和横坡角
	Body_IMUdata_t imu_body;      //车架IMU观测数据，测量车架的俯仰角
	GNSSdata_t gnss;              //GNSS观测数据
	double imu_time;			  //IMU时间戳
	double rotation;              //编码器输出，铲刀相对旋转角度， deg
	int balde_install_flag;		  //铲刀传感器安装误差标定1：位置1标定开始 2：位置2标定开始 3：位置3标定开始
	int body_install_falg;		  //车体传感器安装误差标定1：标定开始
	int rotation_install_falg;	  //旋转编码器安装误差标定1：标定开始
	int bgnss_update;             //GNSS观测数据更新标志： 0-未更新 1-更新
	int bbaldeimu_update;         //铲刀IMU数据更新： 0-未更新 1-更新
	int bbodyimu_update;          //车架IMU数据更新： 0-未更新 1-更新
};
typedef struct ATdata ATdata_t;
void ATdata_init(ATdata_t* atd);
void ATdata_reset(ATdata_t* atd);
//上位机配置信息
struct config
{
	double gyostd_thr;        //静态判断的陀螺std阈值， deg/s
	double accstd_thr;        //          加计       ， g
	double gyo_noise_RP;      //水平角度跟踪的陀螺噪声，deg/s
	double gyo_noise_Y;       //航向角度跟踪的陀螺噪声，deg/s
	int acc_noise_option;     //加速度计观测定权策略
	double leverx_R;          //右控制点的X轴杆臂，     m
	double levery_R;          //          Y             m
	double leverz_R;          //          Z             m
	double leverx_L;          //左控制点的X轴杆臂，     m
	double levery_L;          //          Y             m
	double leverz_L;          //          Z             m
	double acc_bias[3];		  //六面体标定加速度零偏    g
	double acc_coefficient[9];//六面体比例系数矩阵
};
typedef struct config config_t;
//输出结果数据
struct result
{
	double gnsstime;           //结果时间戳，      gnss周内秒， 20Hz
	/*铲刀结果*/
	double roll_balde;         //铲刀横滚角，      rad 
	double pitch_balde;        //铲刀俯仰角，      rad
	double heading_balde;      //铲刀航向角，      rad
	double gnss_speed_ver;     //gnss主天线竖直速度，  m/s
	double gnss_alt;           //gnss主天线高程，      m
	double rotation;		   //旋转传感器角度，     rad
	/*车体结果*/
	double roll_body;          //车架横滚角，      rad  
	double pitch_body;         //车架俯仰角，      rad
	/*控制所需参数*/
	double cross_slope_balde;  //铲刀横坡角roll,   rad
	double tilt_slope_balde;   //铲刀纵坡角pitch,  rad
	double yaw_balde;		   //标定后铲刀航向yaw rad
	double CPR_blh[3];         //右控制点位置，    rad，rad，m
	double CPL_blh[3];         //左控制点位置，    rad，rad，m
	double gyoy_balde;         //去零偏的横滚角速度  rad/s
	double ver_balde;		   //铲刀垂直速度(中间控制点)
	double cross_slope_body;   //车架横坡角roll,   rad
	double tilt_slope_body;    //车架纵坡角pitch,  rad
	double expect_slope_angle; //期望横坡角        rad

	int state;                 //解算状态 0:未初始化 1：解算中
	int balde_install_result;  //铲刀安装误差标定状态1：位置1标定完成 2：位置2标定完成 3：铲刀安装误差标定完成
	int body_install_result;   //车体安装误差标定状态0:标定失败 1：车体安装误差标定完成			  
	int rotation_install_result;//0：标定失败 1：旋转编码器标定完成
};
typedef result result_t;
/*************平地机项目接口定义***************/
//上位机重置配置参数接口
int UI_set_config(config_t* cfg);
//角度跟踪、高程融合解算接口
int Grader_AttTrack_Process(ATdata_t* atd, result* res);
//铲刀期望横坡角计算接口
int Grader_Slope_Process(double Design_slope,ATdata_t* atd, result* res);
#endif