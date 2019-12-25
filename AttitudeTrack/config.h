#ifndef __CONFIGURE_CONFIG_H__
#define __CONFIGURE_CONFIG_H__
#include <string.h>
#include "parson.h"
#define WIN32_DEBUG 1
#define ANDROID_DEBUG 0
//#define MG_SOFT_VER "MG.v1.0.0"     /*第一版算法库*/
//#define MG_SOFT_DATA "20191018"
//#define MG_SOFT_VER "MG.v1.1.0"     /*增加双天线安装误差标定、双位置法安装误差标定*/
//#define MG_SOFT_DATA "20191029"
//#define MG_SOFT_VER "MG.v1.1.1"     /*不规范代码完善*/
//#define MG_SOFT_DATA "20191029"
//#define MG_SOFT_VER "MG.v1.1.2"     /*高程融合算法优化，IMU部分加入推算高程观测，结果更平滑*/
//#define MG_SOFT_DATA "20191029"
//#define MG_SOFT_VER "MG.v1.1.3"     /*铲刀天向速度改成铲刀中间位置天向速度*/
//#define MG_SOFT_DATA "20191030"     /*函数接口参数单位统一，加速度：g，角速度rad/s,角度：rad*/
//#define MG_SOFT_VER "MG.v1.1.4"     /*算法中增加IMU数据降频仿真、gnss中断仿真检测*/
//#define MG_SOFT_DATA "20191031"     
//#define MG_SOFT_VER "MG.v1.2.1"     /*算法修改双位置法安装误差标定流程，参考天宝方案*/
//#define MG_SOFT_DATA "20191111" 
//#define MG_SOFT_VER "MG.v1.2.2"     /*算法接口增加安装误差标定上位机交互标志*/
//#define MG_SOFT_DATA "20191112"
//#define MG_SOFT_VER "MG.v1.2.3"     /*算法接口增加上位机配置文件参数重置接口*/
//#define MG_SOFT_DATA "20191114"
//#define MG_SOFT_VER "MG.v1.2.4"      /*高程融合优化，增加高程原始数据平滑*/
//#define MG_SOFT_DATA "20191212"
#define MG_SOFT_VER "MG.v1.2.5"       /*安卓动态库版本，EKF矩阵运算为c语言*/
#define MG_SOFT_DATA "20191219"
#if WIN32_DEBUG
#define  ATTITUDE_TRACK_CONFIG_PATH  "G:\\平地机项目\\config\\AttitudeTrack.cfg"
#else
#define  ATTITUDE_TRACK_CONFIG_PATH  "/attcfg/AttitudeTrack.cfg"
#endif

struct AttTrackCfg
{
	/*陀螺零偏*/
	double gbiasx;
	double gbiasy;
	double gbiasz;
	/*加计零偏*/
	double abiasx;
	double abiasy;
	double abiasz;
	/*静态陀螺std阈值*/
	double gstdxthr;
	double gstdythr;
	double gstdzthr;
	/*静态加计std阈值*/
	double astdxthr;
	double astdythr;
	double astdzthr;
	/*陀螺噪声*/
	double gnoisex;
	double gnoisey;
	double gnoisez;
	/*加计噪声*/
	double anoisex;
	double anoisey;
	double anoisez;
	/*陀螺零偏稳定性*/
	double gstabilityx;
	double gstabilityy;
	double gstabilityz;
	/*加计零偏稳定性*/
	double astabilityx;
	double astabilityy;
	double astabilityz;

	int gshaftx;
	int gshafty;
	int gshaftz;
	int ashaftx;
	int ashafty;
	int ashaftz;
	/*双天线航向std阈值*/
	double gnssyawstd_thr;
	double gnssyawvar;
	double gnssrollstd_thr;
	/*铲刀臂杆值*/
	double leverrightx;
	double leverrighty;
	double leverrightz;
	double leverleftx;
	double leverlefty;
	double leverleftz;
	/*安装误差角*/
	double balde_install_roll;
	double balde_install_pitch;
	double balde_install_yaw;
	double mainfall_install_pitch;
	/*旋转编码器零位*/
	double rotation_zero;
};
typedef AttTrackCfg AttTrackCfg_t;

int init_config();
void save_config();
void creat_config();
//陀螺零偏
int config_set_gyo_staticbiasx(double num);
double config_get_gyo_staticbiasx();
int config_set_gyo_staticbiasy(double num);
double config_get_gyo_staticbiasy();
int config_set_gyo_staticbiasz(double num);
double config_get_gyo_staticbiasz();
//加计零偏
int config_set_acc_staticbiasx(double num);
double config_get_acc_staticbiasx();
int config_set_acc_staticbiasy(double num);
double config_get_acc_staticbiasy();
int config_set_acc_staticbiasz(double num);
double config_get_acc_staticbiasz();
//陀螺静态判断std阈值
int config_set_gyo_stdx_thr(double num);
double config_get_gyo_stdx_thr();
int config_set_gyo_stdy_thr(double num);
double config_get_gyo_stdy_thr();
int config_set_gyo_stdz_thr(double num);
double config_get_gyo_stdz_thr();
//加计静态判断std阈值
int config_set_acc_stdx_thr(double num);
double config_get_acc_stdx_thr();
int config_set_acc_stdy_thr(double num);
double config_get_acc_stdy_thr();
int config_set_acc_stdz_thr(double num);
double config_get_acc_stdz_thr();
//陀螺角速度量测噪声
int config_set_gyo_noisex(double num);
double config_get_gyo_noisex();
int config_set_gyo_noisey(double num);
double config_get_gyo_noisey();
int config_set_gyo_noisez(double num);
double config_get_gyo_noisez();
//加计线速度量测噪声
int config_set_acc_noisex(double num);
double config_get_acc_noisex();
int config_set_acc_noisey(double num);
double config_get_acc_noisey();
int config_set_acc_noisez(double num);
double config_get_acc_noisez();
// 陀螺零偏稳定性
int config_set_gyo_stabilityx(double num);
double config_get_gyo_stabilityx();
int config_set_gyo_stabilityy(double num);
double config_get_gyo_stabilityy();
int config_set_gyo_stabilityz(double num);
double config_get_gyo_stabilityz();
//加计零偏稳定性
int config_set_acc_stabilityx(double num);
double config_get_acc_stabilityx();
int config_set_acc_stabilityy(double num);
double config_get_acc_stabilityy();
int config_set_acc_stabilityz(double num);
double config_get_acc_stabilityz();
//陀螺轴系转换
int config_set_gyo_shaftx(int num);
int config_get_gyo_shaftx();
int config_set_gyo_shafty(int num);
int config_get_gyo_shafty();
int config_set_gyo_shaftz(int num);
int config_get_gyo_shaftz();
//加计轴系转换
int config_set_acc_shaftx(int num);
int config_get_acc_shaftx();
int config_set_acc_shafty(int num);
int config_get_acc_shafty();
int config_set_acc_shaftz(int num);
int config_get_acc_shaftz();
//双天线航向std判断阈值和观测噪声
int config_set_gnssyawstd_thr(double num);
double config_get_gnssyawstd_thr();
int config_set_gnssyaw_var(double num);
double config_get_gnssyaw_var();
//双天线横滚std判断阈值
int config_set_gnssrollstd_thr(double num);
double config_get_gnssrollstd_thr();
//右控制点杆臂
int config_set_right_leverx(double num);
double config_get_right_leverx();
int config_set_right_levery(double num);
double config_get_right_levery();
int config_set_right_leverz(double num);
double config_get_right_leverz();
//左控制点杆臂
int config_set_left_leverx(double num);
double config_get_left_leverx();
int config_set_left_levery(double num);
double config_get_left_levery();
int config_set_left_leverz(double num);
double config_get_left_leverz();
//铲刀安装误差
int config_set_balde_install_roll(double num);
double config_get_balde_install_roll();
int config_set_balde_install_pitch(double num);
double config_get_balde_install_pitch();
int config_set_balde_install_yaw(double num);
double config_get_balde_install_yaw();
//车体安装误差
int config_set_mainfall_install_pitch(double num);
double config_get_mainfall_install_pitch();
//旋转编码器零位
int config_set_rotation_install(double num);
double config_get_rotation_install();

void generate_default_config_ADIS16460();
void generate_default_config_BDF06();

void generate_default_config_IS203();
void generate_default_config_Grader();//平地机配置文件
void get_config(AttTrackCfg_t* cfg);
#endif

