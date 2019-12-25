#ifndef __ATPROCESS_H__
#define __ATPROCESS_H__
#include "AttInit.h"
#include "DataStruct.h"
#include "ComFunc.h"
#include "config.h"
#include "ElevationEstimate.h"
#include "StaticDetect.h"
#include "Calibrate.h"
#include "kalmanfilter.h"
#define ACC_SMOOTH_NOISE 1
#define ACC_LINEAR_NOISE 0
#define DEBUG_SAVE 1
#define SaveElePro 0
#define Heading_2ant_Err 0
#define ACC_BIAS_CAL 0
#define RAD_PI_PI(x) {if((x)>PI) (x)-=2*PI;   else if((x)<-PI) (x)+=2*PI;}
#define ACC_SMOOTH_WIN 5
class ATProcess
{
public:
	bool battinit;
	bool bkfinit;
	bool bprocessinit;
	bool heading_installerr_ok;
	bool rotation_zero_ok;
	bool Acc_Bias_ok;
	bool position1_ok;
	bool position2_ok;
	bool position3_ok;
	double dt_baldeimu;     //����IMU�������
	double tpre_baldeimu;
	double dt_mainfallimu;  //����IMU�������
	double tpre_mainfallimu;
	//������̬ NED
	double integ_att_blade[3];
	double roll_balde;      //rad
	double pitch_balde;
	double heading_balde; 
	double Cb2n_balde[9];
	double qua_balde[4];
	double crossslope_balde;  //�������½� rad
	double tiltslope_balde;   //�������½� rad
	//������̬ NED
	double tiltslop_mainfall; //�������½� rad
	//������ƫ���ٶ�
	double gyo_bias_balde[3];    //�������ٶ�
	double gyo_bias_mainfall[3]; //���ܽ��ٶ�

	double gyobias_balde[3];
	double gyobias_mainfall[3];
	double accpre_balde[3];
	double gyopre_balde[3];
	double accpre_mainfall[3];
	double gyopre_mainfall[3];
	double rotation_zero;

	int num_accnorm_balde;        //�Ӽ�ģֵС����ֵ����
	int num_gyonorm_balde;        //����ģֵС����ֵ����
	int num_accnorm_mainfall;     //�Ӽ�ģֵС����ֵ����
	int num_gyonorm_mainfall;     //����ģֵС����ֵ����
	//ƽ���˲�������
	vector<double> queue_ax_balde;
	vector<double> queue_ay_balde;
	vector<double> queue_az_balde;
	vector<double> queue_ax_mainfall;
	vector<double> queue_ay_mainfall;
	vector<double> queue_az_mainfall;
	double smooth_acc_balde[3];
	double smooth_acc_mainfall[3];
	//�������������ݴ����Ƿ�����
	int num_nogetbaldeimu;        
	int num_nogetmainfallimu;
	int num_nogetgnss;


	int blade_install_result;
	int body_install_result;
	int rotation_install_result;
	//���ò����ṹ��
	AttTrackCfg_t cfg;
	Cal_Install_Error cal;
	AttTrackData idata;     //����Ԫ�۲�����
	AttInit attinitial;     //��̬��ʼ��
	KF ekf_pitch;
	KF ekf_roll;
	KF ekf_yaw;
	KF ekf_pitch_mainfall;
public:
	ATProcess();
	ATProcess(double dtime);
	~ATProcess();
	int Acc_Amooth(AttTrackData& iatd,int option);
	int Rotation_Zero_Err(AttTrackData& iatd);
	int Two_Position_Install_Err(AttTrackData& iatd);
	int process_MotorGrader(AttTrackData& iatd, int bstatic); 
};
#endif

