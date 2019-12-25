#ifndef __ATTTRACK_LIB_H__
#define __ATTTRACK_LIB_H__
#include "stdio.h"
// ����IMU���Դ������۲�����
struct Balde_IMUdata
{
	double imutimetarget;   //IMU�����ʱ�������/���룬100Hz���������ʱ����
	double accx;            //���ٶȼ�X�����   g
	double accy;            //        Y         g
	double accz;            //        Z         g
	double gyox;            //����X�������     rad/s
	double gyoy;            //    Y             rad/s
	double gyoz;            //    Z             rad/s
};
typedef struct Balde_IMUdata Balde_IMUdata_t;
// ����IMU���Դ������۲�����
struct Body_IMUdata
{
	double imutimetarget;   //IMU�����ʱ�������/���룬100Hz���������ʱ����
	double accx;            //���ٶȼ�X�����   g
	double accy;            //        Y         g
	double accz;            //        Z         g
	double gyox;            //����X�������     rad/s
	double gyoy;            //    Y             rad/s
	double gyoz;            //    Z             rad/s
};
typedef struct Body_IMUdata Body_IMUdata_t;
//MC100 GNSS�۲�����
struct GNSSdata
{
	double gnsstimetarget;   //GNSS�۲��ʱ����������룬10Hz
	double lat;              //γ��, WGS84       rad
	double lon;              //���ȣ�WGS84       rad
	double alt;              //�̣߳�WGS84        m
	double gnssyaw;          //˫���ߺ��򣬱�ƫ��(-180 - 180)      rad
	double noise_gyaw;       //˫���ߺ���۲����� rad
	double gnssroll;         //˫���ߺ����       rad
	double noise_groll;      //˫���ߺ���ǹ۲����� rad
	double speed;            //����              m/s
	double speed_ver;        //�߳��ٶ�          m/s
	int state_pos;           //GNSSλ�ý�״̬
	int state_yaw;           //GNSS�����״̬
};
typedef struct GNSSdata GNSSdata_t;
//ƽ�ػ���̬���ٹ۲�����
struct ATdata
{
	Balde_IMUdata_t imu_balde;    //����IMU�۲����ݣ�����������ˮƽ��̬�Ǻͺ��½�
	Body_IMUdata_t imu_body;      //����IMU�۲����ݣ��������ܵĸ�����
	GNSSdata_t gnss;              //GNSS�۲�����
	double imu_time;			  //IMUʱ���
	double rotation;              //��������������������ת�Ƕȣ� deg
	int balde_install_flag;		  //������������װ���궨1��λ��1�궨��ʼ 2��λ��2�궨��ʼ 3��λ��3�궨��ʼ
	int body_install_falg;		  //���崫������װ���궨1���궨��ʼ
	int rotation_install_falg;	  //��ת��������װ���궨1���궨��ʼ
	int bgnss_update;             //GNSS�۲����ݸ��±�־�� 0-δ���� 1-����
	int bbaldeimu_update;         //����IMU���ݸ��£� 0-δ���� 1-����
	int bbodyimu_update;          //����IMU���ݸ��£� 0-δ���� 1-����
};
typedef struct ATdata ATdata_t;
void ATdata_init(ATdata_t* atd);
void ATdata_reset(ATdata_t* atd);
//��λ��������Ϣ
struct config
{
	double gyostd_thr;        //��̬�жϵ�����std��ֵ�� deg/s
	double accstd_thr;        //          �Ӽ�       �� g
	double gyo_noise_RP;      //ˮƽ�Ƕȸ��ٵ�����������deg/s
	double gyo_noise_Y;       //����Ƕȸ��ٵ�����������deg/s
	int acc_noise_option;     //���ٶȼƹ۲ⶨȨ����
	double leverx_R;          //�ҿ��Ƶ��X��˱ۣ�     m
	double levery_R;          //          Y             m
	double leverz_R;          //          Z             m
	double leverx_L;          //����Ƶ��X��˱ۣ�     m
	double levery_L;          //          Y             m
	double leverz_L;          //          Z             m
	double acc_bias[3];		  //������궨���ٶ���ƫ    g
	double acc_coefficient[9];//���������ϵ������
};
typedef struct config config_t;
//����������
struct result
{
	double gnsstime;           //���ʱ�����      gnss�����룬 20Hz
	/*�������*/
	double roll_balde;         //��������ǣ�      rad 
	double pitch_balde;        //���������ǣ�      rad
	double heading_balde;      //��������ǣ�      rad
	double gnss_speed_ver;     //gnss��������ֱ�ٶȣ�  m/s
	double gnss_alt;           //gnss�����߸̣߳�      m
	double rotation;		   //��ת�������Ƕȣ�     rad
	/*������*/
	double roll_body;          //���ܺ���ǣ�      rad  
	double pitch_body;         //���ܸ����ǣ�      rad
	/*�����������*/
	double cross_slope_balde;  //�������½�roll,   rad
	double tilt_slope_balde;   //�������½�pitch,  rad
	double yaw_balde;		   //�궨���������yaw rad
	double CPR_blh[3];         //�ҿ��Ƶ�λ�ã�    rad��rad��m
	double CPL_blh[3];         //����Ƶ�λ�ã�    rad��rad��m
	double gyoy_balde;         //ȥ��ƫ�ĺ�����ٶ�  rad/s
	double ver_balde;		   //������ֱ�ٶ�(�м���Ƶ�)
	double cross_slope_body;   //���ܺ��½�roll,   rad
	double tilt_slope_body;    //�������½�pitch,  rad
	double expect_slope_angle; //�������½�        rad

	int state;                 //����״̬ 0:δ��ʼ�� 1��������
	int balde_install_result;  //������װ���궨״̬1��λ��1�궨��� 2��λ��2�궨��� 3��������װ���궨���
	int body_install_result;   //���尲װ���궨״̬0:�궨ʧ�� 1�����尲װ���궨���			  
	int rotation_install_result;//0���궨ʧ�� 1����ת�������궨���
};
typedef result result_t;
/*************ƽ�ػ���Ŀ�ӿڶ���***************/
//��λ���������ò����ӿ�
int UI_set_config(config_t* cfg);
//�Ƕȸ��١��߳��ںϽ���ӿ�
int Grader_AttTrack_Process(ATdata_t* atd, result* res);
//�����������½Ǽ���ӿ�
int Grader_Slope_Process(double Design_slope,ATdata_t* atd, result* res);
#endif