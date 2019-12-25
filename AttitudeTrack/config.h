#ifndef __CONFIGURE_CONFIG_H__
#define __CONFIGURE_CONFIG_H__
#include <string.h>
#include "parson.h"
#define WIN32_DEBUG 1
#define ANDROID_DEBUG 0
//#define MG_SOFT_VER "MG.v1.0.0"     /*��һ���㷨��*/
//#define MG_SOFT_DATA "20191018"
//#define MG_SOFT_VER "MG.v1.1.0"     /*����˫���߰�װ���궨��˫λ�÷���װ���궨*/
//#define MG_SOFT_DATA "20191029"
//#define MG_SOFT_VER "MG.v1.1.1"     /*���淶��������*/
//#define MG_SOFT_DATA "20191029"
//#define MG_SOFT_VER "MG.v1.1.2"     /*�߳��ں��㷨�Ż���IMU���ּ�������̹߳۲⣬�����ƽ��*/
//#define MG_SOFT_DATA "20191029"
//#define MG_SOFT_VER "MG.v1.1.3"     /*���������ٶȸĳɲ����м�λ�������ٶ�*/
//#define MG_SOFT_DATA "20191030"     /*�����ӿڲ�����λͳһ�����ٶȣ�g�����ٶ�rad/s,�Ƕȣ�rad*/
//#define MG_SOFT_VER "MG.v1.1.4"     /*�㷨������IMU���ݽ�Ƶ���桢gnss�жϷ�����*/
//#define MG_SOFT_DATA "20191031"     
//#define MG_SOFT_VER "MG.v1.2.1"     /*�㷨�޸�˫λ�÷���װ���궨���̣��ο��챦����*/
//#define MG_SOFT_DATA "20191111" 
//#define MG_SOFT_VER "MG.v1.2.2"     /*�㷨�ӿ����Ӱ�װ���궨��λ��������־*/
//#define MG_SOFT_DATA "20191112"
//#define MG_SOFT_VER "MG.v1.2.3"     /*�㷨�ӿ�������λ�������ļ��������ýӿ�*/
//#define MG_SOFT_DATA "20191114"
//#define MG_SOFT_VER "MG.v1.2.4"      /*�߳��ں��Ż������Ӹ߳�ԭʼ����ƽ��*/
//#define MG_SOFT_DATA "20191212"
#define MG_SOFT_VER "MG.v1.2.5"       /*��׿��̬��汾��EKF��������Ϊc����*/
#define MG_SOFT_DATA "20191219"
#if WIN32_DEBUG
#define  ATTITUDE_TRACK_CONFIG_PATH  "G:\\ƽ�ػ���Ŀ\\config\\AttitudeTrack.cfg"
#else
#define  ATTITUDE_TRACK_CONFIG_PATH  "/attcfg/AttitudeTrack.cfg"
#endif

struct AttTrackCfg
{
	/*������ƫ*/
	double gbiasx;
	double gbiasy;
	double gbiasz;
	/*�Ӽ���ƫ*/
	double abiasx;
	double abiasy;
	double abiasz;
	/*��̬����std��ֵ*/
	double gstdxthr;
	double gstdythr;
	double gstdzthr;
	/*��̬�Ӽ�std��ֵ*/
	double astdxthr;
	double astdythr;
	double astdzthr;
	/*��������*/
	double gnoisex;
	double gnoisey;
	double gnoisez;
	/*�Ӽ�����*/
	double anoisex;
	double anoisey;
	double anoisez;
	/*������ƫ�ȶ���*/
	double gstabilityx;
	double gstabilityy;
	double gstabilityz;
	/*�Ӽ���ƫ�ȶ���*/
	double astabilityx;
	double astabilityy;
	double astabilityz;

	int gshaftx;
	int gshafty;
	int gshaftz;
	int ashaftx;
	int ashafty;
	int ashaftz;
	/*˫���ߺ���std��ֵ*/
	double gnssyawstd_thr;
	double gnssyawvar;
	double gnssrollstd_thr;
	/*�����۸�ֵ*/
	double leverrightx;
	double leverrighty;
	double leverrightz;
	double leverleftx;
	double leverlefty;
	double leverleftz;
	/*��װ����*/
	double balde_install_roll;
	double balde_install_pitch;
	double balde_install_yaw;
	double mainfall_install_pitch;
	/*��ת��������λ*/
	double rotation_zero;
};
typedef AttTrackCfg AttTrackCfg_t;

int init_config();
void save_config();
void creat_config();
//������ƫ
int config_set_gyo_staticbiasx(double num);
double config_get_gyo_staticbiasx();
int config_set_gyo_staticbiasy(double num);
double config_get_gyo_staticbiasy();
int config_set_gyo_staticbiasz(double num);
double config_get_gyo_staticbiasz();
//�Ӽ���ƫ
int config_set_acc_staticbiasx(double num);
double config_get_acc_staticbiasx();
int config_set_acc_staticbiasy(double num);
double config_get_acc_staticbiasy();
int config_set_acc_staticbiasz(double num);
double config_get_acc_staticbiasz();
//���ݾ�̬�ж�std��ֵ
int config_set_gyo_stdx_thr(double num);
double config_get_gyo_stdx_thr();
int config_set_gyo_stdy_thr(double num);
double config_get_gyo_stdy_thr();
int config_set_gyo_stdz_thr(double num);
double config_get_gyo_stdz_thr();
//�Ӽƾ�̬�ж�std��ֵ
int config_set_acc_stdx_thr(double num);
double config_get_acc_stdx_thr();
int config_set_acc_stdy_thr(double num);
double config_get_acc_stdy_thr();
int config_set_acc_stdz_thr(double num);
double config_get_acc_stdz_thr();
//���ݽ��ٶ���������
int config_set_gyo_noisex(double num);
double config_get_gyo_noisex();
int config_set_gyo_noisey(double num);
double config_get_gyo_noisey();
int config_set_gyo_noisez(double num);
double config_get_gyo_noisez();
//�Ӽ����ٶ���������
int config_set_acc_noisex(double num);
double config_get_acc_noisex();
int config_set_acc_noisey(double num);
double config_get_acc_noisey();
int config_set_acc_noisez(double num);
double config_get_acc_noisez();
// ������ƫ�ȶ���
int config_set_gyo_stabilityx(double num);
double config_get_gyo_stabilityx();
int config_set_gyo_stabilityy(double num);
double config_get_gyo_stabilityy();
int config_set_gyo_stabilityz(double num);
double config_get_gyo_stabilityz();
//�Ӽ���ƫ�ȶ���
int config_set_acc_stabilityx(double num);
double config_get_acc_stabilityx();
int config_set_acc_stabilityy(double num);
double config_get_acc_stabilityy();
int config_set_acc_stabilityz(double num);
double config_get_acc_stabilityz();
//������ϵת��
int config_set_gyo_shaftx(int num);
int config_get_gyo_shaftx();
int config_set_gyo_shafty(int num);
int config_get_gyo_shafty();
int config_set_gyo_shaftz(int num);
int config_get_gyo_shaftz();
//�Ӽ���ϵת��
int config_set_acc_shaftx(int num);
int config_get_acc_shaftx();
int config_set_acc_shafty(int num);
int config_get_acc_shafty();
int config_set_acc_shaftz(int num);
int config_get_acc_shaftz();
//˫���ߺ���std�ж���ֵ�͹۲�����
int config_set_gnssyawstd_thr(double num);
double config_get_gnssyawstd_thr();
int config_set_gnssyaw_var(double num);
double config_get_gnssyaw_var();
//˫���ߺ��std�ж���ֵ
int config_set_gnssrollstd_thr(double num);
double config_get_gnssrollstd_thr();
//�ҿ��Ƶ�˱�
int config_set_right_leverx(double num);
double config_get_right_leverx();
int config_set_right_levery(double num);
double config_get_right_levery();
int config_set_right_leverz(double num);
double config_get_right_leverz();
//����Ƶ�˱�
int config_set_left_leverx(double num);
double config_get_left_leverx();
int config_set_left_levery(double num);
double config_get_left_levery();
int config_set_left_leverz(double num);
double config_get_left_leverz();
//������װ���
int config_set_balde_install_roll(double num);
double config_get_balde_install_roll();
int config_set_balde_install_pitch(double num);
double config_get_balde_install_pitch();
int config_set_balde_install_yaw(double num);
double config_get_balde_install_yaw();
//���尲װ���
int config_set_mainfall_install_pitch(double num);
double config_get_mainfall_install_pitch();
//��ת��������λ
int config_set_rotation_install(double num);
double config_get_rotation_install();

void generate_default_config_ADIS16460();
void generate_default_config_BDF06();

void generate_default_config_IS203();
void generate_default_config_Grader();//ƽ�ػ������ļ�
void get_config(AttTrackCfg_t* cfg);
#endif

