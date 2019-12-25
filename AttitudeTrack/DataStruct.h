#ifndef __DATASTRUCT_H__
#define __DATASTRUCT_H__
#include "ComFunc.h"
#include "AttTrack_lib.h"
#include "config.h"
//���嵼��ϵΪNED������ϵΪFRD
//#define I80S_ADIS16460
//#define 
#define IS203
#define GNSS_UPDATE_TIME 50  //GNSS������� 50ms 20hz
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
	time_t time;        /* time (s) expressed by standard time_t ��1970.01.01 0�뵽���ڵ����� long int*/
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
	double imutime_balde;      //����IMU����ʱ�������
	double utc_time;
	double gyo_balde[3];       //����XYZ���ݽ��ٶȣ�rad/s��ǰ����
	double acc_balde[3];       //����XYZ�ӼƼ��ٶȣ�m/s2,ǰ����	
	int bbaldeimu_update;      //����IMU���ݸ��±�־

	double imutime_mainfall;      //����IMU����ʱ�������
	double integ_pitch;			  //�������ݻ��ָ�����
	double gyo_mainfall[3];       //����XYZ���ݽ��ٶȣ�rad/s��ǰ����
	double acc_mainfall[3];       //����XYZ�ӼƼ��ٶȣ�m/s2,ǰ����
	int bmainfallimu_update;      //����IMU���ݸ��±�־

	double rotation_balde;     //���������ת�Ƕȣ�rad/s

	bool gnss_alt_err;	       //gnss�߳��쳣��־
	double gnsstime;           //GPSʱ���
	double gnsstime_pre;	   //GPSǰһ��ʱ���
	int gnss_lost_num;		   //GPS�жϵ���
	double BLH_ant[3];         //������λ����λ�ã�γ�ȣ����ȣ��̣߳�rad,rad,m��
	double Mean_Gnss_Alt;	   //ǰ�����߳̾�ֵ
	double Gnss_Alt_Pre;
	double speed;              //gnss����
	double speed_ver;          //gnss��ֱ�ٶ�
	int gstate_pos;            //GNSS��λ��״̬
	double yaw_2ant;           //˫���ߺ���deg,��ƫ��
	double roll_2ant;          //˫���ߺ���ǣ�deg
	double pitch_2ant;	       //˫���߸����ǣ�deg
	double gyaw_noise;         //˫���ߺ����׼��
	double groll_noise;        //˫���ߺ���Ǳ�׼��
	int gstate_2ant;           //˫���ߺ����״̬
	int bgnssupdate;           //gnss�����

	double ep[6];			//utcʱ��
	int week;				//gnss��
	double weektime_sec;	//gnss
	double imu_time;		//IMUʱ���
	
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
void Data_Check(AttTrackData* atdata);//�����쳣ֵ��⣬�Ƿ񶪵�
void Gnss_time_decode(AttTrackData* atdata);
#endif


