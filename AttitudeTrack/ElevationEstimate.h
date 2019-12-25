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
	//ƽ���˲�������
	vector<double> queue_gnss_alt;
	vector<double> queue_gnss_ver;
	double  alt;        //���θ� m  ����Ϊ��
	double  vu;         //�̷߳����ٶ� m/s  ����Ϊ��
	double  accalt;     //�̷߳�����ٶ�  m/s2
	double  accbias;    //���Ƶ���ƫ��� m/s2
	double  anzpre;     //����ϵ��Z��Ӽ�
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
	KF kfelv;           //��׼kalman�˲���ģ��
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
//����ϵ��ά����ת��������ˮƽ����ϵ��NED,ǰ����
void Conver_b2lever(double roll, double pitch, double var_b[3], double var_lever[3]);
#endif
