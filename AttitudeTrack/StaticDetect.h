#ifndef __STATICDETECT_H__
#define __STATICDETECT_H__
#include <vector>
#define glv_g0            9.80665
using namespace std;
/*------------by dhf,2017.08.24---------*/
class StaticDetect
{
public:
	int N;                 //�������ڴ�С
	vector<double> vax;    //�Ӽ�X�����ݴ�  m/s2
	vector<double> vay;    //    Y
	vector<double> vaz;    //    Z
	vector<double> vgx;    //����X          deg/s
	vector<double> vgy;    //    Y
	vector<double> vgz;    //    Z
	double accvar;         //�Ӽƹ۲���������
	double gyovar;         //����
	double thre;           //�ж���ֵ
	int bfinshinit;
	int num_gnssstatic;    //GNSS������̬����
public:
	StaticDetect() {};
	~StaticDetect() {};
	StaticDetect(int winlen, double thread);
	int calstaticstd(double acc[], double gyo[], int bgnssupdate, double gpsvn[], int option);
	int detect_SHOE(double acc[], double gyo[], int bgnssupdate, double gpsvn[]);
	int detect_ARED(double acc[], double gyo[], int bgnssupdate, double gpsvn[]);
	int detect_AMVD(double acc[], double gyo[], int bgnssupdate, double gpsvn[]);
	int detect_AMD(double acc[], double gyo[], int bgnssupdate, double gpsvn[]);
};

//̽��Ӽƺ�����ģֵ��std
int detectstatic_std(double acc[], double gyo[]);
#endif