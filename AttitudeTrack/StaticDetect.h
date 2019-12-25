#ifndef __STATICDETECT_H__
#define __STATICDETECT_H__
#include <vector>
#define glv_g0            9.80665
using namespace std;
/*------------by dhf,2017.08.24---------*/
class StaticDetect
{
public:
	int N;                 //滑动窗口大小
	vector<double> vax;    //加计X轴数据窗  m/s2
	vector<double> vay;    //    Y
	vector<double> vaz;    //    Z
	vector<double> vgx;    //陀螺X          deg/s
	vector<double> vgy;    //    Y
	vector<double> vgz;    //    Z
	double accvar;         //加计观测噪声方差
	double gyovar;         //陀螺
	double thre;           //判断阈值
	int bfinshinit;
	int num_gnssstatic;    //GNSS连续静态计数
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

//探测加计和陀螺模值的std
int detectstatic_std(double acc[], double gyo[]);
#endif