#ifndef __TEST_H__
#define __TEST_H__
//��̬���١��̹߳����㷨�ӿ���֤
#include "AttTrack_lib.h"
#include "ATProcess.h"
//#define PI 3.14159265358979323846
#define D2R (PI/180.0)
#define R2D (180.0/PI)
//#define WORK_PATH				 "G:/ƽ�ػ���Ŀ/CGI310����/��������/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	 "test_1031.txt"

//#define WORK_PATH				"G:/ƽ�ػ���Ŀ/2019.1.16����-�Ͼ�/5/"
//#define TEST_RAW_FILE_PATH		WORK_PATH
//#define TEST_RAW_FILE_NAME	    "116data05.txt"

//#define WORK_PATH				 "G:/ƽ�ػ���Ŀ/11.18����/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	 "test04.txt"

//#define WORK_PATH				 "E:/ƽ�ػ���Ŀ/12.5����/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	 "953228983.txt"


//#define WORK_PATH				 "E:/ƽ�ػ���Ŀ/12.5����/�߳�/2/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	 "953231840.txt"

//#define WORK_PATH				 "E:/ƽ�ػ���Ŀ/12.06����/��̬�߳�-�̶��Ƕ�/�̶�����/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	 "953307486.txt"


//#define WORK_PATH				 "E:/ƽ�ػ���Ŀ/12.06����/��̬�߳�-�̶��Ƕ�/�̶�����/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	 "gga.txt"

//#define WORK_PATH				 "G:/ƽ�ػ���Ŀ/12.06����/�������/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	 "data2.txt"


//#define WORK_PATH				 "E:/ƽ�ػ���Ŀ/12.06����/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	 "953298901.txt"


//#define WORK_PATH				 "G:/ƽ�ػ���Ŀ/12.06����/��̬�߳�-�̶��Ƕ�/�̶�����/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	 "953307486.txt"

//#define WORK_PATH				 "G:/ƽ�ػ���Ŀ/12.06����/��̬�߳�-�̶��Ƕ�/�̶����/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	 "953307486.txt"

//#define WORK_PATH				 "G:/ƽ�ػ���Ŀ/12.06����/��̬�߳�-�̶��Ƕ�/�̶�����/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	 "953307486.txt"

//#define WORK_PATH				 "G:/ƽ�ػ���Ŀ/12.16����/��̬/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	 "UB482.txt"



//#define WORK_PATH				 "G:/ƽ�ػ���Ŀ/12.06����/��̬�߳���֤/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	 "953298901.txt"

#define WORK_PATH				 "G:/ƽ�ػ���Ŀ/12.06����/��̬�߳�-���⶯��/"
#define TEST_RAW_FILE_PATH	 WORK_PATH
#define TEST_RAW_FILE_NAME	 "953307486.txt"

//#define WORK_PATH				 "G:/ƽ�ػ���Ŀ/12.06����/��̬�߳�-���⶯��/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	 "953307486.txt"

//#define WORK_PATH				 "G:/ƽ�ػ���Ŀ/12.13����/�忨�̶߳ԱȲ���/��̬�߳�/"
//#define TEST_RAW_FILE_PATH	 WORK_PATH
//#define TEST_RAW_FILE_NAME	 "953307486.txt"
static int Read_imu_num = 0;
struct gyo_angle_test
{
	double balde_acc[3];
	double balde_gyo[3];
	double mainfall_acc[3];
	double mainfall_gyo[3];
	bool test_init;
	bool binit_gyaw;
	bool binit_gbias;
	double gyo_bias[3];
	double gyo_angle[3];
	double mean_gyaw;
	double std_gyaw;
	double mean_acc[3];
	double mean_gyo[3];
	double acc_angle[3];
	vector<double> vgpsyaw;  //0-360
	vector<double> vax;
	vector<double> vay;
	vector<double> vaz;
	vector<double> vgx;
	vector<double> vgy;
	vector<double> vgz;

};

struct gpgga_data {
	double time;
	double lat;
	double lon;
	double alt;
	int stat;
};
typedef struct gyo_angle_test gyo_angle_test_t;
int decode_buf2struct_IS203(char* buf, ATdata_t* atd);
int decode_buf2struct_Grader(char* buf, ATdata_t* atd);
int test_init(gyo_angle_test_t* gyo_test);
int Gyro_init(ATdata_t* atd,gyo_angle_test_t* gyo_test);
int Gyro_Angle_Process(ATdata_t* atd, gyo_angle_test_t* gyo_test);
void GPGGA_decode(struct gpgga_data* gga_data, FILE *file_fp);
#endif