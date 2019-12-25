#include "test.h"
ATdata_t MG_data;
result_t MG_result;
static int testinit = 0;
static int init_num = 0;
static int gyo_init = 0;
double mean_acc[3] = { 0 };
double mean_gyo[3] = { 0 };
//void main()
//{
//	FILE *fp;
//	FILE *fpp;
//	char buff[208];
//	fp = fopen("G:/平地机项目/2019.1.16数据-南京/1/116data01.txt", "rt");//原始数据路径
//	fscanf(fp, "%s", buff);
//	if (!fp)
//	{
//		printf("cannot open file01\n");
//	}
//	fpp = fopen("G:/平地机项目/2019.1.16数据-南京/1/testout.txt", "wt");//结果保存路径
//	if (!fpp)
//	{
//		printf("cannot open file02\n");
//	}
//	while (!feof(fp))
//	{
//		int decode_falg = 0;
//		decode_falg =decode_buf2struct_IS203(buff, &MG_data);//测试数据解码
//		int MG_flag = 0;
//		if (decode_falg)
//		{
//			MG_flag = AttTrack_Process_struct(&MG_data, &MG_result);//接口测试
//			fprintf(fpp, "%f, %f, %f, %f, %f, %f, %f\r\n",
//				MG_result.pitch_balde*R2D, MG_result.roll_balde*R2D, MG_result.heading_balde*R2D,
//				MG_result.cross_slope_balde*R2D, MG_result.tilt_slope_balde*R2D, MG_result.pitch_mainfall*R2D,
//				MG_result.tilt_slope_mainfall*R2D);
//		}
//	}
//}
int decode_buf2struct_Grader(char* buf, ATdata_t* atd)
{
	if (strlen(buf)<5) return 0;
	double val[12] = { 0.0 };
	if (strstr(buf, "IMU2,"))//IMU2
	{
		Str2Array(buf + 4, ",", val);

		atd->imu_balde.imutimetarget = val[0]; //直接是采样间隔 100Hz
		atd->imu_balde.accx = val[1] * GN;//2,-1,-3
		atd->imu_balde.accy = val[2] * GN;
		atd->imu_balde.accz = val[3] * GN;
		atd->imu_balde.gyox = val[4]*D2R;
		atd->imu_balde.gyoy = val[5] * D2R;
		atd->imu_balde.gyoz = val[6] * D2R;
		atd->bbaldeimu_update = 1;
		return 2;
	}
	else if (strstr(buf, "IMU1,"))//imu1
	{
		Str2Array(buf + 4, ",", val);
		atd->imu_body.imutimetarget = val[0];//2,-1 ,-3
		atd->imu_body.accx = val[2] * GN;
		atd->imu_body.accy = val[1] * GN;
		atd->imu_body.accz = val[3] * GN;
		atd->imu_body.gyox = val[5] * D2R;
		atd->imu_body.gyoy = val[4] * D2R;
		atd->imu_body.gyoz = val[6] * D2R;
		atd->bbodyimu_update = 1;
		return 2;
	}
	else if (strstr(buf, "GNSS,"))
	{
		Str2Array(buf + 5, ",", val);
		atd->gnss.gnsstimetarget = val[0];//采样间隔20hz
		atd->gnss.lat = val[2];
		atd->gnss.lon = val[1];
		atd->gnss.alt = val[3];
		double yaw_tmp = 0;
		yaw_tmp = val[4]*D2R + PI / 2;//根据双天线安装方向确定(左正右辅)
		RAD_PI_PI(yaw_tmp);
		atd->gnss.gnssroll = val[6] * D2R;
		atd->gnss.gnssyaw = yaw_tmp;
		atd->gnss.speed = val[8];
		atd->gnss.speed_ver = val[9];
		atd->gnss.state_pos = (int)val[10];
		atd->gnss.state_yaw = (int)val[11];
		atd->bgnss_update = 1;
		return 1;
	}
	else
	{
		return 0;
	}
}
int decode_buf2struct_IS203(char* buf, ATdata_t* atd)
{
	if (strlen(buf)<5) return 0;
	double val[10] = { 0.0 };
	if (strstr(buf, "IMU,"))
	{
		Str2Array(buf + 4, ",", val);
		Read_imu_num++;
		atd->imu_balde.imutimetarget = val[0]; //直接是采样间隔 200hz
		atd->imu_balde.accx = val[1];
		atd->imu_balde.accy = val[2];
		atd->imu_balde.accz = val[3];
		atd->imu_balde.gyox = val[4];
		atd->imu_balde.gyoy = val[5];
		atd->imu_balde.gyoz = val[6];


		atd->imu_body.imutimetarget = val[0];
		atd->imu_body.accx = val[1];
		atd->imu_body.accy = val[2];
		atd->imu_body.accz = val[3];
		atd->imu_body.gyox = val[4];
		atd->imu_body.gyoy = val[5];
		atd->imu_body.gyoz = val[6];
		atd->bbodyimu_update = 1;
#if 0
		if (Read_imu_num == 2)//1/4降采样
		{
			atd->imu_balde.imutimetarget = val[0] * 2;//采样时间*4
			atd->bbaldeimu_update = 1;
			Read_imu_num = 0;
			return 2;
		}
		else
		{
			atd->bbaldeimu_update = 0;
			return 0;
		}
#endif
		atd->bbaldeimu_update = 1;
		return 2;
	}
	else if (strstr(buf, "GNSS,"))
	{
		Str2Array(buf + 5, ",", val);
		atd->gnss.gnsstimetarget = val[0];
		atd->gnss.lat = val[1];
		atd->gnss.lon = val[2];
		atd->gnss.alt = val[3];
		double yaw_tmp = 0;
		yaw_tmp = val[4];
		RAD_PI_PI(yaw_tmp);
		atd->gnss.gnssyaw = yaw_tmp;
		atd->gnss.speed = val[5];
		atd->gnss.speed_ver = val[6];
		atd->gnss.state_pos = (int)val[7];
		atd->gnss.state_yaw = (int)val[8];
		atd->bgnss_update = 1;
		return 1;
	}
	else
	{
		return 0;
	}
}

int test_init(gyo_angle_test_t* gyo_test)
{
	gyo_test->test_init = false;
	gyo_test->binit_gbias = false;
	gyo_test->binit_gyaw = false;
	gyo_test->mean_gyaw = 0.0;
	gyo_test->mean_gyaw = 0.0;
	for (int i = 0; i < 3; i++)
	{
		gyo_test->gyo_angle[i] = 0.0;
		gyo_test->gyo_bias[i] = 0.0;
		gyo_test->balde_acc[i] = 0.0;
		gyo_test->balde_gyo[i] = 0.0;
		gyo_test->mainfall_acc[i] = 0.0;
		gyo_test->mainfall_gyo[i] = 0.0;
		//gyo_test->mean_gyo[i] = 0.0;
		//gyo_test->mean_acc[i] = 0.0;
	}
	return 1;
}
int Gyro_init(ATdata_t* atd, gyo_angle_test_t* gyo_test)
{
		/*------------IMU1水平姿态和陀螺零偏初始化-------------*/
		if (atd->bbaldeimu_update)
		{
			if (init_num < 1000) //100HZ 10秒
			{
				mean_acc[0]+= gyo_test->balde_acc[0];
				mean_acc[1] += gyo_test->balde_acc[1];
				mean_acc[2] += gyo_test->balde_acc[2];
				mean_gyo[0] += gyo_test->balde_gyo[0];
				mean_gyo[1] += gyo_test->balde_gyo[1];
				mean_gyo[2] += gyo_test->balde_gyo[2];
				init_num++;
				return 0;
			}
			else
			{
				for (int i = 0; i < 3; i++)
				{
					mean_acc[i] /= 1000;
					mean_gyo[i] /= 1000;
				}
				gyo_test->gyo_bias[0] = mean_gyo[0];
				gyo_test->gyo_bias[1] = mean_gyo[1];
				gyo_test->gyo_bias[2] = mean_gyo[2];
				//计算横滚和俯仰角
				gyo_test->gyo_angle[0] = atan2(-mean_acc[1], -mean_acc[2]);//姿态定义？
				gyo_test->gyo_angle[1] = atan2(mean_acc[0] , sqrt(mean_acc[1] * mean_acc[1] + mean_acc[2] * mean_acc[2]));
				return 1;
			}
			return 0;
		}
		return 0;
}
int Gyro_Angle_Process(ATdata_t* atd, gyo_angle_test_t* gyo_test) 
{
	if (!testinit)
	{
		test_init(gyo_test);
		testinit = 1;
		return 0;
	}
	gyo_test->balde_acc[0] = atd->imu_balde.accy;
	gyo_test->balde_acc[1] = -atd->imu_balde.accx;
	gyo_test->balde_acc[2] = -atd->imu_balde.accz;
	gyo_test->balde_gyo[0] = atd->imu_balde.gyoy;
	gyo_test->balde_gyo[1] = -atd->imu_balde.gyox;
	gyo_test->balde_gyo[2] = -atd->imu_balde.gyoz;
	if (testinit)
	{
		if (!gyo_init)
		{
			int angle_init = 0;
			//要改 加入双天线航向解状态和GPS位置初始化 修改阈值 返回值反映初始化失败原因
			angle_init = Gyro_init(atd, gyo_test);
			if (angle_init)
			{
				gyo_test->gyo_angle[2] = atd->gnss.gnssyaw;
				gyo_init = 1;
			}
		}
	}
	if (gyo_init)
	{
		gyo_test->gyo_angle[0] += (gyo_test->balde_gyo[0] - gyo_test->gyo_bias[0])*atd->imu_balde.imutimetarget;
		gyo_test->gyo_angle[1] += (gyo_test->balde_gyo[1] - gyo_test->gyo_bias[1])*atd->imu_balde.imutimetarget;
		gyo_test->gyo_angle[2] += (gyo_test->balde_gyo[2] - gyo_test->gyo_bias[2])*atd->imu_balde.imutimetarget;
		gyo_test->acc_angle[0]= atan2(-gyo_test->balde_acc[1], -gyo_test->balde_acc[2]);
		gyo_test->acc_angle[1]= atan2(gyo_test->balde_acc[0], sqrt(gyo_test->balde_acc[1] * gyo_test->balde_acc[1] +
			gyo_test->balde_acc[2] * gyo_test->balde_acc[2]));
		RAD_PI_PI(gyo_test->gyo_angle[2]);
		gyo_test->acc_angle[2] = atd->gnss.gnssyaw;
		return 1;
	}
	return 0;
}

void GPGGA_decode(struct gpgga_data* gga_data, FILE *file_fp)
{
	char NAME[3];
	char test[10000];
	char gyro[218];
	memset(NAME, 0x00, sizeof(NAME));
	//double time1;
	static double time0 = 0;
	fscanf(file_fp, "%s", test);
	printf("NAME=%s\n",test);
	int flag = 0;
	int m, i, b[40];
	char ch;
	ch = ',';
	m = (int)strlen(test);
	gtime_t gps_time;
	double ep[6] = { 0 };
	ep[0] = 2019;
	ep[1] = 12;
	ep[2] = 16;
	/*##########################IMU数据############################*/
	double alt_err = 0;
	int *week = 0;
	for (i = 0; i < m; ++i)
	{
		if (test[i] == ch)
		{

			b[flag] = i + 1;
			if (flag == 0)
			{
				strncpy(gyro, test + b[flag], 8);
				double time= atof(gyro);
				ep[3] = (int)(time / 10000);
				ep[4] = (int)((time  - ep[3] * 10000)/100);
				ep[5] = time - ep[3]*10000-ep[4]*100;
				gps_time = epoch2time(ep);
				//printf("%f\n", time);
				//printf("%f-%f-%f:%f:%f:%f\n", ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]);
				double week_sec=time2gpst(gps_time, week);
				gga_data->time = week_sec;
			}
			if (flag == 1)
			{
				strncpy(gyro, test + b[flag], 12);
				double lat = 0;
				lat= atof(gyro);
				int int_lat = (int)(lat / 100);
				gga_data->lat = (lat - int_lat * 100) / 60+ int_lat;
			}
			if (flag == 3)
			{
				strncpy(gyro, test + b[flag], 12);
				double lon = 0;
				lon= atof(gyro);
				int int_lon = (int)(lon / 100);
				gga_data->lon = (lon - int_lon * 100) / 60 + int_lon;
			}
			if (flag == 5)
			{
				strncpy(gyro, test + b[flag], 3);
				gga_data->stat = (int)atof(gyro);
			}
			if (flag == 10)
			{
				strncpy(gyro, test + b[flag], 6);
				alt_err = atof(gyro);
			}
			if (flag == 8)
			{
				strncpy(gyro, test + b[flag], 6);
				gga_data->alt= atof(gyro)+1.08;
			}
			flag += 1;
		}
	}
#if DEBUG_SAVE
		outdhf(12, "%d/%02d/%02d %02d:%02d:%02f %.9f %.9f %.3f %d %d\r\n",
			(int)ep[0], (int)ep[1], (int)ep[2], (int)ep[3], (int)ep[4], ep[5],
			gga_data->lon, gga_data->lat, gga_data->alt,
			gga_data->stat, 1);
#endif
}