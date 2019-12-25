//#include "stdafx.h"
#include <iostream>
#include "ATProcess.h"
#include "AttTrack_lib.h"
#include "test.h"
using namespace std;
int main1()//gga语句解析
{
	char lcfile[1024] = { 0 };
	char cOutFilePath[1024] = { 0 };
	sprintf(cOutFilePath, "%s/OUT_gga/", TEST_RAW_FILE_PATH);//结果保存路径
	//sprintf(lcfile, "%s%s", TEST_RAW_FILE_PATH, TEST_RAW_FILE_NAME);//原始数据文件
	//FILE *fraw = fopen(lcfile, "rt");
	//if (fraw == NULL)
	//{
	//	printf("opendatafile err!\n");
	//}
	fopendhf(cOutFilePath);
	//char line[1024];
	ATdata_t atdata = { 0 };
	result_t result = { 0 };
	gyo_angle_test_t ym_test;
	//config_t ym_cfg;
	vector<double> gga_alt;
	struct gpgga_data gga_data;
	FILE *fp;
	//fgets(line, MAXLEN, fraw);
	//注意轴系和单位的调整 以及数据顺序
	//int temp = decode_buf2struct_IS203(line, &atdata);
	//int temp = decode_buf2struct_Grader(line, &atdata);
	fp = fopen("G:/平地机项目/12.16数据/动态/BD982.log", "rt");//1 3 2
	while (!feof(fp))
	{
		double mean_alt = 0;
		GPGGA_decode(&gga_data, fp);
		if (gga_alt.size() < 5) //10秒
		{
			gga_alt.push_back(gga_data.alt);
		}
		else
		{
			gga_alt.push_back(gga_data.alt);
			gga_alt.erase(gga_alt.begin());
			int num = (int)gga_alt.size();
			for (int i = 1; i < num - 1; i++)
			{
				mean_alt += gga_alt[i];
			}
			mean_alt /= (num - 2);
		}
#if DEBUG_SAVE
		if (mean_alt)
		{
			outdhf(11, "%f,%.9f,%.9f,%f,%d\r\n",
				gga_data.time, gga_data.lat, gga_data.lon, gga_data.alt,gga_data.stat);
		}
#endif
	}
	return 0;
}
int main2()//加速度解析
{
	char lcfile[1024] = { 0 };
	char cOutFilePath[1024] = { 0 };
	sprintf(cOutFilePath, "%s/", TEST_RAW_FILE_PATH);//结果保存路径
	sprintf(lcfile, "%s%s", TEST_RAW_FILE_PATH, TEST_RAW_FILE_NAME);//原始数据文件
	FILE *fraw = fopen(lcfile, "rt");
	if (fraw == NULL)
	{
		printf("opendatafile err!\n");
	}
	fopendhf(cOutFilePath);

	char line[1024];
	ATdata_t atdata = { 0 };
	result_t result = { 0 };
	gyo_angle_test_t ym_test;
	config_t ym_cfg;
	ym_cfg.acc_bias[0] = 0.002;
	ym_cfg.acc_bias[1] = 0.002;
	ym_cfg.acc_bias[2] = 0.002;
	//init_config();
	//UI_set_config(&ym_cfg);//配置文件重启测试
	while (!feof(fraw))
	{
		fgets(line, MAXLEN, fraw);
		//注意轴系和单位的调整 以及数据顺序
		//int temp = decode_buf2struct_IS203(line, &atdata);
		int temp = decode_buf2struct_Grader(line, &atdata);
		if (temp == 1)
		{
			continue;
		}
#if DEBUG_SAVE
		if (atdata.bbaldeimu_update)
		{
			outdhf(11, "%f,%f,%f,%f,%f,%f\r\n",
				atdata.imu_balde.accx, atdata.imu_balde.accy, atdata.imu_balde.accz,
				atdata.imu_body.accx, atdata.imu_body.accy, atdata.imu_body.accz);
		}
#endif
	}
	return 0;
}
int main()
{
	char lcfile[1024] = { 0 };
	char cOutFilePath[1024] = { 0 };
	sprintf(cOutFilePath, "%s/OUT/", TEST_RAW_FILE_PATH);//结果保存路径
	sprintf(lcfile, "%s%s", TEST_RAW_FILE_PATH, TEST_RAW_FILE_NAME);//原始数据文件
	FILE *fraw = fopen(lcfile, "rt");
	if (fraw == NULL)
	{
		printf("opendatafile err!\n");
	}
	fopendhf(cOutFilePath);

	char line[1024];
	ATdata_t atdata = { 0 };
	result_t result = { 0 };
	gyo_angle_test_t ym_test;
	config_t ym_cfg;
	ym_cfg.acc_bias[0] = 0.002;
	ym_cfg.acc_bias[1] = 0.002;
	ym_cfg.acc_bias[2] = 0.002;
	//init_config();
	//UI_set_config(&ym_cfg);//配置文件重启测试
	while (!feof(fraw))
	{
		fgets(line, MAXLEN, fraw);
		//注意轴系和单位的调整 以及数据顺序
		//int temp = decode_buf2struct_IS203(line, &atdata);
		int temp = decode_buf2struct_Grader(line, &atdata);
		if (temp == 1)
		{
			continue;
		}
		if (atdata.gnss.gnsstimetarget > 73016 && atdata.gnss.gnsstimetarget < 73018)//仿真gnss信号中断
		{
			//atdata.bgnss_update = false;
			//atdata.gnss.gnsstimetarget = 0;
		}
/*--------------------IMU时间戳---------------------------*/
		double ep[6] = {0};
		gtime_t gt_chc = { 0 };
		static int gnss_week = 0;
		static double imu_sec = 0.0;
		static double week_sec = 0.0;
		if (atdata.gnss.gnsstimetarget&&atdata.bgnss_update)
		{
			double gnss_time = atdata.gnss.gnsstimetarget;
			gt_chc.time = (int)gnss_time;
			gt_chc.sec = gnss_time - (int)gnss_time;
			/*gnss秒转换为UTC时间*/
			/*gnss秒转换为周、周内秒*/
			week_sec = time2gpst(gt_chc, &gnss_week);
			//printf("week_sec:%f\r\n", week_sec);
			time2epoch(gpst2utc(gt_chc), ep);
		}
		if (atdata.bgnss_update)
		{
			imu_sec = week_sec;
		}
		else if (atdata.bbaldeimu_update)
		{
			if (week_sec)
			{
				week_sec += atdata.imu_balde.imutimetarget;//imu时间戳
				imu_sec = week_sec;
			}
		}
		atdata.imu_time = imu_sec;
		//printf("imu_sec:%f,%f,%f\r\n", week_sec,imu_sec, atdata.imu_balde.imutimetarget);
/*-----------------------算法处理接口-----------------------*/
		int test_ret = Gyro_Angle_Process(&atdata, &ym_test);
		int ret = Grader_AttTrack_Process(&atdata, &result);
#if DEBUG_SAVE
		if (atdata.bgnss_update)
		{
			outdhf(12, "%d/%02d/%02d %02d:%02d:%02f %.9f %.9f %.3f %d %d\r\n",
				(int)ep[0], (int)ep[1], (int)ep[2], (int)ep[3], (int)ep[4],ep[5],
				atdata.gnss.lon,atdata.gnss.lat,atdata.gnss.alt,
				atdata.gnss.state_pos,1);
		}
		if(atdata.bbaldeimu_update&&ret&&gnss_week&&result.roll_balde)
		{
			outdhf(11, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%.9f\r\n",
				ym_test.gyo_angle[0] * R2D, ym_test.gyo_angle[1] * R2D, ym_test.gyo_angle[2] * R2D,
				ym_test.acc_angle[0] * R2D, ym_test.acc_angle[1] * R2D, ym_test.acc_angle[2] * R2D,
				result.roll_balde*R2D, result.pitch_balde*R2D, result.heading_balde*R2D,
				result.CPL_blh[2],result.CPR_blh[2],result.gnss_speed_ver,
				atdata.gnss.alt,atdata.gnss.gnssroll*R2D,
				result.CPL_blh[0]*R2D, result.CPL_blh[1] * R2D,
				result.CPR_blh[0] * R2D, result.CPR_blh[1] * R2D,
				imu_sec);
		}
#endif
		ATdata_reset(&atdata);
	}
	return 0;
}