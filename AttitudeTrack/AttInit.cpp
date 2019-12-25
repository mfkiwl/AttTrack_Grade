//#include "stdafx.h"
#include "AttInit.h"


AttInit::AttInit()
{
	bias_gx1 = bias_gy1 = bias_gz1 =0;
	std_gx1 = std_gy1 = std_gz1 =0;
	std_ax1 = std_ay1 = std_az1 =0;

	bias_gx2 = bias_gy2 = bias_gz2 = 0;
	std_gx2 = std_gy2 = std_gz2 = 0;
	std_ax2 = std_ay2 = std_az2 = 0;

	mean_gyaw = 0;
	std_gyaw = 0;
	mean_groll = 0;
	std_groll = 0;

	roll1 = pitch1 = roll2 = pitch2 = 0;
	cross_slope1 = tilt_slope1 = tilt_slope2 = 0;  

	bfinshinit = 0;
	binit_gyaw = 0;
	binit_groll = 0;
	binit_att_balde = 0;
	binit_att_mainfall = 0;
	binit_gbias_balde = 0;
	binit_gbias_mainfall = 0;
}


AttInit::~AttInit()
{
}

//三轴加表和三轴陀螺
int AttInit::process(AttTrackData iatd, AttTrackCfg_t* confg)
{
/*------------双天线姿态初始化-------------*/
	if (iatd.bgnssupdate) //同时要判断双天线解算状态
	{
		if (vgpsyaw.size() < Heading_wlen) //10秒
		{
			vgpsyaw.push_back(iatd.yaw_2ant);//双天线航向角（双天线安装方式？）
			vgpsroll.push_back(iatd.roll_2ant);//双天线俯仰角
		}
		else
		{
			vgpsyaw.push_back(iatd.yaw_2ant);
			vgpsyaw.erase(vgpsyaw.begin());
			vgpsroll.push_back(iatd.roll_2ant);
			vgpsroll.erase(vgpsroll.begin());
		}

		if (!binit_gyaw && vgpsyaw.size() > Heading_wlen-1)
		{
			int num = (int)vgpsyaw.size();
			sort(vgpsyaw.begin(), vgpsyaw.end());//排序
			for (int i = 1; i < num - 1; i++)
			{
				mean_gyaw += vgpsyaw[i];
			}
			mean_gyaw /= (num - 2);
#if WIN32_DEBUG
			printf("double_ant_yaw: %f\n", mean_gyaw*R2D);
#endif
			for (int i = 1; i < num - 1; i++)
			{
				std_gyaw += (vgpsyaw[i] - mean_gyaw)*(vgpsyaw[i] - mean_gyaw);
			}
			std_gyaw = sqrt(std_gyaw / (num - 3));
#if WIN32_DEBUG
			printf("double_ant_yaw_std: %f\n", std_gyaw*R2D);
#endif
			if (std_gyaw < confg->gnssyawstd_thr*D2R)//双天线航向std阈值判断
			{
				binit_gyaw = 1;
			}
			else
			{
#if WIN32_DEBUG
				printf("双天线静态航向std超限！\n"); //需要考虑角度边界的跳变
#endif
			}
		}
		if (!binit_groll && vgpsroll.size() > Heading_wlen-1)
		{
			int num = (int)vgpsroll.size();
			sort(vgpsroll.begin(), vgpsroll.end());
			for (int i = 1; i < num - 1; i++)
			{
				mean_groll += vgpsroll[i];
			}
			mean_groll /= (num - 2);
#if WIN32_DEBUG
			//printf("double_ant_roll: %f\n", mean_groll*R2D);
#endif
			for (int i = 1; i < num - 1; i++)
			{
				std_groll += (vgpsroll[i] - mean_groll)*(vgpsroll[i] - mean_groll);
			}
			std_groll = sqrt(std_groll / (num - 3));
#if WIN32_DEBUG
			//printf("double_ant_roll_std: %f\n", std_groll*R2D);
#endif
			if (std_groll < confg->gnssrollstd_thr*D2R)//双天线俯仰角std阈值判断
			{
				binit_groll = 1;
			}
			else
			{
#if WIN32_DEBUG
				//printf("双天线静态横滚角std超限！\n"); //需要考虑角度边界的跳变
#endif
			}
		}
	}
/*------------IMU1水平姿态和陀螺零偏初始化-------------*/
	if (iatd.bbaldeimu_update)
	{
		if (vax1.size() < IMU_wlen) //100HZ 10秒
		{
			vax1.push_back(iatd.acc_balde[0]);
			vay1.push_back(iatd.acc_balde[1]);
			vaz1.push_back(iatd.acc_balde[2]);
			vgx1.push_back(iatd.gyo_balde[0]);
			vgy1.push_back(iatd.gyo_balde[1]);
			vgz1.push_back(iatd.gyo_balde[2]);
		}
		else
		{
			vax1.push_back(iatd.acc_balde[0]);
			vay1.push_back(iatd.acc_balde[1]);
			vaz1.push_back(iatd.acc_balde[2]);
			vgx1.push_back(iatd.gyo_balde[0]);
			vgy1.push_back(iatd.gyo_balde[1]);
			vgz1.push_back(iatd.gyo_balde[2]);
			vax1.erase(vax1.begin());
			vay1.erase(vay1.begin());
			vaz1.erase(vaz1.begin());
			vgx1.erase(vgx1.begin());
			vgy1.erase(vgy1.begin());
			vgz1.erase(vgz1.begin());
		}
		if ((!binit_att_balde || !binit_gbias_balde) && vax1.size() > IMU_wlen-1)
		{
			sort(vax1.begin(), vax1.end());
			sort(vay1.begin(), vay1.end());
			sort(vaz1.begin(), vaz1.end());
			sort(vgx1.begin(), vgx1.end());
			sort(vgy1.begin(), vgy1.end());
			sort(vgz1.begin(), vgz1.end());
			double meanax = 0, meanay = 0, meanaz = 0, meangx = 0, meangy = 0, meangz = 0;
			int num = (int)vax1.size();
			for (int i = 1; i < num - 1; i++)
			{
				meanax += vax1[i];
				meanay += vay1[i];
				meanaz += vaz1[i];
				meangx += vgx1[i];
				meangy += vgy1[i];
				meangz += vgz1[i];
			}
			meanax /= (num - 2);
			meanay /= (num - 2);
			meanaz /= (num - 2);
			meangx /= (num - 2);
			meangy /= (num - 2);
			meangz /= (num - 2);
#if WIN32_DEBUG
			printf("IMU_balde_gyobias:%f, %f, %f\n", meangx*R2D, meangy*R2D, meangz*R2D);
#endif
			if (meangx > 1.0 * D2R || meangy > 1.0 * D2R || meangz > 1.0 * D2R)
			{
#if WIN32_DEBUG
				printf("陀螺1零偏大于1deg！！\n");
#endif
			}
			for (int i = 1; i < num - 1; i++)
			{
				std_ax1 += (vax1[i] - meanax)*(vax1[i] - meanax);
				std_ay1 += (vay1[i] - meanay)*(vay1[i] - meanay);
				std_az1 += (vaz1[i] - meanaz)*(vaz1[i] - meanaz);
				std_gx1 += (vgx1[i] - meangx)*(vgx1[i] - meangx);
				std_gy1 += (vgy1[i] - meangy)*(vgy1[i] - meangy);
				std_gz1 += (vgz1[i] - meangz)*(vgz1[i] - meangz);
			}
			std_ax1 = sqrt(std_ax1 / (num - 3));
			std_ay1 = sqrt(std_ay1 / (num - 3));
			std_az1 = sqrt(std_az1 / (num - 3));
			std_gx1 = sqrt(std_gx1 / (num - 3));
			std_gy1 = sqrt(std_gy1 / (num - 3));
			std_gz1 = sqrt(std_gz1 / (num - 3));
#if WIN32_DEBUG
			printf("IMU_balde_acc_static_std: %f, %f, %f\n", std_ax1, std_ay1, std_az1);
			printf("IMU_balde_gyo_static_std: %f, %f, %f\n", std_gx1, std_gy1, std_gz1);
#endif
			if (std_gx1 > confg->gstdxthr || std_gy1 > confg->gstdythr || std_gz1 > confg->gstdzthr) //0.001
			{
#if WIN32_DEBUG
				printf("陀螺1静态std超限！\n");
#endif
			}
			else
			{
				bias_gx1 = meangx;
				bias_gy1 = meangy;
				bias_gz1 = meangz;
				binit_gbias_balde = 1;
			}
			if (std_ax1 > confg->astdxthr || std_ay1 > confg->astdythr || std_az1 > confg->astdzthr) // 0.05
			{
#if WIN32_DEBUG
				printf("加计1静态std超限！\n");
#endif
			}
			else
			{
				//计算横滚和俯仰角
				roll1 = atan2(-meanay, -meanaz);//姿态定义？
				pitch1 = atan2(meanax , sqrt(meanay*meanay + meanaz*meanaz));
#if WIN32_DEBUG
				printf("Balde_static_att: roll %f, pitch %f, yaw %f\n", roll1 * R2D, pitch1 * R2D, mean_gyaw * R2D);
#endif
				cross_slope1 = atan2(sin(roll1)*cos(pitch1), sqrt(sin(roll1)*sin(roll1)*sin(pitch1)*sin(pitch1) + cos(roll1)*cos(roll1)));
				tilt_slope1 = pitch1;
#if WIN32_DEBUG
				printf("Balde_static_tiltangle:  %f,  %f\n", cross_slope1 * R2D, tilt_slope1 * R2D);
#endif
				binit_att_balde = 1;
			}
		}
	}
/*------------IMU2水平姿态和陀螺零偏初始化-------------*/
	if (iatd.bmainfallimu_update)
	{
		if (vax2.size() < IMU_wlen) //100HZ 10秒
		{
			vax2.push_back(iatd.acc_mainfall[0]);
			vay2.push_back(iatd.acc_mainfall[1]);
			vaz2.push_back(iatd.acc_mainfall[2]);
			vgx2.push_back(iatd.gyo_mainfall[0]);
			vgy2.push_back(iatd.gyo_mainfall[1]);
			vgz2.push_back(iatd.gyo_mainfall[2]);
		}
		else
		{
			vax2.push_back(iatd.acc_mainfall[0]);
			vay2.push_back(iatd.acc_mainfall[1]);
			vaz2.push_back(iatd.acc_mainfall[2]);
			vgx2.push_back(iatd.gyo_mainfall[0]);
			vgy2.push_back(iatd.gyo_mainfall[1]);
			vgz2.push_back(iatd.gyo_mainfall[2]);
			vax2.erase(vax2.begin());
			vay2.erase(vay2.begin());
			vaz2.erase(vaz2.begin());
			vgx2.erase(vgx2.begin());
			vgy2.erase(vgy2.begin());
			vgz2.erase(vgz2.begin());
		}
		if ((!binit_att_mainfall || !binit_gbias_mainfall) && vax2.size() > IMU_wlen-1)
		{
			sort(vax2.begin(), vax2.end());
			sort(vay2.begin(), vay2.end());
			sort(vaz2.begin(), vaz2.end());
			sort(vgx2.begin(), vgx2.end());
			sort(vgy2.begin(), vgy2.end());
			sort(vgz2.begin(), vgz2.end());
			double meanax = 0, meanay = 0, meanaz = 0, meangx = 0, meangy = 0, meangz = 0;
			int num = (int)vax1.size();
			for (int i = 1; i < num - 1; i++)
			{
				meanax += vax2[i];
				meanay += vay2[i];
				meanaz += vaz2[i];
				meangx += vgx2[i];
				meangy += vgy2[i];
				meangz += vgz2[i];
			}
			meanax /= (num - 2);
			meanay /= (num - 2);
			meanaz /= (num - 2);
			meangx /= (num - 2);
			meangy /= (num - 2);
			meangz /= (num - 2);
#if WIN32_DEBUG
			printf("IMU_mainfall_gyobias:%f, %f, %f\n", meangx*R2D, meangy*R2D, meangz*R2D);
#endif
			if (meangx > 1.0 * D2R || meangy > 1.0 * D2R || meangz > 1.0 * D2R)
			{
#if WIN32_DEBUG
				printf("陀螺1零偏大于1deg！！\n");
#endif
			}
			for (int i = 1; i < num - 1; i++)
			{
				std_ax2 += (vax2[i] - meanax)*(vax2[i] - meanax);
				std_ay2 += (vay2[i] - meanay)*(vay2[i] - meanay);
				std_az2 += (vaz2[i] - meanaz)*(vaz2[i] - meanaz);
				std_gx2 += (vgx2[i] - meangx)*(vgx2[i] - meangx);
				std_gy2 += (vgy2[i] - meangy)*(vgy2[i] - meangy);
				std_gz2 += (vgz2[i] - meangz)*(vgz2[i] - meangz);
			}
			std_ax2 = sqrt(std_ax2 / (num - 3));
			std_ay2 = sqrt(std_ay2 / (num - 3));
			std_az2 = sqrt(std_az2 / (num - 3));
			std_gx2 = sqrt(std_gx2 / (num - 3));
			std_gy2 = sqrt(std_gy2 / (num - 3));
			std_gz2 = sqrt(std_gz2 / (num - 3));
#if WIN32_DEBUG
			printf("IMU_mainfall_acc_static_std: %f, %f, %f\n", std_ax2, std_ay2, std_az2);
			printf("IMU_mainfall_gyo_static_std: %f, %f, %f\n", std_gx2, std_gy2, std_gz2);
#endif
			if (std_gx2 > confg->gstdxthr || std_gy2 > confg->gstdythr || std_gz2 > confg->gstdzthr) //0.001
			{
#if WIN32_DEBUG
				printf("陀螺2静态std超限！\n");
#endif
			}
			else
			{
				bias_gx2 = meangx;
				bias_gy2 = meangy;
				bias_gz2 = meangz;
				binit_gbias_mainfall = 1;
			}
			if (std_ax2 > confg->astdxthr || std_ay2 > confg->astdythr || std_az2 > confg->astdzthr) // 0.05
			{
#if WIN32_DEBUG
				printf("加计2静态std超限！\n");
#endif
			}
			else
			{
				//计算横滚和俯仰角
				roll2 = atan2(-meanay , -meanaz);
				pitch2 = atan2(meanax , sqrt(meanay*meanay + meanaz*meanaz));
#if WIN32_DEBUG
				printf("IMU_mainfall_static_pitch: pitch %f\n", pitch2 * R2D);
#endif
				tilt_slope2 = pitch2;
#if WIN32_DEBUG
				printf("IMU_mainfall_static_tiltangle:  %f\n", tilt_slope2 * R2D);
#endif
				binit_att_mainfall = 1;
			}
		}
	}
	bfinshinit = (binit_gyaw&&binit_groll&&binit_att_balde&&binit_gbias_balde&&binit_att_mainfall&&binit_gbias_mainfall);
	//无车架传感器
	//bfinshinit = (binit_gyaw&&binit_groll&&binit_att_balde&&binit_gbias_balde);
	//bfinshinit = (binit_att_balde&&binit_gbias_balde);
	return bfinshinit;
}
