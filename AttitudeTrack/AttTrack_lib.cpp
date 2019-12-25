//#include "stdafx.h"
#include <iostream>
#include "ATProcess.h"
#include "AttTrack_lib.h"
#include "Calibrate.h"
#include "DataStruct.h"
using namespace std;
int read_imu_num = 0;
int cfg_reset = 0;
void setresult(result* res, AttTrackData* iatd, ATProcess* at, ElevationEstimate* elv, double leverR[], double leverL[])
{
	res->gnsstime = iatd->gnsstime;
	/*铲刀结果*/
	res->roll_balde = at->roll_balde;
	res->pitch_balde = at->pitch_balde;
	res->heading_balde = at->heading_balde;
	res->cross_slope_balde = at->crossslope_balde;    //铲刀横坡角
	res->tilt_slope_balde = at->tiltslope_balde;      //铲刀纵坡角
	res->gyoy_balde = at->gyo_bias_balde[1];		  //铲刀去除零偏后横滚角速度
	res->gnss_alt = elv->kf_alt;					  //gnss主天线高程
	res->gnss_speed_ver = elv->kf_vu;			      //gnss主天线天向速度
	double cptemp[3] = { 0 };
	double BLH_main_ant[3] = { iatd->BLH_ant[0], iatd->BLH_ant[1], iatd->BLH_ant[2] };
	if (elv->kf_alt)
	{
		BLH_main_ant[2] = elv->kf_alt;//kf融合高程
	}
	double att[3] = { at->roll_balde,at->pitch_balde ,at->heading_balde };//NED
	//double att[3] = { at->roll_balde,0 ,0};//NED
	double wib[3] = { iatd->gyo_balde[0],iatd->gyo_balde[1],iatd->gyo_balde[2] };
	titl_compen(att, BLH_main_ant, leverR, cptemp);//右控制点高程
	for (int i = 0; i < 3; i++)
	{
		res->CPR_blh[i] = cptemp[i];
		//double ecef_tmp[3] = { 0 };
		//pos2ecef(cptemp, ecef_tmp);
		//ecef2enu(cptemp, ecef_tmp, res->CPR_blh);
		//res->CPR_blh[2] = cptemp[2];
	}
	titl_compen(att, BLH_main_ant, leverL, cptemp);//左控制点高程
	for (int i = 0; i < 3; i++)
	{
		res->CPL_blh[i] = cptemp[i];
		//double ecef_tmp[3] = { 0 };
		//pos2ecef(cptemp, ecef_tmp);
		//ecef2enu(cptemp, ecef_tmp, res->CPL_blh);
		//res->CPL_blh[2] = cptemp[2];
	}
	double vtemp_R[3] = { 0 };
	double vtemp_L[3] = { 0 };
	double vgnss[3] = { 0,0,elv->kf_vu };
	double lever_Middle[3] = { leverR[0],leverR[1]/2,leverR[2] };
	Lever_ver(BLH_main_ant, att, wib, vgnss, vtemp_R, lever_Middle);//铲刀中间控制点垂直速度
	res->ver_balde = vtemp_R[2];//铲刀垂直速度（中间值）
	/*车体结果*/
	res->tilt_slope_body = at->tiltslop_mainfall;
	/*解算状态*/
	if (at->battinit)
	{
		res->state = 1;
	}
	else
	{
		res->state = 0;
	}
	/*安装误差标定返回标志*/
	if (at->rotation_zero_ok)
	{
		res->rotation = at->rotation_zero;
	}
	res->balde_install_result = at->blade_install_result;		//铲刀传感器标定结果
	res->body_install_result = at->body_install_result;			//车体传感器标定结果
	res->rotation_install_result = at->rotation_install_result; //旋转编码器标定结果
}

void ATdata_init(ATdata_t* atd)
{
	atd->imu_balde.imutimetarget = 0;
	atd->imu_balde.accx = 0;
	atd->imu_balde.accy = 0;
	atd->imu_balde.accz = 0;
	atd->imu_balde.gyox = 0;
	atd->imu_balde.gyoy = 0;
	atd->imu_balde.gyoz = 0;
	atd->bbaldeimu_update = 0;

	atd->imu_body.imutimetarget = 0;
	atd->imu_body.accx = 0;
	atd->imu_body.accy = 0;
	atd->imu_body.accz = 0;
	atd->imu_body.gyox = 0;
	atd->imu_body.gyoy = 0;
	atd->imu_body.gyoz = 0;
	atd->bbodyimu_update = 0;

	atd->gnss.gnsstimetarget = 0;
	atd->gnss.lat = 0;
	atd->gnss.lon = 0;
	atd->gnss.alt = 0;
	atd->gnss.state_pos = 0;
	atd->gnss.speed = 0;
	atd->gnss.speed_ver = 0;
	atd->gnss.gnssyaw = 0;
	atd->gnss.noise_gyaw = 0;
	atd->gnss.gnssroll = 0;
	atd->gnss.noise_groll = 0;
	atd->gnss.state_yaw = 0;
	atd->bgnss_update = 0;
}

void ATdata_reset(ATdata_t* atd)
{
	atd->bbaldeimu_update = 0;
	atd->bbodyimu_update = 0;
	atd->bgnss_update = 0;
}

int Grader_AttTrack_Process(ATdata_t* atd, result* res)
{
	static AttTrackData iatd;
	static ATProcess at(0.001); //注意采样率100hz
	static ElevationEstimate elev;
	static double leverright[3] = { 0 };
	static double leverleft[3] = { 0 };
	static double balde_gyro_angle[3] = { 0 };
	if (!at.bprocessinit)
	{
		if (!init_config())//检测路径下是否存在配置文件
		{
			generate_default_config_Grader();//不存在即生成配置文件
		}
		get_config(&at.cfg);//获取配置参数
		leverright[0] = at.cfg.leverrightx;
		leverright[1] = at.cfg.leverrighty;
		leverright[2] = at.cfg.leverrightz;
		leverleft[0] = at.cfg.leverleftx;
		leverleft[1] = at.cfg.leverlefty;
		leverleft[2] = at.cfg.leverleftz;
		iatd.Init();
		at.cal.Cal_Install_Error_init();
		//if (at.cfg.abiasx&&at.cfg.abiasy&&at.cfg.install_pitch&&at.cfg.install_roll)
		if (at.cfg.balde_install_pitch&&at.cfg.balde_install_roll)
		{
			at.Acc_Bias_ok = true;
		}
		if (at.cfg.balde_install_yaw)
		{
			at.heading_installerr_ok=true;
		}
		at.bprocessinit = true;
	}
	if (cfg_reset)//上位机重置配置文件，需要重新读取并生效
	{
		get_config(&at.cfg);//获取最新配置参数
		cfg_reset = 0;
	}
	decode(atd, &iatd, &at.cfg); //数据解码+IMU轴系调整
	if (atd->bgnss_update)
	{
		Gnss_time_decode(&iatd);//gnss时间戳解码
		Data_Check(&iatd);//gnss中断、异常判断
		//printf("gnss_week:%d   gnss_week_sec:%f\r\n", iatd.week, iatd.weektime_sec);
	}
	int Rotation_Zero_flag = at.Rotation_Zero_Err(iatd);//旋转编码器零位
	int install_err_flag= at.Two_Position_Install_Err(iatd);//双位置法安装误差标定
#if 1
	if (!at.battinit)
	{
		int finshattinit = 0;
		//要改 加入双天线航向解状态和GPS位置初始化 修改阈值 返回值反映初始化失败原因
		finshattinit = at.attinitial.process(iatd, &at.cfg);
		if (finshattinit==1)
		{
			//铲刀传感器相关
			at.roll_balde = at.attinitial.roll1;
			at.pitch_balde = at.attinitial.pitch1;
			at.heading_balde = at.attinitial.mean_gyaw;
			at.integ_att_blade[0]=at.attinitial.roll1;
			at.integ_att_blade[1] = at.attinitial.pitch1;
			at.integ_att_blade[2] = at.attinitial.mean_gyaw;
			balde_gyro_angle[0]= at.attinitial.roll1;//纯陀螺积分角度，测试对比用
			balde_gyro_angle[1] = at.attinitial.pitch1;
			balde_gyro_angle[2] = at.attinitial.mean_gyaw;
			a2mat_ned(at.attinitial.roll1, at.attinitial.pitch1, at.attinitial.mean_gyaw,at.Cb2n_balde);
			m2qua_ned(at.Cb2n_balde, at.qua_balde);
			at.crossslope_balde = at.attinitial.cross_slope1;//水平偏转角
			at.tiltslope_balde = at.attinitial.tilt_slope1;//垂直偏转角
			at.gyobias_balde[0] = at.attinitial.bias_gx1;
			at.gyobias_balde[1] = at.attinitial.bias_gy1;
			at.gyobias_balde[2] = at.attinitial.bias_gz1;
			Mequalm(iatd.acc_balde, 3, 1, at.accpre_balde);
			Mequalm(iatd.gyo_balde, 3, 1, at.gyopre_balde);
			at.tpre_baldeimu = iatd.imutime_balde - at.dt_baldeimu;
			//车架传感器相关
			at.tiltslop_mainfall = at.attinitial.tilt_slope2;
			at.gyobias_mainfall[0] = at.attinitial.bias_gx2;
			at.gyobias_mainfall[1] = at.attinitial.bias_gy2;
			at.gyobias_mainfall[2] = at.attinitial.bias_gz2;
			Mequalm(iatd.acc_mainfall, 3, 1, at.accpre_mainfall);
			Mequalm(iatd.gyo_mainfall, 3, 1, at.gyopre_mainfall);
			at.tpre_mainfallimu = iatd.imutime_mainfall - at.dt_mainfallimu;
			iatd.integ_pitch= at.attinitial.tilt_slope2;

			elev.initstate(iatd.BLH_ant[2], iatd.speed_ver);//gnss高程融合

			at.battinit = true;
		}
	}
	//要改 根据IS203的实际数据确定阈值  +  根据GPS识别静态?? 自适应计算阈值
	int bstatic = detectstatic_std(iatd.acc_balde, iatd.gyo_balde);
	bstatic = 0;
	if (at.battinit)
	{
		for (int i = 0; i < 3; i++)
		{
			balde_gyro_angle[i] += iatd.gyo_balde[i] * at.tpre_baldeimu;//陀螺积分角度
		}
		at.process_MotorGrader(iatd, bstatic);	
#if DEBUG_SAVE
		outdhf(0, "%f,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%d,%d,%f,%f,%f,%f,%f\n",
			iatd.imutime_balde, iatd.acc_balde[0], iatd.acc_balde[1], iatd.acc_balde[2], iatd.gyo_balde[0], iatd.gyo_balde[1], iatd.gyo_balde[2], iatd.bbaldeimu_update,
			iatd.imutime_mainfall, iatd.acc_mainfall[0], iatd.acc_mainfall[1], iatd.acc_mainfall[2], iatd.gyo_mainfall[0], iatd.gyo_mainfall[1], iatd.gyo_mainfall[2], iatd.bmainfallimu_update,
			iatd.gnsstime, iatd.BLH_ant[0], iatd.BLH_ant[1], iatd.BLH_ant[2], iatd.speed, iatd.speed_ver, iatd.gstate_pos, iatd.yaw_2ant, iatd.roll_2ant, iatd.gyaw_noise, iatd.groll_noise, iatd.gstate_2ant, iatd.bgnssupdate,
			iatd.rotation_balde, balde_gyro_angle[0] * R2D, balde_gyro_angle[1] * R2D, balde_gyro_angle[2] * R2D, iatd.Mean_Gnss_Alt);
#endif
		//要改 根据推土机的实际情况
		double att[3] = { at.roll_balde ,at.pitch_balde ,at.heading_balde };
		if (iatd.bbaldeimu_update||iatd.bgnssupdate) //IMU更新后检查GNSS是否更新，若更新则更新公共结构体
		{
			//elev.Alt_Complement_Filter(iatd.acc_balde, at.dt_baldeimu, at.roll_balde, at.pitch_balde, iatd.BLH_ant[2], iatd.speed_ver, iatd.gstate_pos, iatd.bgnssupdate, bstatic);
			elev.update_kf(iatd,att,bstatic);
		}
	}
#endif
	//结果结构体赋值
	if (iatd.bbaldeimu_update)
	{
		setresult(res, &iatd, &at,&elev, leverright, leverleft);
		read_imu_num++;
		if (read_imu_num % 100 == 0)
		{
			//printf("%d-%d-%d:%02d:%02d:%06.3f   CPL_alt:%f   CPR_alt:%f   speed_ver:%f\r\n", (int)iatd.ep[0], (int)iatd.ep[1], (int)iatd.ep[2],
			//	(int)iatd.ep[3], (int)iatd.ep[4], iatd.ep[5],
			//	res->CPL_alt, res->CPR_alt, res->gnss_speed_ver);
		}
	}
	iatd.Reset();
	return 1;
}
int UI_set_config(config_t* cfg)
{
	//重新写入配置文件
	init_config();
	config_set_gyo_stdx_thr(cfg->gyostd_thr); 
	config_set_gyo_stdy_thr(cfg->gyostd_thr);
	config_set_gyo_stdz_thr(cfg->gyostd_thr);
	config_set_acc_stdx_thr(cfg->accstd_thr);
	config_set_acc_stdy_thr(cfg->accstd_thr);
	config_set_acc_stdz_thr(cfg->accstd_thr);

	config_set_right_leverx(cfg->leverx_R);
	config_set_right_levery(cfg->levery_R);
	config_set_right_leverz(cfg->leverz_R);
	config_set_left_leverx(cfg->leverx_L);
	config_set_left_levery(cfg->levery_L);
	config_set_left_leverz(cfg->leverz_L);

	config_set_acc_staticbiasx(cfg->acc_bias[0]);
	config_set_acc_staticbiasy(cfg->acc_bias[1]);
	config_set_acc_staticbiasz(cfg->acc_bias[2]);

	config_set_gyo_stdx_thr(0.3);  //具体数值,判断静态
	config_set_gyo_stdy_thr(0.3);
	config_set_gyo_stdz_thr(0.3);
	config_set_acc_stdx_thr(0.1);
	config_set_acc_stdy_thr(0.1);
	config_set_acc_stdz_thr(0.1);
	cfg_reset = 1;//配置文件重置标志
	return 1;
}
int Grader_Slope_Process(double Design_slope,ATdata_t* atd, result* res)
{
	if (res->state)//姿态跟踪进入解算
	{
		double T = atd->rotation;
		double R = res->tilt_slope_body;
		double CS = Design_slope;
		res->expect_slope_angle = sin(T)*tan(R) + cos(T)*tan(CS);
		return 1;
	}
	else
	{
		res->expect_slope_angle = 0.0;
		return 0;
	}
}
