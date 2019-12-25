//#include "stdafx.h"
#include "ATProcess.h"


ATProcess::ATProcess()
{
	battinit = false;
	bkfinit = false;
	bprocessinit = false;
	heading_installerr_ok = false;
	rotation_zero_ok = false;
	Acc_Bias_ok = false;
	position1_ok = false;
	position2_ok = false;
	position3_ok = false;
	dt_baldeimu = 0.01;
	dt_mainfallimu = 0.01;
	tpre_baldeimu = 0;
	tpre_mainfallimu = 0;
	roll_balde = 0;            
	pitch_balde = 0;
	heading_balde = 0;
	crossslope_balde = 0;
	tiltslope_balde = 0;
	rotation_zero = 0;

	tiltslop_mainfall = 0;
	blade_install_result = 0;
	body_install_result = 0;
	rotation_install_result = 0;
	num_accnorm_balde = 0;
	num_gyonorm_balde = 0;
	num_accnorm_mainfall = 0;
	num_gyonorm_mainfall = 0;

	num_nogetbaldeimu = 0;
	num_nogetmainfallimu = 0;
	num_nogetgnss = 0;

	for (int i = 0; i < 3; i++)
	{
		gyo_bias_balde[i] = 0.0;
		gyo_bias_mainfall[i] = 0.0;
		gyobias_balde[i] = 0.0;
		gyopre_balde[i] = 0.0;
		accpre_balde[i] = 0.0;
		gyobias_mainfall[i] = 0.0;
		gyopre_mainfall[i] = 0.0;
		accpre_mainfall[i] = 0.0;
		smooth_acc_balde[i] = 0.0;
		smooth_acc_mainfall[i] = 0.0;
	}
	for (int i = 0; i < 4; i++)
	{
		qua_balde[i] = 0.0;
	}
	for (int i = 0; i < 9; i++)
	{
		Cb2n_balde[i] = 0;
	}
}
ATProcess::ATProcess(double dtime)
{
	battinit = false;
	bkfinit = false;
	bprocessinit = false;
	dt_baldeimu = dtime;
	dt_mainfallimu = dtime;
	tpre_baldeimu = 0;
	tpre_mainfallimu = 0;
	roll_balde = 0;
	pitch_balde = 0;
	heading_balde = 0;
	crossslope_balde = 0;
	tiltslope_balde = 0;

	tiltslop_mainfall = 0;

	num_accnorm_balde = 0;
	num_gyonorm_balde = 0;
	num_accnorm_mainfall = 0;
	num_gyonorm_mainfall = 0;

	num_nogetbaldeimu = 0;
	num_nogetmainfallimu = 0;
	num_nogetgnss = 0;

	for (int i = 0; i < 3; i++)
	{
		gyo_bias_balde[i] = 0.0;
		gyo_bias_mainfall[i] = 0.0;
		gyobias_balde[i] = 0.0;
		gyopre_balde[i] = 0.0;
		accpre_balde[i] = 0.0;
		gyobias_mainfall[i] = 0.0;
		gyopre_mainfall[i] = 0.0;
		accpre_mainfall[i] = 0.0;
		integ_att_blade[i] = 0.0;
	}
	for (int i = 0; i < 4; i++)
	{
		qua_balde[i] = 0.0;
	}
	for (int i = 0; i < 9; i++)
	{
		Cb2n_balde[i] = 0;
	}
}
ATProcess::~ATProcess()
{
}

int ATProcess::process_MotorGrader(AttTrackData& iatd, int bstatic)
{
	num_nogetbaldeimu++;
	num_nogetmainfallimu++;
	num_nogetgnss++;
	if (!bkfinit)
	{
		//滤波器的状态为角度误差和零偏误差，所以噪声阵Q为陀螺角速度噪声和零偏不稳定性
		//CRS03: 噪声：0.025deg/s 零偏稳定性：3.5deg/h
		//SCC2230-D08 功率谱密度：0.008deg/s/sqrt(hz)  零偏稳定性：2deg/hr
		//SCC2230-B15（z轴陀螺）：噪声：0.12deg/s  10hz
		//SCC2130-B15（x轴陀螺）：噪声：0.12deg/s  
		/*2019.12.6 徐州测试传感器 x\y为SCC2130-B15 z为SCC2230-B15*/
		double gnoise = 0.12*D2R, gstabilityx = 2 * D2R / 3600.0;//10hz
		double dxk0[2] = { 1.0*D2R,0.1*D2R };  //设定值影响收敛的速度
		double noise_baldegx[2] = { gnoise,gstabilityx };//x、y轴陀螺为SCC2230-D08
		double noise_baldegy[2] = { gnoise,gstabilityx };
		double noise_baldegz[2] = { 0.12*D2R,gstabilityx };//z轴陀螺为SCC2230-B15
		double noise_mainfallgy[2] = { gnoise,gstabilityx };
/*********俯仰角滤波器内存分配，参数初始********/
		ekf_pitch.kfmalloc(ATT_NUMX, ATT_NUMV);
		ekf_pitch.kfinit();
		//ekf_pitch.setXk(dxk0);
		ekf_pitch.setPxk(dxk0);
		ekf_pitch.setQk(noise_baldegy);
/*********横滚角滤波器内存分配，参数初始********/
		ekf_roll.kfmalloc(ATT_NUMX, ATT_NUMV);
		ekf_roll.kfinit();
		//ekf_roll.setXk(dxk0);
		ekf_roll.setPxk(dxk0);
		ekf_roll.setQk(noise_baldegx);
/*********航向角滤波器内存分配，参数初始********/
		ekf_yaw.kfmalloc(ATT_NUMX, ATT_NUMV);
		ekf_yaw.kfinit();
		//ekf_yaw.setXk(dxk0);
		ekf_yaw.setPxk(dxk0);
		ekf_yaw.setQk(noise_baldegz);
/*********车架俯仰角滤波器内存分配，参数初始********/
		ekf_pitch_mainfall.kfmalloc(ATT_NUMX, ATT_NUMV);
		ekf_pitch_mainfall.kfinit();
		//ekf_pitch_mainfall.setXk(dxk0);
		ekf_pitch_mainfall.setPxk(dxk0);
		ekf_pitch_mainfall.setQk(noise_mainfallgy);
		queue_ax_balde.clear();
		queue_ay_balde.clear();
		queue_az_balde.clear();
		queue_ax_mainfall.clear();
		queue_ay_mainfall.clear();
		queue_az_mainfall.clear();
		bkfinit = true;
	}
/*--------------------------------铲刀传感器数据处理--------------------------------------------*/
	if (iatd.bbaldeimu_update)
	{
		dt_baldeimu = iatd.imutime_balde;//IMU采样间隔
		double accmean[3] = { 0 }, gyomean[3] = { 0 };
		Maddn(accpre_balde, iatd.acc_balde, accmean, 3, 1);
		Maddn(gyopre_balde, iatd.gyo_balde, gyomean, 3, 1);
		Mmul(accmean, 3, 1, 0.5);
		Mmul(gyomean, 3, 1, 0.5);
		Mequalm(iatd.acc_balde, 3, 1, accpre_balde);
		Mequalm(iatd.gyo_balde, 3, 1, gyopre_balde);
		Mminn(gyomean, gyobias_balde, gyo_bias_balde, 3, 1); //陀螺数据取均值
/*-----------纯陀螺积分角度---------------*/
		integ_att_blade[0] += (gyomean[0]- attinitial.bias_gx1) * dt_baldeimu;
		integ_att_blade[1] += (gyomean[1] - attinitial.bias_gy1) * dt_baldeimu;
		integ_att_blade[2] += (gyomean[2] - attinitial.bias_gz1) * dt_baldeimu;
/*----------姿态更新 - 四元数法-----------*/	
#if 1  	
		double gyoincrement[3] = { 0 };
		Mmuln(gyo_bias_balde, 3, 1, dt_baldeimu, gyoincrement);
		qupdt(qua_balde, gyoincrement);
		double atttemp[3] = { 0.0 };
		q2mat_ned(qua_balde, Cb2n_balde);
		m2att_ned(Cb2n_balde, atttemp);
#if 1
		roll_balde    = atttemp[0];
		pitch_balde   = atttemp[1];
		heading_balde = atttemp[2];
#endif
		//roll_balde += (gyo_bias_balde[0] * dt_baldeimu);//陀螺积分横滚角(零偏反馈修正)
		//pitch_balde += (gyo_bias_balde[1] * dt_baldeimu);//陀螺积分俯仰角(零偏反馈修正)
		//heading_balde += (gyo_bias_balde[2] * dt_baldeimu);//陀螺积分航向角(零偏反馈修正)
#endif
/*----------滤波器时间更新-----------*/
/*----------俯仰角滤波器时间更新-----------*/
		ekf_pitch.setPhi(iatd.imutime_balde,1);
		ekf_pitch.TUpdate(iatd.imutime_balde, 0);
/*----------横滚角滤波器时间更新-----------*/
		ekf_roll.setPhi(iatd.imutime_balde,1);
		ekf_roll.TUpdate(iatd.imutime_balde, 0);
/*----------航向角滤波器时间更新-----------*/
		ekf_yaw.setPhi(iatd.imutime_balde,1);
		ekf_yaw.TUpdate(iatd.imutime_balde, 0);
/*----------构造观测量和量测噪声-----------*/	
//铲刀为控制对象，且动态变化频率高，应用高频观测
		double noise_roll[1] = { 0 }, noise_pitch[1] = { 0 };
		double normacc = fabs(sqrt(iatd.acc_balde[0] * iatd.acc_balde[0] + iatd.acc_balde[1] * iatd.acc_balde[1] + iatd.acc_balde[2] * iatd.acc_balde[2]) / GN - 1.0);
#if ACC_SMOOTH_NOISE	/************加速度计开窗平滑，滤除高频信号***********/
		int acc_smooth_falg = Acc_Amooth(iatd, 0);
		double roll_acc = 0;
		double pitch_acc = 0;
		if (acc_smooth_falg)
		{
			roll_acc = atan2(-smooth_acc_balde[1], -smooth_acc_balde[2]);
			pitch_acc = atan2(smooth_acc_balde[0], sqrt(smooth_acc_balde[1] * smooth_acc_balde[1] + smooth_acc_balde[2] * smooth_acc_balde[2]));//抬头为正
		}
		else
		{
			roll_acc = atan2(-accmean[1], -accmean[2]);
			pitch_acc = atan2(accmean[0], sqrt(accmean[1] * accmean[1] + accmean[2] * accmean[2]));//抬头为正
		}
		noise_roll[0] = 3.5*D2R;
		noise_pitch[0] = 4.5*D2R;
#endif		
#if ACC_LINEAR_NOISE  /*******************噪声动静态区分*************/
		double roll_acc = atan2(-accmean[1], -accmean[2]);
		double pitch_acc = atan2(accmean[0], sqrt(accmean[1] * accmean[1] + accmean[2] * accmean[2]));
		//double roll_acc = atan2(-iatd.acc_balde[1], -iatd.acc_balde[2]);
		//double pitch_acc = atan2(iatd.acc_balde[0], sqrt(iatd.acc_balde[1] * iatd.acc_balde[1] + iatd.acc_balde[2] * iatd.acc_balde[2]));
		/*------------干扰加速度判断------------*/
		// 1.加速度计的模值 < 阈值
		// 2.前后历元加速度计计算的角度差值 < 阈值
		// 3.对应旋转轴角速度模值 < 阈值
		// 4.新息 < 阈值
		if (normacc < 0.01)  //10mg 准静态
		{
			num_accnorm_balde++;
		}
		else
		{
			num_accnorm_balde = 0;
		}
		if (num_accnorm_balde > 200)  //准静态超过1秒
		{
			noise_roll[0] = 1.5*D2R;
			noise_pitch[0] = 1.5*D2R;
			/********增加静态的处理，加计开窗平滑*********/
		}
		else
		{
			noise_roll[0] = (1.5 + (normacc - 0.01) * 80)*D2R;
			noise_pitch[0] = (1.5 + (normacc - 0.01) * 80)*D2R;
		}
		if (normacc > 0.03)
		{
			noise_roll[0] = 12*D2R;
			noise_pitch[0] = 12*D2R;
		}
#endif
#if DEBUG_SAVE
		outdhf(1, "%f, %f, %f, %f, %f, %f, %f, %f, %f,%f, %f, %f,%f,%f,%f,%f\n",iatd.imutime_balde, 
			roll_acc*R2D, roll_balde*R2D, 
			pitch_acc*R2D,pitch_balde*R2D,
			heading_balde*R2D, roll_acc*R2D, pitch_acc*R2D, iatd.yaw_2ant*R2D,
			integ_att_blade[0]*R2D, integ_att_blade[1] * R2D, integ_att_blade[2] * R2D,
			iatd.roll_2ant*R2D, gyomean[0]*R2D, dt_baldeimu, normacc);
		outdhf(10, "%f,%f,%f\n", heading_balde*R2D, iatd.yaw_2ant*R2D, gyomean[2]*R2D);
#endif
		double zk_roll[1] = { 0 }, zk_pitch[1] = { 0 };
		zk_roll[0] = roll_balde - roll_acc;
		zk_pitch[0] = pitch_balde - pitch_acc;
/*----------俯仰角滤波器量测更新-----------*/
		ekf_pitch.setRk(noise_pitch, 0);
		ekf_pitch.setHk(1);
		ekf_pitch.MUpdate(zk_pitch);
/*----------横滚角滤波器量测更新-----------*/
		ekf_roll.setRk(noise_roll, 0);
		ekf_roll.setHk(1);
		ekf_roll.MUpdate(zk_roll);
/*----------反馈修正-----------*/
/*----------俯仰角反馈修正-------*/
		pitch_balde -= ekf_pitch.xk[0];
		gyobias_balde[1] += ekf_pitch.xk[1];
/*----------横滚角反馈修正-------*/
		roll_balde -= ekf_roll.xk[0];
		gyobias_balde[0] += ekf_roll.xk[1];
		num_nogetbaldeimu = 0;
#if DEBUG_SAVE
		outdhf(4, "%f, %f, %f, %f, %f, %f, %f\n",
			(ekf_roll.xk[0]),(ekf_roll.xk[1]), (ekf_pitch.xk[0]),(ekf_pitch.xk[1]), gyobias_balde[0] * R2D, gyobias_balde[1] * R2D, iatd.imutime_balde);
#endif
/*-----------状态清零------------*/
		ekf_pitch.xk[0] = 0;
		ekf_pitch.xk[1] = 0;
		ekf_roll.xk[0] = 0;
		ekf_roll.xk[1] = 0;
	}
#if 1
/*--------------------------------------双天线航向数据处理-----------------------------------------*/
	if (iatd.bgnssupdate)
	{
/*----------构造观测量和量测噪声-----------*/
		double zk_yaw[1] = { 0 };
		zk_yaw[0] = heading_balde - iatd.yaw_2ant;
		/***根据新息判断航向是否临界变换，若临界则不更新***/
		if (fabs(zk_yaw[0]) > PI)
		{
			RAD_PI_PI(zk_yaw[0]);
		}
		//double noise_yaw[1] = { iatd.gyaw_noise*3 }; //0.1 时间延迟等影响
		double noise_yaw[1] = { 0.3*D2R * 1 }; //0.1 时间延迟等影响
/*----------航向角滤波器量测更新-----------*/
		ekf_yaw.setRk(noise_yaw, 0);
		ekf_yaw.setHk(1);
		ekf_yaw.MUpdate(zk_yaw);
/*----------航向角反馈修正-------*/
		heading_balde -= ekf_yaw.xk[0];
		gyobias_balde[2] += ekf_yaw.xk[1];
#if DEBUG_SAVE
		outdhf(5, "%f, %f, %f, %f\n",
			sqrt(ekf_yaw.xk[0]), sqrt(ekf_yaw.xk[1]), gyobias_balde[2] * R2D, iatd.imutime_balde);
#endif
/*---------------状态置零-----------------*/
		ekf_yaw.xk[0] = 0;
		ekf_yaw.xk[1] = 0;
		num_nogetgnss = 0;
	}
	RAD_PI_PI(heading_balde);
	double atti[3] = { roll_balde,pitch_balde,heading_balde };
	a2mat_ned(atti, Cb2n_balde);
	m2qua_ned(Cb2n_balde, qua_balde);
	//计算倾斜角度
	att2tilt(atti, &crossslope_balde, &tiltslope_balde);

/*------------------------------------------车架传感器数据处理---------------------------------------------*/
	if (iatd.bmainfallimu_update) 
	{
		dt_mainfallimu = iatd.imutime_mainfall;

		double accmean[3] = { 0 }, gyomean[3] = { 0 };
		Maddn(accpre_mainfall, iatd.acc_mainfall, accmean, 3, 1);
		Maddn(gyopre_mainfall, iatd.gyo_mainfall, gyomean, 3, 1);
		Mmul(accmean, 3, 1, 0.5);
		Mmul(gyomean, 3, 1, 0.5);
		Mequalm(iatd.acc_mainfall, 3, 1, accpre_mainfall);
		Mequalm(iatd.gyo_mainfall, 3, 1, gyopre_mainfall);
		Mminn(gyomean, gyobias_mainfall, gyo_bias_mainfall, 3, 1); //陀螺数据取均值
 /*----------姿态更新 - 角速度积分-----------*/
#if 1  	
		double gyoincrement[3] = { 0 };
		Mmuln(gyo_bias_mainfall, 3, 1, dt_mainfallimu, gyoincrement);
		tiltslop_mainfall += gyoincrement[1];
		iatd.integ_pitch += gyoincrement[1];
#endif
/*----------滤波器时间更新-----------*/
/*----------车架俯仰角滤波器时间更新-----------*/
		ekf_pitch_mainfall.setPhi(iatd.imutime_balde,1);
		ekf_pitch_mainfall.TUpdate(iatd.imutime_balde, 0);
/*----------构造观测量和量测噪声-----------*/
//车架传感器测量的是地形起伏和趋势，是低频信号，可对加速度数据进行平滑处理
		double noise_pitch[1] = { 0 };
		double normacc = fabs(sqrt(iatd.acc_mainfall[0] * iatd.acc_mainfall[0] + iatd.acc_mainfall[1] * iatd.acc_mainfall[1] + iatd.acc_mainfall[2] * iatd.acc_mainfall[2]) / GN - 1.0);
#if ACC_SMOOTH_NOISE	/************加速度计开窗平滑，滤除高频信号***********/
		int acc_smooth_falg = Acc_Amooth(iatd, 1);
		double pitch_acc_body = 0;
		if (acc_smooth_falg)
		{
			pitch_acc_body = atan2(smooth_acc_mainfall[0], sqrt(smooth_acc_mainfall[1] * smooth_acc_mainfall[1] + smooth_acc_mainfall[2] * smooth_acc_mainfall[2]));
		}
		else
		{
			pitch_acc_body = atan2(accmean[0], sqrt(accmean[1] * accmean[1] + accmean[2] * accmean[2]));
		}
		noise_pitch[0] =4.5*D2R;
#endif		
#if ACC_LINEAR_NOISE   /*******************噪声动静态区分*************/
		double pitch_acc_body = atan2(accmean[0], sqrt(accmean[1] * accmean[1] + accmean[2] * accmean[2]));
		if (normacc < 0.01)  //10mg 准静态
		{
			num_accnorm_mainfall++;
		}
		else
		{
			num_accnorm_mainfall = 0;
		}
		if (num_accnorm_mainfall > 200)  //准静态超过1秒
		{
			noise_pitch[0] = 1.5*D2R;
			/********增加静态的处理，加计开窗平滑*********/
		}
		else
		{
			noise_pitch[0] = (1.5 + (normacc - 0.01) * 50)*D2R;
		}
		if (normacc > 0.02)
		{
			noise_pitch[0] = 8.5*D2R;
		}
#endif
		double zk_pitch[1] = { 0 };
		zk_pitch[0] = tiltslop_mainfall - pitch_acc_body;
/*----------车架俯仰角滤波器量测更新-----------*/
		ekf_pitch_mainfall.setRk(noise_pitch, 0);
		ekf_pitch_mainfall.setHk(1);
		ekf_pitch_mainfall.MUpdate(zk_pitch);
/*----------反馈修正-----------*/
/*----------俯仰角反馈修正-------*/
		tiltslop_mainfall -= ekf_pitch_mainfall.xk[0];
		gyobias_mainfall[1] += ekf_pitch_mainfall.xk[1];
#if DEBUG_SAVE
		outdhf(2, "%f, %f, %f,%f,%f,%f,%f\n", iatd.imutime_mainfall, tiltslop_mainfall*R2D, pitch_acc_body*R2D, iatd.integ_pitch*R2D,
			ekf_pitch_mainfall.xk[0], gyobias_mainfall[1], normacc);

		outdhf(6, "%f, %f, %f, %f\n",
			sqrt(ekf_pitch_mainfall.xk[0]), sqrt(ekf_pitch_mainfall.xk[0]), gyobias_mainfall[1] * R2D, iatd.imutime_mainfall);
#endif
/*------------状态置零-----------*/
		ekf_pitch_mainfall.xk[0] = 0;
		ekf_pitch_mainfall.xk[1] = 0;
		num_nogetbaldeimu = 0;
	}
#endif
	return 1;
}

int ATProcess::Two_Position_Install_Err(AttTrackData& iatd)
{
	//俯仰、横滚安装误差标定，加计零偏标定（出厂做六位置法标定）
	if (!Acc_Bias_ok&&ACC_BIAS_CAL)
	{
/*----------------------------铲刀俯仰角安装误差---------------------------*/
		if (!position1_ok&&iatd.balde_install_falg==1)//上位机发送位置1标定状态
		{
			int pos1_flag = 0;
			/*位置1：调整铲刀与车辆垂直，铲刀放置在地上，同时记录铲刀两端位置标记点*/
			/*位置1：调整天线杆上铅垂线至垂直，可认为当前铲刀安装平面处于水平*/
			/*位置1：并记录车辆前后四个车轮的位置，同时采集三轴加速度的原始数据*/
			/*位置1：加速度质量满足std阈值，则位置1采样完成，发送上位机交互标志*/
			/*位置1：上位机提示位置1标定结果，需要转换车辆位置*/
			pos1_flag = cal.Acc_Bias_2pos_cal(iatd, 1);
			if (pos1_flag)
			{
				position1_ok = true;//位置1采样完成
				blade_install_result = 1;//返回上位机
			}
		}
/*----------------------------铲刀横滚角安装误差---------------------------*/
		if (position1_ok&&!position2_ok&&iatd.balde_install_falg==2)//上位机发送位置2标定状态
		{
			int pos2_flag = 0;
			/*位置2：车辆转换180度，调整铲刀位置与位置1标记点重合，同时铲刀放置在地上*/
			/*位置2：上位机发送标定状态，采集位置2数据，采集完毕返回状态，上位机提示*/
			/*位置2：标定成功：上位机显示横滚角安装误差标定成功，上位机提示转换下一个位置*/
			pos2_flag = cal.Acc_Bias_2pos_cal(iatd, 2);
			if (pos2_flag)
			{
				position2_ok = true;//位置2采样完成
				blade_install_result = 2;//返回上位机
			}
		}
/*----------------------------车体俯仰角安装误差---------------------------*/
		if (position1_ok&&position2_ok&&!position3_ok&&iatd.balde_install_falg==3)//上位机发送位置3标定状态
		{
			int pos3_flag = 0;
			/*位置3：调整车辆位置，车辆前后四个车轮与位置1标记车轮位置重合*/
			/*位置3：上位机发送标定状态，采集位置3数据，采集完毕返回状态，上位机提示*/
			/*位置3：标定成功：上位机显示俯仰角角安装误差标定成功，上位机提示标定完成*/
			pos3_flag = cal.Acc_Bias_2pos_cal(iatd, 3);
			if (pos3_flag)
			{
				position3_ok = true;//位置3采样完成
			}
		}
		if (position1_ok&&position2_ok&&position3_ok)
		{
			int install_flag = 0;
			install_flag=cal.Acc_Bias_2pos_cal(iatd, 4);
			if (install_flag)
			{
				Acc_Bias_ok = true;//双位置安装标定完成，需返回上位机标定成功状态
				cfg.balde_install_roll = cal.blade_installangle[0];
				cfg.balde_install_pitch = cal.blade_installangle[1];
				cfg.mainfall_install_pitch = cal.mainfall_installangle[1];
				blade_install_result = 3;//返回上位机
			}
		}
	}
	if (Acc_Bias_ok)
	{
		double acc_bias[3] = {cfg.abiasx,cfg.abiasy,cfg.abiasz };//三轴加速度零偏需要六位置法出厂标定
		cal.Comp_Acc_bias(&iatd, acc_bias);//3轴加计零偏补偿
		printf("balde_install_err:%f,%f\n", cfg.balde_install_roll*R2D, cfg.balde_install_pitch*R2D);
		/*铲刀安装误差补偿*/
		cal.Comp_InstallErr_Acc(iatd.acc_balde, cfg.balde_install_roll, cfg.balde_install_pitch);//俯仰、横滚安装偏差补偿
		cal.Comp_InstallErr_Acc(iatd.gyo_balde, cfg.balde_install_roll, cfg.balde_install_pitch);
		/*车体安装误差补偿*/
		cal.Comp_InstallErr_Acc(iatd.acc_mainfall, 0.0, cfg.mainfall_install_pitch);
		cal.Comp_InstallErr_Acc(iatd.gyo_mainfall, 0.0, cfg.mainfall_install_pitch);
		return 1;
	}
	else 
	{
		return 0;
	}
}
int ATProcess::Acc_Amooth(AttTrackData& iatd, int option)
{
	if (option == 0)//铲刀IMU加速度平滑
	{
		double sum_ax = 0,sum_ay=0,sum_az=0;
		if ((int)queue_ax_balde.size() < ACC_SMOOTH_WIN)
		{
			queue_ax_balde.push_back(iatd.acc_balde[0]);
			queue_ay_balde.push_back(iatd.acc_balde[1]);
			queue_az_balde.push_back(iatd.acc_balde[2]);
			return 0;
		}
		else
		{
			queue_ax_balde.erase(queue_ax_balde.begin());
			queue_ax_balde.push_back(iatd.acc_balde[0]);
			queue_ay_balde.erase(queue_ay_balde.begin());
			queue_ay_balde.push_back(iatd.acc_balde[1]);
			queue_az_balde.erase(queue_az_balde.begin());
			queue_az_balde.push_back(iatd.acc_balde[2]);
			int num = (int)queue_ax_balde.size();
			for (int i = 0; i < num; i++)
			{
				sum_ax += queue_ax_balde[i];
				sum_ay += queue_ay_balde[i];
				sum_az += queue_az_balde[i];
			}
			smooth_acc_balde[0] = sum_ax / num;
			smooth_acc_balde[1] = sum_ay / num;
			smooth_acc_balde[2] = sum_az / num;
			return 1;
		}
	}
	else if (option == 1)//车架IMU加速度平滑
	{
		double sum_ax = 0, sum_ay = 0, sum_az = 0;
		if ((int)queue_ax_mainfall.size() < ACC_SMOOTH_WIN)
		{
			queue_ax_mainfall.push_back(iatd.acc_mainfall[0]);
			queue_ay_mainfall.push_back(iatd.acc_mainfall[1]);
			queue_az_mainfall.push_back(iatd.acc_mainfall[2]);
			return 0;
		}
		else
		{
			queue_ax_mainfall.erase(queue_ax_mainfall.begin());
			queue_ax_mainfall.push_back(iatd.acc_mainfall[0]);
			queue_ay_mainfall.erase(queue_ay_mainfall.begin());
			queue_ay_mainfall.push_back(iatd.acc_mainfall[1]);
			queue_az_mainfall.erase(queue_az_mainfall.begin());
			queue_az_mainfall.push_back(iatd.acc_mainfall[2]);
			int num = (int)queue_ax_balde.size();
			for (int i = 0; i < num; i++)
			{
				sum_ax += queue_ax_mainfall[i];
				sum_ay += queue_ay_mainfall[i];
				sum_az += queue_az_mainfall[i];
			}
			smooth_acc_mainfall[0] = sum_ax / num;
			smooth_acc_mainfall[1] = sum_ay / num;
			smooth_acc_mainfall[2] = sum_az / num;
			return 1;
		}
	}
	else
	{
		return 0;
	}
}
int ATProcess::Rotation_Zero_Err(AttTrackData& iatd)
{
	if (!rotation_zero_ok&&Heading_2ant_Err&&iatd.rotation_install_flag==1)//改成上位机交互的标志
	{
		int cal_flag = 0;
		cal_flag = cal.InstallErr_Rotation(iatd.rotation_balde);//双天线安装误差标定
		if (cal_flag)
		{
			rotation_zero = cal.installrotation;
			rotation_zero_ok = true;//需返回上位机标定成功状态
			rotation_install_result = 1;//返回上位机标志
			return 1;
		}
		else
		{
			rotation_install_result = 0;//返回上位机标志
			return 0;
		}
	}
	else
	{
		return 0;
	}
}