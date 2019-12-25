#include "ElevationEstimate.h"
#include "ComFunc.h"
#include "ATProcess.h"

ElevationEstimate::ElevationEstimate() {}
ElevationEstimate::~ElevationEstimate() {}
int ElevationEstimate::initstate(double gnssalt, double gnssvu)
{
	alt = gnssalt;
	vu = gnssvu;
	accalt = 0;
	accbias = 0;
	anzpre = 0;
	Acc_Alt = 0;
	Gnss_Alt = 0;
	tmp_vu = 0;
	tmp_alt = 0;
	for (int i = 0; i < 3; i++)
	{
		accpre[i] = 0;
	}
	//ekfelv = EKF(4, 3);
	//kfelv = KF(4, 3);
	binitkf = true;
	bcompfilter = true;
	return 0;
}
//int ElevationEstimate::update_ekf(double acc[3], double dt, double roll, double pitch, double gnssalt, double gnssvn, int gpsstate,int bgnssupdata,int bstatic)
//{											
//	if (binitkf)
//	{
//		ekfelv = EKF(4, 3);
//		//高程 高程速度 加速度 零偏
//		double dxk0[4] = { 0.02,0.02,0.005*GN, 0.00001*GN };
//		//double dxk0[4] = { 0.05,0.05,0.015*GN, 0.005*GN };
//		ekfelv.setPxk(dxk0);
//		//double noise[4] = { 0.1,0.1,0.005*GN,0.000001*GN }; 
//		//double noise[4] = { 0.05,0.01,0.003*GN,0.000001*GN };
//		double noise[4] = { 0.1,0.1,0.01*GN,0.000001*GN };
//		ekfelv.setQk(noise);
//		binitkf = false;
//	}
//	double accmean[3] = { 0 };
//	Maddn(accpre, acc, accmean, 3, 1);
//	Mmul(accmean, 3, 1, 0.5);
//	Mequalm(acc, 3, 1, accpre);
//	/**********加速度观测量构造**********/
//	double Cb2lever[9] = { 0 }, Croll[9] = { 0 }, Cpitch[9] = { 0 }, Ctemp[9] = { 0 };
//	double cphi = cos(roll); double sphi = sin(roll);
//	double cthe = cos(pitch); double sthe = sin(pitch);
//	Croll[0 * 3 + 0] = 1.0;
//	Croll[1 * 3 + 1] = cphi;  Croll[1 * 3 + 2] = sphi;
//	Croll[2 * 3 + 1] = -sphi; Croll[2 * 3 + 2] = cphi;
//	Cpitch[0 * 3 + 0] = cthe;                  Cpitch[0 * 3 + 2] = -sthe;
//	Cpitch[1 * 3 + 1] = 1.0;
//	Cpitch[2 * 3 + 0] = sthe;                  Cpitch[2 * 3 + 2] = cthe;
//	Mmulnm(Croll, Cpitch, 3, 3, 3, Ctemp);
//	Mtn(Ctemp, 3, 3, Cb2lever);   //调平
//	double f_lever[3] = { 0 };
//	Mmulnm(Cb2lever, accmean, 3, 3, 1, f_lever);
//	outdhf(10, "%f,%f,%f,%f,%f,%f\n", acc[0], acc[1], acc[2], f_lever[0], f_lever[1], f_lever[2]);
//	/**********牛顿力学方程**********/
//	//vu += (-(fn[2] + GN - accbias)*dt);
//	//vu += accalt*dt;
//	//vu += (accalt - accbias)*dt;
//
//	//vu += (accalt + accbias)*dt;
//	//alt += vu*dt;
//
//	vu += accalt*dt;
//	alt += vu*dt;
//	/**********滤波更新************/
//	ekfelv.setPhi(dt, 1);
//	ekfelv.TUpdate(dt);
//
//	double accz = -(f_lever[2] + GN - accbias);
//	//double accz = -(fn[2] + GN);
//	double accnoise = 0.001*GN; //1mg
//	if (bgnssupdata)
//	{
//		double zk[3] = { 0 }, noise[3] = {0};
//		zk[0] = alt - gnssalt;
//		zk[1] = vu - gnssvn;
//		zk[2] = accalt - accz;
//		ekfelv.setzk(zk, 3);
//
//		if (bstatic)
//		{
//			noise[0] = 0.01;
//			noise[1] = 0.01;
//			//环境噪声
//			//noise[2] = 0.2;
//			//加计噪声datasheet
//			//noise[2] = 0.0012*GN*5;
//			noise[2] = accnoise*30;
//		}
//		else
//		{
//			//noise[0] = 0.01;
//			//noise[1] = 0.01;
//			//noise[2] = 0.0012*GN * 5;
//			noise[0] = 0.02;
//			noise[1] = 0.03;
//			noise[2] = accnoise*30;
//		}
//		ekfelv.setRk(noise, 3);
//		ekfelv.setHk(3,1);
//		ekfelv.MUpdate(noise, zk);
//		alt -= ekfelv.xk[0];
//		vu -= ekfelv.xk[1];
//		accalt -= ekfelv.xk[2];
//		accbias += ekfelv.xk[3];
//		ekfelv.xk.zeros();
//	}
//	else
//	{
//		double zk[1] = { 0 }, noise[1] = { 0 };
//		zk[0] = accalt - accz;
//		ekfelv.setzk(zk, 1);
//		if (bstatic)
//		{
//			//noise[0] = 0.0012*GN * 5;
//			noise[0] = accnoise*30;
//		}
//		else
//		{
//			//noise[0] = 0.0012*GN * 5;
//			noise[0] = accnoise * 30;
//		}
//		ekfelv.setRk(noise, 1);
//
//		ekfelv.setHk(1,2);
//		ekfelv.MUpdate(noise, zk);
//		alt -= ekfelv.xk[0];
//		vu -= ekfelv.xk[1];
//		accalt -= ekfelv.xk[2];
//		accbias += ekfelv.xk[3];
//		ekfelv.xk.zeros();
//	}
//#ifdef SaveElePro
//	outdhf(1, "%f,%f,%f,%f,%f,%f,%f,%d\n", alt, vu, accalt, accbias, gnssalt,gnssvn,accz,bstatic);
//	outdhf(2, "%f,%f,%f,%f\n", sqrt(ekfelv.Pxk[0]), sqrt(ekfelv.Pxk[5]), sqrt(ekfelv.Pxk[10]), sqrt(ekfelv.Pxk[15]));
//#endif
//	return 1;
//}

int ElevationEstimate::update_kf(AttTrackData& iatd, double att[3], int bstatic)

{
	static int gnss_smooth_flag = 0;
	double accmean[3] = { 0 };
	double normacc = sqrt(iatd.acc_balde[0] * iatd.acc_balde[0] + iatd.acc_balde[1] * iatd.acc_balde[1] + iatd.acc_balde[2] * iatd.acc_balde[2]) - GN;
	Maddn(accpre, iatd.acc_balde, accmean, 3, 1);
	Mmul(accmean, 3, 1, 0.5);
	Mequalm(iatd.acc_balde, 3, 1, accpre);
	/**********加速度观测量构造 - 加速度调平**********/
	double f_lever[3] = { 0 };
	Conver_b2lever(att[0], att[1], accmean, f_lever); //NED坐标系
	int gnss_alt_err_flag = 0;
		//Gnss_alt_Err(iatd);//返回高程错误标志
	if (iatd.bgnssupdate)
	{
		gnss_smooth_flag=Gnss_Amooth(iatd, 0);
	}
	if (gnss_alt_err_flag)//高程异常
	{
		//mean_Alt = iatd.BLH_ant[2];//取上一时刻高程值
	}
	if (binitkf)
	{
		//高程 高程速度 加速度 零偏
		double xk0[4] = { iatd.BLH_ant[2] ,iatd.speed_ver ,f_lever[2] + GN,0 };
		double dxk0[4] = { 0.05,0.1,0.01*GN, 0.008*GN };
		double noise[4] = { 0.01,0.01,0.01*GN,0.000001*GN };
/*********高程融合滤波器内存分配，参数初始********/
		kfelv.kfmalloc(ALT_NUMX, ALT_NUMV);
		kfelv.kfinit();
		kfelv.setXk(xk0);
		kfelv.setPxk(dxk0);
		kfelv.setQk(noise);
		pre_alt = iatd.BLH_ant[2];
		tmp_alt = iatd.BLH_ant[2];
		queue_gnss_alt.clear();
		queue_gnss_ver.clear();
		binitkf = false;
	}
	//outdhf(10, "%f,%f,%f,%f,%f,%f\n", acc[0], acc[1], acc[2], f_lever[0], f_lever[1], f_lever[2]);
/*-----------滤波器时间更新----------*/
	kfelv.setPhi(iatd.imutime_balde,0);
	kfelv.TUpdate(iatd.imutime_balde,0);
/*-----------滤波器量测更新----------*/
/*----------GNSS 高程和天向速度观测----------*/
#if 0
	if (iatd.bgnssupdate)  
	{
		double zk[2] = { 0 }, noise[2] = { 0 };
		//zk[0] = iatd.BLH_ant[2];
		//zk[1] = iatd.speed_ver;
		zk[0] = mean_Alt;//对原始高程进行平滑
		zk[1] = mean_ver;
		kfelv.setzk(zk, 2);
		if (iatd.gstate_pos == 4)
		{
			noise[0] = 0.005;  //GNSS高程观测噪声0.008
			noise[1] = 0.02;  //GNSS天向速度估测0.02
			if (gnss_alt_err_flag)
			{
			//noise[0] = 0.1;  //GNSS高程观测噪声0.008
			//noise[1] = 0.05;  //GNSS天向速度估测0.02
			}
		}
		else
		{
			noise[0] = 0.02;  //GNSS高程观测噪声0.005
			noise[1] = 0.05;  //GNSS天向速度估测0.02
		}
		kfelv.setRk(noise, 2);
		kfelv.setHk(2, 1);
		kfelv.MUpdate(noise, zk);
		kfelv.xk[0] = (kfelv.xk[0] + pre_alt) / 2;
		if (gnss_alt_err_flag)
		{
			//tmp_vu = kfelv.xk[1];
			//tmp_alt = kfelv.xk[0];
		}
		pre_alt = kfelv.xk[0];
	}
#endif
	/*----------IMU-Z轴加速度、速度观测----------*/
	if (gnss_smooth_flag)
	{
		tmp_vu = mean_ver;
		tmp_alt = mean_Alt;
	}
	else
	{
		tmp_vu = iatd.speed_ver;
		tmp_alt = iatd.BLH_ant[2];
	}
#if 1
	//else  
	if(iatd.bbaldeimu_update)
	{
		double zk[3] = { 0 }, noise[3] = { 0 };
#if 0
		zk[0] = -(f_lever[2] + GN - kfelv.xk[3]);  //高程加速度
		tmp_vu -= (f_lever[2] + GN - kfelv.xk[3])*dt;//天向速度
#endif
#if 1
		zk[0] = -(f_lever[2] + GN);  //高程加速度天向为正
		tmp_vu -= (f_lever[2] + GN)*iatd.imutime_balde;
#endif
		/*观测量：垂直加速度、推算垂直速度、推算高程*/
		tmp_alt += tmp_vu*iatd.imutime_balde;//推算高程
		zk[1] = tmp_vu;//天向实时推算速度
		zk[2] = tmp_alt;//推算高程
//需处理干扰加速度
	    noise[0] = 0.01*GN * 1;
		noise[1] = 0.01;//0.01
		noise[2] = 0.01;//0.01
		kfelv.setRk(noise,0);
		kfelv.setHk(0);
		kfelv.MUpdate(zk);
		pre_alt = kfelv.xk[0];
	}
#endif
	//pre_alt = iatd.BLH_ant[2];
	kf_alt = kfelv.xk[0];//输出高程数据
	kf_vu = kfelv.xk[1];//输出竖直速度
#if DEBUG_SAVE
	if (mean_Alt)
	{
		outdhf(3, "%f,%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%f\n", kfelv.xk[0], kfelv.xk[1], kfelv.xk[2], kfelv.xk[3], f_lever[2],
			iatd.BLH_ant[2], iatd.speed_ver, bstatic,normacc,
			mean_Alt, gnss_alt_err_flag, iatd.imu_time);
		outdhf(7, "%f,%f,%f,%f\n", (kfelv.Pxk[0]), (kfelv.Pxk[5]), sqrt(kfelv.Pxk[10]), sqrt(kfelv.Pxk[15]));
	}
#endif
	return 1;
}

int ElevationEstimate::Alt_Complement_Filter(double acc[3], double dt, double roll, double pitch, double gnssalt, double gnssvn, int gpsstate, int bgnssupdata, int bstatic)
{
	if (bcompfilter)
	{
		if (bgnssupdata)
		{
			Acc_Alt = gnssalt;
			Gnss_Alt = gnssalt;
			bcompfilter = false;
		}
		return 0;
	}
	double accmean[3] = { 0 };
	Maddn(accpre, acc, accmean, 3, 1);
	Mmul(accmean, 3, 1, 0.5);
	Mequalm(acc, 3, 1, accpre);
	/**********加速度观测量构造 - 加速度调平**********/
	double f_lever[3] = { 0 };
	Conver_b2lever(roll, pitch, accmean, f_lever); //NED坐标系
	if (!bcompfilter)
	{
		if (bgnssupdata)
		{
			Gnss_Alt = gnssalt;
		}
		else
		{
			//Acc_Alt += gnssvn*dt + (f_lever[2] + GN)*dt*dt / 2;
			//Acc_Alt = Gnss_Alt+gnssvn*dt + (f_lever[2] + GN)*dt*dt / 2;
			Acc_Alt += gnssvn*dt;
		}
		kf_alt = Gnss_Alt*Ki + Acc_Alt*(1-Ki);
		outdhf(3, "%f,%f,%f,%f,%f,%f,%f,%d,%f\n", kf_alt, gnssvn,0,0, f_lever[2],gnssalt,gnssvn, bstatic, Acc_Alt);
		return 1;
	}
	return 0;
}




void Conver_b2lever(double roll, double pitch, double var_b[3], double var_lever[3])
{
	double Cb2lever[9] = { 0 }, Croll[9] = { 0 }, Cpitch[9] = { 0 }, Ctemp[9] = { 0 };
	double cphi = cos(roll); double sphi = sin(roll);
	double cthe = cos(pitch); double sthe = sin(pitch);
	Croll[0 * 3 + 0] = 1.0;
	Croll[1 * 3 + 1] = cphi;  Croll[1 * 3 + 2] = sphi;
	Croll[2 * 3 + 1] = -sphi; Croll[2 * 3 + 2] = cphi;
	Cpitch[0 * 3 + 0] = cthe;                  Cpitch[0 * 3 + 2] = -sthe;
	Cpitch[1 * 3 + 1] = 1.0;
	Cpitch[2 * 3 + 0] = sthe;                  Cpitch[2 * 3 + 2] = cthe;
	Mmulnm(Croll, Cpitch, 3, 3, 3, Ctemp);
	Mtn(Ctemp, 3, 3, Cb2lever);   //调平
	Mmulnm(Cb2lever, var_b, 3, 3, 1, var_lever);
}
int ElevationEstimate::Gnss_Amooth(AttTrackData& iatd, int option)
{
	if (option == 0)//铲刀IMU加速度平滑
	{
		double sum_gnss_alt = 0, sum_gnss_ver = 0;
		if ((int)queue_gnss_alt.size() < ACC_SMOOTH_WIN)
		{
			queue_gnss_alt.push_back(iatd.BLH_ant[2]);
			queue_gnss_ver.push_back(iatd.speed_ver);
			return 0;
		}
		else
		{
			queue_gnss_alt.erase(queue_gnss_alt.begin());
			queue_gnss_alt.push_back(iatd.BLH_ant[2]);
			queue_gnss_ver.erase(queue_gnss_ver.begin());
			queue_gnss_ver.push_back(iatd.speed_ver);
			int num = (int)queue_gnss_alt.size();
			for (int i = 0; i < num; i++)
			{
				sum_gnss_alt += queue_gnss_alt[i];
				sum_gnss_ver += queue_gnss_ver[i];
			}
			mean_Alt = sum_gnss_alt / num;
			mean_ver = sum_gnss_ver / num;
			return 1;
		}
	}
	else
	{
		return 0;
	}
}

int ElevationEstimate::Gnss_alt_Err(AttTrackData& iatd)
{
	if (iatd.bgnssupdate)
	{
		if (vgpsalt.size() < Alt_WIN)
		{
			vgpsalt.push_back(iatd.BLH_ant[2]);//高程速度
			vgpspitch.push_back(iatd.roll_2ant);
			return 0;
		}
		else
		{
			vgpsalt.push_back(iatd.BLH_ant[2]);
			vgpsalt.erase(vgpsalt.begin());
			vgpspitch.push_back(iatd.roll_2ant);
			vgpspitch.erase(vgpspitch.begin());
			double mean_ver = 0.0, mean_pitch = 0.0;
			double std_ver = 0.0,std_pitch=0.0;
			int num = (int)vgpsalt.size();
			for (int i = 1; i < num - 1; i++)
			{
				mean_ver += vgpsalt[i];
				mean_pitch += vgpspitch[i];
			}
			mean_ver /= (num - 2);
			mean_pitch /= (num - 2);
			for (int i = 1; i < num - 1; i++)
			{
				std_ver += (vgpsalt[i] - mean_ver)*(vgpsalt[i] - mean_ver);
				std_pitch += (vgpspitch[i] - mean_pitch)*(vgpspitch[i] - mean_pitch);
			}
			if (std_ver > 0.02)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
	}
	else
	{
		return 0;
	}
}