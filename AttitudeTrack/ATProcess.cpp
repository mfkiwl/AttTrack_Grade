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
		//�˲�����״̬Ϊ�Ƕ�������ƫ������������QΪ���ݽ��ٶ���������ƫ���ȶ���
		//CRS03: ������0.025deg/s ��ƫ�ȶ��ԣ�3.5deg/h
		//SCC2230-D08 �������ܶȣ�0.008deg/s/sqrt(hz)  ��ƫ�ȶ��ԣ�2deg/hr
		//SCC2230-B15��z�����ݣ���������0.12deg/s  10hz
		//SCC2130-B15��x�����ݣ���������0.12deg/s  
		/*2019.12.6 ���ݲ��Դ����� x\yΪSCC2130-B15 zΪSCC2230-B15*/
		double gnoise = 0.12*D2R, gstabilityx = 2 * D2R / 3600.0;//10hz
		double dxk0[2] = { 1.0*D2R,0.1*D2R };  //�趨ֵӰ���������ٶ�
		double noise_baldegx[2] = { gnoise,gstabilityx };//x��y������ΪSCC2230-D08
		double noise_baldegy[2] = { gnoise,gstabilityx };
		double noise_baldegz[2] = { 0.12*D2R,gstabilityx };//z������ΪSCC2230-B15
		double noise_mainfallgy[2] = { gnoise,gstabilityx };
/*********�������˲����ڴ���䣬������ʼ********/
		ekf_pitch.kfmalloc(ATT_NUMX, ATT_NUMV);
		ekf_pitch.kfinit();
		//ekf_pitch.setXk(dxk0);
		ekf_pitch.setPxk(dxk0);
		ekf_pitch.setQk(noise_baldegy);
/*********������˲����ڴ���䣬������ʼ********/
		ekf_roll.kfmalloc(ATT_NUMX, ATT_NUMV);
		ekf_roll.kfinit();
		//ekf_roll.setXk(dxk0);
		ekf_roll.setPxk(dxk0);
		ekf_roll.setQk(noise_baldegx);
/*********������˲����ڴ���䣬������ʼ********/
		ekf_yaw.kfmalloc(ATT_NUMX, ATT_NUMV);
		ekf_yaw.kfinit();
		//ekf_yaw.setXk(dxk0);
		ekf_yaw.setPxk(dxk0);
		ekf_yaw.setQk(noise_baldegz);
/*********���ܸ������˲����ڴ���䣬������ʼ********/
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
/*--------------------------------�������������ݴ���--------------------------------------------*/
	if (iatd.bbaldeimu_update)
	{
		dt_baldeimu = iatd.imutime_balde;//IMU�������
		double accmean[3] = { 0 }, gyomean[3] = { 0 };
		Maddn(accpre_balde, iatd.acc_balde, accmean, 3, 1);
		Maddn(gyopre_balde, iatd.gyo_balde, gyomean, 3, 1);
		Mmul(accmean, 3, 1, 0.5);
		Mmul(gyomean, 3, 1, 0.5);
		Mequalm(iatd.acc_balde, 3, 1, accpre_balde);
		Mequalm(iatd.gyo_balde, 3, 1, gyopre_balde);
		Mminn(gyomean, gyobias_balde, gyo_bias_balde, 3, 1); //��������ȡ��ֵ
/*-----------�����ݻ��ֽǶ�---------------*/
		integ_att_blade[0] += (gyomean[0]- attinitial.bias_gx1) * dt_baldeimu;
		integ_att_blade[1] += (gyomean[1] - attinitial.bias_gy1) * dt_baldeimu;
		integ_att_blade[2] += (gyomean[2] - attinitial.bias_gz1) * dt_baldeimu;
/*----------��̬���� - ��Ԫ����-----------*/	
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
		//roll_balde += (gyo_bias_balde[0] * dt_baldeimu);//���ݻ��ֺ����(��ƫ��������)
		//pitch_balde += (gyo_bias_balde[1] * dt_baldeimu);//���ݻ��ָ�����(��ƫ��������)
		//heading_balde += (gyo_bias_balde[2] * dt_baldeimu);//���ݻ��ֺ����(��ƫ��������)
#endif
/*----------�˲���ʱ�����-----------*/
/*----------�������˲���ʱ�����-----------*/
		ekf_pitch.setPhi(iatd.imutime_balde,1);
		ekf_pitch.TUpdate(iatd.imutime_balde, 0);
/*----------������˲���ʱ�����-----------*/
		ekf_roll.setPhi(iatd.imutime_balde,1);
		ekf_roll.TUpdate(iatd.imutime_balde, 0);
/*----------������˲���ʱ�����-----------*/
		ekf_yaw.setPhi(iatd.imutime_balde,1);
		ekf_yaw.TUpdate(iatd.imutime_balde, 0);
/*----------����۲�������������-----------*/	
//����Ϊ���ƶ����Ҷ�̬�仯Ƶ�ʸߣ�Ӧ�ø�Ƶ�۲�
		double noise_roll[1] = { 0 }, noise_pitch[1] = { 0 };
		double normacc = fabs(sqrt(iatd.acc_balde[0] * iatd.acc_balde[0] + iatd.acc_balde[1] * iatd.acc_balde[1] + iatd.acc_balde[2] * iatd.acc_balde[2]) / GN - 1.0);
#if ACC_SMOOTH_NOISE	/************���ٶȼƿ���ƽ�����˳���Ƶ�ź�***********/
		int acc_smooth_falg = Acc_Amooth(iatd, 0);
		double roll_acc = 0;
		double pitch_acc = 0;
		if (acc_smooth_falg)
		{
			roll_acc = atan2(-smooth_acc_balde[1], -smooth_acc_balde[2]);
			pitch_acc = atan2(smooth_acc_balde[0], sqrt(smooth_acc_balde[1] * smooth_acc_balde[1] + smooth_acc_balde[2] * smooth_acc_balde[2]));//̧ͷΪ��
		}
		else
		{
			roll_acc = atan2(-accmean[1], -accmean[2]);
			pitch_acc = atan2(accmean[0], sqrt(accmean[1] * accmean[1] + accmean[2] * accmean[2]));//̧ͷΪ��
		}
		noise_roll[0] = 3.5*D2R;
		noise_pitch[0] = 4.5*D2R;
#endif		
#if ACC_LINEAR_NOISE  /*******************��������̬����*************/
		double roll_acc = atan2(-accmean[1], -accmean[2]);
		double pitch_acc = atan2(accmean[0], sqrt(accmean[1] * accmean[1] + accmean[2] * accmean[2]));
		//double roll_acc = atan2(-iatd.acc_balde[1], -iatd.acc_balde[2]);
		//double pitch_acc = atan2(iatd.acc_balde[0], sqrt(iatd.acc_balde[1] * iatd.acc_balde[1] + iatd.acc_balde[2] * iatd.acc_balde[2]));
		/*------------���ż��ٶ��ж�------------*/
		// 1.���ٶȼƵ�ģֵ < ��ֵ
		// 2.ǰ����Ԫ���ٶȼƼ���ĽǶȲ�ֵ < ��ֵ
		// 3.��Ӧ��ת����ٶ�ģֵ < ��ֵ
		// 4.��Ϣ < ��ֵ
		if (normacc < 0.01)  //10mg ׼��̬
		{
			num_accnorm_balde++;
		}
		else
		{
			num_accnorm_balde = 0;
		}
		if (num_accnorm_balde > 200)  //׼��̬����1��
		{
			noise_roll[0] = 1.5*D2R;
			noise_pitch[0] = 1.5*D2R;
			/********���Ӿ�̬�Ĵ����Ӽƿ���ƽ��*********/
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
/*----------�������˲����������-----------*/
		ekf_pitch.setRk(noise_pitch, 0);
		ekf_pitch.setHk(1);
		ekf_pitch.MUpdate(zk_pitch);
/*----------������˲����������-----------*/
		ekf_roll.setRk(noise_roll, 0);
		ekf_roll.setHk(1);
		ekf_roll.MUpdate(zk_roll);
/*----------��������-----------*/
/*----------�����Ƿ�������-------*/
		pitch_balde -= ekf_pitch.xk[0];
		gyobias_balde[1] += ekf_pitch.xk[1];
/*----------����Ƿ�������-------*/
		roll_balde -= ekf_roll.xk[0];
		gyobias_balde[0] += ekf_roll.xk[1];
		num_nogetbaldeimu = 0;
#if DEBUG_SAVE
		outdhf(4, "%f, %f, %f, %f, %f, %f, %f\n",
			(ekf_roll.xk[0]),(ekf_roll.xk[1]), (ekf_pitch.xk[0]),(ekf_pitch.xk[1]), gyobias_balde[0] * R2D, gyobias_balde[1] * R2D, iatd.imutime_balde);
#endif
/*-----------״̬����------------*/
		ekf_pitch.xk[0] = 0;
		ekf_pitch.xk[1] = 0;
		ekf_roll.xk[0] = 0;
		ekf_roll.xk[1] = 0;
	}
#if 1
/*--------------------------------------˫���ߺ������ݴ���-----------------------------------------*/
	if (iatd.bgnssupdate)
	{
/*----------����۲�������������-----------*/
		double zk_yaw[1] = { 0 };
		zk_yaw[0] = heading_balde - iatd.yaw_2ant;
		/***������Ϣ�жϺ����Ƿ��ٽ�任�����ٽ��򲻸���***/
		if (fabs(zk_yaw[0]) > PI)
		{
			RAD_PI_PI(zk_yaw[0]);
		}
		//double noise_yaw[1] = { iatd.gyaw_noise*3 }; //0.1 ʱ���ӳٵ�Ӱ��
		double noise_yaw[1] = { 0.3*D2R * 1 }; //0.1 ʱ���ӳٵ�Ӱ��
/*----------������˲����������-----------*/
		ekf_yaw.setRk(noise_yaw, 0);
		ekf_yaw.setHk(1);
		ekf_yaw.MUpdate(zk_yaw);
/*----------����Ƿ�������-------*/
		heading_balde -= ekf_yaw.xk[0];
		gyobias_balde[2] += ekf_yaw.xk[1];
#if DEBUG_SAVE
		outdhf(5, "%f, %f, %f, %f\n",
			sqrt(ekf_yaw.xk[0]), sqrt(ekf_yaw.xk[1]), gyobias_balde[2] * R2D, iatd.imutime_balde);
#endif
/*---------------״̬����-----------------*/
		ekf_yaw.xk[0] = 0;
		ekf_yaw.xk[1] = 0;
		num_nogetgnss = 0;
	}
	RAD_PI_PI(heading_balde);
	double atti[3] = { roll_balde,pitch_balde,heading_balde };
	a2mat_ned(atti, Cb2n_balde);
	m2qua_ned(Cb2n_balde, qua_balde);
	//������б�Ƕ�
	att2tilt(atti, &crossslope_balde, &tiltslope_balde);

/*------------------------------------------���ܴ��������ݴ���---------------------------------------------*/
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
		Mminn(gyomean, gyobias_mainfall, gyo_bias_mainfall, 3, 1); //��������ȡ��ֵ
 /*----------��̬���� - ���ٶȻ���-----------*/
#if 1  	
		double gyoincrement[3] = { 0 };
		Mmuln(gyo_bias_mainfall, 3, 1, dt_mainfallimu, gyoincrement);
		tiltslop_mainfall += gyoincrement[1];
		iatd.integ_pitch += gyoincrement[1];
#endif
/*----------�˲���ʱ�����-----------*/
/*----------���ܸ������˲���ʱ�����-----------*/
		ekf_pitch_mainfall.setPhi(iatd.imutime_balde,1);
		ekf_pitch_mainfall.TUpdate(iatd.imutime_balde, 0);
/*----------����۲�������������-----------*/
//���ܴ������������ǵ�����������ƣ��ǵ�Ƶ�źţ��ɶԼ��ٶ����ݽ���ƽ������
		double noise_pitch[1] = { 0 };
		double normacc = fabs(sqrt(iatd.acc_mainfall[0] * iatd.acc_mainfall[0] + iatd.acc_mainfall[1] * iatd.acc_mainfall[1] + iatd.acc_mainfall[2] * iatd.acc_mainfall[2]) / GN - 1.0);
#if ACC_SMOOTH_NOISE	/************���ٶȼƿ���ƽ�����˳���Ƶ�ź�***********/
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
#if ACC_LINEAR_NOISE   /*******************��������̬����*************/
		double pitch_acc_body = atan2(accmean[0], sqrt(accmean[1] * accmean[1] + accmean[2] * accmean[2]));
		if (normacc < 0.01)  //10mg ׼��̬
		{
			num_accnorm_mainfall++;
		}
		else
		{
			num_accnorm_mainfall = 0;
		}
		if (num_accnorm_mainfall > 200)  //׼��̬����1��
		{
			noise_pitch[0] = 1.5*D2R;
			/********���Ӿ�̬�Ĵ����Ӽƿ���ƽ��*********/
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
/*----------���ܸ������˲����������-----------*/
		ekf_pitch_mainfall.setRk(noise_pitch, 0);
		ekf_pitch_mainfall.setHk(1);
		ekf_pitch_mainfall.MUpdate(zk_pitch);
/*----------��������-----------*/
/*----------�����Ƿ�������-------*/
		tiltslop_mainfall -= ekf_pitch_mainfall.xk[0];
		gyobias_mainfall[1] += ekf_pitch_mainfall.xk[1];
#if DEBUG_SAVE
		outdhf(2, "%f, %f, %f,%f,%f,%f,%f\n", iatd.imutime_mainfall, tiltslop_mainfall*R2D, pitch_acc_body*R2D, iatd.integ_pitch*R2D,
			ekf_pitch_mainfall.xk[0], gyobias_mainfall[1], normacc);

		outdhf(6, "%f, %f, %f, %f\n",
			sqrt(ekf_pitch_mainfall.xk[0]), sqrt(ekf_pitch_mainfall.xk[0]), gyobias_mainfall[1] * R2D, iatd.imutime_mainfall);
#endif
/*------------״̬����-----------*/
		ekf_pitch_mainfall.xk[0] = 0;
		ekf_pitch_mainfall.xk[1] = 0;
		num_nogetbaldeimu = 0;
	}
#endif
	return 1;
}

int ATProcess::Two_Position_Install_Err(AttTrackData& iatd)
{
	//�����������װ���궨���Ӽ���ƫ�궨����������λ�÷��궨��
	if (!Acc_Bias_ok&&ACC_BIAS_CAL)
	{
/*----------------------------���������ǰ�װ���---------------------------*/
		if (!position1_ok&&iatd.balde_install_falg==1)//��λ������λ��1�궨״̬
		{
			int pos1_flag = 0;
			/*λ��1�����������복����ֱ�����������ڵ��ϣ�ͬʱ��¼��������λ�ñ�ǵ�*/
			/*λ��1���������߸���Ǧ��������ֱ������Ϊ��ǰ������װƽ�洦��ˮƽ*/
			/*λ��1������¼����ǰ���ĸ����ֵ�λ�ã�ͬʱ�ɼ�������ٶȵ�ԭʼ����*/
			/*λ��1�����ٶ���������std��ֵ����λ��1������ɣ�������λ��������־*/
			/*λ��1����λ����ʾλ��1�궨�������Ҫת������λ��*/
			pos1_flag = cal.Acc_Bias_2pos_cal(iatd, 1);
			if (pos1_flag)
			{
				position1_ok = true;//λ��1�������
				blade_install_result = 1;//������λ��
			}
		}
/*----------------------------��������ǰ�װ���---------------------------*/
		if (position1_ok&&!position2_ok&&iatd.balde_install_falg==2)//��λ������λ��2�궨״̬
		{
			int pos2_flag = 0;
			/*λ��2������ת��180�ȣ���������λ����λ��1��ǵ��غϣ�ͬʱ���������ڵ���*/
			/*λ��2����λ�����ͱ궨״̬���ɼ�λ��2���ݣ��ɼ���Ϸ���״̬����λ����ʾ*/
			/*λ��2���궨�ɹ�����λ����ʾ����ǰ�װ���궨�ɹ�����λ����ʾת����һ��λ��*/
			pos2_flag = cal.Acc_Bias_2pos_cal(iatd, 2);
			if (pos2_flag)
			{
				position2_ok = true;//λ��2�������
				blade_install_result = 2;//������λ��
			}
		}
/*----------------------------���帩���ǰ�װ���---------------------------*/
		if (position1_ok&&position2_ok&&!position3_ok&&iatd.balde_install_falg==3)//��λ������λ��3�궨״̬
		{
			int pos3_flag = 0;
			/*λ��3����������λ�ã�����ǰ���ĸ�������λ��1��ǳ���λ���غ�*/
			/*λ��3����λ�����ͱ궨״̬���ɼ�λ��3���ݣ��ɼ���Ϸ���״̬����λ����ʾ*/
			/*λ��3���궨�ɹ�����λ����ʾ�����ǽǰ�װ���궨�ɹ�����λ����ʾ�궨���*/
			pos3_flag = cal.Acc_Bias_2pos_cal(iatd, 3);
			if (pos3_flag)
			{
				position3_ok = true;//λ��3�������
			}
		}
		if (position1_ok&&position2_ok&&position3_ok)
		{
			int install_flag = 0;
			install_flag=cal.Acc_Bias_2pos_cal(iatd, 4);
			if (install_flag)
			{
				Acc_Bias_ok = true;//˫λ�ð�װ�궨��ɣ��践����λ���궨�ɹ�״̬
				cfg.balde_install_roll = cal.blade_installangle[0];
				cfg.balde_install_pitch = cal.blade_installangle[1];
				cfg.mainfall_install_pitch = cal.mainfall_installangle[1];
				blade_install_result = 3;//������λ��
			}
		}
	}
	if (Acc_Bias_ok)
	{
		double acc_bias[3] = {cfg.abiasx,cfg.abiasy,cfg.abiasz };//������ٶ���ƫ��Ҫ��λ�÷������궨
		cal.Comp_Acc_bias(&iatd, acc_bias);//3��Ӽ���ƫ����
		printf("balde_install_err:%f,%f\n", cfg.balde_install_roll*R2D, cfg.balde_install_pitch*R2D);
		/*������װ����*/
		cal.Comp_InstallErr_Acc(iatd.acc_balde, cfg.balde_install_roll, cfg.balde_install_pitch);//�����������װƫ���
		cal.Comp_InstallErr_Acc(iatd.gyo_balde, cfg.balde_install_roll, cfg.balde_install_pitch);
		/*���尲װ����*/
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
	if (option == 0)//����IMU���ٶ�ƽ��
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
	else if (option == 1)//����IMU���ٶ�ƽ��
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
	if (!rotation_zero_ok&&Heading_2ant_Err&&iatd.rotation_install_flag==1)//�ĳ���λ�������ı�־
	{
		int cal_flag = 0;
		cal_flag = cal.InstallErr_Rotation(iatd.rotation_balde);//˫���߰�װ���궨
		if (cal_flag)
		{
			rotation_zero = cal.installrotation;
			rotation_zero_ok = true;//�践����λ���궨�ɹ�״̬
			rotation_install_result = 1;//������λ����־
			return 1;
		}
		else
		{
			rotation_install_result = 0;//������λ����־
			return 0;
		}
	}
	else
	{
		return 0;
	}
}