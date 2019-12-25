#include "Calibrate.h"

void Cal_Install_Error::Cal_Install_Error_init(void)
{
	winlen = 6000;//ȡ��ֹ1min���ݣ�����Ƶ��100hz��
	winlen1 = 6000;
	max1 = may1 = maz1 = max2 = may2 = maz2 =max3=may3=maz3= 0;
	sax1 = say1 = saz1 = sax2 = say2 = saz2 =sax3=say3=saz3= GN;
	std_head = std_rotat = 0;
	mean_head = mean_rotat = 0;
	blade_installangle[0] = 0;
	blade_installangle[1] = 0;
	mainfall_installangle[0] = 0;
	mainfall_installangle[1] = 0;
	installyaw = 0;
	accx_bias = 0;
	accy_bias = 0;
	accz_bias = 0;
}
int Cal_Install_Error::Acc_Bias_2pos_cal(AttTrackData atdata, int option)
{
		if (option == 1)
		{
			if (ax1.size() < winlen)
			{
				ax1.push_back(atdata.acc_balde[0]);
				ay1.push_back(atdata.acc_balde[1]);
				az1.push_back(atdata.acc_balde[2]);
				return 0;  //��һλ�ò���δ���
			}
			else
			{
				max1 = GetAveStd(ax1, 0);
				sax1 = GetAveStd(ax1, 1);
				may1 = GetAveStd(ay1, 0);
				say1 = GetAveStd(ay1, 1);
				maz1 = GetAveStd(az1, 0);
				saz1 = GetAveStd(az1, 1);
				if (sax1 < 0.001*GN && say1 < 0.001*GN && saz1 < 0.001*GN)
				{
					return 1;  //��һλ�ò������
				}
				else
				{
					ax1.clear();
					ay1.clear();
					az1.clear();
					return 0;  //��һλ��std����
				}
			}
		}
		if (option == 2)
		{
			if (ax2.size() < winlen)
			{
				ax2.push_back(atdata.acc_balde[0]);
				ay2.push_back(atdata.acc_balde[1]);
				az2.push_back(atdata.acc_balde[2]);
				return 0;  //�ڶ�λ�ò���δ���
			}
			else
			{
				max2 = GetAveStd(ax2, 0);
				sax2 = GetAveStd(ax2, 1);
				may2 = GetAveStd(ay2, 0);
				say2 = GetAveStd(ay2, 1);
				maz2 = GetAveStd(az2, 0);
				saz2 = GetAveStd(az2, 1);
				if (sax2 < 0.001*GN && say2 < 0.001*GN && saz2 < 0.001*GN)
				{
					return 1;  //�ڶ�λ�ò������
				}
				else
				{
					ax2.clear();
					ay2.clear();
					az2.clear();
					return 0;  //�ڶ�λ��std����
				}
			}
		}
		if (option == 3)
		{
			if (ax3.size() < winlen)
			{
				ax3.push_back(atdata.acc_balde[0]);
				ay3.push_back(atdata.acc_balde[1]);
				az3.push_back(atdata.acc_balde[2]);
				return 0;  //����λ�ò���δ���
			}
			else
			{
				max3 = GetAveStd(ax3, 0);
				sax3 = GetAveStd(ax3, 1);
				may3 = GetAveStd(ay3, 0);
				say3 = GetAveStd(ay3, 1);
				maz3 = GetAveStd(az3, 0);
				saz3 = GetAveStd(az3, 1);
				if (sax3 < 0.001*GN && say3 < 0.001*GN && saz3 < 0.001*GN)
				{
					return 1;  //����λ�ò������
				}
				else
				{
					ax3.clear();
					ay3.clear();
					az3.clear();
					return 0;  //����λ��std����
				}
			}
		}
		if (option == 4)
		{
			/*��ǰ������ϵ*/
			double balde_pitch_install = atan2(max1, sqrt(maz1*maz1+ may1*may1));//λ��1
			double balde_roll_install= (asin(-may1 / GN) + asin(-may2 / GN)) / 2;//λ��1+λ��2
			double mainfall_pitch_install= (asin(-max1 / GN) + asin(-max3 / GN)) / 2;//λ��1+λ��3
			blade_installangle[0] = balde_roll_install;
			blade_installangle[1] = balde_pitch_install;
			mainfall_installangle[1] = mainfall_pitch_install;
			printf("blade_roll_err:%f  blade_pitch_err:%f mainfall_pitch_err:%f \r\n",
				balde_roll_install, balde_pitch_install, mainfall_pitch_install);
			config_set_balde_install_roll(balde_roll_install*R2D);				   //�궨���д�������ļ�
			config_set_balde_install_pitch(balde_pitch_install*R2D);               //�궨���д�������ļ�
			config_set_mainfall_install_pitch(mainfall_pitch_install*R2D);         //�궨���д�������ļ�
			return 1;                                      //��װ��������� 
		}
		return 0;
}
int Cal_Install_Error::InstallErr_Rotation(double rotation)
{
	if (rotat1.size() < winlen1)
	{
		rotat1.push_back(rotation);
		return 0;
	}
	else
	{
		std_rotat = GetAveStd(rotat1, 1);
		mean_rotat = GetAveStd(rotat1, 0);
		if (fabs(std_rotat) < 0.5)//��ֹʱ��ת������std��ֵ
		{
			installrotation =mean_rotat;//��ת��������λֵ
			config_set_rotation_install(installrotation);//�궨���д�������ļ�
			return 1;
		}
		else
		{
			rotat1.clear();
			return 0;
		}
	}
}

void Cal_Install_Error::Comp_Acc_bias(AttTrackData* atdata, double acc_bias[3])
{
	for (int i = 0; i < 3; i++)
	{
		atdata->acc_balde[i] -= acc_bias[i];
	}
}
void Cal_Install_Error::Comp_InstallErr_Acc(double acc[3], double installroll, double installpitch)
{
	double cphi = cos(installroll); double sphi = sin(installroll);
	double cthe = cos(installpitch); double sthe = sin(installpitch);
	//calculate levering matrix
	double C3[9] = { 0 }, C2[9] = { 0 };
	/*��x����ת �����*/
	C3[0 * 3 + 0] = 1.0;
	C3[1 * 3 + 1] = cphi;  C3[1 * 3 + 2] = sphi;
	C3[2 * 3 + 1] = -sphi; C3[2 * 3 + 2] = cphi;
	/*��y����ת ������*/
	C2[0 * 3 + 0] = cthe;                        C2[0 * 3 + 2] = -sthe;
	                       C2[1 * 3 + 1] = 1.0;
	C2[2 * 3 + 0] = sthe;                        C2[2 * 3 + 2] = cthe;

	double Clevel[3 * 3] = { 0.0 }, Ctemp[3 * 3] = { 0.0 };
	Mmulnm(C3, C2, 3, 3, 3, Ctemp);//NEDϵ ��ת˳�� Cnb=XYZ
	Mtn(Ctemp, 3, 3, Clevel);

	double acc_comp[3] = { 0.0 };
	Mmulnm(Clevel, acc, 3, 3, 1, acc_comp);

	Mequalm(acc_comp, 3, 1, acc);
}