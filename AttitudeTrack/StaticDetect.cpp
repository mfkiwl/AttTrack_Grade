#include "StaticDetect.h"
#include "ComFunc.h"
StaticDetect::StaticDetect(int winlen, double thread)
{
	N = winlen;
	accvar = 0;
	gyovar = 0;
	thre = thread;
	bfinshinit = 0;
	num_gnssstatic = 0;
}

int StaticDetect::calstaticstd(double acc[], double gyo[], int bgnssupdate, double gpsvn[], int option)
{
	if (bfinshinit)
	{
		return 1;
	}
	if (vax.size() < N)
	{
		vax.push_back(acc[0]);
		vay.push_back(acc[1]);
		vaz.push_back(acc[2]);
		vgx.push_back(gyo[0] * R2D);
		vgy.push_back(gyo[1] * R2D);
		vgz.push_back(gyo[2] * R2D);
	}
	else
	{
		vax.push_back(acc[0]);
		vay.push_back(acc[1]);
		vaz.push_back(acc[2]);
		vgx.push_back(gyo[0] * R2D);
		vgy.push_back(gyo[1] * R2D);
		vgz.push_back(gyo[2] * R2D);
		vax.erase(vax.begin());
		vay.erase(vay.begin());
		vaz.erase(vaz.begin());
		vgx.erase(vgx.begin());
		vgy.erase(vgy.begin());
		vgz.erase(vgz.begin());
	}
	if (bgnssupdate)
	{
		double gspeed = sqrt(gpsvn[0] * gpsvn[0] + gpsvn[1] * gpsvn[1] + gpsvn[2] * gpsvn[2]);
		if (gspeed < 0.05)
		{
			num_gnssstatic++;
			if (num_gnssstatic > 5) //连续静止>5秒
			{
				//计算观测噪声
				double stdax = GetAveStd(vax, 1);
				double stday = GetAveStd(vay, 1);
				double stdaz = GetAveStd(vaz, 1);
				double stdgx = GetAveStd(vgx, 1);
				double stdgy = GetAveStd(vgy, 1);
				double stdgz = GetAveStd(vgz, 1);
				accvar = (stdax*stdax + stday*stday + stdaz*stdaz) / 3;
				gyovar = (stdgx*stdgx + stdgy*stdgy + stdgz*stdgz) / 3; //观测噪声方差
																		//计算阈值
				if (option == 0)
				{
					double meanax = GetAveStd(vax, 0);
					double meanay = GetAveStd(vay, 0);
					double meanaz = GetAveStd(vaz, 0);
					double accmeannorm = sqrt(meanax*meanax + meanay*meanay + meanaz*meanaz);
					double normmeanax = meanax*glv_g0 / accmeannorm;
					double normmeanay = meanay*glv_g0 / accmeannorm;
					double normmeanaz = meanaz*glv_g0 / accmeannorm;
					double thread = 0.0;
					for (int i = 0; i < vax.size(); i++)
					{
						thread += ((vax[i] - normmeanax)*(vax[i] - normmeanax) + (vay[i] - normmeanay)*(vay[i] - normmeanay) + (vaz[i] - normmeanaz)*(vaz[i] - normmeanaz)) / accvar
							+ (vgx[i] * vgx[i] + vgy[i] * vgy[i] + vgz[i] * vgz[i]) / gyovar;
					}
					thread /= N;
					thre = thread * 1.5;
				}
				if (option == 1)
				{
					double thread = 0.0;
					for (int i = 0; i < vax.size(); i++)
					{
						thread += vgx[i] * vgx[i] + vgy[i] * vgy[i] + vgz[i] * vgz[i];
					}
					thread /= (gyovar*N);
					thre = thread * 1.5;
				}
				if (option == 2)
				{
					double meanax = GetAveStd(vax, 0);
					double meanay = GetAveStd(vay, 0);
					double meanaz = GetAveStd(vaz, 0);
					double thread = 0.0;
					for (int i = 0; i < vax.size(); i++)
					{
						thread += (vax[i] - meanax)*(vax[i] - meanax) + (vay[i] - meanay)*(vay[i] - meanay) + (vaz[i] - meanaz)*(vaz[i] - meanaz);
					}
					thread /= (accvar*N);
					thre = thread * 1.5;
				}
				if (option == 3)
				{
					double thread = 0.0;
					for (int i = 0; i < vax.size(); i++)
					{
						thread += (sqrt(vax[i] * vax[i] + vay[i] * vay[i] + vaz[i] * vaz[i]) - glv_g0)*(sqrt(vax[i] * vax[i] + vay[i] * vay[i] + vaz[i] * vaz[i]) - glv_g0);
					}
					thread /= (accvar*N);
					thre = thread * 1.5;
				}
				bfinshinit = 1;
			}
		}
		else
		{
			num_gnssstatic = 0;
			vax.clear();
			vay.clear();
			vaz.clear();
			vgx.clear();
			vgy.clear();
			vgz.clear();
		}
	}
}

int StaticDetect::detect_SHOE(double acc[], double gyo[], int bgnssupdate, double gpsvn[])
{
	if (!bfinshinit) //未完成观测噪声的计算
	{
		calstaticstd(acc, gyo, bgnssupdate, gpsvn, 0);
		return 0;
	}
	if (vax.size() < N)
	{
		vax.push_back(acc[0]);
		vay.push_back(acc[1]);
		vaz.push_back(acc[2]);
		vgx.push_back(gyo[0] * R2D);
		vgy.push_back(gyo[1] * R2D);
		vgz.push_back(gyo[2] * R2D);
		return 0;
	}
	else
	{
		vax.push_back(acc[0]);
		vay.push_back(acc[1]);
		vaz.push_back(acc[2]);
		vgx.push_back(gyo[0] * R2D);
		vgy.push_back(gyo[1] * R2D);
		vgz.push_back(gyo[2] * R2D);
		vax.erase(vax.begin());
		vay.erase(vay.begin());
		vaz.erase(vaz.begin());
		vgx.erase(vgx.begin());
		vgy.erase(vgy.begin());
		vgz.erase(vgz.begin());
	}
	double meanax = GetAveStd(vax, 0);
	double meanay = GetAveStd(vay, 0);
	double meanaz = GetAveStd(vaz, 0);
	double accmeannorm = sqrt(meanax*meanax + meanay*meanay + meanaz*meanaz);
	double normmeanax = meanax*glv_g0 / accmeannorm;
	double normmeanay = meanay*glv_g0 / accmeannorm;
	double normmeanaz = meanaz*glv_g0 / accmeannorm;
	double thread = 0.0;
	for (int i = 0; i < vax.size(); i++)
	{
		thread += ((vax[i] - normmeanax)*(vax[i] - normmeanax) + (vay[i] - normmeanay)*(vay[i] - normmeanay) + (vaz[i] - normmeanaz)*(vaz[i] - normmeanaz)) / accvar
			+ (vgx[i] * vgx[i] + vgy[i] * vgy[i] + vgz[i] * vgz[i]) / gyovar;
	}
	thread /= N;
	int ret = 0;
	if (thread < thre)
	{
		ret = 1;
	}
	double speed = sqrt(gpsvn[0] * gpsvn[0] + gpsvn[1] * gpsvn[1] + gpsvn[2] * gpsvn[2]);
	outdhf(0, "%f,%f,%f,%d\n", thread, speed, thre, ret);
	return ret;
}

int StaticDetect::detect_ARED(double acc[], double gyo[], int bgnssupdate, double gpsvn[])
{
	if (!bfinshinit) //未完成观测噪声的计算
	{
		calstaticstd(acc, gyo, bgnssupdate, gpsvn, 1);
		return 0;
	}
	if (vax.size() < N)
	{
		vax.push_back(acc[0]);
		vay.push_back(acc[1]);
		vaz.push_back(acc[2]);
		vgx.push_back(gyo[0] * R2D);
		vgy.push_back(gyo[1] * R2D);
		vgz.push_back(gyo[2] * R2D);
		return 0;
	}
	else
	{
		vax.push_back(acc[0]);
		vay.push_back(acc[1]);
		vaz.push_back(acc[2]);
		vgx.push_back(gyo[0] * R2D);
		vgy.push_back(gyo[1] * R2D);
		vgz.push_back(gyo[2] * R2D);
		vax.erase(vax.begin());
		vay.erase(vay.begin());
		vaz.erase(vaz.begin());
		vgx.erase(vgx.begin());
		vgy.erase(vgy.begin());
		vgz.erase(vgz.begin());
	}
	double thread = 0.0;
	for (int i = 0; i < vax.size(); i++)
	{
		thread += vgx[i] * vgx[i] + vgy[i] * vgy[i] + vgz[i] * vgz[i];
	}
	thread /= (gyovar*N);
	int ret = 1;
	if (thread < thre)
	{
		ret = 0;
	}
	double speed = sqrt(gpsvn[0] * gpsvn[0] + gpsvn[1] * gpsvn[1] + gpsvn[2] * gpsvn[2]);
	outdhf(0, "%f,%f,%f,%d\n", thread, speed, thre, ret);
	return ret;
}

int StaticDetect::detect_AMVD(double acc[], double gyo[], int bgnssupdate, double gpsvn[])
{
	if (!bfinshinit) //未完成观测噪声的计算
	{
		calstaticstd(acc, gyo, bgnssupdate, gpsvn, 2);
		return 0;
	}
	if (vax.size() < N)
	{
		vax.push_back(acc[0]);
		vay.push_back(acc[1]);
		vaz.push_back(acc[2]);
		vgx.push_back(gyo[0] * R2D);
		vgy.push_back(gyo[1] * R2D);
		vgz.push_back(gyo[2] * R2D);
		return 0;
	}
	else
	{
		vax.push_back(acc[0]);
		vay.push_back(acc[1]);
		vaz.push_back(acc[2]);
		vgx.push_back(gyo[0] * R2D);
		vgy.push_back(gyo[1] * R2D);
		vgz.push_back(gyo[2] * R2D);
		vax.erase(vax.begin());
		vay.erase(vay.begin());
		vaz.erase(vaz.begin());
		vgx.erase(vgx.begin());
		vgy.erase(vgy.begin());
		vgz.erase(vgz.begin());
	}
	double meanax = GetAveStd(vax, 0);
	double meanay = GetAveStd(vay, 0);
	double meanaz = GetAveStd(vaz, 0);
	double thread = 0.0;
	for (int i = 0; i < vax.size(); i++)
	{
		thread += (vax[i] - meanax)*(vax[i] - meanax) + (vay[i] - meanay)*(vay[i] - meanay) + (vaz[i] - meanaz)*(vaz[i] - meanaz);
	}
	thread /= (accvar*N);
	int ret = 1;
	if (thread < thre)
	{
		ret = 0;
	}
	double speed = sqrt(gpsvn[0] * gpsvn[0] + gpsvn[1] * gpsvn[1] + gpsvn[2] * gpsvn[2]);
	outdhf(0, "%f,%f,%f,%d\n", thread, speed, thre, ret);
	return ret;
}

int StaticDetect::detect_AMD(double acc[], double gyo[], int bgnssupdate, double gpsvn[])
{
	if (!bfinshinit) //未完成观测噪声的计算
	{
		calstaticstd(acc, gyo, bgnssupdate, gpsvn, 3);
		return 0;
	}
	if (vax.size() < N)
	{
		vax.push_back(acc[0]);
		vay.push_back(acc[1]);
		vaz.push_back(acc[2]);
		vgx.push_back(gyo[0] * R2D);
		vgy.push_back(gyo[1] * R2D);
		vgz.push_back(gyo[2] * R2D);
		return 0;
	}
	else
	{
		vax.push_back(acc[0]);
		vay.push_back(acc[1]);
		vaz.push_back(acc[2]);
		vgx.push_back(gyo[0] * R2D);
		vgy.push_back(gyo[1] * R2D);
		vgz.push_back(gyo[2] * R2D);
		vax.erase(vax.begin());
		vay.erase(vay.begin());
		vaz.erase(vaz.begin());
		vgx.erase(vgx.begin());
		vgy.erase(vgy.begin());
		vgz.erase(vgz.begin());
	}
	double thread = 0.0;
	for (int i = 0; i < vax.size(); i++)
	{
		thread += (sqrt(vax[i] * vax[i] + vay[i] * vay[i] + vaz[i] * vaz[i]) - glv_g0)*(sqrt(vax[i] * vax[i] + vay[i] * vay[i] + vaz[i] * vaz[i]) - glv_g0);
	}
	thread /= (accvar*N);
	int ret = 1;
	if (thread < thre)
	{
		ret = 0;
	}
	double speed = sqrt(gpsvn[0] * gpsvn[0] + gpsvn[1] * gpsvn[1] + gpsvn[2] * gpsvn[2]);
	outdhf(0, "%f,%f,%f,%d\n", thread, speed, thre, ret);
	return ret;
}

int detectstatic_std(double acc[], double gyo[])
{
	static vector<double> accnorm, gyonorm;  
	static int num = 0;
	static int numturn = 0;
	static int bstatic = 0;
	double iaccn = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
	double igyon = sqrt(gyo[0] * gyo[0] + gyo[1] * gyo[1] + gyo[2] * gyo[2]);
	if (accnorm.size() < 100) //1秒或0.5秒
	{
		accnorm.push_back(iaccn);
		gyonorm.push_back(igyon);
		return 0;
	}
	else
	{
		accnorm.erase(accnorm.begin());
		gyonorm.erase(gyonorm.begin());
		accnorm.push_back(iaccn);
		gyonorm.push_back(igyon);
	}
	double anstd = GetAveStd(accnorm, 1);
	double gnstd = GetAveStd(gyonorm, 1);
	//if (anstd < 0.008 && gnstd < 0.001)
	if (anstd < 0.015 && gnstd < 0.003)
	{
		num++;
		numturn = 0;
		if (num > 5)
		{
			bstatic = 1;
		}
		if (num > 100001)
		{
			num = 101;
		}
	}
	else
	{
		numturn++;
		if (numturn > 3)
		{
			num = 0;
			bstatic = 0;
		}
		if (numturn > 100001)
		{
			numturn = 101;
		}
	}
	//outdhf(10, "%f,%f,%d\n", anstd, gnstd,bstatic);
	return bstatic;
}
