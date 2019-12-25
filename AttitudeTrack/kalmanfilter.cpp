//#include "stdafx.h"
#include "kalmanfilter.h"
//EKF
int EKF::setxk(double xk0[], int nox)
{
	for (int i = 0; i <nox; i++)
	{
		xk[i][0] = xk0[i];
	}
	return 0;
}

int EKF::setPxk(double dpxk0[], int nox)
{
	for (int i = 0; i <nox; i++)
	{
		Pxk[i][i] = dpxk0[i] * dpxk0[i];
	}
	return 0;
}

int EKF::setPhi(double dt)
{
	Phi[0][0] = 1;
	Phi[0][1] = dt;
	Phi[1][0] = 0;
	Phi[1][1] = 1;
	return 0;
}


int EKF::setQk(double Qnoise[], int nox)
{
	for (int i = 0; i <nox; i++)
	{
		Qk[i][i] = Qnoise[i] * Qnoise[i];
	}
	return 0;
}

int EKF::setHk(int num, int option)
{
	Hk[0][0] = 1;
	Hk[0][1] = 0;
	return 0;
}

int EKF::setRk(double Rnoise[], int noz)
{
	for (int i = 0; i < noz; i++)
	{
		Rk[i][i] = Rnoise[i] * Rnoise[i];
	}
	return 0;
}

int EKF::setzk(double zki[], int noz)
{
	for (int i = 0; i < noz; i++)
	{
		zk[i][0] = zki[i];
	}
	return 0;
}

int EKF::TUpdate(double dt,int nox)
{
	//xk = Phi*xk;
	double  xk_temp[2] = { 0 };
	double  Pxk_temp[2][2] = { 0 };
	for (int i = 0; i < nox; i++)
	{
		xk_temp[i] = Phi[i][0] * xk[0][0] + Phi[i][1] * xk[1][0];
	}

	for (int i = 0; i < nox; i++)
	{
		xk[i][0] = xk_temp[i];
	}
	//Pxk = Phi*Pxk*(Phi.t()) + Qk*dt;
	//printf("xk_temp:%f,%f\n", xk_temp[0], xk_temp[1]);
	for (int i = 0; i < nox; i++)
	{
		Pxk_temp[i][0] = Phi[i][0] * Pxk[0][0] + Phi[i][1] * Pxk[1][0];
		Pxk_temp[i][1] = Phi[i][0] * Pxk[0][1] + Phi[i][1] * Pxk[1][1];
	}
	for (int i = 0; i < nox; i++)
	{
		Pxk[i][0] = Pxk_temp[i][0] * Phi[0][0] + Pxk_temp[i][1] * Phi[0][1] + Qk[i][0] * dt;
		Pxk[i][1] = Pxk_temp[i][0] * Phi[1][0] + Pxk_temp[i][1] * Phi[1][1] + Qk[i][1] * dt;
	}
	//free(Phi);
	//free(Qk);
	return 0;
}

int EKF::MUpdate(double Rnoise[], double zki[],int nox)
{
	//mat xk_1 = xk;
	double xk_1[2] = { 0 };
	for (int i = 0; i < nox; i++)
	{
		xk_1[i] = xk[i][0];
	}

	//mat Pxk_1 = Pxk;
	double Pxk_1[2][2] = { 0 };
	for (int i = 0; i < nox; i++)
	{
		Pxk_1[i][0] = Pxk[i][0];
		Pxk_1[i][1] = Pxk[i][1];
	}
	//mat PHt = Pxk_1*(Hk.t());
	double PHt[2][1] = { 0 };
	for (int i = 0; i < nox; i++)
	{
		PHt[i][0] = Pxk_1[i][0] * Hk[0][0] + Pxk_1[i][1] * Hk[0][1];
	}
	//mat HPHtR = Hk*PHt + Rk;
	double HPHtR = 0;
	HPHtR = Hk[0][0] * PHt[0][0] + Hk[0][1] * PHt[1][0] + Rk[0][0];


	//mat Kk = PHt*inv(HPHtR);
	double Kk[2] = { 0 };
	Kk[0] = PHt[0][0] * (1 / HPHtR);//多维矩阵求逆？
	Kk[1] = PHt[1][0] * (1 / HPHtR);

	//// test
	//mat kxk = Kk*(zk - Hk*xk_1);

	//xk = xk_1 + Kk*(zk - Hk*xk_1);
	for (int i = 0; i < nox; i++)
	{
		xk[i][0] = xk_1[i] + Kk[i] * (zk[0][0] - (Hk[0][0] * xk_1[0] + Hk[0][1] * xk_1[1]));
	}


	//Pxk = Pxk_1 - Kk*HPHtR*(Kk.t());
	for (int i = 0; i < nox; i++)
	{
		Pxk[i][0] = Pxk_1[i][0] - Kk[i] * HPHtR*Kk[0];
		Pxk[i][1] = Pxk_1[i][1] - Kk[i] * HPHtR*Kk[1];

	}
	//Pxk = (Pxk + Pxk.t())*0.5;

	double Pxk_2[2][2] = { 0 };
	for (int i = 0; i < nox; i++)
	{
		Pxk_2[i][0] = Pxk[i][0];
		Pxk_2[i][1] = Pxk[i][1];
	}
	for (int i = 0; i < nox; i++)
	{
		Pxk[i][0] = (Pxk_2[i][0] + Pxk_2[0][i]) / 2;
		Pxk[i][1] = (Pxk_2[i][1] + Pxk_2[1][i]) / 2;
	}
	//free(Hk);//释放内存
	//free(zk);
	//free(Rk);
	return 0;
}
//KF
void KF::kfmalloc(int row, int col)
{
	ROW = row;
	COL = col;
	xk = (double*)__ml_zero(sizeof(double)*ROW * 1);
	xkpre = (double*)__ml_zero(sizeof(double)*ROW * 1);
	Pxk = (double*)__ml_zero(sizeof(double)*ROW*ROW);
	Phi = (double*)__ml_zero(sizeof(double)*ROW*ROW);
	Qk = (double*)__ml_zero(sizeof(double)*ROW*ROW);
	Hk = (double*)__ml_zero(sizeof(double)*COL*ROW);
	Rk = (double*)__ml_zero(sizeof(double)*COL*COL);
}
void KF::setRk(double Rnoise[],int option)
{
	if (option == 0)
	{
		for (int i = 0; i<COL; i++)
		{
			Rk[i*COL + i] = SQR(Rnoise[i]);
		}
	}
}
void KF::setHk(int option)
{
	if (option == 0)
	{
		Hk[0 * ROW + 2] = 1;
		Hk[1 * ROW + 1] = 1;
		Hk[2 * ROW + 0] = 1;
	}
	else if (option == 1)
	{
		Hk[0 * ROW + 0] = 1;
		Hk[1 * ROW + 0] = 1;
	}

}
void KF::setPxk(double xk0[])
{
	for (int i = 0; i<ROW; i++)
	{
		Pxk[i*ROW + i] = SQR(xk0[i]);
	}
}
void KF::setPhi(double dt,int option)
{
	if (option == 0)
	{
		Mequal(Phi, ROW, ROW, 0);
		Phi[0 * ROW + 0] = 1; Phi[0 * ROW + 1] = dt; Phi[0 * ROW + 2] = 0.5*dt*dt; Phi[0 * ROW + 3] = -0.5*dt*dt;
		Phi[1 * ROW + 1] = 1; Phi[1 * ROW + 2] = dt; Phi[1 * ROW + 3] = -dt;
		Phi[2 * ROW + 2] = 1;
		Phi[3 * ROW + 3] = 1;
	}
	else if (option == 1)
	{
		Mequal(Phi, ROW, ROW, 0);
		Phi[0 * ROW + 0] = 1; Phi[0 * ROW + 1] = dt;
		Phi[1 * ROW + 0] = 0; Phi[1 * ROW + 1] = 1;
	}

}
void KF::setXk(double xk0[])
{
	for (int i = 0; i<ROW; i++)
	{
		xk[i] = xk0[i];
	}
}
void KF::setQk(double Qnoise[])
{
	for (int i = 0; i<ROW; i++)
	{
		Qk[i*ROW + i] = SQR(Qnoise[i]);
	}
}
void KF::kfinit()
{
	memset(xk, 0, sizeof(double)*ROW * 1);
	memset(xkpre, 0, sizeof(double)*ROW * 1);
	memset(Phi, 0, sizeof(double)*ROW*ROW);
	memset(Hk, 0, sizeof(double)*COL*ROW);
	memset(Rk, 0, sizeof(double)*COL * COL);
	memset(Pxk, 0, sizeof(double)*ROW*ROW);
	memset(Qk, 0, sizeof(double)*ROW*ROW);
}

void KF:: TUpdate(double dt, int option)
{
	if (!option)
	{
		double* temp = (double*)__ml_zero(sizeof(double)*ROW * 1);
		double* temp1 = (double*)__ml_zero(sizeof(double)*ROW*ROW);
		double* temp2 = (double*)__ml_zero(sizeof(double)*ROW*ROW);

		/*dsf90:xk  = Phi X xk*/
		Mmulnm(Phi, xk, ROW, ROW, 1, temp);
		Mequalm(temp, ROW, 1, xk);
		Mmulnm(Phi, Pxk, ROW, ROW, ROW, temp1);
		Mtn(Phi, ROW, ROW, temp2);
		Mmulnm(temp1, temp2, ROW, ROW, ROW, Pxk);
#if 1
		/*dsf90:Pxk = Phi X Pxk X Mt(Phi) + Qk*dt*/
		Mmuln(Qk, ROW, ROW, dt, temp1);
		Madd(Pxk, temp1, ROW, ROW);
#else   
		/*dsf90:修正：Pxk = Phi X Pxk X Mt(Phi) + Qk*dt^2*/
		Mmuln(Qk, ROW, ROW, dt*dt, temp1);
		Madd(Pxk, temp1, ROW, ROW);
#endif
		//printf("xk:%f,%f,%f,%f\n", xk[0], xk[1], xk[2], xk[3]);
		//printf("Qk:%f,%f,%f,%f\n", Qk[0], Qk[5], Qk[10], Qk[15]);
		//printf("Pxk:%f,%f,%f,%f\n", sqrt(Pxk[0]), sqrt(Pxk[5]), sqrt(Pxk[10]), sqrt(Pxk[15]));
		gilc_free(temp);  temp = NULL;
		gilc_free(temp1); temp1 = NULL;
		gilc_free(temp2); temp2 = NULL;
	}
}

void KF::MUpdate(double ZK[])
{
	double* xkk_1 = (double*)__ml_zero(sizeof(double)*ROW * 1);
	double* Pxkk_1 = (double*)__ml_zero(sizeof(double)*ROW*ROW);
	double* Pxykk_1 = (double*)__ml_zero(sizeof(double)*ROW*COL);
	double* Pykk_1 = (double*)__ml_zero(sizeof(double)*COL*COL);
	double* Kk = (double*)__ml_zero(sizeof(double)*ROW*COL);
	double* ykk_1 = (double*)__ml_zero(sizeof(double)*COL * 1);
	double* dxk = (double*)__ml_zero(sizeof(double)*ROW * 1);
	double* Hkt = (double*)__ml_zero(sizeof(double)*ROW*COL);
	double* invPy = (double*)__ml_zero(sizeof(double)*COL*COL);
	double* KPyKt = (double*)__ml_zero(sizeof(double)*ROW*ROW);

	Mequalm(xk, ROW, 1, xkk_1);
	Mequalm(Pxk, ROW, ROW, Pxkk_1);

	Mtn(Hk, COL, ROW, Hkt);
	Mmulnm(Pxkk_1, Hkt, ROW, ROW, COL, Pxykk_1);
	Mmulnm(Hk, Pxykk_1, COL, ROW, COL, Pykk_1);
	Madd(Pykk_1, Rk, COL, COL);
	Minvn(Pykk_1, COL, invPy);
	Mmulnm(Pxykk_1, invPy, ROW, COL, COL, Kk);
	Mmulnm(Hk, xkk_1, COL, ROW, 1, ykk_1);
	Mminn(ZK, ykk_1, ykk_1, COL, 1);
	Mmulnm(Kk, ykk_1, ROW, COL, 1, dxk);
	Maddn(xkk_1, dxk, xk, ROW, 1);

	double* KkHk = (double*)__ml_zero(sizeof(double)*ROW*ROW);
	double* KkHkPxkk_1 = (double*)__ml_zero(sizeof(double)*ROW*ROW);

	/*dsf90:Pxk  	= Pxk - Kk X Hk X Pxk*/
	Mmulnm(Kk, Hk, ROW, COL, ROW, KkHk);
	Mmulnm(KkHk, Pxkk_1, ROW, ROW, ROW, KkHkPxkk_1);
	Mminn(Pxkk_1, KkHkPxkk_1, Pxk, ROW, ROW);
	//Mmul(Pxk,ROW,ROW,0.995);

	gilc_free(KkHk);       KkHk = NULL;
	gilc_free(KkHkPxkk_1); KkHkPxkk_1 = NULL;
	/*end add*/
	gilc_free(xkk_1);
	gilc_free(Pxkk_1);
	gilc_free(Pxykk_1);
	gilc_free(Pykk_1);
	gilc_free(Kk);
	gilc_free(ykk_1);
	gilc_free(dxk);
	gilc_free(Hkt);
	gilc_free(invPy);
	gilc_free(KPyKt);
	xkk_1 = NULL;
	Pxkk_1 = NULL;
	Pxykk_1 = NULL;
	Pykk_1 = NULL;
	Kk = NULL;
	ykk_1 = NULL;
	dxk = NULL;
	Hkt = NULL;
	invPy = NULL;
	KPyKt = NULL;
}
void KF::kffree()
{
	if (xk != NULL)
	{
		gilc_free(xk); xk = NULL;
	}
	if (xkpre != NULL)
	{
		gilc_free(xkpre); xkpre = NULL;
	}
	if (Phi != NULL)
	{
		gilc_free(Phi); Phi = NULL;
	}
	if (Hk != NULL)
	{
		gilc_free(Hk); Hk = NULL;
	}
	if (Pxk != NULL)
	{
		gilc_free(Pxk); Pxk = NULL;
	}
	if (Qk != NULL)
	{
		gilc_free(Qk); Qk = NULL;
	}
	if (Rk != NULL)
	{
		gilc_free(Rk); Rk = NULL;
	}
}
