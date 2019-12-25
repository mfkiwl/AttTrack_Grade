#include "Matrix.h"
//EKF
void EKF::InitialMatrix_xk(int nox,int noz)
{
	int m =nox;
	int n =noz;
	int i,j;
	xk = (double**)malloc(m*sizeof(double*));//行分配内存
	for (i = 0;i < m;i++)
	{
		xk[i] = (double*)malloc(n*sizeof(double));//列分配内存
	}

		for (i = 0;i<m;i++)
		{
			for (j = 0;j <n;j++)
			{
				xk[i][0] =0;
			}
		}
}
void EKF::InitialMatrix_Pxk(int nox, int noz)
{
	int m =nox;
	int n =noz;
	int i,j;
	Pxk = (double**)malloc(m*sizeof(double*));
	for (i = 0;i<m;i++)
		Pxk[i] = (double*)malloc(m*sizeof(double));
	for (i = 0;i<m;i++)
	{
		for (j = 0;j < m;j++)
		{
			Pxk[i][j] = 0;
		}
	}
}
void EKF::InitialMatrix_Phi(int nox, int noz)
{
	int m = nox;
	int n = noz;
	int i,j;
	Phi = (double**)malloc(m*sizeof(double*));
	for (i = 0;i<2;i++)
		Phi[i] = (double*)malloc(m*sizeof(double));
	for (i = 0;i<m;i++)
	{
		for (j = 0;j < m;j++)
		{
			Phi[i][j] = 0;
		}
	}
}
void EKF::InitialMatrix_Qk(int nox, int noz)
{
	int m = nox;
	int n = noz;
	int i,j;
	Qk = (double**)malloc(m*sizeof(double*));
	for (i = 0;i<m;i++)
		Qk[i] = (double*)malloc(m*sizeof(double));
	for (i = 0;i<m;i++)
	{
		for (j = 0;j < m;j++)
		{
			Qk[i][j] = 0;
		}
	}
}
void EKF::InitialMatrix_Hk(int nox, int noz)
{
	int m =nox;
	int n = noz;
	int i,j;
	Hk = (double**)malloc(m*sizeof(double*));
	for (i = 0;i<m;i++)
		Hk[i] = (double*)malloc(n*sizeof(double));
	for (i = 0;i<n;i++)
	{
		for (j = 0;j < m;j++)
		{
			Hk[i][j] = 0;
		}
	}
}
void EKF::InitialMatrix_Rk(int nox, int noz)
{
	int m = nox;
	int n = noz;
	int i,j;
	Rk = (double**)malloc(n*sizeof(double*));
	for (i = 0;i<n;i++)
		Rk[i] = (double*)malloc(n*sizeof(double));
	for (i = 0;i<n;i++)
	{
		for (j = 0;j < n;j++)
		{
			Rk[i][j] = 0;
		}
	}
}
void EKF::InitialMatrix_zk(int nox, int noz)
{
	int m = nox;
	int n = noz;
	int i;
	zk = (double**)malloc(n*sizeof(double*));
	for (i = 0;i<n;i++)
		zk[i] = (double*)malloc(n*sizeof(double));
	for (i = 0;i<n;i++)
	{
			zk[i][0] = 0;
	}
}




