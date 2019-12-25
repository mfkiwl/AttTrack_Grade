#ifndef __KALMANFILTER_H__
#define __KALMANFILTER_H__
#include <math.h>
#include "ComFunc.h"
#include "Matrix.h"
#define ALT_NUMX 4
#define ALT_NUMV 3
#define ATT_NUMX 2
#define ATT_NUMV 1
#define SQR(x)      ((x)*(x))
//×ËÌ¬¸ú×ÙEKF
class EKF
{
public:
	double **xk;
	double **Pxk;
	double **Phi;
	double **Qk;
	double **Hk;
	double **Rk;
	double **zk;
public:
	void InitialMatrix_xk(int nox, int noz);
	void InitialMatrix_Pxk(int nox, int noz);
	void InitialMatrix_Phi(int nox, int noz);
	void InitialMatrix_Qk(int nox, int noz);
	void InitialMatrix_Hk(int nox, int noz);
	void InitialMatrix_Rk(int nox, int noz);
	void InitialMatrix_zk(int nox, int noz);
	int setxk(double xk0[], int nox);
	int setPxk(double dpxk0[], int nox);
	int setPhi(double dt);
	int setQk(double Qnoise[], int nox);
	int setHk(int num, int option);
	int setRk(double Rnoise[], int noz);
	int setzk(double zki[], int noz);
	int TUpdate(double dt, int nox);
	int MUpdate(double Rnoise[], double zki[] , int nox);
};
//¸ß³ÌÈÚºÏKF
class KF
{
public:
	int ROW, COL;
	double* xk;
	double* Pxk;
	double* Phi;
	double* Qk;
	double* Hk;
	double* Rk;
	double* xkpre;
	void kfmalloc(int row, int col);                               
	void kffree();                    
	void kfinit();                                      
	void setPxk(double xk0[]);
	void setPhi(double dt, int option);
	void setQk(double Qnoise[]);
	void setXk(double xk0[]);
	void setHk(int option);
	void setRk(double Rnoise[],int option);
	void TUpdate(double dt, int option);
	void MUpdate(double ZK[]);
};
#endif
