#include "ComFunc.h"

/*-----------------log define by DHF ----------------------*/
static FILE *fpoutprocess_balde = NULL;
static FILE *fpoutprocess_mainfall = NULL;
static FILE *fpoutprocess_elv = NULL;
static FILE *fpoutraw = NULL;
static FILE *fpoutkf_rp_balde = NULL;
static FILE *fpoutkf_yawb = NULL;
static FILE *fpoutkf_pitchm = NULL;
static FILE *fpoutkf_elv = NULL;
static FILE *fpouttest = NULL;
static FILE *fpout_ymtest = NULL;
static FILE *fpout_gnss_pos = NULL;
extern void fopendhf(const char *file)
{
	char fprocess1[128] = { 0 };
	sprintf(fprocess1, "%sprocess_balde.txt", file);
	if (!*file || !(fpoutprocess_balde = fopen(fprocess1, "w")))
	{
		printf("can not open outfile-process crated by dhf!\n");
		fpoutprocess_balde = NULL;
	}

	char fprocess2[128] = { 0 };
	sprintf(fprocess2, "%sprocess_mainfall.txt", file);
	if (!*file || !(fpoutprocess_mainfall = fopen(fprocess2, "w")))
	{
		printf("can not open outfile-process crated by dhf!\n");
		fpoutprocess_mainfall = NULL;
	}

	char fprocess_elv[128] = { 0 };
	sprintf(fprocess_elv, "%sprocess_elv.txt", file);
	if (!*file || !(fpoutprocess_elv = fopen(fprocess_elv, "w")))
	{
		printf("can not open outfile-process crated by dhf!\n");
		fpoutprocess_elv = NULL;
	}

	char fraw[128] = { 0 };
	sprintf(fraw, "%srawdata.txt", file);
	if (!*file || !(fpoutraw = fopen(fraw, "w")))
	{
		printf("can not open outfile-test crated by dhf\n!");
		fpoutraw = NULL;
	}

	char fkf1[128] = { 0 };
	sprintf(fkf1, "%skf_balde_rp.txt", file);
	if (!*file || !(fpoutkf_rp_balde = fopen(fkf1, "w")))
	{
		printf("can not open outfile-test crated by dhf\n!");
		fpoutkf_rp_balde = NULL;
	}

	char fkf3[128] = { 0 };
	sprintf(fkf3, "%skf_balde_yaw.txt", file);
	if (!*file || !(fpoutkf_yawb = fopen(fkf3, "w")))
	{
		printf("can not open outfile-test crated by dhf\n!");
		fpoutkf_yawb = NULL;
	}

	char fkf4[128] = { 0 };
	sprintf(fkf4, "%skf_mainfall_pitch.txt", file);
	if (!*file || !(fpoutkf_pitchm = fopen(fkf4, "w")))
	{
		printf("can not open outfile-test crated by dhf\n!");
		fpoutkf_pitchm = NULL;
	}

	char fkf5[128] = { 0 };
	sprintf(fkf5, "%skf_elv.txt", file);
	if (!*file || !(fpoutkf_elv = fopen(fkf5, "w")))
	{
		printf("can not open outfile-test crated by dhf\n!");
		fpoutkf_elv = NULL;
	}

	char ftest[128] = { 0 };
	sprintf(ftest, "%stest.txt", file);
	if (!*file || !(fpouttest = fopen(ftest, "w")))
	{
		printf("can not open outfile-test crated by dhf\n!");
		fpouttest = NULL;
	}
	char ftest_ym[128] = { 0 };
	sprintf(ftest_ym, "%symtest.txt", file);
	if (!*file || !(fpout_ymtest = fopen(ftest_ym, "w")))
	{
		printf("can not open outfile-test crated by ym\n!");
		fpout_ymtest = NULL;
	}
	char fgnss_pos[128] = { 0 };
	sprintf(fgnss_pos, "%sgnss_pos.txt", file);
	if (!*file || !(fpout_gnss_pos = fopen(fgnss_pos, "w")))
	{
		printf("can not open outfile-test crated by ym\n!");
		fpout_gnss_pos = NULL;
	}
}
extern void fclosedhf(void)
{
	if (fpoutprocess_balde)
	{
		fclose(fpoutkf_rp_balde);
		fpoutkf_rp_balde = NULL;
	}
	if (fpoutprocess_mainfall)
	{
		fclose(fpoutprocess_mainfall);
		fpoutprocess_mainfall = NULL;
	}
	if (fpoutprocess_elv)
	{
		fclose(fpoutprocess_elv);
		fpoutprocess_elv = NULL;
	}
	if (fpoutraw)
	{
		fclose(fpoutraw);
		fpoutraw = NULL;
	}
	if (fpoutkf_rp_balde)
	{
		fclose(fpoutkf_rp_balde);
		fpoutkf_rp_balde = NULL;
	}
	if (fpoutkf_yawb)
	{
		fclose(fpoutkf_yawb);
		fpoutkf_yawb = NULL;
	}
	if (fpoutkf_pitchm)
	{
		fclose(fpoutkf_pitchm);
		fpoutkf_pitchm = NULL;
	}
	if (fpoutkf_elv)
	{
		fclose(fpoutkf_elv);
		fpoutkf_elv = NULL;
	}
	if (fpouttest)
	{
		fclose(fpouttest);
		fpouttest = NULL;
	}
	if (fpout_ymtest)
	{
		fclose(fpout_ymtest);
		fpout_ymtest = NULL;
	}
	if (fpout_gnss_pos)
	{
		fclose(fpout_gnss_pos);
		fpout_gnss_pos = NULL;
	}
}

extern void outdhf(int option, const char *format, ...)
{
	va_list ap;
	switch (option)
	{
	case 0:
		if (!fpoutraw) return;
		va_start(ap, format); vfprintf(fpoutraw, format, ap); va_end(ap);
		fflush(fpoutraw);
		break;
	case 1:
		if (!fpoutprocess_balde) return;
		va_start(ap, format); vfprintf(fpoutprocess_balde, format, ap); va_end(ap);
		fflush(fpoutprocess_balde);
		break;
	case 2:
		if (!fpoutprocess_mainfall) return;
		va_start(ap, format); vfprintf(fpoutprocess_mainfall, format, ap); va_end(ap);
		fflush(fpoutprocess_mainfall);
		break;
	case 3:
		if (!fpoutprocess_elv) return;
		va_start(ap, format); vfprintf(fpoutprocess_elv, format, ap); va_end(ap);
		fflush(fpoutprocess_elv);
		break;
	case 4:
		if (!fpoutkf_rp_balde) return;
		va_start(ap, format); vfprintf(fpoutkf_rp_balde, format, ap); va_end(ap);
		fflush(fpoutkf_rp_balde);
		break;
	case 5:
		if (!fpoutkf_yawb) return;
		va_start(ap, format); vfprintf(fpoutkf_yawb, format, ap); va_end(ap);
		fflush(fpoutkf_yawb);
		break;
	case 6:
		if (!fpoutkf_pitchm) return;
		va_start(ap, format); vfprintf(fpoutkf_pitchm, format, ap); va_end(ap);
		fflush(fpoutkf_pitchm);
		break;
	case 7:
		if (!fpoutkf_elv) return;
		va_start(ap, format); vfprintf(fpoutkf_elv, format, ap); va_end(ap);
		fflush(fpoutkf_elv);
		break;
	case 10:
		if (!fpouttest) return;
		va_start(ap, format); vfprintf(fpouttest, format, ap); va_end(ap);
		fflush(fpouttest);
		break;
	case 11:
		if (!fpout_ymtest) return;
		va_start(ap, format); vfprintf(fpout_ymtest, format, ap); va_end(ap);
		fflush(fpout_ymtest);
		break;
	case 12:
		if (!fpout_gnss_pos) return;
		va_start(ap, format); vfprintf(fpout_gnss_pos, format, ap); va_end(ap);
		fflush(fpout_gnss_pos);
		break;
	default:
		break;
	}
}

extern void Str2Array(const char *str, const char *sep, double *val)
{
	char *p, _str[1024];
	double d[MAXVAL] = { 0.0 };
	int i, j;

	strcpy(_str, str);
	for (i = 0, p = strtok(_str, sep); p&&i<MAXVAL; p = strtok(NULL, sep), i++) {
		d[i] = atof(p);
	}

	for (j = 0; j<i; j++) val[j] = d[j];
}
extern int checkstr(const char* str, const char* substr)
{
	int i, j, check, count = 0;
	int len = (int)strlen(str);        /*取得字符串长度，不包括'\0'*/
	int sublen = (int)strlen(substr);
	if (len<sublen)
	{
		return 0;
	}
	for (i = 0; i<len; i++)
	{
		check = 1;                /*检测标记*/
		for (j = 0; j + i<len&&j<sublen; j++)        /*逐个字符进行检测，在sublen长度内，一旦出现不同字符便将check置为0*/
		{
			if (str[i + j] != substr[j])
			{
				check = 0;
				break;
			}
		}
		if (check == 1)           /*在sublen长度内的字符都相等*/
		{
			break;
			//count++;
			//i=i+sublen;           /*调整检测起始位置*/
		}
	}
	return i;
}

extern double GetAveStd(vector<double> a, int opt)
{
	int n = (int)a.size();
	double avg = 0.0, std = 0.0, rms = 0.0, sum = 0.0;

	if (n == 0) return 99999.9;

	for (int i = 0; i<n; i++) {
		sum += a[i];
		rms += a[i] * a[i];
	}
	avg = sum / n;

	if (opt == 0) return avg;

	sum = 0.0;
	for (int i = 0; i<n; i++) {
		sum += (a[i] - avg)*(a[i] - avg);
	}

	std = sqrt(sum / double(n - 1));
	rms = sqrt(rms / double(n));

	if (opt == 1) return std;
	else if (opt == 2) return rms;

	return 0.0;
}

extern void matmul(const char *tr, int n, int k, int m, double alpha,
	const double *A, const double *B, double beta, double *C)
{
	double d;
	int i, j, x, f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);

	for (i = 0; i<n; i++) for (j = 0; j<k; j++) {
		d = 0.0;
		switch (f) {
		case 1: for (x = 0; x<m; x++) d += A[i + x*n] * B[x + j*m]; break;
		case 2: for (x = 0; x<m; x++) d += A[i + x*n] * B[j + x*k]; break;
		case 3: for (x = 0; x<m; x++) d += A[x + i*m] * B[x + j*m]; break;
		case 4: for (x = 0; x<m; x++) d += A[x + i*m] * B[j + x*k]; break;
		}
		if (beta == 0.0) C[i + j*n] = alpha*d; else C[i + j*n] = alpha*d + beta*C[i + j*n];
	}
}
/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
extern double dot(const double *a, const double *b, int n)
{
	double c = 0.0;

	while (--n >= 0) c += a[n] * b[n];
	return c;
}
/* transform ecef to geodetic postion ------------------------------------------
* transform ecef position to geodetic position
* args   : double *r        I   ecef position {x,y,z} (m)
*          double *pos      O   geodetic position {lat,lon,h} (rad,m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
extern void ecef2pos(const double *r, double *pos)
{
	double e2 = FE_WGS84*(2.0 - FE_WGS84), r2 = dot(r, r, 2), z, zk, v = RE_WGS84, sinp;

	for (z = r[2], zk = 0.0; fabs(z - zk) >= 1E-4;) {
		zk = z;
		sinp = z / sqrt(r2 + z*z);
		v = RE_WGS84 / sqrt(1.0 - e2*sinp*sinp);
		z = r[2] + v*e2*sinp;
	}
	pos[0] = r2>1E-12 ? atan(z / sqrt(r2)) : (r[2]>0.0 ? PI / 2.0 : -PI / 2.0);
	pos[1] = r2>1E-12 ? atan2(r[1], r[0]) : 0.0;
	pos[2] = sqrt(r2 + z*z) - v;
}
/* transform geodetic to ecef position -----------------------------------------
* transform geodetic position to ecef position
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *r        O   ecef position {x,y,z} (m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
extern void pos2ecef(const double *pos, double *r)
{
	double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);
	double e2 = FE_WGS84*(2.0 - FE_WGS84), v = RE_WGS84 / sqrt(1.0 - e2*sinp*sinp);

	r[0] = (v + pos[2])*cosp*cosl;
	r[1] = (v + pos[2])*cosp*sinl;
	r[2] = (v*(1.0 - e2) + pos[2])*sinp;
}
/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
extern void xyz2enu(const double *pos, double *E)
{
	double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);

	E[0] = -sinl;      E[3] = cosl;       E[6] = 0.0;
	E[1] = -sinp*cosl; E[4] = -sinp*sinl; E[7] = cosp;
	E[2] = cosp*cosl;  E[5] = cosp*sinl;  E[8] = sinp;
}
/* transform ecef vector to local tangental coordinate -------------------------
* transform ecef vector to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *r        I   vector in ecef coordinate {x,y,z}
*          double *e        O   vector in local tangental coordinate {e,n,u}
* return : none
*-----------------------------------------------------------------------------*/
extern void ecef2enu(const double *pos, const double *r, double *e)
{
	double E[9];

	xyz2enu(pos, E);
	matmul("NN", 3, 1, 3, 1.0, E, r, 0.0, e);
}
/* transform local vector to ecef coordinate -----------------------------------
* transform local tangental coordinate vector to ecef
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *e        I   vector in local tangental coordinate {e,n,u}
*          double *r        O   vector in ecef coordinate {x,y,z}
* return : none
*-----------------------------------------------------------------------------*/
extern void enu2ecef(const double *pos, const double *e, double *r)
{
	double E[9];

	xyz2enu(pos, E);
	matmul("TN", 3, 1, 3, 1.0, E, e, 0.0, r);
}

void Maddn(double *a, double *b, double *c, int m, int n)
{
	for (int i = 0; i<m; i++)
		for (int j = 0; j<n; j++)
		{
			*(c + i*n + j) = *(a + i*n + j) + *(b + i*n + j);
		}
}

void Mminn(double *a, double *b, double *c, int m, int n)
{
	int i, j;

	for (i = 0; i<m; i++)
		for (j = 0; j<n; j++)
		{
			*(c + i*n + j) = *(a + i*n + j) - *(b + i*n + j);
		}
}

void Mmulnm(double *a, double *b, int m, int n, int k, double *c)
{
	int i, j, l, u;
	for (i = 0; i <= m - 1; i++)
		for (j = 0; j <= k - 1; j++)
		{
			u = i*k + j;
			c[u] = 0.0;
			for (l = 0; l <= n - 1; l++)
				c[u] = c[u] + a[i*n + l] * b[l*k + j];
		}
}

void Mmul(double *a, int m, int n, double b)
{
	for (int i = 0; i<m; i++)
	{
		for (int j = 0; j<n; j++)
		{
			*(a + i*n + j) = *(a + i*n + j)*b;
		}
	}
}

void Mmuln(double *a, int m, int n, double b, double* c)
{
	for (int i = 0; i<m; i++)
	{
		for (int j = 0; j<n; j++)
		{
			*(c + i*n + j) = *(a + i*n + j)*b;
		}
	}
}

void Mtn(double *a, int m, int n, double *b)
{
	for (int l = 0; l<n; l++)
		for (int k = 0; k<m; k++)
		{
			b[l*m + k] = a[k*n + l];
		}

}

void Mequalm(double *M, int m, int n, double *N)
{
	for (int i = 0; i<m; i++)
	{
		for (int j = 0; j<n; j++)
		{
			*(N + i*n + j) = *(M + i*n + j);
		}
	}
}

void askew(double v[], double m[])
{
	m[0 * 3 + 0] = 0.0;   m[0 * 3 + 1] = -v[2]; m[0 * 3 + 2] = v[1];
	m[1 * 3 + 0] = v[2];  m[1 * 3 + 1] = 0.0;   m[1 * 3 + 2] = -v[0];
	m[2 * 3 + 0] = -v[1]; m[2 * 3 + 1] = v[0];  m[2 * 3 + 2] = 0.0;
}

void m2qua_ned(double m[], double q[])
{
	double s[5] = { 0.0 };
	s[4] = m[0 * 3 + 0] + m[1 * 3 + 1] + m[2 * 3 + 2];
	s[0] = 1.0 + s[4];
	s[1] = 1.0 + 2.0*m[0 * 3 + 0] - s[4];
	s[2] = 1.0 + 2.0*m[1 * 3 + 1] - s[4];
	s[3] = 1.0 + 2.0*m[2 * 3 + 2] - s[4];
	int index = 0;
	double max = s[0];
	for (int k = 1; k<4; k++)
	{
		if (s[k]>max)
		{
			index = k;
			max = s[k];
		}
	}
	switch (index)
	{
	case 0:
		q[0] = 0.5*sqrt(s[0]);
		q[1] = 0.25*(m[2 * 3 + 1] - m[1 * 3 + 2]) / q[0];
		q[2] = 0.25*(m[0 * 3 + 2] - m[2 * 3 + 0]) / q[0];
		q[3] = 0.25*(m[1 * 3 + 0] - m[0 * 3 + 1]) / q[0];
		break;
	case 1:
		q[1] = 0.5*sqrt(s[1]);
		q[2] = 0.25*(m[1 * 3 + 0] + m[0 * 3 + 1]) / q[1];
		q[3] = 0.25*(m[0 * 3 + 2] + m[2 * 3 + 0]) / q[1];
		q[0] = 0.25*(m[2 * 3 + 1] - m[1 * 3 + 2]) / q[1];
		break;
	case 2:
		q[2] = 0.5*sqrt(s[2]);
		q[3] = 0.25*(m[2 * 3 + 1] + m[1 * 3 + 2]) / q[2];
		q[0] = 0.25*(m[0 * 3 + 2] - m[2 * 3 + 0]) / q[2];
		q[1] = 0.25*(m[1 * 3 + 0] + m[0 * 3 + 1]) / q[2];
		break;
	case 3:
		q[3] = 0.5*sqrt(s[3]);
		q[0] = 0.25*(m[1 * 3 + 0] - m[0 * 3 + 1]) / q[3];
		q[1] = 0.25*(m[0 * 3 + 2] + m[2 * 3 + 0]) / q[3];
		q[2] = 0.25*(m[2 * 3 + 1] + m[1 * 3 + 2]) / q[3];
		break;
	}
}

void q2mat_ned(double qua[], double m[])
{
	double q11, q12, q13, q14, q22, q23, q24, q33, q34, q44;
	q11 = qua[0] * qua[0]; q12 = qua[0] * qua[1]; q13 = qua[0] * qua[2]; q14 = qua[0] * qua[3];
	q22 = qua[1] * qua[1]; q23 = qua[1] * qua[2]; q24 = qua[1] * qua[3];
	q33 = qua[2] * qua[2]; q34 = qua[2] * qua[3];
	q44 = qua[3] * qua[3];
	m[0 * 3 + 0] = q11 + q22 - q33 - q44; m[0 * 3 + 1] = 2 * (q23 - q14);     m[0 * 3 + 2] = 2 * (q24 + q13);
	m[1 * 3 + 0] = 2 * (q23 + q14);     m[1 * 3 + 1] = q11 - q22 + q33 - q44; m[1 * 3 + 2] = 2 * (q34 - q12);
	m[2 * 3 + 0] = 2 * (q24 - q13);     m[2 * 3 + 1] = 2 * (q34 + q12);     m[2 * 3 + 2] = q11 - q22 - q33 + q44;
}

void m2att_ned(double m[], double a[])
{
	a[0] = atan2(m[2 * 3 + 1], m[2 * 3 + 2]);
	a[1] = asin(-m[2 * 3 + 0]);
	a[2] = atan2(m[1 * 3 + 0], m[0 * 3 + 0]);
}

void m2att_ned(double m[], double* roll, double* pitch, double* yaw)
{
	*roll  = atan2(m[2 * 3 + 1], m[2 * 3 + 2]);
	*pitch = asin(-m[2 * 3 + 0]);
	*yaw   = atan2(m[1 * 3 + 0], m[0 * 3 + 0]);
}

void a2mat_ned(double att[], double m[])
{
	double phi = att[0];
	double theta = att[1];
	double psi = att[2];
	double cpsi = cos(psi); double spsi = sin(psi);
	double cthe = cos(theta); double sthe = sin(theta);
	double cphi = cos(phi); double sphi = sin(phi);
	double C1[9] = { 0.0 };
	double C2[9] = { 0.0 };
	double C3[9] = { 0.0 };
	double m1[9] = { 0.0 };
	double m2[9] = { 0.0 };
	C1[0 * 3 + 0] = cpsi;  C1[0 * 3 + 1] = spsi;
	C1[1 * 3 + 0] = -spsi; C1[1 * 3 + 1] = cpsi;
	C1[2 * 3 + 2] = 1.0;
	C2[0 * 3 + 0] = cthe;                  C2[0 * 3 + 2] = -sthe;
	C2[1 * 3 + 1] = 1.0;
	C2[2 * 3 + 0] = sthe;                  C2[2 * 3 + 2] = cthe;
	C3[0 * 3 + 0] = 1.0;
	C3[1 * 3 + 1] = cphi; C3[1 * 3 + 2] = sphi;
	C3[2 * 3 + 1] = -sphi; C3[2 * 3 + 2] = cphi;
	Mmulnm(C3, C2, 3, 3, 3, m1);
	Mmulnm(m1, C1, 3, 3, 3, m2);
	Mtn(m2, 3, 3, m);
}

void a2mat_ned(double roll, double pitch, double yaw, double m[])
{
	double phi = roll;
	double theta = pitch;
	double psi = yaw;
	double cpsi = cos(psi); double spsi = sin(psi);
	double cthe = cos(theta); double sthe = sin(theta);
	double cphi = cos(phi); double sphi = sin(phi);
	double C1[9] = { 0.0 };
	double C2[9] = { 0.0 };
	double C3[9] = { 0.0 };
	double m1[9] = { 0.0 };
	double m2[9] = { 0.0 };
	C1[0 * 3 + 0] = cpsi;  C1[0 * 3 + 1] = spsi;
	C1[1 * 3 + 0] = -spsi; C1[1 * 3 + 1] = cpsi;
	C1[2 * 3 + 2] = 1.0;
	C2[0 * 3 + 0] = cthe;                  C2[0 * 3 + 2] = -sthe;
	C2[1 * 3 + 1] = 1.0;
	C2[2 * 3 + 0] = sthe;                  C2[2 * 3 + 2] = cthe;
	C3[0 * 3 + 0] = 1.0;
	C3[1 * 3 + 1] = cphi; C3[1 * 3 + 2] = sphi;
	C3[2 * 3 + 1] = -sphi; C3[2 * 3 + 2] = cphi;
	Mmulnm(C3, C2, 3, 3, 3, m1);
	Mmulnm(m1, C1, 3, 3, 3, m2);
	Mtn(m2, 3, 3, m);
}

void qupdt(double qnb0[], double rv_nb[])
{
	double qnb1[4] = { 0.0 };
	double n2, c, s, n, n_2, nq;
	n2 = rv_nb[0] * rv_nb[0] + rv_nb[1] * rv_nb[1] + rv_nb[2] * rv_nb[2];
	if (n2<1.0e-8)
	{
		c = 1.0 - n2*(1.0 / 8.0 - n2 / 384.0);
		s = 1.0 / 2 - n2*(1.0 / 48.0 - n2 / 3840.0);
	}
	else
	{
		n = sqrt(n2); n_2 = n / 2;
		c = cos(n_2); s = sin(n_2) / n;
	}
	double q2[4] = { 0.0 };
	q2[0] = c; q2[1] = s*rv_nb[0]; q2[2] = s*rv_nb[1]; q2[3] = s*rv_nb[2];
	// q = qmul(q1, q2);
	qnb1[0] = qnb0[0] * q2[0] - qnb0[1] * q2[1] - qnb0[2] * q2[2] - qnb0[3] * q2[3];
	qnb1[1] = qnb0[0] * q2[1] + qnb0[1] * q2[0] + qnb0[2] * q2[3] - qnb0[3] * q2[2];
	qnb1[2] = qnb0[0] * q2[2] + qnb0[2] * q2[0] + qnb0[3] * q2[1] - qnb0[1] * q2[3];
	qnb1[3] = qnb0[0] * q2[3] + qnb0[3] * q2[0] + qnb0[1] * q2[2] - qnb0[2] * q2[1];
	// normalization
	n2 = qnb1[0] * qnb1[0] + qnb1[1] * qnb1[1] + qnb1[2] * qnb1[2] + qnb1[3] * qnb1[3];
	if (n2>1.000001 || n2<0.999999)
	{
		nq = 1.0 / sqrt(n2);
		qnb1[0] = qnb1[0] * nq; qnb1[1] = qnb1[1] * nq; qnb1[2] = qnb1[2] * nq; qnb1[3] = qnb1[3] * nq;
	}
	for (int i = 0; i < 4; i++)
	{
		qnb0[i] = qnb1[i];
	}
}

void att2tilt(double att[], double* roll, double* pitch)
{
	*roll = atan2(sin(att[0])*cos(att[1]), sqrt(sin(att[0])*sin(att[0])*sin(att[1])*sin(att[1]) + cos(att[0])*cos(att[0])));
	*pitch = att[1];
}
#if 0
int Cal_InstallErr_Acc(vector<double> vaccx, vector<double> vaccy, vector<double> vaccz, double Install_Acc[2])
{
	if (vaccx.size() < 100) return 0;
	sort(vaccx.begin(), vaccx.end());
	sort(vaccy.begin(), vaccy.end());
	sort(vaccz.begin(), vaccz.end());
	double axmean = 0, aymean = 0, azmean = 0, axstd = 0, aystd = 0, azstd = 0;
	int num = vaccx.size();
	for (int i = 1; i < num - 1; i++)
	{
		axmean += vaccx[i];
		aymean += vaccy[i];
		azmean += vaccz[i];
	}
	axmean /= (num - 2);
	aymean /= (num - 2);
	azmean /= (num - 2);
	for (int i = 1; i < num - 1; i++)
	{
		axstd += (vaccx[i] - axmean)*(vaccx[i] - axmean);
		aystd += (vaccy[i] - aymean)*(vaccy[i] - aymean);
		azstd += (vaccz[i] - azmean)*(vaccz[i] - azmean);
	}
	axstd = sqrt(axstd / (num - 3));
	aystd = sqrt(aystd / (num - 3));
	azstd = sqrt(azstd / (num - 3));

	if (axstd > 0.1 || aystd > 0.1 || azstd > 0.1)
	{
		printf("水平校正阶段加计std超限！\n");
		vaccx.clear();
		vaccy.clear();
		vaccz.clear();
		return 0;
	}

	//计算加计的安装误差角

	double roll = 0.0, pitch = 0.0;
	roll = atan(aymean / azmean);
	pitch = atan(axmean / sqrt(aymean*aymean + azmean*azmean));

	Install_Acc[0] = roll;
	Install_Acc[1] = pitch;
	return 1;  //加计安装误差计算完成
	
}

void Comp_InstallErr_Acc(double acc[3], double Install_Acc[2])
{
	double roll = Install_Acc[0];
	double pitch = Install_Acc[1];
	//calculate levering matrix
	double Cr[3 * 3] = { 1.0,       0.0,       0.0,
		                 0.0, cos(roll), sin(roll),
		                 0.0, -sin(roll), cos(roll) };
	double Cp[3 * 3] = { cos(pitch), 0.0, -sin(pitch),
		                        0.0, 1.0,         0.0,
		                 sin(pitch), 0.0,  cos(pitch) };
	double Clevel[3 * 3] = { 0.0 }, Ctemp[3 * 3] = { 0.0 };
	Mmulnm(Cr, Cp, 3, 3, 3, Ctemp);
	Mtn(Ctemp, 3, 3, Clevel);
	double acc_comp[3] = { 0.0 };
	Mmulnm(Clevel, acc, 3, 3, 1, acc_comp);
	for (int i = 0; i < 3; i++)
	{
		acc[i] = acc_comp[i];
	}
}
#endif

extern void cal_lever_ned(double att[3], double lever_b[3], double lever_ned[3])
{
	double roll = att[0], pitch = att[1], yaw = att[2];
	double cr = cos(roll), sr = sin(roll), cp = cos(pitch), sp = sin(pitch), cy = cos(yaw), sy = sin(yaw);
	double Cr[3 * 3] = { 1.0, 0.0, 0.0,
		                 0.0, cr, sr,
		                 0.0, -sr, cr };
	double Cp[3 * 3] = { cp, 0.0, -sp,
		                 0.0, 1.0, 0.0,
		                 sp, 0.0, cp };
	double Cy[3 * 3] = { cy, sy, 0.0,
		                -sy, cy, 0.0,
		                0.0, 0.0, 1.0 };
	double Ctemp[3 * 3] = { 0.0 };
	double Cn2b[3 * 3] = { 0.0 }, Cb2n[3 * 3] = { 0.0 };
	Mmulnm(Cr, Cp, 3, 3, 3, Ctemp);
	Mmulnm(Ctemp, Cy, 3, 3, 3, Cn2b);
	Mtn(Cn2b, 3, 3, Cb2n);

	Mmulnm(Cb2n, lever_b, 3, 3, 1, lever_ned);
}

extern void lever_compen(double blh_ant[3], double lever_ned[3], double blh_target[3], int option)
{
	double lever_ECEF[3] = { 0.0 }, ant_ECEF[3] = { 0.0 }, target_ECEF[3] = { 0.0 };
	double a = RE_WGS84, f = FE_WGS84, b = (1 - f)*a;
	double e = sqrt(a*a - b*b) / a, e2 = e*e;
	double sl = sin(blh_ant[0]), cl = cos(blh_ant[0]);
	double tl = sl / cl, sl2 = sl*sl;
	double sq = 1 - e2*sl*sl, sq2 = sqrt(sq);
	double RMh = a*(1 - e2) / sq / sq2 + blh_ant[2];
	double RNh = a / sq2 + blh_ant[2];
	double clRNh = cl*RNh;
	double Mpv[3 * 3] = { 0.0 }, lever_blh[3] = { 0.0 };

	switch (option)
	{
	case 0:
		//ECEF2BLH
		pos2ecef(blh_ant, ant_ECEF);
		//printf("blh_ant:%f,%f,%f\n", blh_ant[0]*R2D, blh_ant[1] * R2D, blh_ant[2]);
		enu2ecef(blh_ant, lever_ned, lever_ECEF);
		Maddn(ant_ECEF, lever_ECEF, target_ECEF, 3, 1);
		//printf("lever_ECEF:%f,%f,%f\n", lever_ECEF[0], lever_ECEF[1], lever_ECEF[2]);
		ecef2pos(target_ECEF, blh_target);
		break;
	case 1:
		//ENU m2rad
		Mpv[0 * 3 + 1] = 1.0 / RMh;
		Mpv[1 * 3 + 0] = 1.0 / clRNh;
		Mpv[2 * 3 + 2] = 1.0;
		Mmulnm(Mpv, lever_ned, 3, 3, 1, lever_blh);
		Maddn(blh_ant, lever_blh, blh_target, 3, 1);
		break;
	default:
		break;
	}
}
extern void Lever_ver(double pos[3],double att[3], double wib[3], double vn1[3], double vn2[3], double lever[3])
{
	double CW[9] = {0.0};
	double askeww[3 * 3] = { 0.0 };
	double tempvel[3] = { 0.0 };
	double wnb[3] = { 0.0 };
	double Cnb[3 * 3] = { 0.0 };
	double Cbn[3 * 3] = { 0.0 };
	double Ctwie[3 * 1] = { 0.0 };
	double roll = att[0], pitch = att[1], yaw = att[2];
	double cr = cos(roll), sr = sin(roll), cp = cos(pitch), sp = sin(pitch), cy = cos(yaw), sy = sin(yaw);
	double Cr[3 * 3] = { 1.0, 0.0, 0.0,
		0.0, cr, sr,
		0.0, -sr, cr };
	double Cp[3 * 3] = { cp, 0.0, -sp,
		0.0, 1.0, 0.0,
		sp, 0.0, cp };
	double Cy[3 * 3] = { cy, sy, 0.0,
		-sy, cy, 0.0,
		0.0, 0.0, 1.0 };
	double Ctemp[3 * 3] = { 0.0 };
	Mmulnm(Cr, Cp, 3, 3, 3, Ctemp);//NED系坐标转换Cnb=XYZ
	Mmulnm(Ctemp, Cy, 3, 3, 3, Cnb);
	Mtn(Cnb, 3, 3, Cbn);
	/*dsf90:wbnb    = wbib - Cbn X wnin;*/
	/*dsf90:CW	   = Cnb X (wbnb X);*/
	double wnie[3] = { 0.0 };
	double wnen[3] = { 0.0 };
	double wnin[3] = { 0.0 };
	double wie = 7.2921151467e-5;
	double RE = 6378137.0;
	double sl = sin(pos[0]); double cl = cos(pos[0]); double tl = sl / cl;
	double a = RE;
	double f = (1.0 / 298.257);
	double b = (1 - f)*RE;
	double e = sqrt(a*a - b*b) / a;	double e2 = e*e;
	double sq = 1 - e2*sl*sl, sq2 = sqrt(sq);
	double RMh = RE*(1 - e2) / sq / sq2 + pos[2];
	double RNh = RE / sq2 + pos[2];
	double v1, v2, v3;
	v1 = 0; v2 = 0; v3 = 0;
	wnie[0] = 0.0;
	wnie[1] = wie*cl;
	wnie[2] = wie*sl;
	wnen[0] = -v2 / RMh;
	wnen[1] = v1 / RNh;
	wnen[2] = v1 / (RNh*tl);
	Maddn(wnie, wnen, wnin, 3, 1);
	Mmulnm(Cbn, wnin, 3, 3, 1, Ctwie);
	Mminn(wib, Ctwie, wnb, 3, 1);

	askew(wnb, askeww);
	Mmulnm(Cnb, askeww, 3, 3, 3, CW);

	Mmulnm(CW, lever, 3, 3, 1, tempvel);
	Maddn(vn1, tempvel, vn2, 3, 1);
}
extern void titl_compen(double att[3], double blh_ant[3], double lever_b[3], double blh_target[3])
{
	double lever_ned[3] = { 0.0 };
	cal_lever_ned(att, lever_b, lever_ned);
	//printf("att:%f,%f,%f\n", att[0]*R2D, att[1] * R2D, att[2] * R2D);
	//printf("lever_b:%f,%f,%f\n", lever_b[0], lever_b[1], lever_b[2]);
	//printf("lever_ned:%f,%f,%f\n", lever_ned[0], lever_ned[1], lever_ned[2]);
	lever_compen(blh_ant, lever_ned, blh_target);
}

double FilterQueue(double NEW_DATA, double *QUEUE, char n)
{
	double max = 0, min = 0;
	double sum = 0;
	int i = 0;
	static int count = 0;
	if (count<n)
	{
		QUEUE[count] = NEW_DATA;
		sum = NEW_DATA;
	}
	else
	{
		for (i = n - 1; i != 0; i--)
		{
			QUEUE[i] = QUEUE[i - 1];
		} //队列更新
		QUEUE[0] = NEW_DATA;
		max = QUEUE[0];
		min = QUEUE[0];
		sum = QUEUE[0];
		for (i = n - 1; i != 0; i--)
		{
			if (QUEUE[i] > max)      max = QUEUE[i]; //比较并更新最大值
			else if (QUEUE[i] < min) min = QUEUE[i]; //比较并更新最小值
			sum += QUEUE[i];
		}
		sum = sum - max - min;              //四舍五入
		sum = sum / (n - 2);
	}
	count++;
	if (count == 1000) count = n + 100;
	return sum;
}

double FilterVector(double NEW_DATA, vector<double>& QUEUE, int& count, char n)
{
	double max = 0, min = 0;
	double sum = 0;
	int i = 0;
	if (count<n)
	{
		if (QUEUE.size() < n)
		{
			QUEUE.push_back(NEW_DATA);
		}
		else
		{
			QUEUE[count] = NEW_DATA;           
		}
		sum = NEW_DATA;
	}
	else
	{
		QUEUE.erase(QUEUE.begin());
		QUEUE.push_back(NEW_DATA);
		max = QUEUE[0];
		min = QUEUE[0];
		sum = QUEUE[0];
		for (i = n - 1; i != 0; i--)
		{
			if (QUEUE[i] > max)      max = QUEUE[i]; 
			else if (QUEUE[i] < min) min = QUEUE[i]; 
			sum += QUEUE[i];
		}
		sum = sum - max - min;            
		sum = sum / (n - 2);
	}
	count++;
	if (count == 1000) count = n + 100;
	return sum;
}

double FilterVector2(double NEW_DATA, vector<double>& QUEUE,  char n)
{
	double sum = 0;
	if (QUEUE.size() < n)
	{
		QUEUE.push_back(NEW_DATA);
	}
	else
	{
		QUEUE.erase(QUEUE.begin());
		QUEUE.push_back(NEW_DATA);
		for (int i = 0; i < n;i++)
		{
			sum += QUEUE[i];
		};
		sum = sum / n;
	}
	return sum;
}
void Mequal(double *M, int m, int n, double a)
{
	for (int i = 0; i<m; i++)
	{
		for (int j = 0; j<n; j++)
		{
			*(M + i*n + j) = a;
		}
	}
}
void Madd(double *a, double *b, int m, int n)
{
	for (int i = 0; i<m; i++)
		for (int j = 0; j<n; j++)
		{
			*(a + i*n + j) = *(a + i*n + j) + *(b + i*n + j);
		}
}
double Minvn(double a[], int n, double *b)
{
	for (int i = 0; i<n; i++)
		for (int j = 0; j<n; j++)
		{
			*(b + i*n + j) = *(a + i*n + j);
		}

	int *is, *js, i, j, k, l, u, v;
	double d, p;
	is = (int *)gilc_malloc(n*sizeof(int));
	js = (int *)gilc_malloc(n*sizeof(int));
	for (k = 0; k <= n - 1; k++)
	{
		d = 0.0;
		for (i = k; i <= n - 1; i++)
			for (j = k; j <= n - 1; j++)
			{
				l = i*n + j;
				p = fabs(b[l]);
				if (p>d)
				{
					d = p;
					is[k] = i;
					js[k] = j;
				}
			}
		if (d == 0)//d+1.0==1.0 lcc
		{
			gilc_free(is);
			gilc_free(js);
			printf("\nerror:inverse matrix is not exist\n");
			return (0);
		}
		if (is[k] != k)
			for (j = 0; j <= n - 1; j++)
			{
				u = k*n + j; v = is[k] * n + j;
				p = b[u]; b[u] = b[v]; b[v] = p;
			}
		if (js[k] != k)
			for (i = 0; i <= n - 1; i++)
			{
				u = i*n + k;
				v = i*n + js[k];
				p = b[u];
				b[u] = b[v]; b[v] = p;
			}
		l = k*n + k;
		b[l] = 1.0 / b[l];
		for (j = 0; j <= n - 1; j++)
			if (j != k)
			{
				u = k*n + j;
				b[u] = b[u] * b[l];
			}
		for (i = 0; i <= n - 1; i++)
			if (i != k)
				for (j = 0; j <= n - 1; j++)
					if (j != k)
					{
						u = i*n + j;
						b[u] = b[u] - b[i*n + k] * b[k*n + j];
					}
		for (i = 0; i <= n - 1; i++)
			if (i != k)
			{
				u = i*n + k; b[u] = -b[u] * b[l];
			}
	}
	for (k = n - 1; k >= 0; k--)
	{
		if (js[k] != k)
			for (j = 0; j <= n - 1; j++)
			{
				u = k*n + j; v = js[k] * n + j;
				p = b[u];
				b[u] = b[v];
				b[v] = p;
			}
		if (is[k] != k)
			for (i = 0; i <= n - 1; i++)
			{
				u = i*n + k;
				v = i*n + is[k];
				p = b[u];
				b[u] = b[v]; b[v] = p;
			}
	}

	gilc_free(is);
	gilc_free(js);
	return(1);
}

extern void *__ml_zero(int size)
{
	void *p = gilc_malloc(size);
	if (p == NULL) {
		printf("not malloc for val\n");
	}
	memset(p, 0, size);
	return p;
}