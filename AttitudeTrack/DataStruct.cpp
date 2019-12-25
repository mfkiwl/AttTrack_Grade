#include "DataStruct.h"
void AttTrackData::Init()
{
	imutime_balde = imutime_mainfall = gnsstime = 0;
	for (int i = 0; i < 3; i++)
	{
		gyo_balde[i] = 0;
		acc_balde[i] = 0;
		gyo_mainfall[i] = 0;
		acc_mainfall[i] = 0;
		BLH_ant[i] = 0;
	}
	for (int i = 0; i < 6; i++)
	{
		ep[i] = 0.0;
	}
	week = 0;
	weektime_sec = 0.0;
	yaw_2ant =  roll_2ant = 0;
	gyaw_noise = groll_noise = 0;
	gstate_pos = gstate_2ant = 0;
	speed = speed_ver = 0;
	Mean_Gnss_Alt = 0;
	Gnss_Alt_Pre = 0;
	gnsstime_pre = 0;
	gnss_lost_num = 0;
	gnss_alt_err = false;
	integ_pitch = 0.0;
	bbaldeimu_update = bmainfallimu_update = bgnssupdate = 0;
	balde_install_falg = body_install_falg = rotation_install_flag = 0;
}

void AttTrackData::Reset() 
{
	bbaldeimu_update = 0;
	bmainfallimu_update = 0;
	bgnssupdate = 0;
}

AttTrackData& AttTrackData::operator=(const AttTrackData& atdata)
{
	imutime_balde = atdata.imutime_balde;
	imutime_mainfall = atdata.imutime_mainfall;
	gnsstime = atdata.gnsstime;
	for (int i = 0; i < 3; i++)
	{
		gyo_balde[i] = atdata.gyo_balde[i];
		acc_balde[i] = atdata.acc_balde[i];
		gyo_mainfall[i] = atdata.gyo_mainfall[i];
		acc_mainfall[i] = atdata.acc_mainfall[i];
		BLH_ant[i] = atdata.BLH_ant[i];
	}
	yaw_2ant = atdata.yaw_2ant;
	roll_2ant = atdata.roll_2ant;
	gyaw_noise = atdata.gyaw_noise;
	groll_noise = atdata.groll_noise;
	gstate_pos = atdata.gstate_pos;
	gstate_2ant = atdata.gstate_2ant;
	speed = atdata.speed;
	speed_ver = atdata.speed_ver;
	bbaldeimu_update = atdata.bbaldeimu_update;
	bmainfallimu_update = atdata.bmainfallimu_update;
	bgnssupdate = atdata.bgnssupdate;
	return *this;
}

void AttTrackData::Convert_baldeimu()
{
	int i;
	double a[3];
	for (i = 0; i<3; i++)
	{
		a[i] = acc_balde[i];
	}
	acc_balde[0] = a[0] * 9.80665;
	acc_balde[1] = a[1] * 9.80665;
	acc_balde[2] = a[2] * 9.80665;
	for (i = 0; i<3; i++)
	{
		a[i] = gyo_balde[i];
	}
	gyo_balde[0] = a[0] * 0.017453292519943;
	gyo_balde[1] = a[1] * 0.017453292519943;
	gyo_balde[2] = a[2] * 0.017453292519943;
}

void AttTrackData::Convert_mainfallimu()
{
	int i;
	double a[3];
	for (i = 0; i<3; i++)
	{
		a[i] = acc_mainfall[i];
	}
	acc_mainfall[0] = a[0] * 9.80665;
	acc_mainfall[1] = a[1] * 9.80665;
	acc_mainfall[2] = a[2] * 9.80665;
	for (i = 0; i<3; i++)
	{
		a[i] = gyo_mainfall[i];
	}
	gyo_mainfall[0] = a[0] * 0.017453292519943;
	gyo_mainfall[1] = a[1] * 0.017453292519943;
	gyo_mainfall[2] = a[2] * 0.017453292519943;
}

void decode(ATdata_t* atd, AttTrackData* atdata, AttTrackCfg_t* cfg)
{
	if (atd->bbaldeimu_update)//铲刀数据解析
	{
		atdata->imutime_balde = atd->imu_balde.imutimetarget;
		double balde_imu_data[6] = { atd->imu_balde.accx,atd->imu_balde.accy,atd->imu_balde.accz,
			atd->imu_balde.gyox ,atd->imu_balde.gyoy ,atd->imu_balde.gyoz };
		int ashaftx = (int)fabs(cfg->ashaftx); int ashafty = (int)fabs(cfg->ashafty);
		int ashaftz = (int)fabs(cfg->ashaftz); int gshaftx = (int)fabs(cfg->gshaftx);
		int gshafty = (int)fabs(cfg->gshafty); int gshaftz = (int)fabs(cfg->gshaftz);
		atdata->acc_balde[0] = (int)cfg->ashaftx*balde_imu_data[ashaftx-1]/ ashaftx;
		atdata->acc_balde[1] = (int)cfg->ashafty*balde_imu_data[ashafty-1]/ ashafty;
		atdata->acc_balde[2] = (int)cfg->ashaftz*balde_imu_data[ashaftz-1]/ ashaftz;
		atdata->gyo_balde[0] = (int)cfg->gshaftx*balde_imu_data[gshaftx+2]/ gshaftx;
		atdata->gyo_balde[1] = (int)cfg->gshafty*balde_imu_data[gshafty+2]/ gshafty;
		atdata->gyo_balde[2] = (int)cfg->gshaftz*balde_imu_data[gshaftz+2]/ gshaftz;
		atdata->bbaldeimu_update = atd->bbaldeimu_update;
		atdata->imu_time = atd->imu_time;
	}
	if (atd->bbodyimu_update)//车体数据解析
	{
		atdata->imutime_mainfall = atd->imu_body.imutimetarget;
		double mainfall_imu_data[6] = { atd->imu_body.accx,atd->imu_body.accy,atd->imu_body.accz,
			atd->imu_body.gyox ,atd->imu_body.gyoy ,atd->imu_body.gyoz };
		int ashaftx = (int)fabs(cfg->ashaftx); int ashafty = (int)fabs(cfg->ashafty);
		int ashaftz = (int)fabs(cfg->ashaftz); int gshaftx = (int)fabs(cfg->gshaftx);
		int gshafty = (int)fabs(cfg->gshafty); int gshaftz = (int)fabs(cfg->gshaftz);
		atdata->acc_mainfall[0] = (int)cfg->ashaftx*mainfall_imu_data[ashaftx-1] / ashaftx;
		atdata->acc_mainfall[1] = (int)cfg->ashafty*mainfall_imu_data[ashafty-1] / ashafty;
		atdata->acc_mainfall[2] = (int)cfg->ashaftz*mainfall_imu_data[ashaftz-1] / ashaftz;
		atdata->gyo_mainfall[0] = (int)cfg->gshaftx*mainfall_imu_data[gshaftx + 2] / gshaftx;
		atdata->gyo_mainfall[1] = (int)cfg->gshafty*mainfall_imu_data[gshafty + 2] / gshafty;
		atdata->gyo_mainfall[2] = (int)cfg->gshaftz*mainfall_imu_data[gshaftz + 2] / gshaftz;
		atdata->bmainfallimu_update = atd->bbodyimu_update;
	}

	atdata->rotation_balde = atd->rotation * D2R;//旋转传感器数据
	atdata->balde_install_falg = atd->balde_install_flag;//上位机交互铲刀安装误差标定标志
	atdata->body_install_falg = atd->body_install_falg;//上位机交互车体安装误差标定标志
	atdata->rotation_install_flag = atd->rotation_install_falg;//上位机旋转传感器标定标志
	if (atd->bgnss_update)//gnss数据解析
	{
		atdata->gnsstime = atd->gnss.gnsstimetarget;
		atdata->BLH_ant[0] = atd->gnss.lat * D2R;
		atdata->BLH_ant[1] = atd->gnss.lon * D2R;
		atdata->BLH_ant[2] = atd->gnss.alt;
		atdata->speed = atd->gnss.speed;
		atdata->speed_ver = atd->gnss.speed_ver;
		//atdata->yaw_2ant = atd->gnss.gnssyaw * D2R;
		atdata->yaw_2ant = atd->gnss.gnssyaw;
		atdata->roll_2ant = atd->gnss.gnssroll;
		if (atd->gnss.noise_gyaw)
		{
			atdata->gyaw_noise = atd->gnss.noise_gyaw * D2R;
		}
		else
		{
			atdata->gyaw_noise = 0.1 * D2R;
		}
		if (atd->gnss.noise_groll)
		{
			atdata->groll_noise = atd->gnss.noise_groll * D2R;
		}
		else
		{
			atdata->groll_noise = 0.3 * D2R;
		}
		if (atdata->Gnss_Alt_Pre)
		{
			atdata->Mean_Gnss_Alt = (atdata->Gnss_Alt_Pre + atd->gnss.alt) / 2;
		}
		atdata->gstate_pos = atd->gnss.state_pos;
		atdata->gstate_2ant = atd->gnss.state_yaw;
		atdata->bgnssupdate = atd->bgnss_update;
		atdata->Gnss_Alt_Pre= atd->gnss.alt;
	}
}
void Data_Check(AttTrackData* atdata)
{
	/*gnss数据中断识别*/
	if (atdata->gnsstime_pre)
	{
		double time_err = fabs(atdata->gnsstime - atdata->gnsstime_pre);
		double time_thre = (GNSS_UPDATE_TIME+10) / 1000.0;
		if (time_err>time_thre)
		{
			atdata->gnss_lost_num++;
			printf("GNSS lost:%d!! gnss_time=%f\r\n", (int)(time_err*1000/ GNSS_UPDATE_TIME),atdata->gnsstime);
		}
		else
		{
			atdata->gnss_lost_num = 0;
		}
	}
	if (atdata->gnss_lost_num*GNSS_UPDATE_TIME / 1000.0 > 1)//中断时间大于1秒
	{
		printf("GNSS lost time > 1s\r\n");
	}
	/*gnss高程异常识别*/
	if (atdata->Gnss_Alt_Pre)
	{
		if (fabs(atdata->BLH_ant[2] - atdata->Gnss_Alt_Pre) > 0.1)//10cm
		{
			printf("GNSS alt err!!gnss_time=%f  gnss_alt=%f  gnss_alt_pre=%f\r\n", atdata->gnsstime,atdata->BLH_ant[2], atdata->Gnss_Alt_Pre);
			atdata->gnss_alt_err = true;
		}
		else
		{
			atdata->gnss_alt_err = false;
		}
	}
	atdata->gnsstime_pre = atdata->gnsstime;
}
double time2gpst(gtime_t t, int *week)
{
	gtime_t t0 = epoch2time(gpst0);
	time_t sec = t.time - t0.time;
	int w = (int)(sec / (86400 * 7));

	if (week) {
		*week = w;
	}
	return (double)(sec - w * 86400 * 7) + t.sec;
}
gtime_t epoch2time(const double *ep)
{
	const int doy[] = { 1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335 };
	gtime_t time = { 0 };
	int days, sec, year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];

	if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) {
		return time;
	}

	/* leap year if year%4==0 in 1901-2099 */
	days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 + (year % 4 == 0 && mon >= 3 ? 1 : 0);
	sec = (int)floor(ep[5]);
	time.time = (time_t)days * 86400 + (int)ep[3] * 3600 + (int)ep[4] * 60 + sec;
	time.sec = ep[5] - sec;
	return time;
}
void time2epoch(gtime_t t, double *ep)
{
	const int mday[] = { /* # of days in a month */
		31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
		31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
	};
	int days, sec, mon, day;

	/* leap year if year%4==0 in 1901-2099 */
	days = (int)(t.time / 86400);
	sec = (int)(t.time - (time_t)days * 86400);
	for (day = days % 1461, mon = 0; mon < 48; mon++) {
		if (day >= mday[mon]) {
			day -= mday[mon];
		}
		else {
			break;
		}
	}
	ep[0] = 1970 + days / 1461 * 4 + mon / 12;
	ep[1] = mon % 12 + 1;
	ep[2] = day + 1;
	ep[3] = sec / 3600;
	ep[4] = sec % 3600 / 60;
	ep[5] = sec % 60 + t.sec;
}
gtime_t gpst2utc(gtime_t t)
{
	gtime_t tu;
	int i;

	for (i = 0; i < (int)sizeof(leaps) / (int)sizeof(*leaps); i++) {
		tu = timeadd(t, leaps[i][6]);
		if (timediff(tu, epoch2time(leaps[i])) >= 0.0) {
			return tu;
		}
	}
	return t;
}
double timediff(gtime_t t1, gtime_t t2)
{
	return difftime(t1.time, t2.time) + t1.sec - t2.sec;
}
gtime_t timeadd(gtime_t t, double sec)
{
	double tt;

	t.sec += sec;
	tt = floor(t.sec);
	t.time += (int)tt;
	t.sec -= tt;
	return t;
}
void Gnss_time_decode(AttTrackData* atdata)
{
	gtime_t gt_chc = { 0 };
	double gnss_time = atdata->gnsstime;
	gt_chc.time = (int)gnss_time;
	gt_chc.sec = gnss_time - (int)gnss_time;
	/*gnss秒转换为周、周内秒*/
	int gnss_week = 0;
	atdata->weektime_sec= time2gpst(gt_chc, &gnss_week);
	atdata->week = gnss_week;
	//printf("week:%d,sec:%f\r\n", atdata->week, atdata->weektime_sec);
	/*gnss秒转换为UTC时间*/
	time2epoch(gpst2utc(gt_chc), atdata->ep);
	//printf("utc_time:%02d%02d%f\r\n", 
	//	(int)atdata->ep[3], (int)atdata->ep[4], atdata->ep[5]);

}