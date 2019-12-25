//#include "stdafx.h"
#include "config.h"
#include "ComFunc.h"
JSON_Object        *s_cfg_obj = NULL;
JSON_Value         *s_cfg_val = NULL;
int init_config()
{
	s_cfg_val = json_parse_file(ATTITUDE_TRACK_CONFIG_PATH);
	s_cfg_obj = json_object(s_cfg_val);

	if (s_cfg_obj == NULL)
	{
		printf("Init config file error");
		return 0;
	}

	return 1;
}

void save_config()
{
	if (s_cfg_val)
	{
		json_serialize_to_file_pretty(s_cfg_val, ATTITUDE_TRACK_CONFIG_PATH);
	}
}

void creat_config()
{
	s_cfg_val = json_value_init_object();
	s_cfg_obj = json_object(s_cfg_val);
}
int config_set_gyo_staticbiasx(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_bias.gxstaticbias", num);
	save_config();
	return 1;
}
double config_get_gyo_staticbiasx()
{
	return json_object_dotget_number(s_cfg_obj, "imu_bias.gxstaticbias");
}

int config_set_gyo_staticbiasy(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_bias.gystaticbias", num);
	save_config();
	return 1;
}
double config_get_gyo_staticbiasy()
{
	return json_object_dotget_number(s_cfg_obj, "imu_bias.gystaticbias");
}

int config_set_gyo_staticbiasz(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_bias.gzstaticbias", num);
	save_config();
	return 1;
}
double config_get_gyo_staticbiasz()
{
	return json_object_dotget_number(s_cfg_obj, "imu_bias.gzstaticbias");
}

int config_set_acc_staticbiasx(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_bias.axstaticbias", num);
	save_config();
	return 1;
}
double config_get_acc_staticbiasx()
{
	return json_object_dotget_number(s_cfg_obj, "imu_bias.axstaticbias");
}

int config_set_acc_staticbiasy(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_bias.aystaticbias", num);
	save_config();
	return 1;
}
double config_get_acc_staticbiasy()
{
	return json_object_dotget_number(s_cfg_obj, "imu_bias.aystaticbias");
}

int config_set_acc_staticbiasz(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_bias.azstaticbias", num);
	save_config();
	return 1;
}
double config_get_acc_staticbiasz()
{
	return json_object_dotget_number(s_cfg_obj, "imu_bias.azstaticbias");
}


int config_set_gyo_stdx_thr(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_stdthr.gxstdthr", num);
	save_config();
	return 1;
}
double config_get_gyo_stdx_thr()
{
	return json_object_dotget_number(s_cfg_obj, "imu_stdthr.gxstdthr");
}

int config_set_gyo_stdy_thr(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_stdthr.gystdthr", num);
	save_config();
	return 1;
}
double config_get_gyo_stdy_thr()
{
	return json_object_dotget_number(s_cfg_obj, "imu_stdthr.gystdthr");
}

int config_set_gyo_stdz_thr(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_stdthr.gzstdthr", num);
	save_config();
	return 1;
}
double config_get_gyo_stdz_thr()
{
	return json_object_dotget_number(s_cfg_obj, "imu_stdthr.gzstdthr");
}

int config_set_acc_stdx_thr(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_stdthr.axstdthr", num);
	save_config();
	return 1;
}
double config_get_acc_stdx_thr()
{
	return json_object_dotget_number(s_cfg_obj, "imu_stdthr.axstdthr");
}

int config_set_acc_stdy_thr(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_stdthr.aystdthr", num);
	save_config();
	return 1;
}
double config_get_acc_stdy_thr()
{
	return json_object_dotget_number(s_cfg_obj, "imu_stdthr.aystdthr");
}

int config_set_acc_stdz_thr(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_stdthr.azstdthr", num);
	save_config();
	return 1;
}
double config_get_acc_stdz_thr()
{
	return json_object_dotget_number(s_cfg_obj, "imu_stdthr.azstdthr");
}

int config_set_gyo_noisex(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_noise.gxnoise", num);
	save_config();
	return 1;
}
double config_get_gyo_noisex()
{
	return json_object_dotget_number(s_cfg_obj, "imu_noise.gxnoise");
}

int config_set_gyo_noisey(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_noise.gynoise", num);
	save_config();
	return 1;
}
double config_get_gyo_noisey()
{
	return json_object_dotget_number(s_cfg_obj, "imu_noise.gynoise");
}

int config_set_gyo_noisez(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_noise.gznoise", num);
	save_config();
	return 1;
}
double config_get_gyo_noisez()
{
	return json_object_dotget_number(s_cfg_obj, "imu_noise.gznoise");
}

int config_set_acc_noisex(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_noise.axnoise", num);
	save_config();
	return 1;
}
double config_get_acc_noisex()
{
	return json_object_dotget_number(s_cfg_obj, "imu_noise.axnoise");
}

int config_set_acc_noisey(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_noise.aynoise", num);
	save_config();
	return 1;
}
double config_get_acc_noisey()
{
	return json_object_dotget_number(s_cfg_obj, "imu_noise.aynoise");
}

int config_set_acc_noisez(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_noise.aznoise", num);
	save_config();
	return 1;
}
double config_get_acc_noisez()
{
	return json_object_dotget_number(s_cfg_obj, "imu_noise.aznoise");
}

int config_set_gyo_stabilityx(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_stability.gxstability", num);
	save_config();
	return 1;
}
double config_get_gyo_stabilityx()
{
	return json_object_dotget_number(s_cfg_obj, "imu_stability.gxstability");
}

int config_set_gyo_stabilityy(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_stability.gystability", num);
	save_config();
	return 1;
}
double config_get_gyo_stabilityy()
{
	return json_object_dotget_number(s_cfg_obj, "imu_stability.gystability");
}

int config_set_gyo_stabilityz(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_stability.gzstability", num);
	save_config();
	return 1;
}
double config_get_gyo_stabilityz()
{
	return json_object_dotget_number(s_cfg_obj, "imu_stability.gzstability");
}

int config_set_acc_stabilityx(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_stability.axstability", num);
	save_config();
	return 1;
}
double config_get_acc_stabilityx()
{
	return json_object_dotget_number(s_cfg_obj, "imu_stability.axstability");
}

int config_set_acc_stabilityy(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_stability.aystability", num);
	save_config();
	return 1;
}
double config_get_acc_stabilityy()
{
	return json_object_dotget_number(s_cfg_obj, "imu_stability.aystability");
}

int config_set_acc_stabilityz(double num)
{
	json_object_dotset_number(s_cfg_obj, "imu_stability.azstability", num);
	save_config();
	return 1;
}
double config_get_acc_stabilityz()
{
	return json_object_dotget_number(s_cfg_obj, "imu_stability.azstability");
}

int config_set_gyo_shaftx(int num)
{
	json_object_dotset_number(s_cfg_obj, "imu_shaft.gxshaft", num);
	save_config();
	return 1;
}
int config_get_gyo_shaftx()
{
	return (int)json_object_dotget_number(s_cfg_obj, "imu_shaft.gxshaft");
}

int config_set_gyo_shafty(int num)
{
	json_object_dotset_number(s_cfg_obj, "imu_shaft.gyshaft", num);
	save_config();
	return 1;
}
int config_get_gyo_shafty()
{
	return (int)json_object_dotget_number(s_cfg_obj, "imu_shaft.gyshaft");
}

int config_set_gyo_shaftz(int num)
{
	json_object_dotset_number(s_cfg_obj, "imu_shaft.gzshaft", num);
	save_config();
	return 1;
}
int config_get_gyo_shaftz()
{
	return (int)json_object_dotget_number(s_cfg_obj, "imu_shaft.gzshaft");
}

int config_set_acc_shaftx(int num)
{
	json_object_dotset_number(s_cfg_obj, "imu_shaft.axshaft", num);
	save_config();
	return 1;
}
int config_get_acc_shaftx()
{
	return (int)json_object_dotget_number(s_cfg_obj, "imu_shaft.axshaft");
}

int config_set_acc_shafty(int num)
{
	json_object_dotset_number(s_cfg_obj, "imu_shaft.ayshaft", num);
	save_config();
	return 1;
}
int config_get_acc_shafty()
{
	return (int)json_object_dotget_number(s_cfg_obj, "imu_shaft.ayshaft");
}

int config_set_acc_shaftz(int num)
{
	json_object_dotset_number(s_cfg_obj, "imu_shaft.azshaft", num);
	save_config();
	return 1;
}
int config_get_acc_shaftz()
{
	return (int)json_object_dotget_number(s_cfg_obj, "imu_shaft.azshaft");
}

int config_set_right_leverx(double num)
{
	json_object_dotset_number(s_cfg_obj, "lever.rightleverx", num);
	save_config();
	return 1;
}
double config_get_right_leverx()
{
	return json_object_dotget_number(s_cfg_obj, "lever.rightleverx");
}

int config_set_right_levery(double num)
{
	json_object_dotset_number(s_cfg_obj, "lever.rightlevery", num);
	save_config();
	return 1;
}
double config_get_right_levery()
{
	return json_object_dotget_number(s_cfg_obj, "lever.rightlevery");
}

int config_set_right_leverz(double num)
{
	json_object_dotset_number(s_cfg_obj, "lever.rightleverz", num);
	save_config();
	return 1;
}
double config_get_right_leverz()
{
	return json_object_dotget_number(s_cfg_obj, "lever.rightleverz");
}

int config_set_left_leverx(double num)
{
	json_object_dotset_number(s_cfg_obj, "lever.leftleverx", num);
	save_config();
	return 1;
}
double config_get_left_leverx()
{
	return json_object_dotget_number(s_cfg_obj, "lever.leftleverx");
}

int config_set_left_levery(double num)
{
	json_object_dotset_number(s_cfg_obj, "lever.leftlevery", num);
	save_config();
	return 1;
}
double config_get_left_levery()
{
	return json_object_dotget_number(s_cfg_obj, "lever.leftlevery");
}

int config_set_left_leverz(double num)
{
	json_object_dotset_number(s_cfg_obj, "lever.leftleverz", num);
	save_config();
	return 1;
}
double config_get_left_leverz()
{
	return json_object_dotget_number(s_cfg_obj, "lever.leftleverz");
}

int config_set_gnssyawstd_thr(double num)
{
	json_object_dotset_number(s_cfg_obj, "gnss.yawstdthr", num);
	save_config();
	return 1;
}
double config_get_gnssyawstd_thr()
{
	return json_object_dotget_number(s_cfg_obj, "gnss.yawstdthr");
}

int config_set_gnssyaw_var(double num)
{
	json_object_dotset_number(s_cfg_obj, "gnss.yawvar", num);
	save_config();
	return 1;
}
double config_get_gnssyaw_var()
{
	return json_object_dotget_number(s_cfg_obj, "gnss.yawvar");
}

int config_set_gnssrollstd_thr(double num)
{
	json_object_dotset_number(s_cfg_obj, "gnss.rollstdthr", num);
	save_config();
	return 1;
}
double config_get_gnssrollstd_thr()
{
	return json_object_dotget_number(s_cfg_obj, "gnss.rollstdthr");
}

int config_set_balde_install_roll(double num)
{
	json_object_dotset_number(s_cfg_obj, "blade_install.roll", num);
	save_config();
	return 1;
}
double config_get_balde_install_roll()
{
	return json_object_dotget_number(s_cfg_obj, "blade_install.roll");
}

int config_set_balde_install_pitch(double num)
{
	json_object_dotset_number(s_cfg_obj, "blade_install.pitch", num);
	save_config();
	return 1;
}
double config_get_balde_install_pitch()
{
	return json_object_dotget_number(s_cfg_obj, "blade_install.pitch");
}
int config_set_balde_install_yaw(double num)
{
	json_object_dotset_number(s_cfg_obj, "blade_install.yaw", num);
	save_config();
	return 1;
}
double config_get_balde_install_yaw()
{
	return json_object_dotget_number(s_cfg_obj, "blade_install.yaw");
}

int config_set_mainfall_install_pitch(double num)
{
	json_object_dotset_number(s_cfg_obj, "mainfall_install.pitch", num);
	save_config();
	return 1;
}
double config_get_mainfall_install_pitch()
{
	return json_object_dotget_number(s_cfg_obj, "mainfall_install.pitch");
}

int config_set_rotation_install(double num)
{
	json_object_dotset_number(s_cfg_obj, "rotation.zero", num);
	save_config();
	return 1;
}
double config_get_rotation_install()
{
	return json_object_dotget_number(s_cfg_obj, "rotation.zero");
}
void generate_default_config_ADIS16460()
{
	s_cfg_val = json_value_init_object();
	s_cfg_obj = json_object(s_cfg_val);

	config_set_gyo_staticbiasx(0);
	config_set_gyo_staticbiasy(0);
	config_set_gyo_staticbiasz(0);
	config_set_acc_staticbiasx(0);
	config_set_acc_staticbiasy(0);
	config_set_acc_staticbiasz(0);

	config_set_gyo_stdx_thr(0.001);         //具体数值
	config_set_gyo_stdy_thr(0.001);
	config_set_gyo_stdz_thr(0.001);
	config_set_acc_stdx_thr(0.05);
	config_set_acc_stdy_thr(0.05);
	config_set_acc_stdz_thr(0.05);

	config_set_gyo_noisex(0.025);     //deg
	config_set_gyo_noisey(0.025);     //deg
	config_set_gyo_noisez(0.025);     //deg
	config_set_acc_noisex(0.001);     //g
	config_set_acc_noisey(0.001);     //g
	config_set_acc_noisez(0.001);     //g

	config_set_gyo_stabilityx(8.0);   //dph
	config_set_gyo_stabilityy(8.0);   //dph
	config_set_gyo_stabilityz(8.0);   //dph
	config_set_acc_stabilityx(0.00002);  //g
	config_set_acc_stabilityy(0.00002);  //g
	config_set_acc_stabilityz(0.00002);  //g

	config_set_gyo_shaftx(0);
	config_set_gyo_shafty(1);
	config_set_gyo_shaftz(2);
	config_set_acc_shaftx(0);
	config_set_acc_shafty(1);
	config_set_acc_shaftz(2);

	config_set_gnssyawstd_thr(100.1); //deg
	config_set_gnssyaw_var(30.0);      //deg

	config_set_right_leverx(0);
	config_set_right_levery(0);
	config_set_right_leverz(0);
	config_set_left_leverx(0);
	config_set_left_levery(0);
	config_set_left_leverz(0);

	//关闭对象
	//json_serialize_to_file_pretty(s_cfg_val, GILC_TILTMEASURE_CONFIG_PATH);
	//json_value_free(s_cfg_val);
}

void generate_default_config_BDF06()
{
	s_cfg_val = json_value_init_object();
	s_cfg_obj = json_object(s_cfg_val);

	config_set_gyo_staticbiasx(0);
	config_set_gyo_staticbiasy(0);
	config_set_gyo_staticbiasz(0);
	config_set_acc_staticbiasx(0);
	config_set_acc_staticbiasy(0);
	config_set_acc_staticbiasz(0);

	config_set_gyo_stdx_thr(0.001);         //具体数值
	config_set_gyo_stdy_thr(0.001);
	config_set_gyo_stdz_thr(0.001);
	config_set_acc_stdx_thr(0.05);
	config_set_acc_stdy_thr(0.05);
	config_set_acc_stdz_thr(0.05);

	config_set_gyo_noisex(0.025);     //deg
	config_set_gyo_noisey(0.025);     //deg
	config_set_gyo_noisez(0.025);     //deg
	config_set_acc_noisex(0.001);     //g
	config_set_acc_noisey(0.001);     //g
	config_set_acc_noisez(0.001);     //g

	config_set_gyo_stabilityx(8.0);   //dph
	config_set_gyo_stabilityy(8.0);   //dph
	config_set_gyo_stabilityz(8.0);   //dph
	config_set_acc_stabilityx(0.00001);  //g
	config_set_acc_stabilityy(0.00001);  //g
	config_set_acc_stabilityz(0.00001);  //g

	config_set_gyo_shaftx(0);
	config_set_gyo_shafty(1);
	config_set_gyo_shaftz(2);
	config_set_acc_shaftx(0);
	config_set_acc_shafty(1);
	config_set_acc_shaftz(2);

	config_set_gnssyawstd_thr(100.1); //deg
	config_set_gnssyaw_var(30.0);      //deg

	config_set_right_leverx(0);
	config_set_right_levery(0);
	config_set_right_leverz(0);
	config_set_left_leverx(0);
	config_set_left_levery(0);
	config_set_left_leverz(0);

	//关闭对象
	//json_serialize_to_file_pretty(s_cfg_val, GILC_TILTMEASURE_CONFIG_PATH);
	//json_value_free(s_cfg_val);
}
//CRS03 SCA3300
void generate_default_config_IS203()
{
	s_cfg_val = json_value_init_object();
	s_cfg_obj = json_object(s_cfg_val);

	config_set_gyo_staticbiasx(0);
	config_set_gyo_staticbiasy(0);
	config_set_gyo_staticbiasz(0);
	config_set_acc_staticbiasx(0);
	config_set_acc_staticbiasy(0);
	config_set_acc_staticbiasz(0);

	config_set_gyo_stdx_thr(0.003);  //具体数值,判断静态
	config_set_gyo_stdy_thr(0.003);
	config_set_gyo_stdz_thr(0.003);
	config_set_acc_stdx_thr(0.01);
	config_set_acc_stdy_thr(0.01);
	config_set_acc_stdz_thr(0.01);
	/********/
	config_set_gyo_noisex(0.025);     //deg
	config_set_gyo_noisey(0.025);     //deg
	config_set_gyo_noisez(0.025);     //deg
	config_set_acc_noisex(0.0015);     //g
	config_set_acc_noisey(0.0015);     //g
	config_set_acc_noisez(0.0015);     //g

	config_set_gyo_stabilityx(3.5);   //dph
	config_set_gyo_stabilityy(3.5);   //dph
	config_set_gyo_stabilityz(3.5);   //dph
	config_set_acc_stabilityx(0.00002);  //g
	config_set_acc_stabilityy(0.00002);  //g
	config_set_acc_stabilityz(0.00002);  //g
	/********/
	config_set_gyo_shaftx(0);
	config_set_gyo_shafty(1);
	config_set_gyo_shaftz(2);
	config_set_acc_shaftx(0);
	config_set_acc_shafty(1);
	config_set_acc_shaftz(2);

	config_set_gnssyawstd_thr(1.0); //deg
	config_set_gnssyaw_var(0.5);      //deg
	config_set_gnssrollstd_thr(1.0); //deg

	config_set_right_leverx(0);
	config_set_right_levery(0);
	config_set_right_leverz(-1.2);
	config_set_left_leverx(0);
	config_set_left_levery(3);
	config_set_left_leverz(-1.2);

	config_set_balde_install_roll(0);
	config_set_balde_install_pitch(0);

	//关闭对象
	//json_serialize_to_file_pretty(s_cfg_val, GILC_TILTMEASURE_CONFIG_PATH);
	//json_value_free(s_cfg_val);
}
void generate_default_config_Grader()
{
	s_cfg_val = json_value_init_object();
	s_cfg_obj = json_object(s_cfg_val);

	config_set_gyo_staticbiasx(0);
	config_set_gyo_staticbiasy(0);
	config_set_gyo_staticbiasz(0);
	config_set_acc_staticbiasx(0);
	config_set_acc_staticbiasy(0);
	config_set_acc_staticbiasz(0);

	config_set_gyo_stdx_thr(0.3);  //具体数值,判断静态
	config_set_gyo_stdy_thr(0.3);
	config_set_gyo_stdz_thr(0.3);
	config_set_acc_stdx_thr(0.1);
	config_set_acc_stdy_thr(0.1);
	config_set_acc_stdz_thr(0.1);
	/********/
	config_set_gyo_noisex(0.025);     //deg
	config_set_gyo_noisey(0.025);     //deg
	config_set_gyo_noisez(0.025);     //deg
	config_set_acc_noisex(0.0015);     //g
	config_set_acc_noisey(0.0015);     //g
	config_set_acc_noisez(0.0015);     //g

	config_set_gyo_stabilityx(3.5);   //dph
	config_set_gyo_stabilityy(3.5);   //dph
	config_set_gyo_stabilityz(3.5);   //dph
	config_set_acc_stabilityx(0.00002);  //g
	config_set_acc_stabilityy(0.00002);  //g
	config_set_acc_stabilityz(0.00002);  //g
										 /********/
	config_set_gyo_shaftx(2);
	config_set_gyo_shafty(-1);
	config_set_gyo_shaftz(-3);
	config_set_acc_shaftx(2);
	config_set_acc_shafty(-1);
	config_set_acc_shaftz(-3);

	config_set_gnssyawstd_thr(1.0); //deg
	config_set_gnssyaw_var(0.5);      //deg
	config_set_gnssrollstd_thr(1.0); //deg

	config_set_right_leverx(-0.025);
	config_set_right_levery(4.2975);
	config_set_right_leverz(3.24);
	config_set_left_leverx(-0.025);
	config_set_left_levery(0.2975);
	config_set_left_leverz(3.24);

	config_set_balde_install_roll(0);
	config_set_balde_install_pitch(0);
	config_set_mainfall_install_pitch(0);

	config_set_rotation_install(0);
}
void get_config(AttTrackCfg_t* cfg)
{
	cfg->gbiasx = config_get_gyo_staticbiasx();  //具体数值
	cfg->gbiasy = config_get_gyo_staticbiasy();
	cfg->gbiasz = config_get_gyo_staticbiasz();
	cfg->abiasx = config_get_acc_staticbiasx();
	cfg->abiasy = config_get_acc_staticbiasy();
	cfg->abiasz = config_get_acc_staticbiasz();

	cfg->gstdxthr = config_get_gyo_stdx_thr();   //具体数值
	cfg->gstdythr = config_get_gyo_stdy_thr();
	cfg->gstdzthr = config_get_gyo_stdz_thr();
	cfg->astdxthr = config_get_acc_stdx_thr();
	cfg->astdythr = config_get_acc_stdy_thr();
	cfg->astdzthr = config_get_acc_stdz_thr();

	cfg->gnoisex = config_get_gyo_noisex();
	cfg->gnoisey = config_get_gyo_noisey();
	cfg->gnoisez = config_get_gyo_noisez();
	cfg->anoisex = config_get_acc_noisex();
	cfg->anoisey = config_get_acc_noisey();
	cfg->anoisez = config_get_acc_noisez();

	cfg->gstabilityx = config_get_gyo_stabilityx();
	cfg->gstabilityy = config_get_gyo_stabilityy();
	cfg->gstabilityz = config_get_gyo_stabilityz();
	cfg->astabilityx = config_get_acc_stabilityx();
	cfg->astabilityy = config_get_acc_stabilityy();
	cfg->astabilityz = config_get_acc_stabilityz();

	cfg->gshaftx = config_get_gyo_shaftx();
	cfg->gshafty = config_get_gyo_shafty();
	cfg->gshaftz = config_get_gyo_shaftz();
	cfg->ashaftx = config_get_acc_shaftx();
	cfg->ashafty = config_get_acc_shafty();
	cfg->ashaftz = config_get_acc_shaftz();

	cfg->gnssyawstd_thr = config_get_gnssyawstd_thr();
	cfg->gnssyawvar = config_get_gnssyaw_var();
	cfg->gnssrollstd_thr = config_get_gnssrollstd_thr();

	cfg->leverrightx = config_get_right_leverx();
	cfg->leverrighty = config_get_right_levery();
	cfg->leverrightz = config_get_right_leverz();
	cfg->leverleftx = config_get_left_leverx();
	cfg->leverlefty = config_get_left_levery();
	cfg->leverleftz = config_get_left_leverz();

	cfg->balde_install_roll = config_get_balde_install_roll()*D2R;
	cfg->balde_install_pitch = config_get_balde_install_pitch()*D2R;
	cfg->mainfall_install_pitch = config_get_mainfall_install_pitch()*D2R;

	cfg->rotation_zero = config_get_rotation_install();
}