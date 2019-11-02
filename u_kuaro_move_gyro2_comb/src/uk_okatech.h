/*-----------------------------------------------
 * 	uk_okatech.h
 * <Last Update>	H27/10/29
 * <version>		v1.3
 *
 * <MEMO>
 * オカテック制御プログラム
 * uSerialPort.hをインクルード
 * <10.29>	ME使えるようにした
 * 			stop_slow追加
 * ---------------------------------------------*/
#ifndef UK_OKETECH_H_
#define UK_OKETECH_H_

/*-----ヘッダー宣言-------------------------------*/
//自作
#include "uSerialPort.h"
//boost
#include <boost/timer/timer.hpp>
//ROS用
#ifdef __linux__
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>			//ゲームパッド
//#include <sound_play/sound_play.h>		//sound
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Imu.h>
#include <sys/time.h>
#include <vector>

#endif

#include <sstream>
#include <string>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <time.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>

//#include "kbhit.h"
#include <pthread.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
/*-----#define定義-------------------------------*/
//COMポート
#define OKATECH_COMPORT "COM4"

using namespace boost;
using namespace std;

//制御周期
#define MMR_CONTROL_PERIOD 25	//MMR送るタイミング
#define WRITING_SLEEP_TIME 1	//その他書き込み

//Okatechの機構パラメータ
#define OKATECH_D 0.254     	//タイヤ直径
#define OKATECH_TREAD 0.47 		//KUARO幅
#define OKATECH_PR 120000		//パルス*ギア比

//#defineマクロ
#define VELOCITY_TO_ANGULAR_VELOCITY(V) (V/(OKATECH_D*0.50))
#define NANOSEC_TO_SEC(NANOSEC) (NANOSEC*0.000000001)

//リミッター
#define OKATECH_LIMIT_SPEED_V 1.0 // 0.7_2016_10_14
#define OKATECH_LIMIT_SPEED_R VELOCITY_TO_ANGULAR_VELOCITY(OKATECH_LIMIT_SPEED_V)
#define OKATECH_TURN_LIMIT 0.2	//その場回転用
#define OKATECH_LIMIT_CV 2.0

#define CMD_ACC_LIMIT 0.2
#define CMD_ANG_ACC_LIMIT 0.4
#define CMD_CONTROL_PERIOD 10

//円周率
#ifndef M_PI
#define M_PI 3.14159
#endif

//Okatechの回転角度の範囲
//#define MTH_RANGE_0_2PI	//	0≦TH≦2π
//#define MTH_RANGE_PI_PI	//-π≦TH≦π
#define MTH_RANGE_OO_OO	//-∞≦TH≦∞

/**  WRITE_MEを定義すればMEを書き込み制御する	**/
/**	 デフォルトではMMR書き込み				**/
//#define WRITE_ME
#ifdef WRITE_ME
#define WRITE_MESSAGE "\rME\r"
#else
#define WRITE_MESSAGE "\rMMR\r"
#endif

#ifdef __linux__
//ros/sound_play関連
// #define SOUND_ALL 51
// #define SOUND_PASS "/home/urobo/ミュージック/sound"
#endif



/*-----構造体-------------------------------*/
//READがv1,v2で読みにくい?Rodecはこっちの方が
//都合がいいんだよ(´・ω・｀)
struct OKA_WHEEL{
	struct READ{
		double d1, d2;
		double v1, v2;
	}read;

	struct WRITE{
		double vR,vL;
	}write;

	double v;
	double rv;
	double cv;
};

struct OKA_POSITION{
	double x,y,th;
	double dx,dy,dth;
	double dx_v,dy_v,dw;
	double dtime;
};

/*-----クラス宣言-------------------------------*/
//Okatech用オブザーバー
class Observer : public serial::SerialObserver
{
public:
	Observer(){}
	virtual ~Observer()
	{}

private:

#ifdef WRITE_ME
	int interim_1, interim_2, interim_3, interim_4;
#else
	double interim_1, interim_2;
#endif

	static boost::timer::auto_cpu_timer time_interval;
protected:
	void notify(const std::string& str);
};

//引数例　U_OKATECH okatech("COM4")
class U_OKATECH
{
private:
	//TinyPowerの通信用
	serial::SerialPort TinPow;
	//一定周期に書き込む
	boost::thread Write_Const;

	bool thread_end_flag;
	long long int time_limit;
	void writeMMR(void);
	Observer ob;

	bool stop_flag;
	bool old_stop_flag;

public:
	U_OKATECH(std::string Port);
	~U_OKATECH();
	static OKA_WHEEL wheel;
	static OKA_POSITION position;
	void u_write(const std::string& text);
	void runV(double V);
	void runV_R(double V);
	void runV_L(double V);
	void runV_cv(void);
	void runV_cv_slow(void);
	void runR(double R);
	void runR(void);
	static void update_position(void);
	bool wheel1r(double rps);	bool wheel2r(double rps);
	bool wheel1v(double v);		bool wheel2v(double v);
	void stop(void);
	void stop_slow(void);

	void u_Joy_Control(const sensor_msgs::Joy::ConstPtr& joy);
	void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& twist);
	void emergency_func(const std_msgs::Bool::ConstPtr &state);
	bool joy_use_flag;

	//人追従追加分
	void Wheel_Control_Outside(const std_msgs::StringConstPtr& msg);
	void Odometry_Save(const std_msgs::StringConstPtr& msg);
	double KV,KLV,KT,KTD,INVIOLABLE_AREA;
	double _cmd_acc_limit,_cmd_ang_acc_limit,_cmd_control_period;
	std::string hit_word;

	//gyro
	void gyroReceived(const sensor_msgs::Imu::ConstPtr& gyro);



};


	int initial_rs( char *device ,int *fd_rs);
	void gyro_ThreadFunction();
	float cul_rad(int vref,int vrate);
	float ema_cul(float *buf, int rcount);
	int get_ad();

	void Thread_End ( void );
	void imu_publisher(void);
	void odom_default_pub(void);
	void get_odom_trans_default(void);

class IMU{
public:
	boost::thread th_;
	IMU();
	~IMU();
private:
};

#endif
