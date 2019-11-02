/*-----------------------------------------------
 * 	okatech_kuaro.h
 * <Last Update>	H29/09/28
 * <editor> Takafumi ONO
 * <version>		v2.1
 * ---------------------------------------------*/
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <math.h>


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// #ifndef OKETECH_H_
// #define OKETECH_H_

///制御周期
// #define MMR_CONTROL_PERIOD 25	//MMR送るタイミング
// #define WRITING_SLEEP_TIME 1	//その他書き込み
//
// //Okatechの機構パラメータ
// #define OKATECH_D 0.254     	//タイヤ直径[m]
// #define OKATECH_TREAD 0.47 		//KUARO幅[m]
// #define OKATECH_PR 120000		//パルス*ギア比
//
// #define VELOCITY_TO_ANGULAR_VELOCITY(V) (V/(OKATECH_D*0.50))
// #define NANOSEC_TO_SEC(NANOSEC) (NANOSEC*0.000000001)
//
// //速度制限
// #define OKATECH_LIMIT_SPEED_V 0.7
// #define OKATECH_LIMIT_SPEED_R VELOCITY_TO_ANGULAR_VELOCITY(OKATECH_LIMIT_SPEED_V)
// #define OKATECH_TURN_LIMIT 0.2
// #define OKATECH_LIMIT_CV 2.0
//
// #define CMD_ACC_LIMIT 0.2
// #define CMD_ANG_ACC_LIMIT 0.4
// #define CMD_CONTROL_PERIOD 10
//
// //円周率
// #ifndef M_PI
// #define M_PI 3.14159265
// #endif

static const double encoder_get_time = 0.05;

class OKATECH
{
private:

	std::string sub_serial_topic_;
	std::string sub_cmdvel_topic_;
	std::string pub_serial_topic_;
	std::string pub_odom_topic_;
	std::string frame_id, child_frame_id;

	ros::Publisher pub_ToSerial_;
	ros::Publisher pub_odom_;
	ros::Subscriber sub_velcmd_;
	ros::Subscriber sub_Serial_;
	tf::TransformBroadcaster odom_broadcaster;
	geometry_msgs::TransformStamped odom_trans;

	double th_rad_;

public:
	OKATECH();
	~OKATECH();

	void ReceiveComandFromSerial_Callback(const std_msgs::String::ConstPtr& str_msg_);
	void SendComandToSerial_Callback(const geometry_msgs::Twist::ConstPtr& vel_msg);
	std::string ToStringCmd_linear_vel(double linear_vel);
	std::string ToStringCmd_angular_vel(double angular_vel);
	void cmd_timer();

  nav_msgs::Odometry odom_;
	ros::Timer timer_;

};











// /////////////////////////////////////////////////////////////////////////////////
// #include "uSerialPort.h"
// //boost
// #include <boost/timer/timer.hpp>
// #include <boost/thread.hpp>
// #include <boost/bind.hpp>
// //ROS用
// #include <ros/ros.h>
// #include <sensor_msgs/Joy.h>			//ゲームパッド
// //#include <sound_play/sound_play.h>		//sound
// #include <geometry_msgs/Twist.h>		// cmd_vel
// #include <nav_msgs/Odometry.h>
// #include <tf/transform_broadcaster.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/String.h>
// #include <sensor_msgs/Imu.h>
//
// #include <sstream>
// #include <string>
// #include <iostream>
// #include <time.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <math.h>
// #include <vector>
//
// #include <sys/types.h>
// #include <sys/stat.h>
// #include <unistd.h>
// #include <fcntl.h>
// #include <termios.h>
// #include <sys/time.h>
//
// //#include "kbhit.h"
// #include <pthread.h>
// /*-----#define定義-------------------------------*/
// //COMポート
// #define OKATECH_COMPORT "COM4"
//
// using namespace boost;
// using namespace std;
//
// //制御周期
// #define MMR_CONTROL_PERIOD 25	//MMR送るタイミング
// #define WRITING_SLEEP_TIME 1	//その他書き込み
//
// //Okatechの機構パラメータ
// #define OKATECH_D 0.254     	//タイヤ直径
// #define OKATECH_TREAD 0.47 		//KUARO幅
// #define OKATECH_PR 120000		//パルス*ギア比
//
// //#defineマクロ
// #define VELOCITY_TO_ANGULAR_VELOCITY(V) (V/(OKATECH_D*0.50))
// #define NANOSEC_TO_SEC(NANOSEC) (NANOSEC*0.000000001)
//
// //リミッター
// #define OKATECH_LIMIT_SPEED_V 0.7
// #define OKATECH_LIMIT_SPEED_R VELOCITY_TO_ANGULAR_VELOCITY(OKATECH_LIMIT_SPEED_V)
// #define OKATECH_TURN_LIMIT 0.2	//その場回転用
// #define OKATECH_LIMIT_CV 2.0
//
// #define CMD_ACC_LIMIT 0.2
// #define CMD_ANG_ACC_LIMIT 0.4
// #define CMD_CONTROL_PERIOD 10
//
// //円周率
// #ifndef M_PI
// #define M_PI 3.14159265
// #endif
//
// //Okatechの回転角度の範囲
// //#define MTH_RANGE_0_2PI	//	0≦TH≦2π
// //#define MTH_RANGE_PI_PI	//-π≦TH≦π
// #define MTH_RANGE_OO_OO	//-∞≦TH≦∞
//
// /**  WRITE_MEを定義すればMEを書き込み制御する	**/
// /**	 デフォルトではMMR書き込み				**/
// //#define WRITE_ME
// #ifdef WRITE_ME
// #define WRITE_MESSAGE "\rME\r"
// #else
// #define WRITE_MESSAGE "\rMMR\r"
// #endif
//
// #ifdef __linux__
// //ros/sound_play関連
// // #define SOUND_ALL 51
// // #define SOUND_PASS "/home/urobo/ミュージック/sound"
// #endif
//
// /*-----構造体-------------------------------*/
// //READがv1,v2で読みにくい?Rodecはこっちの方が
// //都合がいいんだよ(´・ω・｀)
// struct OKA_WHEEL{
// 	struct READ{
// 		double d1, d2;
// 		double v1, v2;
// 	}read;
//
// 	struct WRITE{
// 		double vR,vL;
// 	}write;
//
// 	double v;
// 	double rv;
// 	double cv;
// };
//
// struct OKA_POSITION{
// 	double x,y,th;
// 	double dx,dy,dth;
// 	double dx_v,dy_v,dw;
// 	double dtime;
// };
//
// /*-----クラス宣言-------------------------------*/
// //Okatech用オブザーバー
// class Observer : public serial::SerialObserver
// {
// public:
// 	Observer(){}
// 	virtual ~Observer()
// 	{}
//
// private:
//
// #ifdef WRITE_ME
// 	int interim_1, interim_2, interim_3, interim_4;
// #else
// 	double interim_1, interim_2;
// #endif
//
// 	static boost::timer::auto_cpu_timer time_interval;
// // protected:
// // 	void notify(const std::string& str);
// };
//
// //引数例　U_OKATECH okatech("COM4")
// class U_OKATECH
// {
// private:
// 	//TinyPowerの通信用
// 	serial::SerialPort TinPow;
// 	//一定周期に書き込む
// 	boost::thread Write_Const;
// 	//一定周期でcmd_velをキャッチする
// 	boost::thread Chatch_Const;
//
// 	bool thread_end_flag;
// 	long long int time_limit;
// 	void writeMMR(void);
// 	void cmd_sts_control(void);
// 	Observer ob;
//
// 	bool stop_flag;
// 	bool old_stop_flag;
//
// 	double global_cmd_v;
// 	double global_cmd_dv;
//
// public:
// 	U_OKATECH(std::string Port);
// 	~U_OKATECH();
// 	static OKA_WHEEL wheel;
// 	static OKA_POSITION position;
// 	void u_write(const std::string& text);
// 	void runV(double V);
// 	void runV_R(double V);
// 	void runV_L(double V);
// 	void runV_cv(void);
// 	void runV_cv_slow(void);
// 	void runR(double R);
// 	void runR(void);
// 	static void update_position(void);
// 	bool wheel1r(double rps);	bool wheel2r(double rps);
// 	bool wheel1v(double v);		bool wheel2v(double v);
// 	void stop(void);
// 	void stop_slow(void);
//
// 	void u_Joy_Control(const sensor_msgs::Joy::ConstPtr& joy);
// 	void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& twist);
// 	void emergency_func(const std_msgs::Bool::ConstPtr &state);
// 	bool joy_use_flag;
//
// 	//人追従追加分
// 	void Wheel_Control_Outside(const std_msgs::StringConstPtr& msg);
// 	void Odometry_Save(const std_msgs::StringConstPtr& msg);
// 	double KV,KLV,KT,KTD,INVIOLABLE_AREA;
// 	double _cmd_acc_limit,_cmd_ang_acc_limit,_cmd_control_period;
// 	std::string hit_word;
//
// 	//gyro
// 	void gyroReceived(const sensor_msgs::Imu::ConstPtr& gyro);
// };
//
//
// 	int initial_rs( char *device ,int *fd_rs);
// 	void gyro_ThreadFunction();
// 	float cul_rad(int vref,int vrate);
// 	float ema_cul(float *buf, int rcount);
// 	int get_ad();
//
// 	void Thread_End ( void );
// 	void imu_publisher(void);
// 	void odom_default_pub(void);
// 	void get_odom_trans_default(void);
//
// class IMU{
// public:
// 	boost::thread th_;
// 	IMU();
// 	~IMU();
// private:
// };
//
// #endif
