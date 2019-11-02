/*-----------------------------------------------
* 	okatech_kuaro.cpp
* <Last Update>	H29/08/27
* <editor> Takafumi ONO
* <version>		v2.0
* ジャイロはまだ実装していない
* ---------------------------------------------*/
#include "okatech_kuaro.h"

//gccの古いバージョンでもto_stringを使えるようにする
namespace patch_std
{
  template < typename T > std::string to_string( const T& n )
  {
    std::ostringstream stm ;
    stm << n ;
    return stm.str() ;
  }
}

OKATECH::OKATECH()
{
  ros::NodeHandle n;

  odom_.header.stamp    = ros::Time::now();
  odom_.header.frame_id = "odom";
  odom_.child_frame_id  = "base_footprint";

  odom_.pose.pose.position.x  = 0.0;
  odom_.pose.pose.position.y  = 0.0;
  odom_.pose.pose.position.z  = 0.0;
  th_rad_ = 0.0;
  odom_.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

  odom_.twist.twist.linear.x  = 0.0;
  odom_.twist.twist.linear.y  = 0.0;
  odom_.twist.twist.linear.z  = 0.0;
  odom_.twist.twist.angular.x = 0.0;
  odom_.twist.twist.angular.y = 0.0;
  odom_.twist.twist.angular.z = 0.0;

  pub_ToSerial_ = n.advertise<std_msgs::String>("serial_send",10);
  pub_odom_ = n.advertise<nav_msgs::Odometry>("odom",10);
  sub_Serial_ = n.subscribe("/serial_receive",10,&OKATECH::ReceiveComandFromSerial_Callback,this);
  sub_velcmd_ = n.subscribe("/cmd_vel",10,&OKATECH::SendComandToSerial_Callback,this);

  //publishの速度
  timer_ = n.createTimer(ros::Duration(encoder_get_time), boost::bind(&OKATECH::cmd_timer, this));

}


OKATECH::~OKATECH()
{

}

// //MMVコマンドをTinyPowerに送る(スレッドで動作)
// void OKATECH::sendMVV(void)
// {
//
//   ros::Rate loop_rate(20);
//
//   while(ros::ok())
//   {
//     TinPow.u_send("\rMVV\r");
//     loop_rate.sleep();
//   }
//
// }

//String型のメッセージを受け取り、オカテックのパラメータに代入
void OKATECH::ReceiveComandFromSerial_Callback(const std_msgs::String::ConstPtr& str_msg_)
{

    std::string str = str_msg_->data;

  	double read_data1_ = 0;
    double read_data2_ = 0;
    int read_data3_ = 0;
    int read_data4_ = 0;
  	static bool ustri_flag = false;
  	static bool read_flag = false;
  	static std::string extraction;

  	for (unsigned int it = 0; it < (unsigned int)str.size(); it++)
  	{
  		if(ustri_flag && str[it] == '>'){
  			read_flag = true;
        break;
  		}
  		if (ustri_flag && str[it] != '\n' && str[it] != '\r'){
  			extraction += str[it];
  		}
  		if (str[it] == ':'){
  			ustri_flag = true;
  		}
  	}

  	if(!read_flag)
    {
      return;
    }
    else
    {
  		ustri_flag = false;
  		read_flag = false;
  		//std::cout <<"extraction"<< extraction << std::endl;
  		sscanf(extraction.c_str(), "%lf,%lf,%d,%d", &read_data1_, &read_data2_, &read_data3_, &read_data4_);
      //std::cout<<"read_data:"<<read_data1_<<","<<read_data2_<<","<<read_data3_<<","<<read_data4_<<std::endl;

      odom_.header.stamp  = ros::Time::now();
    	//set the position
    	odom_.pose.pose.position.x  += read_data1_*cos(th_rad_)*encoder_get_time;
    	odom_.pose.pose.position.y  += read_data1_*sin(th_rad_)*encoder_get_time;
    	odom_.pose.pose.position.z  = 0.0;
      th_rad_ = th_rad_+read_data2_*encoder_get_time;
    	odom_.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, th_rad_);
    	//set the velocity
    	odom_.twist.twist.linear.x  = read_data1_;
    	odom_.twist.twist.linear.y  = 0.0;
    	odom_.twist.twist.linear.z  = 0.0;
    	odom_.twist.twist.angular.x = 0.0;
    	odom_.twist.twist.angular.y = 0.0;
    	odom_.twist.twist.angular.z = read_data2_;

      pub_odom_.publish(odom_);

  		//これを入れると止まっている時などは更新しなくなる
  		//if (fabs(interim_3) + fabs(interim_4) > 1){
  			// OKATECH::wheel.read.d1 = (double)read_data3_;
  			// OKATECH::wheel.read.d2 = (double)read_data4_;
  			// OKATECH::update_position();
  			//std::cout << "(d1 , d2) : " << U_OKATECH::wheel.read.d1 << " , " << U_OKATECH::wheel.read.d2 << std::endl;
  		//}

      extraction.clear();

  	}
}

//call_back関数stringをTinyPowerにシリアル通信で送る
void OKATECH::SendComandToSerial_Callback(const geometry_msgs::Twist::ConstPtr& vel_msg)
{
  std_msgs::String msg_ToTinyPower;

  std::string str = ToStringCmd_linear_vel(vel_msg->linear.x);
  //pub_ToSerial_.publish(msg_ToTinyPower);

  if(vel_msg->linear.x>=0.0)
  {
    str+=ToStringCmd_angular_vel(vel_msg->angular.z);
  }
  else
  {
    str+=ToStringCmd_angular_vel(-vel_msg->angular.z);
  }

  msg_ToTinyPower.data = str;
  pub_ToSerial_.publish(msg_ToTinyPower);

}

//前進速度制御　コマンド例（0.5[m/s]前進：VCX0.5）a
std::string OKATECH::ToStringCmd_linear_vel(double linear_vel)
{
  std::string cmd = "\rVCX" + patch_std::to_string(linear_vel)+"\r";
  //std::cout<<"VCX="<<cmd<<std::endl;
  return cmd;
}

//回頭速度制御　コマンド例（1[rad/s]旋回：VCR1）
std::string OKATECH::ToStringCmd_angular_vel(double angular_vel)
{
  std::string cmd = "\rVCR" + patch_std::to_string(angular_vel)+"\r";
  //std::cout<<"VCR="<<cmd<<std::endl;
  return cmd;
}

//MMVコマンドをTinyPowerに送るタイマー関数
void OKATECH::cmd_timer(void)
{
  std_msgs::String msg;
  msg.data = "\rMVV\r";
  pub_ToSerial_.publish(msg);

  // ros::Rate loop_rate(20);
  //
  // while(ros::ok())
  // {
  //   TinPow.u_send("\rMVV\r");
  //   loop_rate.sleep();
  // }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "TinyPower_control");

  OKATECH okatech;

  ros::spin();

  return 0;
}

// ////////////////////////////////////////////////////
// #include "uk_okatech.h"
// #define MTH_RANGE_PI_PI
// #include <fstream>
//
// OKA_WHEEL U_OKATECH::wheel;
// OKA_POSITION U_OKATECH::position;
//
// //gyro
// double g_rad_now = 0.0;
// double g_drad_now = 0.0;
// double g_rad_ago = 0.0;
// double g_drad = 0;
// double o_drad = 0;
// double o_rad = 0;
// double o_x = 0;
// double o_y = 0;
// double o_dx = 0;
// double o_dy = 0;
// bool gyro_flag = false;
//
// #define G_COM "/dev/sensors/ttyUSBGyro"
// #define BUFFER_SIZE_ 128
// #define bufsize 20
// #define rlimit 5
//
// using namespace boost;
// using namespace std;
//
// int fd_rs;
// ros::Publisher pub;
// ros::Publisher pub_odom;
// char g_filepath [128];
// sensor_msgs::Imu imu;
// int Port;
// char* Sensor_Name;
// char* COM_No;
// int BaudRate;
//
// int len;
// char C;
// char ref_buf[bufsize];
// char rate_buf[bufsize];
// bool space_flag;
// struct timeval s, e;
//
// int rcount;
// float rad_buf[rlimit];
// float tmp;
// bool ema_flag;
// bool read_error_flag;
// unsigned char rbuf[BUFFER_SIZE_];
// float angle_ago;
// float angle_now;
// float angle_now_2;
// float result_rad;
// float time_now;
// float time_ago;
//
// //コンストラクタ
// U_OKATECH::U_OKATECH(std::string Port)
// {
// 	while (true)
// 	{
// 		if (TinPow.open(Port))
// 		{
// 			puts("Linux OKATECH 通信開始");
// 			break;
// 		}
// 		else
// 		{
// 			char judge = '0';
// 			puts("-- OPEN ERROR --");
// 			std::cout << "COMのPORT番号を変更しますか ? (y/n)   ";
// 			std::cin >> judge;
// 			if (judge == 'y')
// 			{
// 				std::cout << "PORT を入力してください\t" << "→  ";
// 				std::cin >> Port;
// 			}
// 			else
// 			{
// 				std::cout << "もう一度接続しますか ? (y/n)   ";
// 				std::cin >> judge;
// 				if (judge == 'y')continue;
// 				puts("プログラムを終了します");
// 				exit(0);
// 			}
// 		}
// 	}
//
// 	// オブザーバー登録
// 	TinPow.attach(&ob);
// 	Write_Const = boost::thread(&U_OKATECH::writeMMR, this);
// 	Chatch_Const = boost::thread(&U_OKATECH::cmd_sts_control, this);
//
// 	//position初期化
// 	position.x    =0.0; position.y    =0.0; position.th   =0.0;
// 	position.dx   =0.0; position.dy   =0.0; position.dth  =0.0;
// 	position.dx_v =0.0; position.dy_v =0.0; position.dw   =0.0;
// 	position.dtime=0.0;
// 	//wheel初期化
// 	wheel.v=0.0; wheel.rv = 0.0; wheel.cv = 0.0;
// 	wheel.write.vR = 0.0; wheel.write.vL = 0.0;
//
// 	time_limit = MMR_CONTROL_PERIOD;
// 	thread_end_flag = false;
// 	joy_use_flag = false;
//
// 	global_cmd_v = 0.0;
// 	global_cmd_dv = 0.0;
// }
//
// //デストラクタ
// U_OKATECH::~U_OKATECH()
// {
// 	stop();
// 	// オブザーバー登録解除
// 	TinPow.detach(&ob);
// 	TinPow.close();
// 	thread_end_flag = true;
// 	if (Write_Const.joinable())Write_Const.join();
// 	if (Chatch_Const.joinable())Chatch_Const.join();
// 	puts("good bye okatech");
// }
//
// //MMR書き込み用
// void U_OKATECH::writeMMR(void)
// {
// 	boost::timer::auto_cpu_timer timer;
// 	timer.start();
// 	while (1)
// 	{
// 		//処理
// 		TinPow.u_send(WRITE_MESSAGE);
// 		if (thread_end_flag)return;
// 		timer.stop();
// 		if (((time_limit * 1000000) - timer.elapsed().wall) >= 0) {
// 			boost::this_thread::sleep_for(boost::chrono::nanoseconds((time_limit * 1000000) - timer.elapsed().wall));
// 		}
// 		timer.start();
// 	}
// }
//
// //書き込む
// void U_OKATECH::u_write(const std::string& text)
// {
// 	if(!TinPow.send(text));
// 	//	std::cout<< "text = " << text<<std::endl;
// }
//
// //wheel1:右車輪
// bool U_OKATECH::wheel1r(double rps)
// {
// 	if (fabs(rps) > OKATECH_LIMIT_SPEED_R)
// 	{
// 		puts("------ DANGER OKATECH_LIMIT_SPEED_R OVER ------");
// 		return false;
// 	}
// 	char str[128]={0};
// 	#ifdef __linux__
// 	sprintf(str, "\r\rRC1RS%5.4lf\r\r", rps);
// 	#endif
// 	//std::cout<<"				wheel1r = "<< str <<std::endl;
// 	u_write(str);
// 	return true;
// }
// //wheel2:左車輪
// bool U_OKATECH::wheel2r(double rps)
// {
// 	if (fabs(rps) > OKATECH_LIMIT_SPEED_R)
// 	{
// 		puts("------ DANGER OKATECH_LIMIT_SPEED_R OVER ------");
// 		return false;
// 	}
// 	char str[128]={0};
// 	#ifdef __linux__
// 	sprintf(str, "\r\rRC2RS%5.4lf\r\r", rps);
// 	#endif
// 	//std::cout<<"				wheel2r = "<< str <<std::endl;
// 	u_write(str); return true;
// }
//
// //wheel1:右車輪
// bool U_OKATECH::wheel1v(double v)
// {
// 	boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
// 	if (fabs(v) > OKATECH_LIMIT_SPEED_V)
// 	{
// 		puts("------ DANGER OKATECH_LIMIT_SPEED_V OVER ------");
// 		return false;
// 	}
// 	wheel.write.vR = v;
// 	wheel1r(VELOCITY_TO_ANGULAR_VELOCITY(v));
// 	return true;
// }
// //wheel2:左車輪
// bool U_OKATECH::wheel2v(double v)
// {
// 	boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
// 	if (fabs(v) > OKATECH_LIMIT_SPEED_V)
// 	{
// 		puts("------ DANGER OKATECH_LIMIT_SPEED_V OVER ------");
// 		return false;
// 	}
// 	wheel.write.vL = v;
// 	wheel2r(VELOCITY_TO_ANGULAR_VELOCITY(v));
// 	return true;
// }
//
// //現在位置の更新
// void U_OKATECH::update_position(void)
// {
// 	const int buf_s = 15;
// 	#ifdef WRITE_ME // ---------------------------------------------------------
// 	static boost::timer::auto_cpu_timer time_interval;
// 	static bool first_process = true;
// 	double updl  = 0;
//
// 	static double read_v1[buf_s];
// 	static double read_v2[buf_s];
// 	static double dtime_[buf_s];
// 	static double updl_[buf_s];
// 	static int ii = 0;
// 	static bool ii_flag = false;
// 	if (first_process)
// 	{
// 		time_interval.start();
// 		first_process = false;
// 	}
// 	else
// 	{
//
// 		time_interval.stop();
// 		read_v1[ii] = wheel.read.d1 / OKATECH_PR * OKATECH_D * M_PI;
// 		read_v2[ii] = wheel.read.d2 / OKATECH_PR * OKATECH_D * M_PI;
// 		dtime_[ii] = NANOSEC_TO_SEC(time_interval.elapsed().wall);
// 		updl_[ii]  = (wheel.read.v1 + wheel.read.v2) * 0.50;
// 		ii++;
// 		if(ii == buf_s)
// 		{
// 			ii_flag = true;
// 			ii = 0;
// 		}
//
// 		if(ii_flag)
// 		{
// 			wheel.read.v1 = read_v1[ii];
// 			wheel.read.v2 = read_v2[ii];
// 			position.dtime = dtime_[ii];
// 			updl  = updl_[ii];
//
// 			//time_interval.stop();
// 			//wheel.read.v1 = wheel.read.d1 / OKATECH_PR * OKATECH_D * M_PI;
// 			//wheel.read.v2 = wheel.read.d2 / OKATECH_PR * OKATECH_D * M_PI;
// 			//position.dtime = NANOSEC_TO_SEC(time_interval.elapsed().wall);
// 			//updl  = (wheel.read.v1 + wheel.read.v2) * 0.50;
// 			//回転角度の微小変化
// 			if(gyro_flag)
// 			{
// 				g_rad_now = -angle_now;
// 				g_drad = g_rad_now - g_rad_ago;
// 				position.dth = g_drad;//gyro
// 				g_rad_ago = g_rad_now;
// 				//std::cout<<"g_rad_now1 = "<<g_rad_now<<std::endl;
// 			}
// 			else
// 			{
// 				position.dth = (wheel.read.v1 - wh eel.read.v2)/ OKATECH_TREAD;
// 			}
// 			o_drad = (wheel.read.v1 - wheel.read.v2)/ OKATECH_TREAD;
// 			o_rad += o_drad;
// 			while (o_rad>M_PI)o_rad -= 2 * M_PI;
// 			while (o_rad<-M_PI)o_rad += 2 * M_PI;
//
// 			position.dx   = updl * cos(position.th);
// 			position.dy   = updl * sin(position.th);
//
// 			//自己位置	 <-	距離の微小変化
// 			position.x   += position.dx;
// 			position.y   += position.dy;
// 			position.th  += position.dth;
//
// 			//x方向速度  y方向速度  回転速度の微小変化
// 			position.dx_v = position.dx / position.dtime;
// 			position.dy_v = position.dy / position.dtime;
// 			position.dw   = position.dth/ position.dtime;
// 			time_interval.start();
//
// 			#ifdef MTH_RANGE_0_2PI
// 			while(position.th>M_PI*2.0)position.th -= 2 * M_PI;
// 			while (position.th < 0)position.th += 2 * M_PI;
// 			#endif //MTH_RANGE_0_2PI
// 			#ifdef MTH_RANGE_PI_PI
// 			while (position.th>M_PI)position.th -= 2 * M_PI;
// 			while (position.th<-M_PI)position.th += 2 * M_PI;
// 			#endif //MTH_RANGE_PI_PI
// 			//座標等出力
// 			//std::cout << "(dl , dth , p) : " << "( " << updl << " , " << position.dth << " , " << updl / position.dth<< ") " << std::endl;
// 			//std::cout << "(v1 , v2 ) : " << "( " << wheel.read.v1  << " , " << wheel.read.v2  << ") " << std::endl;
// 			//std::cout << "(X , Y , TH) : " << "( " << position.x << " , " << position.y << " , " << position.th<< ") " << std::endl;
// 		}
// 	}
// 	#else //WRITE_ME ---------------------------------------------------------
//
// 	static boost::timer::auto_cpu_timer time_interval;
// 	static bool first_process = true;
// 	double updl  = 0;
//
// 	static double read_v1[buf_s];
// 	static double read_v2[buf_s];
// 	static double dtime_[buf_s];
// 	static double updl_[buf_s];
// 	static int ii = 0;
// 	static bool ii_flag = false;
//
// 	if (first_process)
// 	{
// 		time_interval.start();
// 		read_v1[ii] = wheel.read.d1*(OKATECH_D / 2.0);
// 		read_v2[ii] = wheel.read.d2*(OKATECH_D / 2.0);
// 		//wheel.read.v1 = wheel.read.d1*(OKATECH_D / 2.0);
// 		//wheel.read.v2 = wheel.read.d2*(OKATECH_D / 2.0);
// 		ii++;
// 		first_process = false;
// 	}
// 	else
// 	{
// 		time_interval.stop();
// 		read_v1[ii] = wheel.read.d1*(OKATECH_D / 2.0);
// 		read_v2[ii] = wheel.read.d2*(OKATECH_D / 2.0);
// 		dtime_[ii] = NANOSEC_TO_SEC(time_interval.elapsed().wall);
// 		updl_[ii]  = (wheel.read.v1 + wheel.read.v2) * 0.50 * position.dtime;
// 		ii++;
// 		if(ii == buf_s)
// 		{
// 			ii_flag = true;
// 			ii = 0;
// 		}
//
// 		if(ii_flag)
// 		{
// 			wheel.read.v1 = read_v1[ii];
// 			wheel.read.v2 = read_v2[ii];
// 			position.dtime = dtime_[ii];
// 			updl  = updl_[ii];
//
// 			//time_interval.stop();
// 			//	wheel.read.v1 = wheel.read.d1*(OKATECH_D / 2.0);
// 			//wheel.read.v2 = wheel.read.d2*(OKATECH_D / 2.0);
// 			//position.dtime = NANOSEC_TO_SEC(time_interval.elapsed().wall);
// 			//updl  = (wheel.read.v1 + wheel.read.v2) * 0.50 * position.dtime;
// 			//回転角度の微小変化
// 			if(gyro_flag)
// 			{
// 				g_rad_now = -angle_now;
// 				g_drad = g_rad_now - g_rad_ago;
// 				position.dth = g_drad;//gyro
// 				g_rad_ago = g_rad_now;
// 				//	std::cout<<"g_rad_now2 = "<<g_rad_now<<std::endl;
// 			}
// 			else
// 			{
// 				position.dth = (wheel.read.v1 - wheel.read.v2)/ OKATECH_TREAD * position.dtime;
// 			}
// 			imu_publisher();
// 			odom_default_pub();
// 			get_odom_trans_default();
// 			o_drad = (wheel.read.v1 - wheel.read.v2)/ OKATECH_TREAD * position.dtime;
//
// 			while (o_rad>M_PI)o_rad -= 2 * M_PI;
// 			while (o_rad<-M_PI)o_rad += 2 * M_PI;
//
// 			if(position.dth == 0)
// 			{
// 				position.dx   = updl * cos(position.th + position.dth*0.50);
// 				position.dy   = updl * sin(position.th + position.dth*0.50);
// 			}else
// 			{
// 				position.dx = 2.0 * (updl / position.dth)*sin(position.dth*0.5) * cos(position.th + position.dth*0.50);
// 				position.dy = 2.0 * (updl / position.dth)*sin(position.dth*0.5) * sin(position.th + position.dth*0.50);
// 			}
//
// 			if(o_drad == 0)
// 			{
// 				o_dx   = updl * cos(o_rad + o_drad*0.50);
// 				o_dy   = updl * sin(o_rad + o_drad*0.50);
// 			}
// 			else
// 			{
// 				o_dx = 2.0 * (updl / o_drad)*sin(o_drad*0.5) * cos(o_rad + o_drad*0.50);
// 				o_dy = 2.0 * (updl / o_drad)*sin(o_drad*0.5) * sin(o_rad + o_drad*0.50);
// 			}
//
// 			o_x += o_dx;
// 			o_y += o_dy;
// 			o_rad += o_drad;
//
// 			//自己位置	 <-	距離の微小変化
// 			position.x   += position.dx;
// 			position.y   += position.dy;
// 			position.th  += position.dth;
// 			//x方向速度  y方向速度  回転速度の微小変化
// 			position.dx_v = position.dx / position.dtime;
// 			position.dy_v = position.dy / position.dtime;
// 			position.dw   = position.dth/ position.dtime;
//
// 			time_interval.start();
//
// 			#ifdef MTH_RANGE_0_2PI
// 			while(position.th>M_PI*2.0)position.th -= 2 * M_PI;
// 			while (position.th < 0)position.th += 2 * M_PI;
// 			#endif //MTH_RANGE_0_2PI
// 			#ifdef MTH_RANGE_PI_PI
// 			while (position.th>M_PI)position.th -= 2 * M_PI;
// 			while (position.th<-M_PI)position.th += 2 * M_PI;
// 			#endif //MTH_RANGE_PI_PI
// 			//座標等出力
// 			//std::cout << "(dl , dth , p) : " << "( " << updl << " , " << position.dth << " , " << updl / position.dth<< ") " << std::endl;
// 			//std::cout << "(v1 , v2 ) : " << "( " << wheel.read.v1  << " , " << wheel.read.v2  << ") " << std::endl;
// 			//std::cout << "(X , Y , TH, orad) : " << "( " << position.x << " , " << position.y << " , " << position.th<< " , "  << o_rad<<") " << std::endl;
// 		}
// 	}
// 	#endif //WRITE_ME ---------------------------------------------------------
// }
//
// //直進
// void U_OKATECH::runV(double V)
// {
// 	wheel1v(V);
// 	wheel2v(V);
// }
//
// //右車輪
// void U_OKATECH::runV_R(double V)
// {
// 	wheel1v(V);
// }
//
// //左車輪
// void U_OKATECH::runV_L(double V)
// {
// 	wheel2v(V);
// }
//
// //回転
// void U_OKATECH::runR(double R)
// {
// 	wheel.cv = 0.0;
// 	if(R > OKATECH_TURN_LIMIT){R=OKATECH_TURN_LIMIT;}
// 	if(R < -OKATECH_TURN_LIMIT){R=-OKATECH_TURN_LIMIT;}
// 	wheel1v(R);
// 	wheel2v(-R);
// }
//
// //その場回転
// void U_OKATECH::runR(void)
// {
// 	wheel.v = 0.0;
// 	wheel.cv = 0.0;
// 	wheel1v(wheel.rv);
// 	wheel2v(-wheel.rv);
// }
//
// //曲率移動
// void U_OKATECH::runV_cv()
// {
// 	wheel.rv = 0.0;
// 	wheel2v(wheel.v * (1.0 - (wheel.cv * (OKATECH_TREAD / 2.0))));	//左
// 	wheel1v(wheel.v * (1.0 + (wheel.cv * (OKATECH_TREAD / 2.0))));	//右
// }
//
//
// //曲率移動
// void U_OKATECH::runV_cv_slow()
// {
// 	static clock_t s_time = clock();
// 	double cv_v_ago = 0.0, cv_dv_ago = 0.0;
//
// 	if ((double)(clock() - s_time) / CLOCKS_PER_SEC < _cmd_control_period * 0.001)
// 	{
// 		runV_cv_slow();
// 		return;
// 	}
// 	else
// 	{
// 		s_time = clock();
// 	}
// 	cv_v_ago = (wheel.write.vL + wheel.write.vR) * 0.50;
//
// 	if (fabs(cv_v_ago - wheel.v) > _cmd_acc_limit)
// 	wheel.v = (cv_v_ago > wheel.v ? (cv_v_ago - _cmd_acc_limit) : (cv_v_ago + _cmd_acc_limit));
//
// 	wheel.rv = 0.0;
// 	wheel2v(wheel.v * (1.0 - (wheel.cv * (OKATECH_TREAD / 2.0))));	//左
// 	wheel1v(wheel.v * (1.0 + (wheel.cv * (OKATECH_TREAD / 2.0))));	//右
// }
//
// //停止
// void U_OKATECH::stop(void)
// {
// 	wheel.write.vL = 0.0;
// 	wheel.write.vR = 0.0;
// 	wheel.cv = 0.0;
// 	wheel.v=0.0;
// 	wheel.rv = 0.0;
// 	u_write("\rVCX0.0\r");
// 	u_write("\rVCR0.0\r");
// }
//
// //ゆっくり停止
// void U_OKATECH::stop_slow()
// {
// 	int stop_count = (fabs(wheel.write.vL) > fabs(wheel.write.vR)) ? (int)(fabs(wheel.write.vL) * 100) : (int)(fabs(wheel.write.vR) * 100);
// 	if(stop_count < 1)
// 	{
// 		stop();
// 		return;
// 	}
// 	double reduction_r = wheel.write.vR/stop_count;
// 	double reduction_l = wheel.write.vL/stop_count;
// 	for(int ust = 0; ust < stop_count ;ust++)
// 	{
// 		runV_L ( wheel.write.vL - reduction_l );
// 		runV_R ( wheel.write.vR - reduction_r  );
// 		boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
// 	}
// 	stop();
// }
//
// #ifdef __linux__
// //u_Joy_Control
// void U_OKATECH::u_Joy_Control(const sensor_msgs::Joy::ConstPtr& joy)
// {
// 	static int turn_flag = 0;			//初期:0	前進:-1		回転:1
// 	int num;							//ループ用
// 	bool key_remove = true;				//キーから手を離した時を検知するのに使用
// 	//キーパッドが押された時
// 	for (num = 0; num < joy->axes.size(); num++)
// 	{
// 		if (joy->axes[num] != 0)
// 		{
// 			if(num > 3 && turn_flag >= 0)
// 			{
// 				stop_slow();
// 				turn_flag=-1;
// 			}
//
// 			switch (num)
// 			{
// 				case 0:case 1:case 2:case 3:
// 				puts("アナログスティック使用禁止");
// 				return;
// 				break;
// 				case 4:
// 				//左
// 				if (joy->axes[num] > 0)
// 				{
// 					//puts("左");
// 					if (wheel.cv < OKATECH_LIMIT_CV)
// 					{
// 						wheel.cv += 0.5;
// 						runV_cv_slow();
// 					}
// 				}
// 				else
// 				{
// 					//puts("右");
// 					if (wheel.cv > -OKATECH_LIMIT_CV)
// 					{
// 						wheel.cv -= 0.5;
// 						runV_cv_slow();
// 					}
// 				}
// 				break;
// 				case 5:
// 				//上
// 				if (joy->axes[num] > 0)
// 				{
// 					//puts("上");
// 					if (wheel.v < OKATECH_LIMIT_SPEED_V - 0.05)
// 					{
// 						wheel.v += 0.05;
// 						runV_cv_slow();
// 					}
// 				}
// 				else
// 				{
// 					//puts("下");
// 					if (wheel.v > -OKATECH_LIMIT_SPEED_V - 0.05)
// 					{
// 						wheel.v -= 0.05;
// 						runV_cv_slow();
// 					}
// 				}
// 				break;
// 				default:
// 				break;
// 			}
// 			key_remove = false;
// 		}
// 	}
//
// 	if(!joy_use_flag)puts("-----    joy ON    -----");
// 	joy_use_flag = true;
// 	//ボタンが押された時
// 	for (num = 0; num<joy->buttons.size(); num++)
// 	{
// 		if (joy->buttons[num] == 1)
// 		{
// 			switch (num)
// 			{
// 				//STOP	ボタンから手を離したら止まるにしてる場合はコメントアウト
// 				case 2:
// 				stop_slow();
// 				break;
// 				//左回転
// 				case 4: case 6:
// 				if(turn_flag <= 0)
// 				{
// 					turn_flag=1;
// 					stop_slow();
// 				}
// 				if (wheel.rv < OKATECH_TURN_LIMIT-0.02)
// 				{
// 					wheel.rv += 0.02;
// 					runR();
// 				}
// 				break;
// 				//右回転
// 				case 5: case 7:
// 				if(turn_flag <= 0)
// 				{
// 					turn_flag=1;
// 					stop_slow();
// 				}
// 				if (wheel.rv > -OKATECH_TURN_LIMIT+0.02)
// 				{
// 					wheel.rv -= 0.02;
// 					runR();
// 				}
// 				break;
// 				//Cv初期化
// 				case 0:
// 				wheel.cv = 0.0;
// 				runV_cv_slow();
// 				break;
//
// 				//joy OFF
// 				case 1: case 3:
// 				joy_use_flag = false;
// 				puts("-----    joy OFF   -----");
// 				break;
//
// 				default:
// 				break;
// 			}
// 			key_remove = false;
// 		}
// 	}
// 	//キー&パッドから手が離れた	2で止めるようにしていればコメントアウト
// 	/*if(key_remove){
// 	stop();
// }*/
// }
//
// //cmd_vel
// void U_OKATECH::cmdVelReceived(const geometry_msgs::Twist::ConstPtr& twist)
// {
//
// 	double cmd_v,cmd_dv;
//
// 	global_cmd_v  = twist->linear.x;
// 	global_cmd_dv = twist->angular.z * OKATECH_TREAD * 0.5;
// }
//
// //MMR書き込み用
// void U_OKATECH::cmd_sts_control(void)
// {
// 	static clock_t start_time;
//
// 	double cmd_v = 0.0;
// 	double cmd_dv = 0.0;
// 	double cmd_v_ago  = 0.0;
// 	double cmd_dv_ago = 0.0;
//
// 	double finit_time = 0.50; //0.50
// 	double sts_time = 0.0;
// 	double sts_vel = 0.0;
// 	double coef_a, coef_b, coef_c, coef_d;
//
// 	while (1)
// 	{
// 		if (thread_end_flag)return;
//
// 		cmd_v = 0.0;
// 		cmd_dv = 0.0;
// 		cmd_v_ago  = 0.0;
// 		cmd_dv_ago = 0.0;
//
// 		//グローバル変数として，cmdVelReceivedから取得
// 		cmd_v  = global_cmd_v;
// 		cmd_dv = global_cmd_dv;
//
// 		cmd_v_ago  = (wheel.write.vL + wheel.write.vR) * 0.5;
// 		cmd_dv_ago = cmd_v_ago - wheel.write.vL;
//
// 		coef_a = -2 * (cmd_v - cmd_v_ago) / (finit_time * finit_time * finit_time);
// 		coef_b = 3 * (cmd_v - cmd_v_ago) / (finit_time * finit_time);
// 		coef_c = 0.0;
// 		coef_d = cmd_v_ago;
//
//
// 		sts_vel = 0.0;
//
// 		//ros::Rate loop_rate(50);
// 		start_time = clock();
// 		while(1)
// 		{
// 			sts_time = (double)((double)(clock() - start_time)/CLOCKS_PER_SEC);
//
// 			sts_vel = coef_a * sts_time*sts_time*sts_time + coef_b * sts_time*sts_time + coef_c * sts_time + coef_d;
//
// 			cmd_dv = global_cmd_dv;
//
// 			runV_L( sts_vel - cmd_dv);
// 			runV_R( sts_vel + cmd_dv);
//
// 			sts_time = (double)((double)(clock() - start_time)/CLOCKS_PER_SEC);
//
// 			if(sts_time > finit_time)
// 			{
// 				break;
// 			}
//
// 			//loop_rate.sleep();
// 			usleep(10000);
// 		}
//
// 	}
//
// }
//
// void U_OKATECH::emergency_func(const std_msgs::Bool::ConstPtr &state)
// {
// 	// state pass
// 	old_stop_flag = stop_flag;
// 	stop_flag = state->data;
// 	if(joy_use_flag)return;
//
// 	// stop
// 	if (old_stop_flag != stop_flag && stop_flag)
// 	{
// 		ROS_WARN("Emergency Received!");
// 		//stop();
// 	}
// 	else if (old_stop_flag != stop_flag && !stop_flag)
// 	{
// 		ROS_WARN("Emergency Clear!");
// 	}
// }
//
// void U_OKATECH::Wheel_Control_Outside(const std_msgs::StringConstPtr& msg)
// {
// 	if(joy_use_flag)return;
// 	//std::cout<<"yes"<<std::endl;
// 	std::cout<<msg->data.c_str()<<std::endl;
// 	std::string recv_msg =  msg->data.c_str();
// 	//if(recv_msg.find(hit_word.c_str())== std::string::npos)return;
// 	static bool wco_start_flag = true;
// 	double kx,ky,kz;
// 	char set[256] = {0};
// 	std::cout<<recv_msg<<std::endl;
// 	sscanf(recv_msg.c_str(),"%[^,],%lf,%lf,%lf",set,&kx,&ky,&kz);
// 	kx = -kx;
// 	if((kx==0.0)&(kz==0.0))
// 	{
// 		stop();
// 		return;
// 	}
// 	double line,line_v,target_w,target_th,wco_v,wco_dv;
// 	static boost::timer::auto_cpu_timer wco_timer;
// 	static double line_ago,target_th_ago;
//
// 	line = sqrt(pow(kx,2) + pow(kz,2));
// 	if(line < 0.0001)return;
// 	target_th = atan2(kx,kz);
//
// 	if(wco_start_flag)
// 	{
// 		wco_timer.start();
// 		target_w = 0;
// 		line_v = 0;
// 		wco_start_flag = false;
// 	}
// 	else
// 	{
// 		wco_timer.stop();
// 		target_w = (target_th - target_th_ago) / NANOSEC_TO_SEC(wco_timer.elapsed().wall);
// 		line_v = (line - line_ago) / NANOSEC_TO_SEC(wco_timer.elapsed().wall);
// 	}
// 	wco_v  =0.03;//line * KV;// + line_v * KLV;
// 	wco_dv = target_th * KT + target_w * KTD;
//
// 	//値の更新
// 	line_ago = line;
// 	target_th_ago = target_th;
//
// 	if(line > INVIOLABLE_AREA)
// 	{
// 		runV_L( wco_v + wco_dv);
// 		runV_R( wco_v - wco_dv);
// 	}
// 	else
// 	{
// 		//runR(target_th / 0.995 * 0.50);
// 		stop();
// 	}
// }
//
// void U_OKATECH::Odometry_Save(const std_msgs::StringConstPtr& msg)
// {
// 	std::string recv_msg =  msg->data.c_str();
// 	if(recv_msg.find("kinect")== std::string::npos)return;
// 	std::cout<<recv_msg <<std::endl;
// 	int img_num=0;
// 	double x;
// 	double y;
// 	char set[256] = {0};
// 	sscanf(recv_msg.c_str(),"%[^,],%d",set,&img_num);
// 	x = position.x;
// 	y = position.y;
// 	std::ofstream ofs;
// 	ofs.open("/home/robo/practice_ws/src/u_kuaro_move/odometry_save.txt",std::ios::out | std::ios::app);
// 	ofs <<img_num<<","<< x <<","<< y << std::endl;
// 	ofs.close();
// }
//
// //gyro
// void U_OKATECH::gyroReceived(const sensor_msgs::Imu::ConstPtr& gyro)
// {
// 	gyro_flag = true;
// 	tf::Quaternion bq(gyro->orientation.x, gyro->orientation.y, gyro->orientation.z, gyro->orientation.w);
// 	double roll, pitch, yaw;
// 	tf::Matrix3x3(bq).getRPY(roll, pitch, yaw);
// 	//g_rad_now = yaw;
// 	//g_drad_now = gyro->angular_velocity.z;
// }
//
// IMU::IMU()
// {
// 	len = 0;
// 	C = 0;
// 	//bufsize = 20;
// 	space_flag = false;
// 	//count = 0;
// 	rcount = 0;
// 	tmp = 0.0f;
// 	ema_flag = false;
// 	read_error_flag = false;
// 	angle_ago = 0.0f;
// 	angle_now = 0.0f;
// 	angle_now_2 = 0.0f;
// 	ros::NodeHandle n;
// 	ros::NodeHandle nh_odom;
// 	pub = n.advertise<sensor_msgs::Imu>("/imu/data",100);
// 	pub_odom = nh_odom.advertise<nav_msgs::Odometry>("odom_default", 1000);
// 	COM_No = (char*)G_COM;
// 	cout<<"COM_No = "  << COM_No << endl;
// 	if(!initial_rs( COM_No , &fd_rs))
// 	{
// 		ROS_ERROR("com open error");
// 		ros::shutdown();
// 	}
// 	th_ = thread(bind(&gyro_ThreadFunction), this);
// }
//
//
// IMU::~IMU()
// {
// 	this->th_.join();
// 	cout << "end" << endl;
// }
//
// int initial_rs( char *device ,int *fd_rs)
// {
// 	struct termios tio;
// 	memset(&tio, 0, sizeof(tio));
// 	if((*fd_rs = open(device,  O_RDWR | O_NOCTTY | O_NONBLOCK ))<0)
// 	{
// 		printf("INITAL ERR\n");
// 		printf("gyro conect failed\n");
// 		return (-1);
// 	}
// 	else
// 	{
// 		printf("INITAL OK\n");
// 		printf("gyroconected\n");
// 	}
// 	bzero( &tio, sizeof(tio));
// 	tio.c_cflag &= 0;
// 	tio.c_cflag = B38400 | CS8 | CSTOPB | CREAD ;
// 	tio.c_iflag &= 0;
// 	tio.c_oflag &= 0;
// 	tio.c_lflag &= 0;
// 	tcflush( *fd_rs, TCIFLUSH );
// 	tcsetattr( *fd_rs, TCSANOW, &tio );
// 	fcntl( *fd_rs, F_SETFL, FNDELAY );
// 	printf("INITAL SUCCESS\n");
// 	printf("gyro conected\n");
// 	return 1;
// }
//
// void Thread_End ( void )
// {
//
// }
//
// void gyro_ThreadFunction()
// {
// 	bool roop_flag = false;
// 	cout<<"receive now"<<endl;
// 	char key_buf[3] =
// 	{
// 		't',
// 		'\r',
// 		'\n'
// 	};
//
// 	cout<<"send comand 't'"<<endl;
// 	usleep(1000);
//
// 	if(write(fd_rs,&key_buf,sizeof(key_buf)))
// 	{
// 		roop_flag = true;
// 		cout<< " roop_flag == true"<<endl;
// 	}
// 	else
// 	{
// 		cout<< " roop_flag == false"<<endl;
// 	}
// 	if(roop_flag)
// 	{
// 		cout<< "ad start"<<endl;
// 		get_ad();
//
// 	}
// 	else
// 	{
// 		cout<<"終了"<<endl;
// 	}
// }
//
// float cul_rad(int vref,int vrate)
// {
//
// 	float rate = (float)vrate;
// 	float ref = (float)vref;
// 	float diff = rate - ref;
// 	float rad = diff/20.0f*M_PI/180.0f;
// 	if(fabs(rad) < 0.05f*M_PI/180.0f)
// 	{
// 		rad = 0.0f;
// 	}
// 	rad -= 0.0004;//dorihuto hosei
// 	return rad;
// }
//
// float ema_cul(float *buf, int rcount)
// {
// 	float res;
// 	float alfa = 2.0f /(float)(rlimit + 1.0f);
// 	res = buf[rlimit-2] + alfa * (buf[rlimit-1] - buf[rlimit-2]);
// 	return res;
// }
//
// //gyro関係
// int get_ad()
// {
// 	int i = 0;
// 	int count = 0;
// 	gyro_flag = true;
// 	gettimeofday(&s, NULL);
// 	bool range_out_flag = false;
// 	time_now = 0.0f;
// 	time_ago = 0.0f;
// 	float dt = 0.0;
// 	//ros::Rate r(200);
// 	ros::Time current_time;
// 	gettimeofday(&s, NULL);
// 	cout <<" get ad roop"<<endl;
// 	// 受信バッファ全クリ
// 	tcflush(fd_rs,TCIFLUSH);
// 	// 送信バッファ全クリ
// 	tcflush(fd_rs,TCOFLUSH);
// 	imu.header.frame_id = "/imu";
//
// 	while(ros::ok())
// 	{
// 		current_time = ros::Time::now();
// 		imu.header.stamp = current_time;
//
// 		for(int init_count = 0;init_count< bufsize;init_count++)
// 		{
// 			ref_buf[init_count]= '\0';
// 			rate_buf[init_count]= '\0';
// 			rbuf[init_count]= '\0';
// 		}
//
// 		while(ros::ok())
// 		{
// 			//cout<<"roop_2"<<endl;
// 			read_error_flag = false;
//
// 			while(ros::ok())
// 			{
// 				len = read(fd_rs, &C, sizeof(C));
// 				if(len > 0)
// 				{
// 					//cout << "len = "<< len <<endl;
// 					break;
// 				}
// 				else if(len < 0)
// 				{
// 					read_error_flag = true;
// 					cout << "error"<<endl;
// 					break;
// 				}
// 				else
// 				{
// 					//cout<<"len  = 0"<<endl;
// 				}
// 			}
//
// 			if(read_error_flag)continue;
//
// 			if( C == ',')
// 			{
// 				space_flag = true;
// 				ref_buf[i] = '\n';
// 				i = 0;
// 			}
// 			if(C =='\n')
// 			{
// 				space_flag = false;
// 				rate_buf[i] = '\n';
// 				i = 0;
// 				break;
// 			}
//
// 			if(!space_flag)
// 			{
// 				ref_buf[i] = C;
// 				i++;
// 			}
// 			else
// 			{
// 				if(C !=',')
// 				{
// 					rate_buf[i] = C;
// 					i++;
// 				}
// 			}
// 		}
// 		//bool input_flag = false;
// 		int vref = atoi( ref_buf );
// 		int vrate = atoi( rate_buf );
// 		if(vref < 10000|| vrate < 10000||vref > 100000|| vrate > 100000){
// 			//continue;
// 			//input_flag =true;
// 		}
// 		else
// 		{
// 			result_rad = cul_rad(vref, vrate);
// 		}
//
// 		gettimeofday(&e, NULL);
// 		time_now = (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6;
//
// 		dt = time_now-time_ago;
// 		//dt = (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6;
//
// 		if(fabs(result_rad) > 100 * M_PI/180.0f)
// 		{
// 			cout<<"測定範囲外"<<endl;
// 			//result_rad = 100 * M_PI/180.0f;
// 			range_out_flag = true;
// 		}
// 		//----------------ここまで角速度の生データ抽出---------------------------------------------------//
// 		//if(!range_out_flag){
// 		if(rcount > rlimit-2)
// 		{
// 			ema_flag = true;
// 			rad_buf[rlimit-1] = result_rad;
// 		}
// 		else
// 		{
// 			rad_buf[rcount] = result_rad;
// 		}
//
// 		if(ema_flag)
// 		{
// 			tmp = ema_cul(rad_buf, rcount);
// 			rad_buf[rlimit-1] = tmp;
// 			for(int i = 0;i < rlimit-1;i++)
// 			{
// 				rad_buf[i] = rad_buf[i+1];
// 			}
// 		}
// 		else
// 		{
// 			float sum = 0.0;
// 			for(int i = 0; i< rcount+1; i++)
// 			{
// 				sum += rad_buf[i];
// 			}
// 			rad_buf[rcount] = sum/(float)(rcount + 1.0f);
// 		}
// 		angle_now = angle_ago + (rad_buf[rlimit-1] + rad_buf[rlimit-2])*dt*0.50f;//修正オイラー
// 		rcount++;
// 		//}else{
// 		//測定範囲外の時だけ100°/sとしてオイラー法により足しあわせ
// 		//angle_now += result_rad * dt;//オイラー
// 		//}
//
// 		//angle_now_2 += result_rad * dt;//オイラー
// 		//-PI < angle_now <= PI
// 		if(angle_now >= M_PI)
// 		{
// 			angle_now -= 2.0f* M_PI;
// 		}
// 		else if(angle_now < -M_PI)
// 		{
// 			angle_now +=  2.0f * M_PI;
// 		}
// 		//printf("%f\n, ", -angle_now);
// 		//if(angle_now_2 >= M_PI){
// 		//	angle_now_2 -= 2.0f* M_PI;
// 		//}else if(angle_now_2 < -M_PI){
// 		//	angle_now_2 = angle_now_2 + 2.0f* M_PI;
// 		//}
//
// 		angle_ago = angle_now;
// 		/////////////
//
// 		count++;
// 		len = 0;
//
// 		/////////////
// 		time_ago = time_now;
// 	}
// 	return 1;
// }
//
//
// void imu_publisher(void)
// {
// 	geometry_msgs::Quaternion imu_quat= tf::createQuaternionMsgFromYaw(-angle_now);
// 	imu.orientation = imu_quat;
//
// 	imu.header.stamp = ros::Time::now();
// 	imu.header.frame_id = "/imu";
//
// 	imu.angular_velocity.x = 0.0f;
// 	imu.angular_velocity.y = 0.0f;
// 	imu.angular_velocity.z = -result_rad;
//
// 	//printf("%f, ", -angle_now);
// 	//printf("%f\n ", -angle_now*180/M_PI);
// 	pub.publish(imu);
//
// 	//printf("%lf ", -result_rad);
// 	//printf("%lf\n", time_now);
// }
//
//
// void odom_default_pub(void)
// {
// 	nav_msgs::Odometry odom;
// 	odom.header.stamp    = ros::Time::now();
// 	odom.header.frame_id = "odom_default";
// 	odom.child_frame_id  = "base_footprint";
// 	//set the position
// 	odom.pose.pose.position.x  = o_x;
// 	odom.pose.pose.position.y  = o_y;
// 	odom.pose.pose.position.z  = 0.0;
// 	odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, o_rad);
// 	//set the velocity
// 	odom.twist.twist.linear.x  = o_dx;
// 	odom.twist.twist.linear.y  = o_dy;
// 	odom.twist.twist.linear.z  = 0.0;
// 	odom.twist.twist.angular.x = 0.0;
// 	odom.twist.twist.angular.y = 0.0;
// 	odom.twist.twist.angular.z = o_drad;
// 	pub_odom.publish(odom);
//
// 	//printf("%lf ", -result_rad);
// 	//printf("%lf\n", time_now);
// }
//
// void get_odom_trans_default()
// {
// 	tf::TransformBroadcaster odom_d_broadcaster;
// 	geometry_msgs::TransformStamped odom_trans;
// 	odom_trans.header.stamp    = ros::Time::now();
// 	odom_trans.header.frame_id = "odom_default";
// 	odom_trans.child_frame_id  = "base_footprint_odomdefault";
// 	odom_trans.transform.translation.x = o_x;
// 	odom_trans.transform.translation.y = o_y;
// 	odom_trans.transform.translation.z = 0.0;
// 	odom_trans.transform.rotation      = tf::createQuaternionMsgFromRollPitchYaw(0, 0, o_rad);
// 	odom_d_broadcaster.sendTransform(odom_trans);
// }
//
// #endif  // __linux__
