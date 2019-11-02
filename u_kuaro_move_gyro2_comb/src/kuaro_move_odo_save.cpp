/*-----------------------------------------------
 * 	kuaro_move.cpp
 * <Last Update>	H27/10/26
 * <version>		v1.0
 *
 * <MEMO>
 * mainプログラム
 * uk_okatech.hをインクルードしなくても大丈夫
 * u_ros_func.hをインクルードすれば良い
 * ※boostビルドリンク
 * -lboost_thread
 * -lboost_system
 * -lboost_timer
 * ※その他
 * -pthread
 * ---------------------------------------------*/

/*-----ヘッダー宣言-------------------------------*/
//自作
#include "uk_okatech.h"
#include "u_ros_func.h"

#ifdef WIN32
#include<conio.h>
#endif

/*-----MAIN--------------------------------------*/
int main(int argc, char *argv[])
{
	///-------変数定義--------///
	ros::init(argc, argv, "kuaro_move");
	// sound_play::SoundClient sou;sleep(1);sou.startWave(SOUND_PASS"/start.wav");
	U_OKATECH okatech("/dev/ttyUSB0");
	UROS_FUNC uros;
	int frequency;
	bool pb_change;

	///-------Subscriber-----///
	//joy
	ros::NodeHandle joyrcv_node;
   ros::Subscriber chatter_sub = joyrcv_node.subscribe("joy", 100, &U_OKATECH::u_Joy_Control, &okatech);

	// Emergency
	ros::NodeHandle emergency_node;
	ros::Subscriber emergency_sub = emergency_node.subscribe("state", 10, &U_OKATECH::emergency_func, &okatech);

   //cmd_vel
   ros::NodeHandle cmdvel_node;
	ros::Subscriber cmd_vel_sub = cmdvel_node.subscribe<geometry_msgs::Twist>("/cmd_vel", 10 , &U_OKATECH::cmdVelReceived,&okatech);

	//tcp
	ros::NodeHandle lis_node;
	//ros::Subscriber tcp_pub = lis_node.subscribe("/tcp_ros", 100, &U_OKATECH::Wheel_Control_Outside,&okatech);
	// ros::NodeHandle uo_pt("~");
	// uo_pt.param("select_data", okatech.hit_word, std::string("target_point"));
	// uo_pt.param("KV" , okatech.KV,  (double)0.20);
	// uo_pt.param("KLV", okatech.KLV, (double)0.01);
	// uo_pt.param("KT" , okatech.KT,  (double)0.10);
	// uo_pt.param("KTD", okatech.KTD, (double)0.02);
	// uo_pt.param("INVIOLABLE_AREA", okatech.INVIOLABLE_AREA, (double)0.5);

	ros::Subscriber tcp_pub = lis_node.subscribe("/tcp_ros", 100, &U_OKATECH::Odometry_Save,&okatech);

   ///-------Publisher-----///
   ros::NodeHandle odom_node;
	ros::Publisher odom_pub = odom_node.advertise<nav_msgs::Odometry>("odom_default", 1000);

   ros::NodeHandle d_odom_node;
	ros::Publisher d_odom_pub = d_odom_node.advertise<nav_msgs::Odometry>("odom", 1000);

	// param set
   ros::NodeHandle private_nh("~");
	private_nh.param<int>("frequency", frequency, 20);
   ros::Rate loop_rate(frequency);

    /* --------------------------以下mainの処理-------------------------- */
	while (ros::ok())
    {
		uros.ku_data_set(okatech.position);
		//-------odom publish-------//
		d_odom_pub.publish(uros.get_odom_publish_d());
    odom_pub.publish(uros.get_odom_publish());

		ROS_DEBUG("odom publish");
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
