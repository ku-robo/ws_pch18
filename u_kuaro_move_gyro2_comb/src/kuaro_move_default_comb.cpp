/*-----------------------------------------------
 * 	kuaro_move.cpp
 * <Last Update>	H28/10/07
 * <version>		v1.6
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
	U_OKATECH okatech("/dev/sensors/ttyUSBOkatech");
	UROS_FUNC uros;
    tf::TransformBroadcaster odom_broadcaster;
	int frequency;
	bool pb_change;
	IMU imu_node;
	
	///---------param---------///
	ros::NodeHandle uo_pt("~");
	uo_pt.param("CMD_ACC_LIMIT" , okatech._cmd_acc_limit,  (double)CMD_ACC_LIMIT);
	uo_pt.param("CMD_ANG_ACC_LIMIT", okatech._cmd_ang_acc_limit, (double)CMD_ANG_ACC_LIMIT);
	uo_pt.param("CMD_CONTROL_PERIOD" , okatech._cmd_control_period,  (double)CMD_CONTROL_PERIOD);
	
	///-------Subscriber-----///
	//joy
	ros::NodeHandle joyrcv_node;
	ros::Subscriber chatter_sub = joyrcv_node.subscribe("joy", 1, &U_OKATECH::u_Joy_Control, &okatech);

	// Emergency
	ros::NodeHandle emergency_node;
	ros::Subscriber emergency_sub = emergency_node.subscribe("state", 10, &U_OKATECH::emergency_func, &okatech);

	//cmd_vel
	ros::NodeHandle cmdvel_node;
	ros::Subscriber cmd_vel_sub = cmdvel_node.subscribe<geometry_msgs::Twist>("/cmd_vel", 10 , &U_OKATECH::cmdVelReceived,&okatech);

	//gyro
	//ros::NodeHandle gyro_node;
	//ros::Subscriber gyro_sub = gyro_node.subscribe<sensor_msgs::Imu>("/imu/data", 1 , &U_OKATECH::gyroReceived,&okatech);

	///-------Publisher-----///
	ros::NodeHandle d_odom_node;
	ros::Publisher d_odom_pub = d_odom_node.advertise<nav_msgs::Odometry>("odom_d", 1000);

	ros::NodeHandle odom_node;
	ros::Publisher odom_pub = odom_node.advertise<nav_msgs::Odometry>("odom_gyro", 1000);

	// param set
	ros::NodeHandle private_nh("~");
	private_nh.param<int>("frequency", frequency, 50);
	ros::Rate loop_rate(frequency);

	/* --------------------------以下mainの処理-------------------------- */
	while (ros::ok())
    {
		uros.ku_data_set(okatech.position);
		//-------odom transform-----//
    odom_broadcaster.sendTransform(uros.get_odom_trans_gyro());

		//-------odom publish-------//
		d_odom_pub.publish(uros.get_odom_publish_d());
		odom_pub.publish(uros.get_odom_publish());

		ROS_DEBUG("odom publish");
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
