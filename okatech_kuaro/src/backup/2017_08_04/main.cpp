/*-----------------------------------------------
 * 	main.cpp
 * <Last Update>	H29/07/13
 * <version>		v1.0
 * ---------------------------------------------*/

#include "uk_okatech.h"
#include "u_ros_func.h"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "kuaro_move");


	std::string USBport_name_okatech = "/dev/sensors/ttyUSBOkatech";

	//シリアル通信関数入れる．

	U_OKATECH okatech;

	UROS_FUNC uros;
  tf::TransformBroadcaster odom_broadcaster;
	IMU imu_node;

	ros::NodeHandle uo_pt("~");
	uo_pt.param("CMD_ACC_LIMIT" , okatech._cmd_acc_limit,  (double)CMD_ACC_LIMIT);
	uo_pt.param("CMD_ANG_ACC_LIMIT", okatech._cmd_ang_acc_limit, (double)CMD_ANG_ACC_LIMIT);
	uo_pt.param("CMD_CONTROL_PERIOD" , okatech._cmd_control_period,  (double)CMD_CONTROL_PERIOD);

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

	ros::NodeHandle d_odom_node;
	ros::Publisher d_odom_pub = d_odom_node.advertise<nav_msgs::Odometry>("odom_d", 1000);

	ros::NodeHandle odom_node;
	ros::Publisher odom_pub = odom_node.advertise<nav_msgs::Odometry>("odom_gyro", 1000);

	ros::Rate loop_rate(50);

	while (ros::ok())
    {
		uros.ku_data_set(okatech.position);

    odom_broadcaster.sendTransform(uros.get_odom_trans_gyro());

		d_odom_pub.publish(uros.get_odom_publish_d());
		odom_pub.publish(uros.get_odom_publish());

		ROS_DEBUG("odom publish");
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
