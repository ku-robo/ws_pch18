/*-----------------------------------------------
 * 	u_ros_func.cpp
 * <Last Update>	H27/10/26
 * <version>		v1.0
 *
 * <MEMO>
 * rosの記述が長いため作成
 * ---------------------------------------------*/

/*-----ヘッダー宣言-------------------------------*/
#include "u_ros_func.h"

UROS_FUNC::UROS_FUNC(){
	//ros::Time current_time = ros::Time::now();
}


void UROS_FUNC::ku_data_set(OKA_POSITION set_position){
	current_time = ros::Time::now();

	// 差分の計算
	d_position = set_position;
	d_position.x = set_position.x - position.x;
	d_position.y = set_position.y - position.y;

	// 更新
	position = set_position;
}

geometry_msgs::TransformStamped UROS_FUNC::get_odom_trans_gyro(){
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp    = current_time;
	odom_trans.header.frame_id = "odom_gyro";
	odom_trans.child_frame_id  = "base_footprint_odomgyro";
	odom_trans.transform.translation.x = position.x;
	odom_trans.transform.translation.y = position.y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation      = tf::createQuaternionMsgFromRollPitchYaw(0, 0, position.th);
	return odom_trans;
}

nav_msgs::Odometry UROS_FUNC::get_odom_publish(){
	nav_msgs::Odometry odom;
	odom.header.stamp    = current_time;
	odom.header.frame_id = "odom_gyro";
	odom.child_frame_id  = "base_footprint";
	//set the position
	odom.pose.pose.position.x  = position.x;
	odom.pose.pose.position.y  = position.y;
	odom.pose.pose.position.z  = 0.0;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, position.th);
	//set the velocity
	odom.twist.twist.linear.x  = position.dx_v;
	odom.twist.twist.linear.y  = position.dy_v;
	odom.twist.twist.linear.z  = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	odom.twist.twist.angular.z = position.dw;
	return odom;
}

nav_msgs::Odometry UROS_FUNC::get_odom_publish_d(){
	nav_msgs::Odometry odom;
	odom.header.stamp    = current_time;
	odom.header.frame_id = "odom_comb";
	odom.child_frame_id  = "base_footprint";

	//set the position
	odom.pose.pose.position.x  = d_position.x;
	odom.pose.pose.position.y  = d_position.y;
	odom.pose.pose.position.z  = d_position.dth;//dth?
	odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, d_position.th);

	//set the velocity
	odom.twist.twist.linear.x  = d_position.dx_v;
	odom.twist.twist.linear.y  = d_position.dy_v;
	odom.twist.twist.linear.z  = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	odom.twist.twist.angular.z = d_position.dw;
	return odom;
}
