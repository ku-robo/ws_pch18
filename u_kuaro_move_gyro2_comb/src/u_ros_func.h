/*-----------------------------------------------
 * 	u_ros_func.cpp
 * <Last Update>	H27/10/26
 * <version>		v1.0
 *
 * <MEMO>
 * rosの記述が長いため作成
 * ---------------------------------------------*/
#ifndef U_ROS_FUNC_H_
#define U_ROS_FUNC_H_

/*-----ヘッダー宣言-------------------------------*/
#include "uk_okatech.h"

/*-----クラス宣言-------------------------------*/
class UROS_FUNC
{
public:
	UROS_FUNC();
	~UROS_FUNC(){}
	void ku_data_set(OKA_POSITION set_position);
	geometry_msgs::TransformStamped get_odom_trans_gyro();
	nav_msgs::Odometry get_odom_publish();
	nav_msgs::Odometry get_odom_publish_d();

private:
	ros::Time current_time;
	OKA_POSITION position;
	OKA_POSITION d_position;
};

#endif
