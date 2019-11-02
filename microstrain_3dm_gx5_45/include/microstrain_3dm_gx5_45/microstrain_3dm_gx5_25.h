/** ROS node

/*

Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the microstrain_3dm_gx5_45 package.

microstrain_3dm_gx5_45 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

microstrain_3dm_gx5_45 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

*/

//2017_10_23_ono_create
//2017_10_27_軽微な修正

#ifndef _MICROSTRAIN_3DM_GX5_45_H
#define _MICROSTRAIN_3DM_GX5_45_H

// Tell compiler that the following MIP SDI are C functions
extern "C" {
#include "mip_sdk.h"
#include "byteswap_utilities.h"
#include "mip_gx4_imu.h"
#include "mip_gx4_45.h"
}

#include <cstdio>
#include <unistd.h>


// ROS
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_srvs/Empty.h"

#define MIP_SDK_GX4_45_IMU_STANDARD_MODE	0x01
#define MIP_SDK_GX4_45_IMU_DIRECT_MODE	0x02

#define NUM_COMMAND_LINE_ARGUMENTS 3

#define DEFAULT_PACKET_TIMEOUT_MS  1000 //milliseconds

//macro to cause Sleep call to behave as it does for windows
#define Sleep(x) usleep(x*1000.0)

/**
 * \brief Contains functions for micostrain driver
 */
namespace Microstrain
{
  /**
   * \brief Microstrain class
   *
   */
  class Microstrain
  {
  public:
    /**
     * Contructor
     */
    Microstrain();

    /** Destructor */
    ~Microstrain();

    /**
     * Main run loop
     */
    void run();

    //! Nav estimate callback
    void filter_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
    //! @brief AHRS callback
    void ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);

  private:
  //! @brief Reset KF service callback
  bool reset_callback(std_srvs::Empty::Request &req,
		      std_srvs::Empty::Response &resp);
  //! @brief Convience for printing packet stats
  void print_packet_stats();

  // Variables/fields
  //The primary device interface structure
  mip_interface device_interface_;

  //Packet Counters (valid, timeout, and checksum errors)
  u32 filter_valid_packet_count_;
  u32 ahrs_valid_packet_count_;

  u32 filter_timeout_packet_count_;
  u32 ahrs_timeout_packet_count_;

  u32 filter_checksum_error_packet_count_;
  u32 ahrs_checksum_error_packet_count_;

  //Data field storage
  //AHRS
  mip_ahrs_scaled_gyro  curr_ahrs_gyro_;
  mip_ahrs_scaled_accel curr_ahrs_accel_;
  mip_ahrs_scaled_mag   curr_ahrs_mag_;
  mip_ahrs_quaternion  curr_ahrs_quaternion_;

  //FILTER
  mip_filter_llh_pos               curr_filter_pos_;
  mip_filter_ned_velocity          curr_filter_vel_;
  mip_filter_attitude_euler_angles curr_filter_angles_;
  mip_filter_attitude_quaternion   curr_filter_quaternion_;
  mip_filter_compensated_angular_rate curr_filter_angular_rate_;
  mip_filter_llh_pos_uncertainty   curr_filter_pos_uncertainty_;
  mip_filter_ned_vel_uncertainty   curr_filter_vel_uncertainty_;
  mip_filter_euler_attitude_uncertainty curr_filter_att_uncertainty_;
  mip_filter_status curr_filter_status_;

  mip_filter_linear_acceleration curr_filter_linear_acc_;//ono



  // ROS
  ros::Publisher imu_raw_pub_;
  ros::Publisher imu_filtered_pub_;//ono_add
  ros::Publisher nav_status_pub_;
  sensor_msgs::Imu imu_msg_;
  sensor_msgs::Imu imu_filtered_msg_;//ono_add
  nav_msgs::Odometry nav_msg_;
  std_msgs::Int16MultiArray nav_status_msg_;
  std::string imu_frame_id_;
  std::string imu_topic_;//ono_add
  std::string imu_filtered_topic_;//ono_add
  std::string status_topic_;//ono_add
  bool publish_imu_raw_;
  bool publish_imu_filtered_;
  bool setup_complete_flag_;

  // Update rates
  int imu_filtered_rate_;
  int imu_raw_rate_;
  }; // Microstrain class


  // Define wrapper functions that call the Microstrain member functions
#ifdef __cplusplus
  extern "C"
#endif
  {

    /**
     * Callback for KF estimate packets from sensor.
     */
    void filter_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
    /**
     * Callback for AHRS packets from sensor.
     */
    void ahrs_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);

#ifdef __cplusplus
  }
#endif

} // namespace Microstrain

#endif  // _MICROSTRAIN_3DM_GX5_45_H
