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

//2017/10/23 ono_create
//2017/10/25 ジャイロなどの問題を解決
//2017/10/27 加速度などを追加


#include "microstrain_3dm_gx5_25.h"
#include <tf2/LinearMath/Transform.h>
#include <string>
#include <algorithm>



namespace Microstrain
{
  Microstrain::Microstrain():
    // Initialization list
    filter_valid_packet_count_(0),
    ahrs_valid_packet_count_(0),
    filter_timeout_packet_count_(0),
    ahrs_timeout_packet_count_(0),
    filter_checksum_error_packet_count_(0),
    ahrs_checksum_error_packet_count_(0),
    imu_frame_id_("imu"),
    publish_imu_raw_(true),
    publish_imu_filtered_(true)
  {
    // pass
  }
  Microstrain::~Microstrain()
  {
    // pass
  }

  void Microstrain::run()
  {
    // Variables for device configuration, ROS parameters, etc.
    u32 com_port, baudrate;
    bool device_setup = false;
    bool readback_settings = true;
    bool save_settings = true;
    bool auto_init = true;
    u8 auto_init_u8 = 1;
    u8 readback_headingsource = 0;
    u8 readback_auto_init = 0;
    int declination_source;
    u8 declination_source_u8;
    u8 readback_declination_source;
    double declination;

    // Variables
    tf2::Quaternion quat;
    base_device_info_field device_info;
    u8  temp_string[20] = {0};
    u32 bit_result;
    u8  enable = 1;
    u8  data_stream_format_descriptors[10];
    u16 data_stream_format_decimation[10];
    u8  data_stream_format_num_entries = 0;
    u8  readback_data_stream_format_descriptors[10] = {0};
    u16 readback_data_stream_format_decimation[10]  = {0};
    u8  readback_data_stream_format_num_entries     =  0;
    u16 base_rate = 0;
    u16 device_descriptors[128]  = {0};
    u16 device_descriptors_size  = 128*2;
    s16 i;
    u16 j;
    u8  com_mode = 0;
    u8  readback_com_mode = 0;
    float angles[3]             = {0};
    float readback_angles[3]    = {0};
    float offset[3]             = {0};
    float readback_offset[3]    = {0};
    float hard_iron[3]          = {0};
    float hard_iron_readback[3] = {0};
    float soft_iron[9]          = {0};
    float soft_iron_readback[9] = {0};
    u16 estimation_control   = 0, estimation_control_readback = 0;
    u8  heading_source = 0x1;
    float noise[3]          = {0};
    float readback_noise[3] = {0};
    float beta[3]                 = {0};
    float readback_beta[3]        = {0};
    mip_low_pass_filter_settings filter_settings;
    float bias_vector[3]		   = {0};
    u16 duration = 0;
    gx4_imu_diagnostic_device_status_field imu_diagnostic_field;
    gx4_imu_basic_status_field imu_basic_field;
    gx4_45_diagnostic_device_status_field diagnostic_field;
    gx4_45_basic_status_field basic_field;
    mip_filter_external_heading_update_command external_heading_update;
    mip_filter_zero_update_command zero_update_control, zero_update_readback;
    mip_filter_external_heading_with_time_command external_heading_with_time;
    mip_complementary_filter_settings comp_filter_command, comp_filter_readback;

    mip_filter_accel_magnitude_error_adaptive_measurement_command        accel_magnitude_error_command, accel_magnitude_error_readback;
    mip_filter_magnetometer_magnitude_error_adaptive_measurement_command mag_magnitude_error_command, mag_magnitude_error_readback;
    mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command mag_dip_angle_error_command, mag_dip_angle_error_readback;

    // ROS setup
    ros::Time::init();
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    // ROS Parameters
    // Comms Parameters
    std::string port;
    int baud, pdyn_mode;
    private_nh.param("port", port, std::string("/dev/ttyACM0"));
    private_nh.param("baudrate",baud,115200);
    baudrate = (u32)baud;
    // Configuration Parameters
    private_nh.param("device_setup",device_setup,true);
    private_nh.param("readback_settings",readback_settings,true);
    private_nh.param("save_settings",save_settings,false);

    private_nh.param("auto_init",auto_init,true);
    private_nh.param("imu_raw_rate",imu_raw_rate_, 10);
    private_nh.param("imu_filtered_rate",imu_filtered_rate_, 10);//書き換えるlaunchも

    private_nh.param("declination_source",declination_source,2);
    if (declination_source < 1 || declination_source > 3){
      ROS_WARN("declination_source can't be %#04X, must be 1, 2 or 3.  Setting to 2.",declination_source);
      declination_source = 2;
    }
    declination_source_u8 = (u8)declination_source;
    //declination_source_command=(u8)declination_source;
    private_nh.param("declination",declination,0.23);
    private_nh.param("imu_frame_id",imu_frame_id_, std::string("imu"));
    private_nh.param("publish_imu",publish_imu_raw_, true);
    private_nh.param("publish_imu_filltered",publish_imu_filtered_, true);
    private_nh.param("imu_raw_topic",imu_topic_,std::string("imu_raw/data"));
    private_nh.param("imu_filtered_topic",imu_filtered_topic_,std::string("imu_filtered/data"));
    private_nh.param("status_topic",status_topic_,std::string("nav/status"));

    if (publish_imu_raw_)
    {
      imu_raw_pub_ = node.advertise<sensor_msgs::Imu>(imu_topic_,10);
    }
    if (publish_imu_filtered_)
    {
      imu_filtered_pub_= node.advertise<sensor_msgs::Imu>(imu_filtered_topic_,10);
      nav_status_pub_ = node.advertise<std_msgs::Int16MultiArray>(status_topic_,10);
    }

    ros::ServiceServer service = node.advertiseService("reset_kf", &Microstrain::reset_callback, this);

    //Initialize the serial interface to the device
    ROS_INFO("Attempting to open serial port <%s> at <%d> \n",port.c_str(),baudrate);
    if(mip_interface_init(port.c_str(), baudrate, &device_interface_, DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK)
    {
      ROS_FATAL("Couldn't open serial port!  Is it plugged in?");
    }

    // Setup device callbacks
    if(mip_interface_add_descriptor_set_callback(&device_interface_, MIP_FILTER_DATA_SET, this, &filter_packet_callback_wrapper) != MIP_INTERFACE_OK)
    {
	    ROS_FATAL("Can't setup filter callback!");
	    return;
    }
    if(mip_interface_add_descriptor_set_callback(&device_interface_, MIP_AHRS_DATA_SET, this, &ahrs_packet_callback_wrapper) != MIP_INTERFACE_OK)
    {
      ROS_FATAL("Can't setup ahrs callbacks!");
      return;
    }

    ////////////////////////////////////////
    // Device setup
    float dT=0.5;  // common sleep time after setup communications
    if (device_setup)
    {
      // Put device into standard mode - we never really use "direct mode"
      ROS_INFO("Putting device communications into 'standard mode'");
      device_descriptors_size  = 128*2;
      com_mode = MIP_SDK_GX4_45_IMU_STANDARD_MODE;
      while(mip_system_com_mode(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &com_mode) != MIP_INTERFACE_OK){}
      //Verify device mode setting
      ROS_INFO("Verify comm's mode");
      while(mip_system_com_mode(&device_interface_, MIP_FUNCTION_SELECTOR_READ, &com_mode) != MIP_INTERFACE_OK){}
      ROS_INFO("Sleep for a second...");
      ros::Duration(dT).sleep();
      ROS_INFO("Right mode?");

      if(com_mode != MIP_SDK_GX4_45_IMU_STANDARD_MODE)
      {
	       ROS_ERROR("Appears we didn't get into standard mode!");
      }

      // Put into idle mode
      ROS_INFO("Idling Device: Stopping data streams and/or waking from sleep");
      while(mip_base_cmd_idle(&device_interface_) != MIP_INTERFACE_OK){}
      ros::Duration(dT).sleep();

      // AHRS Setup
      // Get base rate
      if (publish_imu_raw_)
      {
      	while(mip_3dm_cmd_get_ahrs_base_rate(&device_interface_, &base_rate) != MIP_INTERFACE_OK){}
      	ROS_INFO("AHRS Base Rate => %d Hz", base_rate);
      	ros::Duration(dT).sleep();
      	// Deterimine decimation to get close to goal rate
      	u8 imu_decimation = (u8)((float)base_rate/ (float)imu_raw_rate_);
      	ROS_INFO("AHRS decimation set to %#04X",imu_decimation);

      	// AHRS Message Format
      	// Set message format
      	ROS_INFO("Setting the AHRS message format");
      	data_stream_format_descriptors[0] = MIP_AHRS_DATA_ACCEL_SCALED;
      	data_stream_format_descriptors[1] = MIP_AHRS_DATA_GYRO_SCALED;
      	data_stream_format_descriptors[2] = MIP_AHRS_DATA_QUATERNION;
      	data_stream_format_decimation[0]  = imu_decimation;//0x32;
      	data_stream_format_decimation[1]  = imu_decimation;//0x32;
      	data_stream_format_decimation[2]  = imu_decimation;//0x32;
      	data_stream_format_num_entries = 3;
      	while(mip_3dm_cmd_ahrs_message_format(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries, data_stream_format_descriptors, data_stream_format_decimation) != MIP_INTERFACE_OK){}
      	ros::Duration(dT).sleep();
      	// Poll to verify
      	ROS_INFO("Poll AHRS data to verify");
      	while(mip_3dm_cmd_poll_ahrs(&device_interface_, MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries, data_stream_format_descriptors) != MIP_INTERFACE_OK){}
      	ros::Duration(dT).sleep();
      	// Save
      	if (save_settings)
      	{
      	  ROS_INFO("Saving AHRS data settings");
      	  while(mip_3dm_cmd_ahrs_message_format(&device_interface_, MIP_FUNCTION_SELECTOR_STORE_EEPROM, 0, NULL,NULL) != MIP_INTERFACE_OK){}
      	  ros::Duration(dT).sleep();
      	}

      	// Declination Source
      	// Set declination
      	ROS_INFO("Setting declination source to %#04X",declination_source_u8);
      	while(mip_filter_declination_source(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &declination_source_u8) != MIP_INTERFACE_OK){}
      	ros::Duration(dT).sleep();
      	//Read back the declination source
      	ROS_INFO("Reading back declination source");
      	while(mip_filter_declination_source(&device_interface_, MIP_FUNCTION_SELECTOR_READ, &readback_declination_source) != MIP_INTERFACE_OK){}
      	if(declination_source_u8 == readback_declination_source)
      	{
      	  ROS_INFO("Success: Declination source set to %#04X", declination_source_u8);
      	}
      	else
      	{
      	  ROS_WARN("Failed to set the declination source to %#04X!", declination_source_u8);
      	}
      	ros::Duration(dT).sleep();
      	if (save_settings)
      	{
      	  ROS_INFO("Saving declination source settings to EEPROM");
      	  while(mip_filter_declination_source(&device_interface_,MIP_FUNCTION_SELECTOR_STORE_EEPROM,NULL) != MIP_INTERFACE_OK)
      	  {}
      	  ros::Duration(dT).sleep();
      	}
      } // end of AHRS setup

      // Filter setup
      if (publish_imu_filtered_)
      {
	       while(mip_3dm_cmd_get_filter_base_rate(&device_interface_, &base_rate) != MIP_INTERFACE_OK){}
      	ROS_INFO("FILTER Base Rate => %d Hz", base_rate);
      	u8 nav_decimation = (u8)((float)base_rate/ (float)imu_filtered_rate_);
      	ros::Duration(dT).sleep();

      	////////// Filter Message Format
      	// Set
      	ROS_INFO("Setting Filter stream format");

        data_stream_format_descriptors[0] = MIP_FILTER_DATA_ATT_QUATERNION;
        data_stream_format_descriptors[1] = MIP_FILTER_DATA_ATT_UNCERTAINTY_EULER;
        data_stream_format_descriptors[2] = MIP_FILTER_DATA_COMPENSATED_ANGULAR_RATE;
        data_stream_format_descriptors[3] = MIP_FILTER_DATA_FILTER_STATUS;
        data_stream_format_decimation[0]  = nav_decimation; //0x32;
        data_stream_format_decimation[1]  = nav_decimation; //0x32;
        data_stream_format_decimation[2]  = nav_decimation; //0x32;
        data_stream_format_decimation[3]  = nav_decimation; //0x32;
        data_stream_format_num_entries = 4;

        while(mip_3dm_cmd_filter_message_format(&device_interface_, MIP_FUNCTION_SELECTOR_READ, &data_stream_format_num_entries,data_stream_format_descriptors, data_stream_format_decimation) != MIP_INTERFACE_OK){}
        ros::Duration(dT).sleep();//ono_MIP_FUNCTION_SELECTOR_WRITEからMIP_FUNCTION_SELECTOR_READに変更

      	// Poll to verify
      	ROS_INFO("Poll filter data to test stream");
      	while(mip_3dm_cmd_poll_filter(&device_interface_, MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries, data_stream_format_descriptors) != MIP_INTERFACE_OK){}
      	ros::Duration(dT).sleep();
      	// Save
      	if (save_settings)
      	{
      	  ROS_INFO("Saving Filter data settings");
      	  while(mip_3dm_cmd_filter_message_format(&device_interface_, MIP_FUNCTION_SELECTOR_STORE_EEPROM, 0, NULL,NULL) != MIP_INTERFACE_OK){}
      	  ros::Duration(dT).sleep();
      	}

      	// Heading Source
      	ROS_INFO("Set heading source to internal mag.");
      	heading_source = 0x1;
      	ROS_INFO("Setting heading source to %#04X",heading_source);
      	while(mip_filter_heading_source(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, &heading_source) != MIP_INTERFACE_OK)
      	{

        }
      	ros::Duration(dT).sleep();

      	ROS_INFO("Read back heading source...");
      	while(mip_filter_heading_source(&device_interface_,MIP_FUNCTION_SELECTOR_READ,&readback_headingsource)!= MIP_INTERFACE_OK){}
      	ROS_INFO("Heading source = %#04X",readback_headingsource);
      	ros::Duration(dT).sleep();

      	if (save_settings)
      	{
      	  ROS_INFO("Saving heading source to EEPROM");
      	  while(mip_filter_heading_source(&device_interface_,MIP_FUNCTION_SELECTOR_STORE_EEPROM,NULL)!= MIP_INTERFACE_OK)
          {

          }
      	  ros::Duration(dT).sleep();
      	}
      }  // end of Filter setup

      // I believe the auto-init pertains to the kalman filter for the -45
      // OR for the complementary filter for the -25  - need to test
      // Auto Initialization
      // Set auto-initialization based on ROS parameter
      ROS_INFO("Setting auto-initinitalization to: %#04X",auto_init);
      auto_init_u8 = auto_init;  // convert bool to u8
      while(mip_filter_auto_initialization(&device_interface_,MIP_FUNCTION_SELECTOR_WRITE,&auto_init_u8) != MIP_INTERFACE_OK)
      {

      }
      ros::Duration(dT).sleep();

      if (readback_settings)
      {
      	// Read the settings back
      	ROS_INFO("Reading back auto-initialization value");
      	while(mip_filter_auto_initialization(&device_interface_, MIP_FUNCTION_SELECTOR_READ, &readback_auto_init)!= MIP_INTERFACE_OK)
      	{

        }
      	ros::Duration(dT).sleep();
      	if (auto_init == readback_auto_init)
        {
          ROS_INFO("Success: Auto init. setting is: %#04X",readback_auto_init);
        }
        else
        {
          ROS_ERROR("Failure: Auto init. setting set to be %#04X, but reads as %#04X",auto_init,readback_auto_init);
        }
      }

      if (save_settings)
      {
      	ROS_INFO("Saving auto init. settings to EEPROM");
      	while(mip_filter_auto_initialization(&device_interface_,MIP_FUNCTION_SELECTOR_STORE_EEPROM,NULL) != MIP_INTERFACE_OK)
      	{

        }
      	ros::Duration(dT).sleep();
      }

      // Enable Data streams
      dT = 0.25;
      if (publish_imu_raw_)
      {
      	ROS_INFO("Enabling AHRS stream");
      	enable = 0x01;
      	while(mip_3dm_cmd_continuous_data_stream(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_AHRS_DATASTREAM, &enable) != MIP_INTERFACE_OK){}
      	ros::Duration(dT).sleep();
      }

      if (publish_imu_filtered_)
      {
      	ROS_INFO("Enabling Filter stream");
      	enable = 0x01;
      	while(mip_3dm_cmd_continuous_data_stream(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_INS_DATASTREAM, &enable) != MIP_INTERFACE_OK){}
        ros::Duration(dT).sleep();
      }

      ROS_INFO("End of device setup - starting streaming");
    }
    else
    {
      ROS_INFO("Skipping device setup and listing for existing streams");
    } // end of device_setup

    // Reset filter - should be for either the KF or CF
    ROS_INFO("Reset filter");
    while(mip_filter_reset_filter(&device_interface_) != MIP_INTERFACE_OK){}
    ros::Duration(dT).sleep();

    // Loop
    // Determine loop rate as 2*(max update rate), but abs. max of 1kHz
    int max_rate = 1;
    if (publish_imu_raw_)
    {
      max_rate = std::max(max_rate,imu_raw_rate_);
    }
    if (publish_imu_filtered_)
    {
      max_rate = std::max(max_rate,imu_filtered_rate_);
    }
    // int spin_rate = std::min(3*max_rate,1000);//なんで×3してる？
    int spin_rate = std::min(max_rate,1000);
    ROS_INFO("Setting spin rate to <%d>",spin_rate);
    ros::Rate r(spin_rate);  // Rate in Hz

    setup_complete_flag_ = true;

    while (ros::ok())
    {
      //Update the parser (this function reads the port and parses the bytes
      mip_interface_update(&device_interface_);
      ros::spinOnce();  // take care of service requests.
      r.sleep();

      //ROS_INFO("Spinning");
    } // end loop

    // close serial port
    mip_sdk_port_close(device_interface_.port_handle);

  } // End of ::run()

  bool Microstrain::reset_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
  {
    ROS_INFO("Reseting the filter");
    while(mip_filter_reset_filter(&device_interface_) != MIP_INTERFACE_OK){}

    return true;
  }


  //////filtered imu///////
  void Microstrain::filter_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
  {
    mip_field_header *field_header;
    u8               *field_data;
    u16              field_offset = 0;

    if(!setup_complete_flag_) return;

    // If we aren't publishing, then return
    if (!publish_imu_filtered_) return;
    // ROS_INFO("Filter callback");
    //The packet callback can have several types, process them all
    switch(callback_type)
    {
    	///
    	//Handle valid packets
    	///

      case MIP_INTERFACE_CALLBACK_VALID_PACKET:
	    {
	      filter_valid_packet_count_++;

    	  ///
    	  //Loop through all of the data fields
    	  ///

	      while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
	      {

  	      ///
  	      // Decode the field
  	      ///

          switch(field_header->descriptor)
		      {

             // Quaternion
             case MIP_FILTER_DATA_ATT_QUATERNION:
             {
               memcpy(&curr_filter_quaternion_, field_data, sizeof(mip_filter_attitude_quaternion));

               //For little-endian targets, byteswap the data field
               mip_filter_attitude_quaternion_byteswap(&curr_filter_quaternion_);

              imu_filtered_msg_.header.seq = filter_valid_packet_count_;
              imu_filtered_msg_.header.stamp = ros::Time::now();
              imu_filtered_msg_.header.frame_id = imu_frame_id_;

              // put into ENU - swap X/Y, invert Z
              imu_filtered_msg_.orientation.x = curr_filter_quaternion_.q[2];
              imu_filtered_msg_.orientation.y = curr_filter_quaternion_.q[1];
              imu_filtered_msg_.orientation.z = -1.0*curr_filter_quaternion_.q[3];
              imu_filtered_msg_.orientation.w = curr_filter_quaternion_.q[0];

            }break;

      		  ///
      		  // Estimated Attitude, Euler Angles
      		  ///

    		    case MIP_FILTER_DATA_ATT_EULER_ANGLES:
    		    {
    		      memcpy(&curr_filter_angles_, field_data, sizeof(mip_filter_attitude_euler_angles));

    		      //For little-endian targets, byteswap the data field
    		      mip_filter_attitude_euler_angles_byteswap(&curr_filter_angles_);

    		    }break;

    		    // Angular Rates
    		    case MIP_FILTER_DATA_COMPENSATED_ANGULAR_RATE:
    		    {
    		      memcpy(&curr_filter_angular_rate_, field_data, sizeof(mip_filter_compensated_angular_rate));

    		      //For little-endian targets, byteswap the data field
    		      mip_filter_compensated_angular_rate_byteswap(&curr_filter_angular_rate_);

    		      imu_filtered_msg_.angular_velocity.x = curr_filter_angular_rate_.x;
    		      imu_filtered_msg_.angular_velocity.y = curr_filter_angular_rate_.y;
    		      imu_filtered_msg_.angular_velocity.z= curr_filter_angular_rate_.z;


    		    }break;

            // liniear acc_ono_add
            case MIP_FILTER_DATA_LINEAR_ACCELERATION:
            {
              memcpy(&curr_filter_linear_acc_, field_data, sizeof(mip_filter_linear_acceleration));

              //For little-endian targets, byteswap the data field
              mip_filter_linear_acceleration_byteswap(&curr_filter_linear_acc_);

              imu_filtered_msg_.linear_acceleration.x = 9.81*curr_filter_linear_acc_.x;
              imu_filtered_msg_.linear_acceleration.y = 9.81*curr_filter_linear_acc_.y;
              imu_filtered_msg_.linear_acceleration.z = 9.81*curr_filter_linear_acc_.z;


            }break;


            // Attitude Uncertainty
            case MIP_FILTER_DATA_ATT_UNCERTAINTY_EULER:
            {
              memcpy(&curr_filter_att_uncertainty_, field_data, sizeof(mip_filter_euler_attitude_uncertainty));
              //For little-endian targets, byteswap the data field
              mip_filter_euler_attitude_uncertainty_byteswap(&curr_filter_att_uncertainty_);
              imu_filtered_msg_.orientation_covariance[0] = curr_filter_att_uncertainty_.roll*curr_filter_att_uncertainty_.roll;
              imu_filtered_msg_.orientation_covariance[4] = curr_filter_att_uncertainty_.pitch*curr_filter_att_uncertainty_.pitch;
              imu_filtered_msg_.orientation_covariance[8] = curr_filter_att_uncertainty_.yaw*curr_filter_att_uncertainty_.yaw;

            }break;

            // // Velocity Uncertainty
            // case MIP_FILTER_DATA_VEL_UNCERTAINTY:
            // {
            //   memcpy(&curr_filter_vel_uncertainty_, field_data, sizeof(mip_filter_ned_vel_uncertainty));
            //
            //   //For little-endian targets, byteswap the data field
            //   mip_filter_ned_vel_uncertainty_byteswap(&curr_filter_vel_uncertainty_);
            //
            //   imu_filtered_msg_.twist.covariance[0] = curr_filter_vel_uncertainty_.east*curr_filter_vel_uncertainty_.east;
            //   imu_filtered_msg_.twist.covariance[4] = curr_filter_vel_uncertainty_.north*curr_filter_vel_uncertainty_.north;
            //   imu_filtered_msg_.twist.covariance[8] = curr_filter_vel_uncertainty_.down*curr_filter_vel_uncertainty_.down;
            //
            // }break;

    		    // Filter Status
    		    case MIP_FILTER_DATA_FILTER_STATUS:
    		    {
    		      memcpy(&curr_filter_status_, field_data, sizeof(mip_filter_status));

    		      //For little-endian targets, byteswap the data field
    		      mip_filter_status_byteswap(&curr_filter_status_);

    		      nav_status_msg_.data.clear();
    		      ROS_DEBUG_THROTTLE(1.0,"Filter Status: %#06X, Dyn. Mode: %#06X, Filter State: %#06X",
    				  curr_filter_status_.filter_state,
    				  curr_filter_status_.dynamics_mode,
    				  curr_filter_status_.status_flags);
    		      nav_status_msg_.data.push_back(curr_filter_status_.filter_state);
    		      nav_status_msg_.data.push_back(curr_filter_status_.dynamics_mode);
    		      nav_status_msg_.data.push_back(curr_filter_status_.status_flags);
    		      nav_status_pub_.publish(nav_status_msg_);

    		    }break;

    		    default: break;
    		  }
	     }

	  // Publish
	  imu_filtered_pub_.publish(imu_filtered_msg_);
	}break;


	///
	//Handle checksum error packets
	///

      case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
	{
	  filter_checksum_error_packet_count_++;
	}break;

	///
	//Handle timeout packets
	///

      case MIP_INTERFACE_CALLBACK_TIMEOUT:
	{
	  filter_timeout_packet_count_++;
	}break;
      default: break;
      }

    print_packet_stats();
  } // filter_packet_callback


  ////////////////////////////////////////////////////////////////////////////////
  //
  // AHRS Packet Callback
  //
  ////////////////////////////////////////////////////////////////////////////////

  void Microstrain::ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
  {
    mip_field_header *field_header;
    u8               *field_data;
    u16              field_offset = 0;

    if(!setup_complete_flag_) return;

    // If we aren't publishing, then return
    if (!publish_imu_raw_) return;
    // ROS_INFO("AHRS callback");
    //The packet callback can have several types, process them all
    switch(callback_type)
    {
  	///
  	//Handle valid packets
  	///

    case MIP_INTERFACE_CALLBACK_VALID_PACKET:
	  {
	    ahrs_valid_packet_count_++;

  	  ///
  	  //Loop through all of the data fields
  	  ///

	    while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
	    {

	      ///
	      // Decode the field
	      ///

	      switch(field_header->descriptor)
		    {
          ///
          // Scaled Accelerometer
          ///

          case MIP_AHRS_DATA_ACCEL_SCALED:
          {
            memcpy(&curr_ahrs_accel_, field_data, sizeof(mip_ahrs_scaled_accel));

            //For little-endian targets, byteswap the data field
            mip_ahrs_scaled_accel_byteswap(&curr_ahrs_accel_);

            // Stuff into ROS message - acceleration in m/s^2
            // Header
            imu_msg_.header.seq = ahrs_valid_packet_count_;
            imu_msg_.header.stamp = ros::Time::now();
            imu_msg_.header.frame_id = imu_frame_id_;
            imu_msg_.linear_acceleration.x = 9.81*curr_ahrs_accel_.scaled_accel[0];
            imu_msg_.linear_acceleration.y = 9.81*curr_ahrs_accel_.scaled_accel[1];
            imu_msg_.linear_acceleration.z = 9.81*curr_ahrs_accel_.scaled_accel[2];

          }break;

          ///
          // Scaled Gyro
          ///
          case MIP_AHRS_DATA_GYRO_SCALED:
          {
            memcpy(&curr_ahrs_gyro_, field_data, sizeof(mip_ahrs_scaled_gyro));

            //For little-endian targets, byteswap the data field
            mip_ahrs_scaled_gyro_byteswap(&curr_ahrs_gyro_);

            imu_msg_.angular_velocity.x = curr_ahrs_gyro_.scaled_gyro[0];
            imu_msg_.angular_velocity.y = curr_ahrs_gyro_.scaled_gyro[1];
            imu_msg_.angular_velocity.z = curr_ahrs_gyro_.scaled_gyro[2];

          }break;

          ///
          // Scaled Magnetometer
          ///

          case MIP_AHRS_DATA_MAG_SCALED:
          {
            memcpy(&curr_ahrs_mag_, field_data, sizeof(mip_ahrs_scaled_mag));

            //For little-endian targets, byteswap the data field
            mip_ahrs_scaled_mag_byteswap(&curr_ahrs_mag_);

          }break;

          // Quaternion
          case MIP_AHRS_DATA_QUATERNION:
          {
            memcpy(&curr_ahrs_quaternion_, field_data, sizeof(mip_ahrs_quaternion));

            //For little-endian targets, byteswap the data field
            mip_ahrs_quaternion_byteswap(&curr_ahrs_quaternion_);
            // put into ENU - swap X/Y, invert Z
            imu_msg_.orientation.x = curr_ahrs_quaternion_.q[2];
            imu_msg_.orientation.y = curr_ahrs_quaternion_.q[1];
            imu_msg_.orientation.z = -1.0*curr_ahrs_quaternion_.q[3];
            imu_msg_.orientation.w = curr_ahrs_quaternion_.q[0];

          }break;

  		    default: break;
        }
	    }

  	  // Publish
  	  imu_raw_pub_.publish(imu_msg_);

  	  }break;

    	///
    	//Handle checksum error packets
    	///

      case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
    	{
    	  ahrs_checksum_error_packet_count_++;
    	}break;

    	///
    	//Handle timeout packets
    	///

      case MIP_INTERFACE_CALLBACK_TIMEOUT:
    	{
    	  ahrs_timeout_packet_count_++;
    	}break;

      default: break;

    }

    print_packet_stats();
  } // ahrs_packet_callback

  void Microstrain::print_packet_stats()
  {
    ROS_DEBUG_THROTTLE(1.0,"%u FILTER (%u errors)    %u AHRS (%u errors)", filter_valid_packet_count_,  filter_timeout_packet_count_ + filter_checksum_error_packet_count_, ahrs_valid_packet_count_, ahrs_timeout_packet_count_ + ahrs_checksum_error_packet_count_);
  } // print_packet_stats

  // Wrapper callbacks
  void filter_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
  {
    Microstrain* ustrain = (Microstrain*) user_ptr;
    ustrain->filter_packet_callback(user_ptr,packet,packet_size,callback_type);
  }

  void ahrs_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
  {
    Microstrain* ustrain = (Microstrain*) user_ptr;
    ustrain->ahrs_packet_callback(user_ptr,packet,packet_size,callback_type);
  }

} // Microstrain namespace
