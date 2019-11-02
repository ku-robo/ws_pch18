/*-----------------------------------------------
* 	okatech_kuaro.cpp
* <Last Update>	H29/09/28
* <editor> Takafumi ONO
* <version>		v2.1
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

  // rosparam
  ros::NodeHandle private_nh("~");
  private_nh.param("sub_serial_topic", sub_serial_topic_, std::string("/serial_receive"));
  private_nh.param("sub_cmdvel_topic", sub_cmdvel_topic_, std::string("/cmd_vel"));
  private_nh.param("pub_serial_topic", pub_serial_topic_, std::string("/serial_send"));
  private_nh.param("pub_odom_topic", pub_odom_topic_, std::string("/odom_default"));
  private_nh.param("frame_id", frame_id, std::string("odom_default"));
  private_nh.param("child_frame_id", child_frame_id, std::string("base_footprint_odom_default"));

  odom_.header.stamp    = ros::Time::now();
  odom_.header.frame_id = frame_id;
  odom_.child_frame_id  = child_frame_id;

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

  pub_ToSerial_ = n.advertise<std_msgs::String>(pub_serial_topic_,10);
  pub_odom_ = n.advertise<nav_msgs::Odometry>(pub_odom_topic_,10);
  sub_Serial_ = n.subscribe(sub_serial_topic_,10,&OKATECH::ReceiveComandFromSerial_Callback,this);
  sub_velcmd_ = n.subscribe(sub_cmdvel_topic_,10,&OKATECH::SendComandToSerial_Callback,this);

  //TinyPowerからオドメトリを取ってくるコマンド送信のタイマー関数
  timer_ = n.createTimer(ros::Duration(encoder_get_time), boost::bind(&OKATECH::cmd_timer, this));

}


OKATECH::~OKATECH()
{

}

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

    //tf::TransformBroadcaster odom_broadcaster;
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id  = child_frame_id;
    odom_trans.header.stamp    = ros::Time::now();
    odom_trans.transform.translation.x = odom_.pose.pose.position.x ;
    odom_trans.transform.translation.y = odom_.pose.pose.position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation      = odom_.pose.pose.orientation;
    odom_broadcaster.sendTransform(odom_trans);
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

}

int main(int argc, char** argv){
  ros::init(argc, argv, "TinyPower_control");

  OKATECH okatech;

  ros::spin();

  return 0;
}
