//version2.4
//2017/10/13 editor ONO
//クレアクトのッジャイロセンサに対応
//差分で角度の積算値を出してみたが、うまく行かない
//2017/10/30 角速度から数値積分するようにした
//2017/11/01 角度を-piからpiになるように変更


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <time.h>
#include <tf/transform_datatypes.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> SyncPolicy;


class ODOM_IMU_COMBINE{
private:

  std::string sub_odom_topic_;
  std::string sub_imu_topic_;
  std::string pub_odom_imu_topic_;
  std::string frame_id, child_frame_id;

  ros::Publisher odom_imu_pub;
  tf::TransformBroadcaster odom_broadcaster;
  nav_msgs::Odometry odom_imu_;

  message_filters::Subscriber<nav_msgs::Odometry> *odom_sub;
  message_filters::Subscriber<sensor_msgs::Imu> *imu_sub;
  message_filters::Synchronizer<SyncPolicy> *sync;

  void odom_subscriber(const nav_msgs::Odometry::ConstPtr &sub);
  void imu_subscriber(const sensor_msgs::Imu::ConstPtr &sub);
  void sync_odom_imu(const nav_msgs::Odometry::ConstPtr &odom, const sensor_msgs::Imu::ConstPtr &imu);

  double last_odom_dist_x_;
  double last_odom_dist_y_;

  bool init_flag_;

  double last_velocity[3];

  double gyro_roll;
  double gyro_pitch;
  double gyro_yaw;

  double start, end;

public:
  ODOM_IMU_COMBINE();
  ~ODOM_IMU_COMBINE();

  void run();

};


ODOM_IMU_COMBINE::ODOM_IMU_COMBINE(){
  ros::NodeHandle n;
  // rosparam
  ros::NodeHandle private_nh("~");
  private_nh.param("sub_odom_topic", sub_odom_topic_, std::string("/odom_default"));
  private_nh.param("sub_imu_topic", sub_imu_topic_, std::string("/imu"));
  private_nh.param("pub_odom_imu_topic", pub_odom_imu_topic_, std::string("/odom_imu"));
  private_nh.param("frame_id", frame_id, std::string("odom_imu"));
  private_nh.param("child_frame_id", child_frame_id, std::string("base_footprint_odom_imu"));

  //SubscriberとPublisherの設定
  odom_imu_pub = n.advertise<nav_msgs::Odometry>(pub_odom_imu_topic_, 1);
  odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>(n,sub_odom_topic_,1);
  imu_sub = new message_filters::Subscriber<sensor_msgs::Imu>(n,sub_imu_topic_,1);
  sync = new  message_filters::Synchronizer<SyncPolicy>(SyncPolicy(100), *odom_sub, *imu_sub);

  //odom_imuの初期化
  odom_imu_.header.frame_id = frame_id;
  odom_imu_.child_frame_id = child_frame_id;

  odom_imu_.pose.pose.position.x = 0.0;
  odom_imu_.pose.pose.position.y = 0.0;
  odom_imu_.pose.pose.position.z = 0.0;

  odom_imu_.pose.pose.orientation.x = 0.0;
  odom_imu_.pose.pose.orientation.y = 0.0;
  odom_imu_.pose.pose.orientation.z = 0.0;
  odom_imu_.pose.pose.orientation.w = 1.0;

  last_odom_dist_x_= 0.0;
  last_odom_dist_y_= 0.0;
  last_velocity[0]=0.0;
  last_velocity[1]=0.0;
  last_velocity[2]=0.0;
  init_flag_=true;
  start = 0;
}

ODOM_IMU_COMBINE::~ODOM_IMU_COMBINE(){
}

//////////////////////////

void ODOM_IMU_COMBINE::sync_odom_imu(const nav_msgs::Odometry::ConstPtr &odom, const sensor_msgs::Imu::ConstPtr &imu){

  odom_imu_.header.stamp  = imu->header.stamp;

  odom_imu_.twist.twist.linear  = odom->twist.twist.linear;
  odom_imu_.twist.twist.angular = odom->twist.twist.angular;

  end = double(imu->header.stamp.sec)+double(imu->header.stamp.nsec)*1e-9;

  if(start == 0.0){
    start = double(imu->header.stamp.sec)+double(imu->header.stamp.nsec)*1e-9;
    return;
  }

  double dt = end-start;

  start = double(imu->header.stamp.sec)+double(imu->header.stamp.nsec)*1e-9;

  //std::cout<<"dt:"<<dt<<std::endl;

  //ドリフト誤差が多い場合
  // if(std::abs(imu->angular_velocity.x)>0.005)gyro_roll += imu->angular_velocity.x*dt;
  // if(std::abs(imu->angular_velocity.y)>0.005)gyro_pitch += imu->angular_velocity.y*dt;
  // if(std::abs(imu->angular_velocity.z)>0.005)gyro_yaw += imu->angular_velocity.z*dt;
  gyro_roll += (imu->angular_velocity.x+last_velocity[0])*dt*0.5;
  gyro_pitch += (imu->angular_velocity.y+last_velocity[1])*dt*0.5;
  gyro_yaw += (imu->angular_velocity.z+last_velocity[2])*dt*0.5;
  last_velocity[0]=imu->angular_velocity.x;
  last_velocity[1]=imu->angular_velocity.y;
  last_velocity[2]=imu->angular_velocity.z;

  //ロールとピッチを使わない場合
  gyro_roll=0.0;
  gyro_pitch=0.0;

  while(gyro_roll>M_PI){
    gyro_roll-=2.0*M_PI;
  }
  while(gyro_roll<-M_PI){
    gyro_roll+=2.0*M_PI;
  }

  while(gyro_pitch>M_PI){
    gyro_pitch-=2.0*M_PI;
  }
  while(gyro_pitch<-M_PI){
    gyro_pitch+=2.0*M_PI;
  }

  while(gyro_yaw>M_PI){
    gyro_yaw-=2.0*M_PI;
  }
  while(gyro_yaw<-M_PI){
    gyro_yaw+=2.0*M_PI;
  }

    odom_imu_.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(gyro_roll, gyro_pitch, gyro_yaw);

  //debug用
  double gyro_roll_deg =  gyro_roll/M_PI*180.0;
  double gyro_pitch_deg = gyro_pitch/M_PI*180.0;
  double gyro_yaw_deg = gyro_yaw/M_PI*180.0;

  //データ確認表示用
  // double gyro_roll_deg = roll/M_PI*180.0;
  // double gyro_pitch_deg = pitch/M_PI*180.0;
  // double gyro_yaw_deg = yaw/M_PI*180.0;
  // std::cout<<"gyro:"<<gyro_roll_deg<<" , "<<gyro_pitch_deg<<" , "<<gyro_yaw_deg<<std::endl;
  // std::cout<<"gyro_yaw:"<<gyro_yaw_deg<<std::endl;

  double odom_dist = hypot (odom->pose.pose.position.x-last_odom_dist_x_,odom->pose.pose.position.y-last_odom_dist_y_);

  if(odom_imu_.twist.twist.linear.x < 0){
    odom_dist = -odom_dist;//速度がマイナスの時は後に下がっているとする
  }

  //ヨー角のみの場合
  odom_imu_.pose.pose.position.x += odom_dist *cos(gyro_yaw);
  odom_imu_.pose.pose.position.y += odom_dist *sin(gyro_yaw);
  odom_imu_.pose.pose.position.z += 0;

  //ヨー角、ピッチを考慮した場合
  // odom_imu_.pose.pose.position.x += odom_dist *cos(gyro_yaw)*sin(gyro_pitch+M_PI/2.0);
  // odom_imu_.pose.pose.position.y += odom_dist *sin(gyro_yaw)*sin(gyro_pitch+M_PI/2.0);
  // odom_imu_.pose.pose.position.z += odom_dist *cos(gyro_pitch+M_PI/2.0);

  odom_imu_pub.publish(odom_imu_);
  last_odom_dist_x_= odom->pose.pose.position.x;
  last_odom_dist_y_= odom->pose.pose.position.y;

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp    = odom_imu_.header.stamp ;
  odom_trans.header.frame_id = frame_id;
  odom_trans.child_frame_id  = child_frame_id;
  odom_trans.transform.translation.x = odom_imu_.pose.pose.position.x ;
  odom_trans.transform.translation.y = odom_imu_.pose.pose.position.y;
  odom_trans.transform.translation.z = odom_imu_.pose.pose.position.z;
  odom_trans.transform.rotation      = odom_imu_.pose.pose.orientation;
  odom_broadcaster.sendTransform(odom_trans);

  //ROS_INFO("odom_gyro_yaw:%lf\n",gyro_yaw*180.0/M_PI);
  //ROS_INFO("odom_gyro_th:%lf\n",odom_gyro_th);

}

void ODOM_IMU_COMBINE::run()
{

  sync->registerCallback(boost::bind(&ODOM_IMU_COMBINE::sync_odom_imu,this, _1, _2));

}

int main(int argc, char** argv){

  ros::init(argc, argv, "odom_imu_combine");

  ODOM_IMU_COMBINE combine;

  combine.run();

  ros::spin();

  return 0;
}
