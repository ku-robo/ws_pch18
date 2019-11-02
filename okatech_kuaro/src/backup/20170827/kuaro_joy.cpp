//参考：ROSのturtlebot_teleopパッケージ

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class KUARO_JOY
{
public:
  KUARO_JOY();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, deadman_buttons_,stop_buttons_,rotation_R_,rotation_L_,straight_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  bool zero_twist_published_;
  bool stop_pressed_;
  ros::Timer timer_;

};

KUARO_JOY::KUARO_JOY():
ph_("~"),
linear_(1),
angular_(0),
deadman_buttons_(4),
l_scale_(0.3),
a_scale_(0.9)
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("buttons_deadman", deadman_buttons_, deadman_buttons_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);
  ph_.param("buttons_stop", stop_buttons_, stop_buttons_);
  ph_.param("buttons_rotation_R", rotation_R_, rotation_R_);
  ph_.param("buttons_rotation_L", rotation_L_, rotation_L_);
  ph_.param("buttons_straight", straight_, straight_);

  deadman_pressed_ = false;
  zero_twist_published_ = false;
  stop_pressed_= false;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &KUARO_JOY::joyCallback, this);
  //publishの速度
  timer_ = nh_.createTimer(ros::Duration(0.05), boost::bind(&KUARO_JOY::publish, this));
}

void KUARO_JOY::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  if(joy->axes[rotation_R_]>=0 && joy->axes[rotation_L_]>=0)
  {
    if(joy->buttons[straight_]<=0)
    {
      vel.angular.z = a_scale_*joy->axes[angular_];
      vel.linear.x = l_scale_*joy->axes[linear_];
      last_published_ = vel;
    }
    else if(joy->buttons[straight_]>0)
    {
      vel.angular.z = 0.0;
      vel.linear.x = l_scale_*joy->axes[linear_];
      last_published_ = vel;
    }
  }
  else if(joy->axes[rotation_R_]<0)
  {
    vel.angular.z = -0.6;
    vel.linear.x = 0;
    last_published_ = vel;
  }
  else if(joy->axes[rotation_L_]<0)
  {
    vel.angular.z = 0.6;
    vel.linear.x = 0;
    last_published_ = vel;
  }
  deadman_pressed_ = joy->buttons[deadman_buttons_];
  stop_pressed_=joy->buttons[stop_buttons_];
}

void KUARO_JOY::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  if (deadman_pressed_ && !stop_pressed_)
  {
    vel_pub_.publish(last_published_);
    zero_twist_published_=false;
  }
  else if((!deadman_pressed_ && !zero_twist_published_) || (stop_pressed_))
  {
    vel_pub_.publish(*new geometry_msgs::Twist());
    zero_twist_published_=true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kuaro_joy");
  KUARO_JOY kuaro_joy;

  ros::spin();
}
