#include <sensor_msgs/Joy.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class JOYSTIC_KUARO{
private:


public:
  JOYSTIC;
  ~JOYSTIC;

  ros::Publisher pub_vel_;
  ros::Subscriber sub_joy_;

};

JOYSTIC::JOYSTIC(){
  	ros::NodeHandle n;

  	pub_vel_= n.advertise<geometry_msgs::Twist>("cmd_vel",10);
  	sub_joy_ = n.subscribe("joy", 10, Callback,this);
}

JOYSTIC::~JOYSTIC(){

}

void JOYSTIC::Callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist cmd_vel;
  //初期:0	前進:-1		回転:1
	static int turn_flag = 0;
  //キーから手を離した時を検知するのに使用
	bool key_remove = true;
	//キーパッドが押された時
	for (int num = 0; num < joy->axes.size(); num++)
	{
		if (joy->axes[num] != 0)
		{
			if(num > 3 && turn_flag >= 0)
			{
				turn_flag=-1;
			}

			switch (num)
			{
        //アナログスティック（1:左、-1：右）
				case 0:
          cmd_vel.twist.twist.linear.x = ;
        break;
        case 1:case 2:case 3:

				break;
        // //方向キー
				// case 4:
				// //左
				// if (joy->axes[num] > 0)
				// {
				// 	//puts("左");
				// 	if (wheel.cv < OKATECH_LIMIT_CV)
				// 	{
				// 		wheel.cv += 0.5;
				// 		runV_cv_slow();
				// 	}
				// }
				// else
				// {
				// 	//puts("右");
				// 	if (wheel.cv > -OKATECH_LIMIT_CV)
				// 	{
				// 		wheel.cv -= 0.5;
				// 		runV_cv_slow();
				// 	}
				// }
				// break;
				// case 5:
				// //上
				// if (joy->axes[num] > 0)
				// {
				// 	//puts("上");
				// 	if (wheel.v < OKATECH_LIMIT_SPEED_V - 0.05)
				// 	{
				// 		wheel.v += 0.05;
				// 		runV_cv_slow();
				// 	}
				// }
				// else
				// {
				// 	//puts("下");
				// 	if (wheel.v > -OKATECH_LIMIT_SPEED_V - 0.05)
				// 	{
				// 		wheel.v -= 0.05;
				// 		runV_cv_slow();
				// 	}
				// }
				// break;
				default:
				break;
			}
			key_remove = false;
		}
	}

	//ABXYボタンが押された時
	for (int num = 0; num < joy->buttons.size(); num++)
	{
		if (joy->buttons[num] == 1)
		{
			switch (num)
			{
				//STOP	ボタンから手を離したら止まるにしてる場合はコメントアウト
				case 2:
				stop_slow();
				break;
				//左回転
				case 4: case 6:
				if(turn_flag <= 0)
				{
					turn_flag=1;
					stop_slow();
				}
				if (wheel.rv < OKATECH_TURN_LIMIT-0.02)
				{
					wheel.rv += 0.02;
					runR();
				}
				break;
				//右回転
				case 5: case 7:
				if(turn_flag <= 0)
				{
					turn_flag=1;
					stop_slow();
				}
				if (wheel.rv > -OKATECH_TURN_LIMIT+0.02)
				{
					wheel.rv -= 0.02;
					runR();
				}
				break;
				//Cv初期化
				case 0:
				wheel.cv = 0.0;
				runV_cv_slow();
				break;

				default:
				break;
			}
			key_remove = false;
		}
	}

  pub_vel_(cmd_vel);
}

int main(){
  ros::init(argc, argv, "joystic_kuaro");

	JOYSTIC_KUARO joy;

	ros::spin();

  return 0;
}
