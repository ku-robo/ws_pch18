/*-----------------------------------------------
 * 	uk_okatech.cpp
 * <Last Update>	H27/11/03
 * <version>		v1.4
 *
 * <MEMO>
 * オカテック制御プログラム
 * wheel1:右車輪
 * wheel2:左車輪
 * <11.03>  joy henkou
 * <10.30>	position.dx_v,position.dx_yが
 * 			更新されないバグを修正
 * <10.29>	ME使えるようにした
 * 			前方をx座標に(以前は前方y)
 * 			ゆっくりと停止追加
 * ---------------------------------------------*/

 /*-----ヘッダー宣言-------------------------------*/
#include "uk_okatech.h"
#ifdef WIN32
#include<conio.h>
#endif

#include <fstream>

OKA_WHEEL U_OKATECH::wheel;
OKA_POSITION U_OKATECH::position;

//gyro
double g_rad_now = 0.0;
double g_rad_ago = 0.0;
double g_drad = 0;
bool gyro_flag = false;



//使用するオブザーバー(Okatech専用)
#ifdef WRITE_ME
void Observer::notify(const std::string& str) {
	interim_1 = 0; interim_2 = 0; interim_3 = 0; interim_4 = 0;
	static bool ustri_flag = false;
	static bool read_flag = false;
	static std::string extraction;

	for (unsigned int it = 0; it < (unsigned int)str.size(); it++)
	{
		if(ustri_flag && str[it] == '>'){
			read_flag = true;break;
		}
		if (ustri_flag && str[it] != '\n' && str[it] != '\r'){
			extraction += str[it];
		}
		if (str[it] == ':'){
			ustri_flag = true;
		}
	}
	if(!read_flag){return;}else{
		ustri_flag = false;
		read_flag = false;
		//std::cout << extraction << std::endl;
#ifdef __linux__
		sscanf(extraction.c_str(), "%d,%d,%d,%d", &interim_1, &interim_2, &interim_3, &interim_4);
#endif
#ifdef WIN32
		sscanf_s(extraction.c_str(), "%d,%d,%d,%d", &interim_1, &interim_2, &interim_3, &interim_4);
#endif
		//これを入れると止まっている時などは更新しなくなる
		//if (fabs(interim_3) + fabs(interim_4) > 1){
			U_OKATECH::wheel.read.d1 = (double)interim_3;
			U_OKATECH::wheel.read.d2 = (double)interim_4;
			U_OKATECH::update_position();
			//std::cout << "(d1 , d2) : " << U_OKATECH::wheel.read.d1 << " , " << U_OKATECH::wheel.read.d2 << std::endl;
		//}
		extraction.clear();
	}
}
#else
void Observer::notify(const std::string& str) {
	interim_1 = 999; interim_2 = 999;
	static bool ustri_flag = false;
	static bool read_flag = false;
	static std::string extraction;
	for (unsigned int it = 0; it < (unsigned int)str.size(); it++)
	{
		if(ustri_flag && str[it] == '>'){
			read_flag = true;break;
		}
		if (ustri_flag && str[it] != '\n' && str[it] != '\r'){
			extraction += str[it];
		}
		if (str[it] == ':'){
			ustri_flag = true;
		}

	}
	if(!read_flag){return;}else{
		ustri_flag = false;
		read_flag = false;
		//std::cout << extraction << std::endl;
#ifdef __linux__
		sscanf(extraction.c_str(), "%lf,%lf", &interim_1, &interim_2);
#endif
#ifdef WIN32
		sscanf_s(extraction.c_str(), "%lf,%lf", &interim_1, &interim_2);
#endif
		if (fabs(interim_1) + fabs(interim_2) < 900){
			U_OKATECH::wheel.read.d1 = interim_1;
			U_OKATECH::wheel.read.d2 = interim_2;
			U_OKATECH::update_position();
			//std::cout << "(d1 , d2) : " << U_OKATECH::wheel.read.d1 << " , " << U_OKATECH::wheel.read.d2 << std::endl;
		}
		extraction.clear();
	}
}
#endif

//コンストラクタ
U_OKATECH::U_OKATECH(std::string Port){
	while (true)
	{
		if (TinPow.open(Port)){
#ifdef __linux__
			puts("Linux OKATECH 通信開始");
#endif
#ifdef WIN32
			puts("Windows OKATECH 通信開始");
#endif
			break;
		}
		else{
			char judge = '0';
			puts("-- OPEN ERROR --");
			std::cout << "COMのPORT番号を変更しますか ? (y/n)   ";
			std::cin >> judge;
			if (judge == 'y'){
				std::cout << "PORT を入力してください\t" << "→  ";
				std::cin >> Port;
			}
			else{
				std::cout << "もう一度接続しますか ? (y/n)   ";
				std::cin >> judge;
				if (judge == 'y')continue;
				puts("プログラムを終了します");
				exit(0);
			}
		}
	}
	// オブザーバー登録
	TinPow.attach(&ob);
	Write_Const = boost::thread(&U_OKATECH::writeMMR, this);
	//position初期化
	//	x					y					th
	position.x    =0.0; position.y    =0.0; position.th   =0.0;
	position.dx   =0.0; position.dy   =0.0; position.dth  =0.0;
	position.dx_v =0.0; position.dy_v =0.0; position.dw   =0.0;
	position.dtime=0.0;
	//wheel初期化
	wheel.v=0.0; wheel.rv = 0.0; wheel.cv = 0.0;
	wheel.write.vR = 0.0; wheel.write.vL = 0.0;

	time_limit = MMR_CONTROL_PERIOD;
	thread_end_flag = false;
}

//デストラクタ
U_OKATECH::~U_OKATECH()
{
	stop();
	// オブザーバー登録解除
	TinPow.detach(&ob);
	TinPow.close();
	thread_end_flag = true;
	if (Write_Const.joinable())Write_Const.join();
	puts("good bye okatech");
}

//MMR書き込み用
void U_OKATECH::writeMMR(void)
{
	boost::timer::auto_cpu_timer timer;
	timer.start();
	while (1)
	{
		//処理
		TinPow.u_send(WRITE_MESSAGE);
		if (thread_end_flag)return;
		timer.stop();
		if (((time_limit * 1000000) - timer.elapsed().wall) >= 0) {
			boost::this_thread::sleep_for(boost::chrono::nanoseconds((time_limit * 1000000) - timer.elapsed().wall));
		}
		timer.start();
	}
}

//書き込む
void U_OKATECH::u_write(const std::string& text){
	TinPow.send(text);
}

//wheel1:右車輪
bool U_OKATECH::wheel1r(double rps){
	if (fabs(rps) > OKATECH_LIMIT_SPEED_R){ puts("------ DANGER OKATECH_LIMIT_SPEED_R OVER ------"); return false; }
	char str[128]={0};
#ifdef __linux__
	sprintf(str, "\r\rRC1RS%5.4lf\r\r", rps);
#endif
#ifdef WIN32
	sprintf_s(str, "\r\rRC1RS%5.4lf\r\r", rps);
#endif
	u_write(str); return true;
}
//wheel2:左車輪
bool U_OKATECH::wheel2r(double rps){
	if (fabs(rps) > OKATECH_LIMIT_SPEED_R){ puts("------ DANGER OKATECH_LIMIT_SPEED_R OVER ------"); return false; }
	char str[128]={0};
#ifdef __linux__
	sprintf(str, "\r\rRC2RS%5.4lf\r\r", rps);
#endif
#ifdef WIN32
	sprintf_s(str, "\r\rRC2RS%5.4lf\r\r", rps);
#endif
	u_write(str); return true;
}

//wheel1:右車輪
bool U_OKATECH::wheel1v(double v){
	boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
	if (fabs(v) > OKATECH_LIMIT_SPEED_V){ puts("------ DANGER OKATECH_LIMIT_SPEED_V OVER ------"); return false; }
	wheel.write.vR = v;
	wheel1r(VELOCITY_TO_ANGULAR_VELOCITY(v));
	return true;
}
//wheel2:左車輪
bool U_OKATECH::wheel2v(double v){
	boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
	if (fabs(v) > OKATECH_LIMIT_SPEED_V){ puts("------ DANGER OKATECH_LIMIT_SPEED_V OVER ------"); return false; }
	wheel.write.vL = v;
	wheel2r(VELOCITY_TO_ANGULAR_VELOCITY(v));
	return true;
}

//現在位置の更新
void U_OKATECH::update_position(void){
#ifdef WRITE_ME // ---------------------------------------------------------
	static boost::timer::auto_cpu_timer time_interval;
	static bool first_process = true;
	double updl  = 0;
	if (first_process){
		time_interval.start();
		first_process = false;
	}else{
		time_interval.stop();
		wheel.read.v1 = wheel.read.d1 / OKATECH_PR * OKATECH_D * M_PI;
		wheel.read.v2 = wheel.read.d2 / OKATECH_PR * OKATECH_D * M_PI;
		position.dtime = NANOSEC_TO_SEC(time_interval.elapsed().wall);
		updl  = (wheel.read.v1 + wheel.read.v2) * 0.50;
		//回転角度の微小変化
		if(gyro_flag){
			g_drad = g_rad_now - g_rad_ago;
			position.dth = g_drad;//gyro
			g_rad_ago = g_rad_now;
			std::cout<<"g_rad_now1 = "<<g_rad_now<<std::endl;
		}else{
			position.dth = (wheel.read.v1 - wh eel.read.v2)/ OKATECH_TREAD;
		}
		
		position.dx   = updl * cos(position.th);
		position.dy   = updl * sin(position.th);

		//自己位置	 <-	距離の微小変化
		position.x   += position.dx;
		position.y   += position.dy;
		position.th  += position.dth;

		//x方向速度  y方向速度  回転速度の微小変化
		position.dx_v = position.dx / position.dtime;
		position.dy_v = position.dy / position.dtime;
		position.dw   = position.dth/ position.dtime;
		time_interval.start();

#ifdef MTH_RANGE_0_2PI
			while(position.th>M_PI*2.0)position.th -= 2 * M_PI;
			while (position.th < 0)position.th += 2 * M_PI;
#endif //MTH_RANGE_0_2PI
#ifdef MTH_RANGE_PI_PI
			while (position.th>M_PI)position.th -= 2 * M_PI;
			while (position.th<-M_PI)position.th += 2 * M_PI;
#endif //MTH_RANGE_PI_PI
		//座標等出力
		//std::cout << "(dl , dth , p) : " << "( " << updl << " , " << position.dth << " , " << updl / position.dth<< ") " << std::endl;
		//std::cout << "(v1 , v2 ) : " << "( " << wheel.read.v1  << " , " << wheel.read.v2  << ") " << std::endl;
		//std::cout << "(X , Y , TH) : " << "( " << position.x << " , " << position.y << " , " << position.th<< ") " << std::endl;
	}

#else //WRITE_ME ---------------------------------------------------------
static boost::timer::auto_cpu_timer time_interval;
	static bool first_process = true;
	double updl  = 0;
	if (first_process){
		time_interval.start();
		wheel.read.v1 = wheel.read.d1*(OKATECH_D / 2.0);
		wheel.read.v2 = wheel.read.d2*(OKATECH_D / 2.0);
		first_process = false;
	}
	else{
		time_interval.stop();
		wheel.read.v1 = wheel.read.d1*(OKATECH_D / 2.0);
		wheel.read.v2 = wheel.read.d2*(OKATECH_D / 2.0);
		position.dtime = NANOSEC_TO_SEC(time_interval.elapsed().wall);
		updl  = (wheel.read.v1 + wheel.read.v2) * 0.50 * position.dtime;
		//回転角度の微小変化
		if(gyro_flag){
			g_drad = g_rad_now - g_rad_ago;
			position.dth = g_drad;//gyro
			g_rad_ago = g_rad_now;
			std::cout<<"g_rad_now2 = "<<g_rad_now<<std::endl;
		}else{
			position.dth = (wheel.read.v1 - wheel.read.v2)/ OKATECH_TREAD * position.dtime;
		}
		
		if(position.dth == 0){
			position.dx   = updl * cos(position.th + position.dth*0.50);
			position.dy   = updl * sin(position.th + position.dth*0.50);
		}else{
			position.dx = 2.0 * (updl / position.dth)*sin(position.dth*0.5) * cos(position.th + position.dth*0.50);
			position.dy = 2.0 * (updl / position.dth)*sin(position.dth*0.5) * sin(position.th + position.dth*0.50);
		}
		//自己位置	 <-	距離の微小変化
		position.x   += position.dx;
		position.y   += position.dy;
		position.th  += position.dth;
		//x方向速度  y方向速度  回転速度の微小変化
		position.dx_v = position.dx / position.dtime;
		position.dy_v = position.dy / position.dtime;
		position.dw   = position.dth/ position.dtime;

		time_interval.start();

#ifdef MTH_RANGE_0_2PI
		while(position.th>M_PI*2.0)position.th -= 2 * M_PI;
		while (position.th < 0)position.th += 2 * M_PI;
#endif //MTH_RANGE_0_2PI
#ifdef MTH_RANGE_PI_PI
		while (position.th>M_PI)position.th -= 2 * M_PI;
		while (position.th<-M_PI)position.th += 2 * M_PI;
#endif //MTH_RANGE_PI_PI
		//座標等出力
		//std::cout << "(dl , dth , p) : " << "( " << updl << " , " << position.dth << " , " << updl / position.dth<< ") " << std::endl;
		//std::cout << "(v1 , v2 ) : " << "( " << wheel.read.v1  << " , " << wheel.read.v2  << ") " << std::endl;
		//std::cout << "(X , Y , TH) : " << "( " << position.x << " , " << position.y << " , " << position.th<< ") " << std::endl;
	}
#endif //WRITE_ME ---------------------------------------------------------
}




//直進
void U_OKATECH::runV(double V){
	wheel1v(V);
	wheel2v(V);
}

//右車輪
void U_OKATECH::runV_R(double V){
	wheel1v(V);
}

//左車輪
void U_OKATECH::runV_L(double V){
	wheel2v(V);
}

//回転
void U_OKATECH::runR(double R){
	wheel.cv = 0.0;
	if(R > OKATECH_TURN_LIMIT){R=OKATECH_TURN_LIMIT;}
	if(R < -OKATECH_TURN_LIMIT){R=-OKATECH_TURN_LIMIT;}
	wheel1v(R);
	wheel2v(-R);
}

//その場回転
void U_OKATECH::runR(void){
	wheel.v = 0.0;
	wheel.cv = 0.0;
	wheel1v(wheel.rv);
	wheel2v(-wheel.rv);
}

//曲率移動
void U_OKATECH::runV_cv(){
	wheel.rv = 0.0;
	wheel2v(wheel.v * (1.0 - (wheel.cv * (OKATECH_TREAD / 2.0))));	//左
	wheel1v(wheel.v * (1.0 + (wheel.cv * (OKATECH_TREAD / 2.0))));	//右
}

//停止
void U_OKATECH::stop(void){
	wheel.write.vL = 0.0;
	wheel.write.vR = 0.0;
	wheel.cv = 0.0;
	wheel.v=0.0;
	wheel.rv = 0.0;
	u_write("\rVCX0.0\r");
	u_write("\rVCR0.0\r");
}

//ゆっくり停止
void U_OKATECH::stop_slow(){
	int stop_count = (fabs(wheel.write.vL) > fabs(wheel.write.vR)) ? (int)(fabs(wheel.write.vL) * 100) : (int)(fabs(wheel.write.vR) * 100);
	if(stop_count < 1){stop();return;}
	double reduction_r = wheel.write.vR/stop_count;
	double reduction_l = wheel.write.vL/stop_count;
	for(int ust = 0; ust < stop_count ;ust++){
		runV_L ( wheel.write.vL - reduction_l );
		runV_R ( wheel.write.vR - reduction_r  );
		boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
	}
	stop();
}

#ifdef __linux__
//u_Joy_Control
void U_OKATECH::u_Joy_Control(const sensor_msgs::Joy::ConstPtr& joy){
	//---------- 関連 ---------------//
	// static sound_play::SoundClient sc;
	static int s_num = 0;
	static int p_num = 0;
	static double volume = 0.5;
	//---------- KUARO関連 ---------------//
	static int turn_flag = 0;			//初期:0	前進:-1		回転:1
	int num;							//ループ用
	bool key_remove = true;				//キーから手を離した時を検知するのに使用
	//キーパッドが押された時
	for (num = 0; num<joy->axes.size(); num++){
		if (joy->axes[num] != 0){
			if(num > 3 && turn_flag >= 0){stop_slow();turn_flag=-1;}
			switch (num){
			case 0:case 1:case 2:case 3:
				puts("アナログスティック使用禁止");
				return;
				break;
			case 4:
				//左
				if (joy->axes[num] > 0){
					//puts("左");
					if (wheel.cv < OKATECH_LIMIT_CV){
						wheel.cv += 0.5;
						runV_cv();
					}
				}else{
					//puts("右");
					if (wheel.cv > -OKATECH_LIMIT_CV){
						wheel.cv -= 0.5;
						runV_cv();
					}
				}
				break;
			case 5:
				//上
				if (joy->axes[num] > 0){
					//puts("上");
					if (wheel.v < OKATECH_LIMIT_SPEED_V - 0.05) {
						wheel.v += 0.05;
						runV_cv();
					}
				}else{
					//puts("下");
					if (wheel.v > -OKATECH_LIMIT_SPEED_V - 0.05) {
						wheel.v -= 0.05;
						runV_cv();
					}
				}
				break;
			default:
				break;
			}
			key_remove = false;
		}
	}
	//ボタンが押された時
	for (num = 0; num<joy->buttons.size(); num++)
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
			case 4:
				if(turn_flag <= 0){turn_flag=1;stop_slow();}
				if (wheel.rv < OKATECH_TURN_LIMIT-0.02) {
					wheel.rv += 0.02;
					runR();
				}
				break;
			//右回転
			case 5:
				if(turn_flag <= 0){turn_flag=1;stop_slow();}
				if (wheel.rv > -OKATECH_TURN_LIMIT+0.02) {
					wheel.rv -= 0.02;
					runR();
				}
				break;
			//Cv初期化
			case 0:
				wheel.cv = 0.0;
				runV_cv();
				break;
			//ポケモンBGM
			// case 1:
			// 	s_num--;
			// 	if(s_num < 1)s_num = SOUND_ALL;
			// 	sc.startWave(SOUND_PASS"/pokemon (" +boost::to_string(s_num)+").wav",volume);
			// 	break;
			// case 3:
			// 	s_num++;
			// 	if(s_num > SOUND_ALL)s_num = 1;
			// 	sc.startWave(SOUND_PASS"/pokemon (" +boost::to_string(s_num)+").wav",volume);
			// 	break;
			//
			// //ピカチュウ
			// case 10:
			// 	p_num--;
			// 	if(p_num < 1)p_num = 3;
			// 	sc.startWave(SOUND_PASS"/pikachu" +boost::to_string(p_num)+".wav",volume);
			// 	break;
			// case 11:
			// 	p_num++;
			// 	if(p_num > 3)p_num = 1;
			// 	sc.startWave(SOUND_PASS"/pikachu" +boost::to_string(p_num)+".wav",volume);
			// 	break;
			//
			// //音量+
			// case 7:
			// 	if(volume <= 0.90)volume+=0.10;
			// 	else volume=1.0;
			// 	std::cout<<"volume : "<<std::noshowpoint<<volume*100.0<<"[％]"<<std::endl;
			// 	break;
			//
			// //音量-
			// case 6:
			// 	if(volume >= 0.10)volume-=0.10;
			// 	if(volume < 0.05) volume =0.0;
			// 	std::cout<<"volume : "<<std::noshowpoint<<volume*100.0<<"[％]"<<std::endl;
			// 	break;

			//sound stop
			// case 12:
			// 	puts("sound stop");
			// 	sc.stopAll();
			// 	puts("--------------------------- joy HELP ---------------------------");
			// 	puts(" 移動		 1:曲率初期化		 2:ポケモン(-)");
			// 	puts("　↑		 3:空ボタン	 	 4:ポケモン(+)");
			// 	puts("←   →	 	 5:左回転		 6:右回転");
			// 	puts("　↓		 7:volume(-)	 	 8:volume(+)");
			// 	puts("　		 9:空ボタン		10:空ボタン");
			// 	puts("　		11:ピカチュウ(-)	12:ピカチュウ(+)");
			// 	puts("　		13:sound stop & HELP");
			// 	puts("----------------------------------------------------------------");
			// 	break;
			default:
				break;
			}
			key_remove = false;
		}
	}
	//キー&パッドから手が離れた	2で止めるようにしていればコメントアウト
	/*if(key_remove){
		stop();
	}*/
}

//cmd_vel
void U_OKATECH::cmdVelReceived(const geometry_msgs::Twist::ConstPtr& twist){

    double vel_x = twist->linear.x;
    double vel_th = twist->angular.z;
    double right_vel = 0.0;
    double left_vel = 0.0;

    // stop flag -> true & speed positive
    if (stop_flag && vel_x > 0.0)
        return;

    if(vel_x == 0){
		// turning
		right_vel = vel_th * OKATECH_TREAD / 2.0;
		left_vel = (-1) * right_vel;
    }else if(vel_th == 0.0){
		// forward / backward
		left_vel = right_vel = vel_x;
    }else{
		// moving doing arcs
		left_vel = vel_x - vel_th * OKATECH_TREAD / 2.0;
		right_vel = vel_x + vel_th * OKATECH_TREAD / 2.0;
    }
	runV_L ( left_vel );
	runV_R ( right_vel );
}

void U_OKATECH::emergency_func(const std_msgs::Bool::ConstPtr &state)
{
	// state pass
	old_stop_flag = stop_flag;
	stop_flag = state->data;

	// stop
	if (old_stop_flag != stop_flag && stop_flag)
	{
		ROS_WARN("Emergency Received!");
		stop();
	}
	else if (old_stop_flag != stop_flag && !stop_flag)
	{
		ROS_WARN("Emergency Clear!");
	}
}

void U_OKATECH::Wheel_Control_Outside(const std_msgs::StringConstPtr& msg){
	//std::cout<<"yes"<<std::endl;
	std::cout<<msg->data.c_str()<<std::endl;
	std::string recv_msg =  msg->data.c_str();
	//if(recv_msg.find(hit_word.c_str())== std::string::npos)return;
	static bool wco_start_flag = true;
	double kx,ky,kz;
	char set[256] = {0};
	std::cout<<recv_msg<<std::endl;
	sscanf(recv_msg.c_str(),"%[^,],%lf,%lf,%lf",set,&kx,&ky,&kz);
  kx = -kx;
	if((kx==0.0)&(kz==0.0)){
		stop();
		return;
	}
	double line,line_v,target_w,target_th,wco_v,wco_dv;
	static boost::timer::auto_cpu_timer wco_timer;
	static double line_ago,target_th_ago;

	line = sqrt(pow(kx,2) + pow(kz,2));
	if(line < 0.0001)return;
	target_th = atan2(kx,kz);

	if(wco_start_flag){
		wco_timer.start();
		target_w = 0;
		line_v = 0;
		wco_start_flag = false;
	}else{
		wco_timer.stop();
		target_w = (target_th - target_th_ago) / NANOSEC_TO_SEC(wco_timer.elapsed().wall);
		line_v = (line - line_ago) / NANOSEC_TO_SEC(wco_timer.elapsed().wall);
	}
	wco_v  =0.03;//line * KV;// + line_v * KLV;
	wco_dv = target_th * KT + target_w * KTD;

	//値の更新
	line_ago = line;
	target_th_ago = target_th;

	if(line > INVIOLABLE_AREA){
		runV_L( wco_v + wco_dv);
		runV_R( wco_v - wco_dv);
	}else{
		//runR(target_th / 0.995 * 0.50);
		stop();
	}
}

void U_OKATECH::Odometry_Save(const std_msgs::StringConstPtr& msg){
	std::string recv_msg =  msg->data.c_str();
	if(recv_msg.find("kinect")== std::string::npos)return;
	std::cout<<recv_msg <<std::endl;
	int img_num=0;
	double x;
	double y;
	char set[256] = {0};
	sscanf(recv_msg.c_str(),"%[^,],%d",set,&img_num);
	x = position.x;
	y = position.y;
	std::ofstream ofs;
	ofs.open("/home/robo/practice_ws/src/u_kuaro_move/odometry_save.txt",std::ios::out | std::ios::app);
		ofs <<img_num<<","<< x <<","<< y << std::endl;
	ofs.close();
}

//gyro
void U_OKATECH::gyroReceived(const sensor_msgs::Imu::ConstPtr& gyro){		
	gyro_flag = true;
	g_rad_now = gyro->orientation.z;
}


#endif  // __linux__
