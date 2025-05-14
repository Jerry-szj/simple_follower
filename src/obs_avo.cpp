/**************************************************************************
功能： // 现有注释：功能说明 (具体功能未在此处详细说明，但从代码看是结合里程计和IMU的避障)
**************************************************************************/
#include <ros/ros.h> // 包含ROS核心库
#include <signal.h> // 包含信号处理相关的头文件 (在此文件中未直接使用)
#include <geometry_msgs/Twist.h> // 包含geometry_msgs/Twist消息类型，用于速度控制
#include <string.h> // 包含C风格字符串处理函数的头文件 (在此文件中未使用)
#include <math.h> // 包含数学函数库 (在此文件中未使用)
#include <iostream> // 包含标准输入输出流库
#include <std_msgs/String.h> // 包含标准消息类型String (在此文件中未使用)
#include <sensor_msgs/Imu.h> // 包含sensor_msgs/Imu消息类型，用于接收IMU数据（加速度、角速度等）
#include <nav_msgs/Odometry.h> // 包含nav_msgs/Odometry消息类型，用于接收里程计数据（位置、速度等）
#include <simple_follower/position.h> // 包含自定义消息类型simple_follower/position，用于表示目标的位置（距离和角度）


using namespace std; // 使用std命名空间

nav_msgs::Odometry odom; // 定义一个全局的Odometry类型的变量odom，用于存储接收到的里程计数据

sensor_msgs::Imu Imu_Data; // 定义一个全局的Imu类型的变量Imu_Data，用于存储接收到的IMU数据

float distance1;    //障碍物距离 // 定义一个全局浮点型变量distance1，用于存储检测到的最近障碍物的距离
float dis_angleX;    //障碍物方向 // 定义一个全局浮点型变量dis_angleX，用于存储检测到的最近障碍物的方向（角度）
int a; // 定义一个全局整型变量a，其用途在acc_judgment函数中赋值，但似乎没有在其他地方被有效使用
//float radar_range ; // (此行被注释掉) 原计划可能用于存储雷达范围的变量
//int radar_count ; // (此行被注释掉) 原计划可能用于雷达计数的变量

/**************************************************************************
函数功能：sub回调函数 // 现有注释：函数功能说明
入口参数：  laserTracker.py // 现有注释：数据来源说明 (应为消息来源的话题)
返回  值：无 // 现有注释：返回值说明
**************************************************************************/
void current_position_Callback(const simple_follower::position& msg)	// 定义名为current_position_Callback的回调函数，参数为接收到的simple_follower::position类型的消息的常量引用
{
	distance1 = msg.distance; // 将接收到的消息中的distance字段（障碍物距离）赋值给全局变量distance1
	dis_angleX = msg.angleX; // 将接收到的消息中的angleX字段（障碍物方向）赋值给全局变量dis_angleX

}


/**************************************************************************
函数功能：底盘运动sub回调函数（原始数据） // 现有注释：函数功能说明 (此回调函数名vel_Callback更像是订阅速度或里程计)
入口参数：cmd_msg  command_recognition.cpp // 现有注释：数据来源说明 (应为消息来源的话题，这里是/odom)
返回  值：无 // 现有注释：返回值说明
**************************************************************************/
void vel_Callback(const nav_msgs::Odometry& msg) // 定义名为vel_Callback的回调函数，参数为接收到的nav_msgs::Odometry类型的消息的常量引用
{
	odom.twist.twist.linear.x = msg.twist.twist.linear.x; // 将接收到的里程计消息中的x方向线速度赋值给全局变量odom的相应字段
	odom.twist.twist.linear.y = msg.twist.twist.linear.y; // 将接收到的里程计消息中的y方向线速度赋值给全局变量odom的相应字段
	
}


void acc_Callback(const sensor_msgs::Imu& msg) // 定义名为acc_Callback的回调函数，参数为接收到的sensor_msgs::Imu类型的消息的常量引用
{
	Imu_Data.linear_acceleration.x = msg.linear_acceleration.x; // 将接收到的IMU消息中的x方向线性加速度赋值给全局变量Imu_Data的相应字段
	Imu_Data.linear_acceleration.y = msg.linear_acceleration.y; // 将接收到的IMU消息中的y方向线性加速度赋值给全局变量Imu_Data的相应字段
	Imu_Data.angular_velocity.z = msg.angular_velocity.z; // 将接收到的IMU消息中的z方向角速度赋值给全局变量Imu_Data的相应字段
}



/**************************************************************************
函数功能：判断障碍物距离是否小于0.75米 // 现有注释：函数功能说明
入口参数：无 // 现有注释：入口参数说明
返回  值：1或0 // 现有注释：返回值说明
**************************************************************************/
int distance_judgment(void) // 定义名为distance_judgment的函数，用于判断障碍物距离是否小于阈值
{
	//int a; // (此行被注释掉) 未使用的变量声明
	if(distance1<=0.75) // 如果全局变量distance1（障碍物距离）小于等于0.75米
		return 1; // 返回1，表示距离过近
	else // 否则
		return 0; // 返回0，表示距离安全
	
}
 
/**************************************************************************
函数功能：判断障碍物方向是否在小车运动趋势方向上 // 现有注释：函数功能说明 (此注释可能不完全准确，下面包含vel_judgment和acc_judgment)
入口参数：无 // 现有注释：入口参数说明
返回  值：1或0 // 现有注释：返回值说明
**************************************************************************/


// 函数vel_judgment：根据当前速度和障碍物角度判断障碍物是否在运动路径上
int vel_judgment(void)
{
	// 2.335 弧度约等于 133.8 度。-2.335 弧度约等于 -133.8 度。
	// 0.785 弧度约等于 45 度。 -0.785 弧度约等于 -45 度。
	// 如果x方向线速度大于0（前进）并且 (障碍物角度大于133.8度 或 小于-133.8度) -- 即障碍物在机器人后方两侧的广阔区域
	if(odom.twist.twist.linear.x > 0 && (dis_angleX > 2.335 || dis_angleX < -2.33))
		return 1; // 返回1，表示障碍物在（前进时的）危险区域 (逻辑似乎是检测后方障碍物)

	// 如果x方向线速度小于0（后退）并且 (障碍物角度在-45度到45度之间) -- 即障碍物在机器人正前方
	else if(odom.twist.twist.linear.x < 0 && dis_angleX > -0.785 && dis_angleX < 0.785)
		return 1; // 返回1，表示障碍物在（后退时的）危险区域 (逻辑是检测前方障碍物)
		
	// 如果y方向线速度大于0（左移，假设标准机器人坐标系）并且 障碍物角度小于0 (障碍物在右半边或正后方)
	else if(odom.twist.twist.linear.y > 0 && dis_angleX < 0)
		return 1; // 返回1，表示障碍物在（左移时的）危险区域

	// 如果y方向线速度小于0（右移）并且 障碍物角度大于0 (障碍物在左半边或正前方)
	else if(odom.twist.twist.linear.y < 0 && dis_angleX > 0)
		return 1; // 返回1，表示障碍物在（右移时的）危险区域
		
	else // 其他情况
		return 0; // 返回0，表示不在危险区域
}

// 函数acc_judgment：根据当前加速度和障碍物角度判断障碍物是否在运动趋势（由加速度指示）的路径上
int acc_judgment(void)
{
	// 如果x方向线性加速度大于0.02（加速前进）并且 (障碍物角度大于133.8度 或 小于-133.8度) -- 即障碍物在机器人后方两侧
	if(Imu_Data.linear_acceleration.x > 0.02 && (dis_angleX > 2.335 || dis_angleX < -2.335))
	{
		a=1; // 设置全局变量a为1 (其后续用途不明确)
		return 1; // 返回1，表示障碍物在（加速前进时的）危险区域
	}

	// 如果x方向线性加速度小于-0.02（加速后退或减速前进）并且 (障碍物角度在-45度到45度之间) -- 即障碍物在机器人正前方
	else if(Imu_Data.linear_acceleration.x < -0.02 && dis_angleX > -0.785 && dis_angleX < 0.785)
	{
		a=1; // 设置全局变量a为1
		return 1; // 返回1，表示障碍物在（加速后退/减速前进时的）危险区域
	}
	//else if(Imu_Data.linear_acceleration.y > 0.01 && dis_angleX < 0) // (此段被注释掉) 原计划可能用于判断y方向加速时的危险区域
	//	return 1;

	//else if(Imu_Data.linear_acceleration.y < -0.01 && dis_angleX > 0) // (此段被注释掉) 原计划可能用于判断y方向加速时的危险区域
	//	return 1;

	else // 其他情况
		return 0; // 返回0，表示不在危险区域

}

// 函数dis_angleX_judgment：综合判断障碍物方向是否危险
int dis_angleX_judgment(void)
{
	if(vel_judgment())//||acc_judgment()) // 如果基于速度的判断为危险 (或者，原计划可能还想加入基于加速度的判断，但acc_judgment()被注释掉了)
		return 1; // 返回1，表示方向危险

	else // 否则
		return 0; // 返回0，表示方向安全					
}



/**************************************************************************
函数功能：主函数 // 现有注释：函数功能说明
入口参数：无 // 现有注释：入口参数说明
返回  值：无 // 现有注释：返回值说明 (main函数标准返回值为int)
**************************************************************************/
int main(int argc, char** argv) // C++程序主函数入口
{
	int turn_fin_flag=0;    //完成转向标志位 // 定义一个整型变量turn_fin_flag并初始化为0 (此变量在后续代码中未使用)
	int temp_count = 0;    //计数变量 // 定义一个整型变量temp_count并初始化为0，用于连续检测到障碍物的计数


	ros::init(argc, argv, "obs_avo");    //初始化ROS节点 // 初始化ROS，节点名称为"obs_avo"

	ros::NodeHandle node;    //创建句柄 // 创建一个ROS节点句柄

	/***创建底盘速度控制话题发布者***/ // 现有注释：说明发布者的功能
	ros::Publisher cmd_vel_Pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1); // 创建一个发布者，发布到"cmd_vel"话题，消息类型为geometry_msgs::Twist，队列大小为1

  	/***创建障碍物方位话题订阅者***/ // 现有注释：说明订阅者的功能
	ros::Subscriber current_position_sub = node.subscribe("/object_tracker/current_position", 1, current_position_Callback); // 订阅障碍物位置信息

	ros::Subscriber vel_sub = node.subscribe("/odom", 1, vel_Callback); // 订阅里程计信息，回调函数为vel_Callback

	ros::Subscriber acc_sub = node.subscribe("imu", 1, acc_Callback); // 订阅IMU信息，回调函数为acc_Callback


	
	double rate2 = 20;    //频率10Hz // 定义循环频率为20Hz (注释写的是10Hz，但代码是20)
	ros::Rate loopRate2(rate2); // 创建ros::Rate对象

	//node.param("/radar_range", radar_range); // (此行被注释掉) 原计划可能从参数服务器读取雷达范围
	//node.param("/radar_count", radar_count); // (此行被注释掉) 原计划可能从参数服务器读取雷达计数相关参数


	while(ros::ok()) // ROS主循环
	{		
		if(distance_judgment() && dis_angleX_judgment())    //判断障碍物的距离和方向 // 如果障碍物距离近 并且 方向判断为危险
		{
			
			temp_count++; // 危险情况计数器加1
			if(temp_count > 0)    //连续计数5️次停止运动防止碰撞，避免雷达有噪点 // 只要temp_count大于0 (即检测到一次危险情况)，就执行停止逻辑 (注释中的“连续计数5次”与代码逻辑不符，这里是>0)
			{
				int i=0; // 定义局部变量i并初始化为0
				if(i<50) // 这个if条件(i<50)在此处似乎没有意义，因为i总是0，所以这个块会执行一次。如果想多次发布停止命令，应该是一个循环。
				{
					cmd_vel_Pub.publish(geometry_msgs::Twist()); // 发布零速度指令，使机器人停止
					i++; // i自增 (但下次循环迭代时i会重新初始化为0，除非这个if在循环内)
					
				}
				temp_count = 0; // 重置危险情况计数器
				a=0; // 重置全局变量a为0 (其原始用途和此重置的意义不明确)
			}
		}
		else temp_count = 0;    //排除雷达噪点 // 如果当前未检测到危险情况 (距离远或方向安全)，则重置计数器
		
		ros::spinOnce(); // 处理ROS消息回调
		loopRate2.sleep(); // 按指定频率休眠
	} 



	return 0; // 主函数返回0
}


