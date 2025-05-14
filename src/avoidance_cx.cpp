/**************************************************************************
功能：雷达避障 // 现有注释：功能说明
**************************************************************************/
#include <ros/ros.h> // 包含ROS核心库，提供节点、话题、服务等ROS编程接口
#include <signal.h> // 包含信号处理相关的头文件 (在此文件中未直接使用)
#include <geometry_msgs/Twist.h> // 包含geometry_msgs/Twist消息类型，用于表示机器人的线速度和角速度
#include <string.h> // 包含C风格字符串处理函数的头文件 (在此文件中未使用，但iostream中的string类更常用)
#include <math.h> // 包含数学函数库 (在此文件中未使用)
#include <iostream> // 包含标准输入输出流库
#include <simple_follower/position.h> // 包含自定义消息类型simple_follower/position，用于表示目标的位置（距离和角度）
#include <std_msgs/Int32.h> // 包含标准消息类型Int32 (在此文件中未使用)
#include <std_msgs/Int8.h> // 包含标准消息类型Int8 (在此文件中未使用)
#include <std_msgs/String.h> // 包含标准消息类型String (在此文件中未使用，但定义了string类型的变量)




using namespace std; // 使用std命名空间，避免在标准库组件前加std::前缀
 


geometry_msgs::Twist cmd_vel_msg;    //速度控制信息数据 // 定义一个全局的Twist类型的变量cmd_vel_msg，用于存储从外部接收到的原始速度指令

float distance1;    //障碍物距离 // 定义一个全局浮点型变量distance1，用于存储检测到的最近障碍物的距离
float dis_angleX;    //障碍物方向 // 定义一个全局浮点型变量dis_angleX，用于存储检测到的最近障碍物的原始方向（角度）
float dis_angleX_change; // 定义一个全局浮点型变量dis_angleX_change，用于存储转换后的障碍物方向（角度）
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
函数功能：底盘运动sub回调函数（原始数据） // 现有注释：函数功能说明
入口参数：cmd_msg  command_recognition.cpp // 现有注释：数据来源说明 (应为消息来源的话题)
返回  值：无 // 现有注释：返回值说明
**************************************************************************/
void cmd_vel_ori_Callback(const geometry_msgs::Twist& msg) // 定义名为cmd_vel_ori_Callback的回调函数，参数为接收到的geometry_msgs::Twist类型的消息的常量引用
{
	cmd_vel_msg.linear.x = msg.linear.x; // 将接收到的消息中的linear.x（线速度x分量）赋值给全局变量cmd_vel_msg的相应字段
	cmd_vel_msg.angular.z = msg.angular.z; // 将接收到的消息中的angular.z（角速度z分量）赋值给全局变量cmd_vel_msg的相应字段
	
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
函数功能：判断障碍物方向是否在小车运动趋势方向上 // 现有注释：函数功能说明
入口参数：无 // 现有注释：入口参数说明
返回  值：1或0 // 现有注释：返回值说明
**************************************************************************/
int dis_angleX_judgment(void) // 定义名为dis_angleX_judgment的函数，用于判断障碍物是否在机器人前进或后退的路径上
{

	// 此处的角度转换逻辑与 avoidance.cpp 中的直接比较不同，它先对 dis_angleX 进行转换
	// dis_angleX 的原始范围可能是 [0, 2π) 或 [-π, π)，转换的目标是将其映射到一个新的范围，使得后续的判断逻辑更直接
	// 如果 dis_angleX >= -1.57 (即 dis_angleX 在 [-π/2, π] 或 [0, 3π/2] 范围内，取决于原始范围)
	//   dis_angleX_change = dis_angleX - 1.57 (将角度减去 π/2)
	// 否则 (dis_angleX < -1.57)
	//   dis_angleX_change = dis_angleX + 1.57 (将角度加上 π/2)
	// 这个转换的目的和效果依赖于 dis_angleX 的原始坐标系定义，目前看其用途不明确，且 dis_angleX_change 未在后续判断中使用。
	if(dis_angleX>=-1.57) // 如果原始障碍物角度大于等于 -π/2
		dis_angleX_change=dis_angleX-1.57; // dis_angleX_change 等于原始角度减去 π/2
	else // 否则 (原始障碍物角度小于 -π/2)
		dis_angleX_change=dis_angleX+1.57; // dis_angleX_change 等于原始角度加上 π/2
		
	// 下面的判断逻辑与 avoidance.cpp 中的 dis_angleX_judgment 存在显著差异，且可能存在逻辑问题。
	// avoidance.cpp 中的逻辑是：前进时，如果障碍物在后方，则返回1；后退时，如果障碍物在前方，则返回1。
	// 此处的逻辑是：
	// 1. 如果机器人期望前进 (cmd_vel_msg.linear.x > 0) 并且 (原始障碍物角度 dis_angleX < 1.57 (π/2) 或者 dis_angleX > -1.57 (-π/2))。
	//    这个条件 (dis_angleX < 1.57 || dis_angleX > -1.57) 实际上等价于 dis_angleX 在 (-π/2, π/2) 之外的整个范围，或者说，只要 dis_angleX 不是正好等于 ±π/2 且不满足特定组合，就可能为真。
	//    更准确地说，这个条件几乎总是为真，除非 dis_angleX 恰好是某个特殊值使得两个子条件都为假（这不可能）。
    //    实际上 (A < C || A > B) 当 B < C 时，如果 A > B 则为真，如果 A < C 则为真。如果 B = -1.57, C = 1.57, 那么这个条件覆盖了所有角度。
    //    因此，前进时，只要 dis_angleX 不是某些特定边界值，此条件可能就为真，意味着几乎总会判断为“障碍物在路径上”。
	if(cmd_vel_msg.linear.x > 0 && (dis_angleX < 1.57 || dis_angleX > -1.57)) // 如果机器人期望前进 并且 (障碍物角度小于π/2 或 大于-π/2) -- 这个角度条件几乎总是成立
		return 1; // 返回1，表示障碍物在运动趋势方向上

	// 2. 如果机器人期望后退 (cmd_vel_msg.linear.x < 0) 并且 (原始障碍物角度 dis_angleX < -1.57 (-π/2) 并且 dis_angleX > 1.57 (π/2))。
	//    这个条件 (dis_angleX < -1.57 && dis_angleX > 1.57) 是不可能同时成立的，一个数不能既小于-π/2又大于π/2。
	//    因此，这个else if分支永远不会执行。
	else if(cmd_vel_msg.linear.x < 0 && dis_angleX < -1.57 && dis_angleX > 1.57) // 如果机器人期望后退 并且 (障碍物角度小于-π/2 且 大于π/2) -- 这个条件永不成立
		return 1; // 返回1 (此代码块永不执行)
		
	else // 其他情况
		return 0; // 返回0，表示障碍物不在运动趋势方向上
		
}


/**************************************************************************
函数功能：主函数 // 现有注释：函数功能说明
入口参数：无 // 现有注释：入口参数说明
返回  值：无 // 现有注释：返回值说明 (main函数标准返回值为int)
**************************************************************************/
int main(int argc, char** argv) // C++程序主函数入口
{
	int temp_count = 0;    //计数变量 // 定义一个整型变量temp_count并初始化为0，用于连续检测到障碍物的计数
	string str1 = "遇到障碍物";    //障碍物字符串 // 定义一个字符串变量str1并赋值 (此变量在后续代码中未使用)

	ros::init(argc, argv, "avoidance_cx");    //初始化ROS节点 // 初始化ROS，节点名称为"avoidance_cx" (与avoidance.cpp中的节点名不同)

	ros::NodeHandle node;    //创建句柄 // 创建一个ROS节点句柄，用于与ROS系统交互

	/***创建底盘速度控制话题发布者***/ // 现有注释：说明发布者的功能
	ros::Publisher cmd_vel_Pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1); // 创建一个发布者，发布到"cmd_vel"话题，消息类型为geometry_msgs::Twist，队列大小为1

	/***创建底盘运动话题订阅者***/ // 现有注释：说明订阅者的功能
	ros::Subscriber vel_sub = node.subscribe("cmd_vel_ori", 1, cmd_vel_ori_Callback); // 创建一个订阅者，订阅"cmd_vel_ori"话题（原始速度指令），消息类型为geometry_msgs::Twist，回调函数为cmd_vel_ori_Callback，队列大小为1

  	/***创建障碍物方位话题订阅者***/ // 现有注释：说明订阅者的功能
	ros::Subscriber current_position_sub = node.subscribe("/object_tracker/current_position", 1, current_position_Callback); // 创建一个订阅者，订阅"/object_tracker/current_position"话题（障碍物位置信息），消息类型为simple_follower::position，回调函数为current_position_Callback，队列大小为1

	
	double rate2 = 30;    //频率30Hz // 定义一个双精度浮点型变量rate2并赋值为30.0，表示循环频率
	ros::Rate loopRate2(rate2); // 创建一个ros::Rate对象，用于控制循环以指定的频率运行 (30Hz)

 
	while(ros::ok()) // 当ROS系统正常运行时，进入主循环
	{
		ros::spinOnce(); // 处理一次ROS消息回调队列中的所有待处理消息
			
		if(distance_judgment() && dis_angleX_judgment())    //判断障碍物的距离和方向 // 如果distance_judgment()返回1 (距离近) 并且 dis_angleX_judgment()返回1 (在路径上，根据其内部逻辑)
		{
			temp_count++; // 连续检测到障碍物的计数器加1
			if(temp_count > 5)    //连续计数5️次停止运动防止碰撞，避免雷达有噪点 // 如果连续检测到障碍物的次数超过5次
			{

				cmd_vel_Pub.publish(geometry_msgs::Twist()); // 发布一个内容为空的Twist消息（即所有速度分量为0），使机器人停止运动
				temp_count = 0; // 重置计数器
			}
		}
		else // 否则 (障碍物距离远，或不在路径上，或不满足连续检测条件)
		{
			temp_count = 0;    //排除雷达噪点 // 重置计数器 (如果之前有计数但未达到阈值，则清零)
			cmd_vel_Pub.publish(cmd_vel_msg);    //将速度指令发送给机器人 // 发布从"cmd_vel_ori"话题接收到的原始速度指令cmd_vel_msg
		}

		ros::spinOnce(); // 再次处理一次ROS消息回调 (在一个循环内多次调用spinOnce通常不是必需的)
		loopRate2.sleep(); // 根据设定的频率休眠，以保持30Hz的循环速率
	} 


	return 0; // 主函数返回0，表示程序正常结束
}

