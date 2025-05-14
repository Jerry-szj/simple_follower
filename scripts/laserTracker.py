#!/usr/bin/env python # 指定脚本的解释器为python

import rospy # 导入ROS的Python客户端库
import _thread, threading # 导入线程模块，_thread为低级模块，threading为高级模块 (此脚本中似乎未使用)
import time # 导入时间模块 (此脚本中似乎未使用)
import numpy as np # 导入NumPy库，用于高效的数值计算，并使用别名np
from sensor_msgs.msg import Joy, LaserScan # 从sensor_msgs.msg模块导入Joy（手柄输入）和LaserScan（激光雷达扫描数据）消息类型 (Joy似乎未使用)
from geometry_msgs.msg import Twist, Vector3 # 从geometry_msgs.msg模块导入Twist（速度指令）和Vector3（三维向量）消息类型 (此脚本中似乎未使用)
from std_msgs.msg import String as StringMsg # 从std_msgs.msg模块导入String（字符串）消息类型，并赋予其别名StringMsg
from simple_follower.msg import position as PositionMsg # 从当前功能包（simple_follower）的msg文件夹中导入自定义的position消息，并赋予其别名PositionMsg
		
class laserTracker: # 定义一个名为laserTracker的类，用于追踪激光雷达扫描数据中的最近目标点
	def __init__(self): # laserTracker类的构造函数
		self.lastScan=None # 初始化lastScan成员变量为None，用于存储上一次的激光雷达扫描数据
		self.winSize = rospy.get_param("~winSize") # 从ROS参数服务器获取名为"winSize"的私有参数值 (用于在上一帧数据中查找相似距离的窗口大小)
		self.deltaDist = rospy.get_param("~deltaDist") # 从ROS参数服务器获取名为"deltaDist"的私有参数值 (判断两次扫描中距离是否足够接近的阈值)
		self.scanSubscriber = rospy.Subscriber("filtered_scan", LaserScan, self.registerScan) # 创建一个ROS订阅者，订阅"filtered_scan"话题（经过滤波的激光雷达数据），回调函数为self.registerScan
		self.positionPublisher = rospy.Publisher("object_tracker/current_position", PositionMsg,queue_size=3) # 创建一个ROS发布者，用于向"object_tracker/current_position"话题发布PositionMsg类型的目标位置信息，队列大小为3
		self.infoPublisher = rospy.Publisher("object_tracker/info", StringMsg, queue_size=3) # 创建一个ROS发布者，用于向"object_tracker/info"话题发布StringMsg类型的追踪器状态信息，队列大小为3

	def registerScan(self, scan_data): # 定义处理激光雷达扫描数据的回调函数，参数为接收到的LaserScan消息
		# registers laser scan and publishes position of closest object (or point rather) # 原有英文注释：注册激光扫描并发布最近对象（或点）的位置
		ranges = np.array(scan_data.ranges) # 将接收到的激光扫描数据中的ranges（距离数组）转换为NumPy数组
		# sort by distance to check from closer to further away points if they might be something real # 原有英文注释：按距离排序，从近到远检查点是否真实
		sortedIndices = np.argsort(ranges) # 对ranges数组进行排序，返回排序后的索引数组 (从小到大)
		
		minDistanceID = None # 初始化最小距离点的索引为None
		minDistance   = float("inf") # 初始化最小距离值为正无穷	
		#rospy.logwarn("88") # (此行被注释掉) 可能的调试日志	

		if(not(self.lastScan is None)): # 如果self.lastScan不为None (即已经接收过至少一次扫描数据)
			# if we already have a last scan to compare to: # 原有英文注释：如果我们已经有上一次扫描数据可以比较
			for i in sortedIndices: # 遍历排序后的索引 (即从最近的点开始检查)
				# check all distance measurements starting from the closest one # 原有英文注释：从最近的开始检查所有距离测量值
				tempMinDistance   = ranges[i] # 获取当前索引i对应的距离值
				
				# now we check if this might be noise: # 原有英文注释：现在我们检查这是否可能是噪声
				# get a window. in it we will check if there has been a scan with similar distance # 原有英文注释：获取一个窗口，在其中检查上一次扫描中是否有相似距离的扫描点
				# in the last scan within that window # 原有英文注释：在上一次扫描的那个窗口内
				
				# we kneed to clip the window so we don"t have an index out of bounds # 原有英文注释：我们需要裁剪窗口以避免索引越界 (kneed应为need)
				windowIndex = np.clip([i-self.winSize, i+self.winSize+1],0,len(self.lastScan)) # 计算用于在上一次扫描数据(self.lastScan)中取窗口的起始和结束索引，并使用np.clip确保索引不越界
				window = self.lastScan[windowIndex[0]:windowIndex[1]] # 从上一次扫描数据中提取出对应窗口的距离值

				with np.errstate(invalid="ignore"): # 使用NumPy的错误状态上下文管理器，忽略无效值（如NaN, inf）比较时产生的警告
					# check if any of the scans in the window (in the last scan) has a distance close enough to the current one # 原有英文注释：检查窗口内（上次扫描中）是否有任何扫描点的距离与当前点足够接近
					if(np.any(abs(window-tempMinDistance)<=self.deltaDist)): # 判断在上一次扫描的窗口内，是否存在至少一个点的距离与当前点(tempMinDistance)的差的绝对值小于等于self.deltaDist
					# this will also be false for all tempMinDistance = NaN or inf # 原有英文注释：对于所有tempMinDistance为NaN或inf的情况，此条件也将为false

						# we found a plausible distance # 原有英文注释：我们找到了一个合理的距离
						minDistanceID = i # 将当前点的索引i赋给minDistanceID
						minDistance = ranges[minDistanceID] # 将当前点的距离赋给minDistance
						break # at least one point was equally close # 原有英文注释：至少有一个点同样接近，所以找到了一个有效的最小值，可以停止循环
						# so we found a valid minimum and can stop the loop # 原有英文注释：所以我们找到了一个有效的最小值，可以停止循环
			
		self.lastScan=ranges # 将当前的激光扫描距离数据(ranges)保存为self.lastScan，供下一次回调使用	
		
		#catches no scan, no minimum found, minimum is actually inf # 原有英文注释：捕获没有扫描、未找到最小值、最小值实际上是inf的情况
		if(minDistance > scan_data.range_max): # 如果找到的最小距离大于激光雷达的最大有效距离 (或者仍为初始的inf)
			#means we did not really find a plausible object # 原有英文注释：意味着我们没有真正找到一个合理的物体
			
			# publish warning that we did not find anything # 原有英文注释：发布警告说我们没有找到任何东西
			rospy.logwarn("laser no object found") # 使用ROS警告日志记录“laser no object found”（激光雷达未找到目标）
			self.infoPublisher.publish(StringMsg("laser:nothing found")) # 通过infoPublisher发布字符串消息“laser:nothing found”
			
		else: # 否则 (找到了一个有效的最小距离点)
			# calculate angle of the objects location. 0 is straight ahead # 原有英文注释：计算物体位置的角度，0度表示正前方
			minDistanceAngle = scan_data.angle_min + minDistanceID * scan_data.angle_increment # 计算最小距离点对应的角度：起始角度 + 索引 * 角度增量
			# here we only have an x angle, so the y is set arbitrarily # 原有英文注释：这里我们只有一个x角度，所以y被任意设置 (PositionMsg的第二个参数)
			self.positionPublisher.publish(PositionMsg(minDistanceAngle, 42, minDistance)) # 通过positionPublisher发布PositionMsg消息，包含计算出的角度、一个固定的值42（可能代表y角度或无效值）、以及最小距离
			#self.infoPublisher.publish(StringMsg("successful")) # (此行被注释掉) 原计划可能在成功找到目标时发布“successful”消息



if __name__ == "__main__": # Python脚本的执行入口点
	print("starting") # 在控制台打印"starting"，表示节点开始启动
	rospy.init_node("laser_tracker") # 初始化ROS节点，节点名称为"laser_tracker"
	tracker = laserTracker() # 创建laserTracker类的一个实例
	print("seems to do something") # 在控制台打印"seems to do something" (似乎在做些什么)
	rospy.logwarn("1") # 使用ROS警告日志记录"1" (可能用于调试)
	try: # 开始一个try块
		rospy.spin() # 进入ROS的主循环，使节点保持运行状态并处理回调函数
	except rospy.ROSInterruptException: # 捕获ROS中断异常
		print("exception") # 在控制台打印"exception"，表示捕获到异常



