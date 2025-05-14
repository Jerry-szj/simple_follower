#!/usr/bin/env python # 指定脚本的解释器为python
#coding=utf-8 # 指定源文件的编码格式为UTF-8

import rospy # 导入ROS的Python客户端库
import _thread, threading # 导入线程模块，_thread为低级模块，threading为高级模块
import time # 导入时间模块，用于时间相关操作
import numpy as np # 导入NumPy库，用于高效的数值计算，并使用别名np
from sensor_msgs.msg import Joy, LaserScan # 从sensor_msgs.msg模块导入Joy（手柄输入）和LaserScan（激光雷达扫描数据）消息类型
from geometry_msgs.msg import Twist, Vector3 # 从geometry_msgs.msg模块导入Twist（速度指令）和Vector3（三维向量）消息类型
from simple_follower.msg import position as PositionMsg # 从当前功能包（simple_follower）的msg文件夹中导入自定义的position消息，并赋予其别名PositionMsg
from std_msgs.msg import String as StringMsg # 从std_msgs.msg模块导入String（字符串）消息类型，并赋予其别名StringMsg
from dynamic_reconfigure.server import Server # 从dynamic_reconfigure.server模块导入Server类，用于创建动态参数配置服务器
from simple_follower.cfg import laser_paramsConfig # 从当前功能包的cfg文件夹中导入由laser_params.cfg生成的配置类型
from std_msgs.msg import Int8 # 从std_msgs.msg模块导入Int8（8位整型）消息类型

angle=[0.0]*3 # 初始化一个名为angle的列表，包含三个浮点数0.0（此变量在后续代码中似乎未使用）
distan=[0.0]*3 # 初始化一个名为distan的列表，包含三个浮点数0.0（此变量在后续代码中似乎未使用）


class Follower: # 定义一个名为Follower的类，封装雷达跟随的主要逻辑
	def __init__(self): # Follower类的构造函数，在创建对象时被调用
	
	        
		# 当我们停止接收来自ps3控制器的Joy消息时，我们将停止所有移动: # 原有注释：说明下面计时器的用途
		self.controllerLossTimer = threading.Timer(1, self.controllerLoss) # 创建一个线程计时器，如果1秒内没有新的手柄消息（通过重置此计时器实现），则调用self.controllerLoss方法
		self.controllerLossTimer.start() # 启动上面创建的计时器线程
		self.switchMode= rospy.get_param("~switchMode") # 从ROS参数服务器获取名为'switchMode'的私有参数值 (决定按钮是切换模式还是按住模式)
		self.max_speed = rospy.get_param("~maxSpeed") # 从ROS参数服务器获取名为'maxSpeed'的私有参数值 (机器人的最大速度)
		self.controllButtonIndex = rospy.get_param("~controllButtonIndex") # 从ROS参数服务器获取名为'controllButtonIndex'的私有参数值 (用于控制激活/非激活状态的按钮索引)

		self.buttonCallbackBusy=False # 初始化一个标志位，用于防止按钮回调被短时间内重复处理，初始为False
		self.active=False # 初始化一个标志位，表示雷达跟随功能是否激活，初始为False（未激活）
		self.i=0 # 初始化一个计数器变量i为0，用于特定的发布逻辑 (publish_flag)

		# PID参数首先是角，距离 # 原有注释：说明PID参数的对应关系
		targetDist = rospy.get_param("~targetDist") # 从ROS参数服务器获取名为'targetDist'的私有参数值 (机器人与目标的期望保持距离)
		global PID_param # 声明PID_param是一个全局变量 (在类方法中使用global通常应谨慎)
		PID_param = rospy.get_param("~PID_controller") # 从ROS参数服务器获取名为'PID_controller'的私有参数 (包含P,I,D增益的字典)

		# Dynamic Reconfigure # 原有注释：说明下面的代码与动态参数配置相关
		self.dynamic_reconfigure_server = Server(laser_paramsConfig, self.reconfigCB) # 创建一个动态参数配置服务器实例，使用laser_paramsConfig指定的参数类型，回调函数为self.reconfigCB

		# 发布速度话题							话题名     消息类型    队列长度 # 原有注释：说明Publisher的参数
		self.cmdVelPublisher = rospy.Publisher("/cmd_vel", Twist, queue_size =3) # 创建一个ROS发布者，用于向'/cmd_vel'话题发布Twist类型的速度指令，消息队列大小为3

		# 来自ps3控制器(游戏手柄)的消息的话题 # 原有注释：说明被注释掉的订阅者用途
		#self.joySubscriber = rospy.Subscriber("joy", Joy, self.buttonCallback) # (此行被注释掉) 原计划订阅'joy'话题（手柄输入），回调函数为self.buttonCallback
		#self.followspeed = rospy.Subscriber("/object_tracker/current_position", PositionMsg,self.positionUpdateCallback) # (此行被注释掉) 原计划订阅目标位置，但与下面的订阅者功能重复

		# tracker话题，给予我们正在追踪的目标的当前位置信息 # 原有注释：说明订阅者用途

		# 订阅目标位置话题											话题名					消息类型				回调函数 # 原有注释：说明Subscriber的参数
		self.positionSubscriber = rospy.Subscriber("/object_tracker/current_position", PositionMsg, self.positionUpdateCallback) # 创建一个ROS订阅者，订阅'/object_tracker/current_position'话题（目标位置信息），回调函数为self.positionUpdateCallback

		# 来自tracker的字符串信息。例如，告诉我们是否丢失了对象 # 原有注释：说明订阅者用途
		self.trackerInfoSubscriber = rospy.Subscriber("/object_tracker/info", StringMsg, self.trackerInfoCallback) # 创建一个ROS订阅者，订阅'/object_tracker/info'话题（追踪器状态信息），回调函数为self.trackerInfoCallback
		
		# 第一个参数是角目标(总是0度)第二个参数是目标距离(比如1米)  # 原有注释：说明PID控制器的目标设定
		
		# 创建simplePID对象				[目标角度,目标距离]		P 				I 				D # 原有注释：说明simplePID的参数
		# self.PID_controller = simplePID([0, targetDist], PID_param["P"], PID_param["I"], PID_param["D"]) # (此行被注释掉) PID控制器实例的初始化实际上在reconfigCB回调函数中进行

		# 当按Ctrl+C终止运行时，调用回调函数controllerLoss使小车速度置零并发布消息 # 原有注释：说明rospy.on_shutdown的用途
		rospy.on_shutdown(self.controllerLoss) # 注册一个回调函数self.controllerLoss，在ROS节点关闭（如Ctrl+C）时被调用

	def publish_flag(self): # 定义一个名为publish_flag的方法
		
		laser_follow_flag=Int8() # 创建一个Int8类型的消息对象
		laser_follow_flag.data=1 # 将消息的data字段设置为1
		laserfwflagPublisher.publish(laser_follow_flag) # 通过全局变量laserfwflagPublisher发布这个消息 (该发布者在主程序块中定义)
		rospy.loginfo("a=%d",laser_follow_flag.data) # 使用ROS日志记录已发布的消息值
		print("11111111111111111111111111") # 在控制台打印一串数字（可能用于调试）

	# Dynamic Reconfigure Config # 原有注释：说明此方法为动态参数配置回调
	def reconfigCB(self,config,level): # 定义动态参数配置的回调函数，参数为配置对象和级别			
		self.max_speed = config.maxSpeed # 将类的成员变量self.max_speed更新为从配置中获取的maxSpeed值
		targetDist = config.targetDist # 将局部变量targetDist更新为从配置中获取的targetDist值
		# 获取PID值 # 原有注释：说明下面代码块的功能
		PID_param["P"] = [config.P_v,config.P_w] # 更新全局字典PID_param中'P'键的值，P_v和P_w分别对应距离和角度的比例增益
		PID_param["I"] = [config.I_v,config.I_w] # 更新全局字典PID_param中'I'键的值，I_v和I_w分别对应距离和角度的积分增益
		PID_param["D"] = [config.D_v,config.D_w] # 更新全局字典PID_param中'D'键的值，D_v和D_w分别对应距离和角度的微分增益
		# 创建simplePID对象				[目标角度,目标距离]		P 				I 				D # 原有注释：说明simplePID实例化参数
		self.PID_controller = simplePID([0, targetDist], PID_param["P"], PID_param["I"], PID_param["D"]) # 使用更新后的参数(目标角度为0，目标距离为targetDist，以及P,I,D增益)创建或重新创建simplePID控制器实例
		rospy.loginfo("max_speed:{},targetDist:{}".format(self.max_speed,targetDist)) # 使用ROS日志记录更新后的最大速度和目标距离
		return config # 返回配置对象，这是动态参数配置回调函数的标准做法
		
	
	def trackerInfoCallback(self, info): # 定义处理追踪器状态信息的回调函数，参数为接收到的StringMsg消息
		# we do not handle any info from the object tracker specifically at the moment. just ignore that we lost the object for example # 原有英文注释：当前不特别处理来自对象追踪器的信息，例如忽略丢失对象的情况
		# 我们不处理此时目标跟踪回传的任何信息，就当做此时没有跟踪的目标物 # 原有中文注释：解释当前对追踪器信息的处理方式
		rospy.logwarn(info.data) # 使用ROS警告日志记录接收到的追踪器信息内容
	
	def positionUpdateCallback(self, position): # 定义处理目标位置更新的回调函数，参数为接收到的PositionMsg消息

		# gets called whenever we receive a new position. It will then update the motorcomand # 原有英文注释：每当收到新位置时调用此方法，然后更新电机指令
		# 每当我们收到一个新位置时，就会被调用。然后它将更新motorcommand # 原有中文注释：解释此回调函数的触发时机和作用

		#if(not(self.active)): # (此行被注释掉) 原计划检查功能是否激活
			#return #if we are not active we will return imediatly without doing anything # (此行被注释掉) 如果未激活，则直接返回不执行后续操作

		angleX= position.angleX # 从接收到的position消息中获取angleX字段 (目标相对角度)
		distance = position.distance # 从接收到的position消息中获取distance字段 (目标相对距离)
		
		if(angleX>-1.57): # 如果目标角度angleX大于-1.57 (约-π/2)
			angleX=angleX-1.57 # 将angleX减去1.57 (约π/2)，此调整与laser_follow.py不同，适配不同雷达坐标系或角度定义
		else : # 否则 (angleX小于等于-1.57)
			angleX=angleX+1.57 # 将angleX加上1.57 (约π/2)，同样是为了调整角度表示
		

		# call the PID controller to update it and get new speeds # 原有英文注释：调用PID控制器更新并获取新速度
		# 调用PID控制器来更新它并获得新的速度 # 原有中文注释：解释下面代码的功能
		[uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angleX, distance]) # 调用PID控制器实例的update方法，传入调整后的角度和当前距离，获取未裁剪的角速度和线速度
			
		# clip these speeds to be less then the maximal speed specified above # 原有英文注释：将速度限制在指定的最大速度内
		# 将这些速度上面规定的最大速度以内 # 原有中文注释：解释下面代码的功能
		angularSpeed = np.clip(-uncliped_ang_speed, -self.max_speed, self.max_speed) # 使用NumPy的clip函数将计算出的角速度(取反后)限制在[-max_speed, max_speed]范围内，此处的取反与laser_follow.py不同
		linearSpeed  = np.clip(-uncliped_lin_speed, -self.max_speed, self.max_speed) # 使用NumPy的clip函数将计算出的线速度（取反后）限制在[-max_speed, max_speed]范围内	
		
		# create the Twist message to send to the cmd_vel topic # 原有英文注释：创建Twist消息并发送到cmd_vel话题
		# 创建Twist消息发送到cmd_vel话题 # 原有中文注释：解释下面代码的功能
		velocity = Twist() # 创建一个Twist类型的消息对象	
		velocity.linear = Vector3(linearSpeed,0,0.) # 设置Twist消息的线速度部分，linearSpeed用于x轴方向，y和z轴为0
		velocity.angular= Vector3(0., 0.,angularSpeed) # 设置Twist消息的角速度部分，angularSpeed用于z轴方向（偏航角速度），x和y轴为0
		self.cmdVelPublisher.publish(velocity) # 通过之前创建的发布者发布这个速度指令消息
		if self.i < 10: # 如果计数器self.i小于10
			self.i = self.i +1 # 计数器self.i自增1
		elif self.i == 10:		#语音识别标志 # 否则如果计数器self.i等于10 (原有注释：语音识别标志)
			self.publish_flag() # 调用self.publish_flag()方法
			self.i = 11 # 将计数器self.i设置为11，以防止再次进入此分支	

	# def buttonCallback(self, joy_data): # (此方法被注释掉) 原计划用于处理手柄按钮输入的回调函数
	# 	# this method gets called whenever we receive a message from the joy stick # (此行被注释掉) 原有英文注释：当收到手柄消息时调用此方法

	# 	# there is a timer that always gets reset if we have a new joy stick message # (此行被注释掉) 原有英文注释：有一个计时器，每次收到新手柄消息时会重置
	# 	# if it runs out we know that we have lost connection and the controllerLoss function will be called # (此行被注释掉) 原有英文注释：如果计时器超时，则表示连接丢失，将调用controllerLoss函数


	# 	self.controllerLossTimer.cancel() # (此行被注释掉) 取消当前的控制器丢失计时器
	# 	self.controllerLossTimer = threading.Timer(0.5, self.controllerLoss) # (此行被注释掉) 创建一个新的0.5秒计时器，超时调用self.controllerLoss
	# 	self.controllerLossTimer.start() # (此行被注释掉) 启动新的计时器

	# 	# if we are in switch mode, one button press will make the follower active / inactive # (此行被注释掉) 原有英文注释：如果处于切换模式，按一次按钮会切换激活/非激活状态
	# 	# but "one" button press will be visible in roughly 10 joy messages (since they get published to fast) # (此行被注释掉) 原有英文注释：但一次按键可能会在约10条手柄消息中体现（因为发布速度快）
	# 	# so we need to drop the remaining 9 # (此行被注释掉) 原有英文注释：所以需要丢弃其余9条

	# 	if self.buttonCallbackBusy: # (此行被注释掉) 如果按钮回调正在处理中
	# 		# we are busy with dealing with the last message # (此行被注释掉) 原有英文注释：我们正忙于处理上一条消息
	# 		return # (此行被注释掉) 直接返回，忽略当前消息
	# 	else: # (此行被注释掉) 否则，按钮回调空闲
	# 		# we are not busy. i.e. there is a real "new" button press # (此行被注释掉) 原有英文注释：我们不忙，即这是一个真实的“新”按键
	# 		# we deal with it in a seperate thread to be able to drop the other joy messages arriving in the mean time # (此行被注释掉) 原有英文注释：我们在一个单独的线程中处理它，以便丢弃期间到达的其他手柄消息
	# 		_thread.start_new_thread(self.threadedButtonCallback,  (joy_data, )) # (此行被注释掉) 使用_thread模块启动一个新线程来执行self.threadedButtonCallback方法

	# def threadedButtonCallback(self, joy_data): # (此方法被注释掉) 原计划用于在单独线程中处理按钮回调的函数
	# 	self.buttonCallbackBusy = True # (此行被注释掉) 设置按钮回调繁忙标志为True，表示开始处理

	# 	if(joy_data.buttons[self.controllButtonIndex]==self.switchMode and self.active): # (此行被注释掉) 如果指定的控制按钮被按下（根据switchMode的值判断按下状态）并且当前功能已激活
	# 		# we are active # (此行被注释掉) 原有英文注释：我们是激活状态
	# 		# switchMode = false: we will always be inactive whenever the button is not pressed (buttons[index]==false) # (此行被注释掉) 原有英文注释：switchMode为false时，按钮未按下则始终非激活
	# 		# switchMode = true: we will only become inactive if we press the button. (if we keep pressing it, # (此行被注释掉) 原有英文注释：switchMode为true时，按按钮才会变为非激活（如果一直按住，则会在0.5秒间隔内交替）
	# 		# we would alternate between active and not in 0.5 second intervalls) # (此行被注释掉) 原有英文注释：我们会在0.5秒的间隔内交替激活和非激活
	# 		rospy.loginfo("stoping") # (此行被注释掉) 使用ROS日志记录“stoping”（正在停止）
	# 		self.stopMoving() # (此行被注释掉) 调用self.stopMoving()方法使机器人停止运动
	# 		self.active = False # (此行被注释掉) 将激活状态self.active设置为False
	# 		rospy.sleep(0.5) # (此行被注释掉) ROS延时0.5秒，防止状态快速切换
	# 	elif(joy_data.buttons[self.controllButtonIndex]==True and not(self.active)): # (此行被注释掉) 否则如果指定的控制按钮被按下并且当前功能未激活
	# 		# if we are not active and just pressed the button (or are constantly pressing it) we become active # (此行被注释掉) 原有英文注释：如果我们未激活且刚按下按钮（或一直按住），则变为激活
	# 		rospy.loginfo("activating") # (此行被注释掉) 使用ROS日志记录“activating”（正在激活）
	# 		self.active = True #enable response # (此行被注释掉) 将激活状态self.active设置为True，并添加注释“enable response”
	# 		rospy.sleep(0.5) # (此行被注释掉) ROS延时0.5秒

	# 	self.buttonCallbackBusy = False # (此行被注释掉) 重置按钮回调繁忙标志为False，表示处理完毕

	# 将速度置零 # 原有中文注释：说明此方法的功能
	def stopMoving(self): # 定义一个名为stopMoving的方法
		velocity = Twist() # 创建一个Twist类型的消息对象
		velocity.linear = Vector3(0.,0.,0.) # 将线速度的x,y,z分量均设置为0
		velocity.angular= Vector3(0.,0.,0.) # 将角速度的x,y,z分量均设置为0
		self.cmdVelPublisher.publish(velocity) # 通过速度发布者发布这个零速度指令

	def controllerLoss(self): # 定义处理控制器（手柄）连接丢失的方法
		# we lost connection so we will stop moving and become inactive # 原有英文注释：我们丢失了连接，所以将停止移动并变为非激活状态
		# 连接丢失，停止移动，并变成不活跃状态 # 原有中文注释：解释此方法执行的动作
		self.stopMoving() # 调用self.stopMoving()方法使机器人停止运动
		self.active = False # 将激活状态self.active设置为False
		rospy.loginfo("lost connection") # 使用ROS日志记录“lost connection”（连接丢失）


		
class simplePID: # 定义一个名为simplePID的类，实现一个简单的离散PID控制器
	"""very simple discrete PID controller""" # 类的文档字符串 (英文): 一个非常简单的离散PID控制器
	"""非常简单的离散PID控制器""" # 类的文档字符串 (中文): 非常简单的离散PID控制器
	def __init__(self, target, P, I, D): # simplePID类的构造函数，参数为目标值和P,I,D增益
		"""Create a discrete PID controller # 方法的文档字符串 (英文): 创建一个离散PID控制器
		each of the parameters may be a vector if they have the same length # 方法的文档字符串 (英文): 如果长度相同，每个参数都可以是一个向量
		
		Args: # 方法的文档字符串 (英文): 参数说明
		target (double) -- the target value(s) # 方法的文档字符串 (英文): target (double) -- 目标值(们)
		P, I, D (double)-- the PID parameter # 方法的文档字符串 (英文): P, I, D (double) -- PID参数

		""" # 方法的文档字符串 (英文): 结束

		""" # 方法的文档字符串 (中文): 开始
		创建一个离散PID控制器 # 方法的文档字符串 (中文): 说明功能
		如果长度相同，每个参数都可以是一个向量 # 方法的文档字符串 (中文): 说明参数特性
		Args: # 方法的文档字符串 (中文): 参数说明
		target(double)——目标值 # 方法的文档字符串 (中文): target参数解释
		P, I, D (double)——PID参数 # 方法的文档字符串 (中文): P,I,D参数解释
		""" # 方法的文档字符串 (中文): 结束

		# check if parameter shapes are compatabile. # 原有英文注释：检查参数形状是否兼容
		# 检查参数形状是否兼容 # 原有中文注释：解释下面if语句的功能
		if(not(np.size(P)==np.size(I)==np.size(D)) or ((np.size(target)==1) and np.size(P)!=1) or (np.size(target )!=1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))): # 检查P,I,D以及target的维度是否满足向量运算的兼容性要求
			raise TypeError("input parameters shape is not compatable") # 如果参数形状不兼容，则抛出TypeError异常
		rospy.loginfo("PID initialised with P:{}, I:{}, D:{}".format(P,I,D)) # 使用ROS日志记录PID控制器初始化时使用的P,I,D值
		self.Kp		=np.array(P) # 将P增益转换为NumPy数组并存储为成员变量self.Kp (比例项系数)
		self.Ki		=np.array(I) # 将I增益转换为NumPy数组并存储为成员变量self.Ki (积分项系数)
		self.Kd		=np.array(D) # 将D增益转换为NumPy数组并存储为成员变量self.Kd (微分项系数)
		self.setPoint   =np.array(target) # 将目标值target转换为NumPy数组并存储为成员变量self.setPoint (设定点)
		
		self.last_error=0 # 初始化上一次的误差值为0
		self.integrator = 0 # 初始化积分累加器为0
		self.integrator_max = float("inf") # 初始化积分累加器的最大值为正无穷（即没有显式的积分饱和限制）
		self.timeOfLastCall = None # 初始化上次调用update方法的时间为None

	def update(self, current_value): # 定义PID控制器的update方法，参数为当前测量值
		"""Updates the PID controller. # 方法的文档字符串 (英文): 更新PID控制器

		Args: # 方法的文档字符串 (英文): 参数说明
			current_value (double): vector/number of same legth as the target given in the constructor # 方法的文档字符串 (英文): current_value (double): 与构造函数中给定的目标长度相同的向量/数值

		Returns: # 方法的文档字符串 (英文): 返回值说明
			controll signal (double): vector of same length as the target # 方法的文档字符串 (英文): controll signal (double): 与目标长度相同的控制信号向量

		""" # 方法的文档字符串 (英文): 结束

		""" # 方法的文档字符串 (中文): 开始
		更新PID控制器。 # 方法的文档字符串 (中文): 说明功能
		Args: # 方法的文档字符串 (中文): 参数说明
			Current_value (double)：与构造函数中给定的目标相同长度的向量/数 # 方法的文档字符串 (中文): current_value参数解释
		Returns: # 方法的文档字符串 (中文): 返回值说明
			controll signal (double))：与目标长度相同的矢量 # 方法的文档字符串 (中文): 控制信号返回值解释
		""" # 方法的文档字符串 (中文): 结束
		current_value=np.array(current_value) # [angleX distance] # 将输入的当前值current_value转换为NumPy数组 (注释表明输入通常是[角度, 距离])
		if(np.size(current_value) != np.size(self.setPoint)): # 检查当前值的维度是否与设定点的维度相同
			raise TypeError("current_value and target do not have the same shape") # 如果维度不同，则抛出TypeError异常
		if(self.timeOfLastCall is None): # 如果是第一次调用update方法 (self.timeOfLastCall为None)
			# the PID was called for the first time. we don"t know the deltaT yet # 原有英文注释：PID第一次被调用，还不知道deltaT
			# PID第一次被调用，我们还不知道时间差 # 原有中文注释：解释原因
			# no controll signal is applied # 原有英文注释：不应用控制信号
			# 没有控制信号被应用 # 原有中文注释：解释结果
			self.timeOfLastCall = time.perf_counter() # 记录当前时间作为上次调用时间
			return np.zeros(np.size(current_value)) # 返回一个与current_value维度相同的零数组作为控制信号

		
		error = self.setPoint - current_value # 计算误差：设定点 - 当前值

		# 偏差较小时停止移动 # 原有中文注释：解释下面代码块的功能 (误差死区)
		if error[0]<0.1 and error[0]>-0.1: # error[0]为角度偏差 # 如果误差向量的第一个元素（角度误差）的绝对值小于0.1
			error[0]=0 # 将角度误差置为0
		if error[1]<0.1 and error[1]>-0.1: # error[1]为距离偏差 # 如果误差向量的第二个元素（距离误差）的绝对值小于0.1
			error[1]=0 # 将距离误差置为0

        # when target is little, amplify velocity by amplify error # 原有英文注释：当目标较近时，通过放大误差来提高速度
        # 当目标很小时，通过放大误差来提高速度 # 原有中文注释：解释下面代码块的功能 (近距离误差放大)
		if error[1]>0 and self.setPoint[1]<1.3:	# 如果距离误差为正（当前距离大于目标距离）并且目标距离设定值小于1.3
			error[1]=error[1]*(1.3/self.setPoint[1]) # 将距离误差按比例放大
		P = error # 比例项的输入即为（可能经过处理的）当前误差
		
		currentTime = time.perf_counter() # 获取当前精确时间
		deltaT      = (currentTime-self.timeOfLastCall) # 计算自上次调用update以来的时间差 (dt)

		# integral of the error is current error * time since last update # 原有英文注释：误差的积分是当前误差乘以时间差
		# 误差的积分是 当前误差*时间差 # 原有中文注释：解释积分项的计算
		self.integrator = self.integrator + (error*deltaT) # 更新积分累加器：当前累加值 + (当前误差 * 时间差)
		I = self.integrator # 积分项的输入为更新后的积分累加器值
		
		# derivative is difference in error / time since last update # 原有英文注释：微分是误差差值除以时间差
		# D是误差的差值 / 时间差 # 原有中文注释：解释微分项的计算
		D = (error-self.last_error)/deltaT # 计算微分项：(当前误差 - 上次误差) / 时间差
		
		self.last_error = error # 更新上次误差为当前误差，为下次计算D项做准备
		self.timeOfLastCall = currentTime # 更新上次调用时间为当前时间
		
		# return controll signal # 原有英文注释：返回控制信号
		# 返回控制信号 # 原有中文注释：解释返回值的含义
		return self.Kp*P + self.Ki*I + self.Kd*D # 计算PID总输出：(Kp*比例项) + (Ki*积分项) + (Kd*微分项)，并返回
		
		
	

			




if __name__ == "__main__": # Python脚本的执行入口点，当脚本被直接运行时执行此块代码
	
	print("starting") # 在控制台打印'starting'，表示节点开始启动
	laserfwflagPublisher = rospy.Publisher("/laser_follow_flag", Int8, queue_size =1) # 创建一个ROS发布者，话题名为'/laser_follow_flag'，消息类型为Int8，队列大小为1 (此发布者在Follower类中使用)
	rospy.init_node("follower") # 初始化ROS节点，节点名称为'follower' (注意：如果此脚本作为follower_cx启动，节点名可能与laser_follow.py冲突，launch文件通常会指定不同的节点名如follower_cx)
	follower = Follower() # 创建Follower类的一个实例
	try: # 开始一个try块，用于捕获可能的异常
		rospy.spin() # 进入ROS的主循环，使节点保持运行状态并处理回调函数，此调用会阻塞直到节点关闭
	except rospy.ROSInterruptException: # 捕获ROS中断异常（例如用户按下Ctrl+C）
		print("exception") # 在控制台打印'exception'，表示捕获到异常



