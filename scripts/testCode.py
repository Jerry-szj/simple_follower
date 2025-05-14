import numpy as np # 导入NumPy库，用于数值计算，别名为np
import copy # 导入copy模块，用于对象的浅拷贝和深拷贝
from unsafe_runaway import * # 从unsafe_runaway模块导入所有内容 (这个模块的具体内容未知，但测试代码依赖它，可能包含simpleTracker和simplePID的定义或其依赖)
import time # 导入time模块，用于时间相关操作，如暂停
from nose.tools import assert_raises # 从nose.tools模块导入assert_raises，用于测试异常抛出




class mockup: # 定义一个名为mockup的简单类，用于创建模拟对象
	pass # 类体为空，仅作为结构占位符
	

def makeLaserData(): # 定义一个函数，用于创建模拟的LaserScan数据对象
	laser_data = mockup() # 创建一个mockup类的实例，作为模拟的LaserScan数据
	laser_data.range_max = 5.6 # 设置模拟激光数据的最大有效距离
	laser_data.angle_min = -2.1 # 设置模拟激光数据的最小扫描角度 (弧度)
	laser_data.angle_increment = 0.06136 # 设置模拟激光数据的角度增量 (弧度)
	laser_data.ranges = list(1+np.random.rand(69)*5) # 生成一个包含69个随机距离值的列表 (范围在1到6之间)，作为模拟的扫描距离读数
	return laser_data # 返回创建的模拟激光数据对象

class Test_simpleTracker: # 定义一个测试类，用于测试simpleTracker类 (假设simpleTracker在此上下文中可访问，可能来自unsafe_runaway)

	
	def setUp(self): # 定义测试用例执行前的设置方法 (nose测试框架会自动调用)
		self.tracker = simpleTracker() # 创建一个simpleTracker类的实例，并将其赋给self.tracker，供测试方法使用
	
	def test_ignor_first_scan(self): # 定义一个测试方法，测试simpleTracker是否会忽略第一次扫描 (或在第一次扫描时抛出特定警告)
		laser_data = makeLaserData() # 创建模拟激光数据
		tracker = simpleTracker() # 创建一个新的simpleTracker实例 (注意：这里未使用self.tracker)
		assert_raises(UserWarning, self.tracker.registerScan,laser_data) # 断言当调用self.tracker.registerScan处理第一次扫描数据时，会抛出UserWarning异常

	def test_unmuted(self): # 定义一个测试方法，测试registerScan方法是否会修改输入的laser_data对象 (unmuted可能指不改变输入)
		laser_data = makeLaserData() # 创建模拟激光数据
		backup = copy.copy(laser_data) # 创建laser_data的浅拷贝，用于后续比较
		try: # 开始一个try块，捕获可能的异常
			angle, distance = self.tracker.registerScan(laser_data) # 调用registerScan方法，期望返回角度和距离 (注意：第一次调用可能因setUp中的tracker实例未处理过数据而行为不同)
		except: # 如果发生任何异常
			pass # 忽略异常，继续执行
		assert backup.ranges == laser_data.ranges # 断言原始laser_data的ranges列表与调用registerScan后的laser_data.ranges列表相同，即输入未被修改
		#print(laser_data.ranges) # (此行被注释掉) 打印激光数据范围
		#print("angle: {}, dist: {}".format(angle, distance))	# (此行被注释掉) 打印计算出的角度和距离

	def test_nan(self): # 定义一个测试方法，测试simpleTracker如何处理包含NaN（非数字）值的激光数据
		laser_data = makeLaserData() # 创建模拟激光数据
		assert_raises(UserWarning, self.tracker.registerScan,laser_data) # 再次测试第一次扫描，期望抛出UserWarning (tracker实例状态可能影响行为)
		laser_data.ranges[12] = float("nan") # 将激光数据中的一个特定距离值设置为NaN
		angle, dist=self.tracker.registerScan(laser_data) # 调用registerScan处理包含NaN的数据 (此时tracker已处理过一次数据)
		#print("angle: {}, dist: {}".format(angle, dist))	# (此行被注释掉) 打印结果角度和距离

	def test_only_nan(self): # 定义一个测试方法，测试simpleTracker如何处理所有距离值都是NaN的激光数据
		laser_data = makeLaserData() # 创建模拟激光数据
		laser_data.ranges = [float("nan") for _ in laser_data.ranges] # 将所有激光距离值都设置为NaN
		assert_raises(UserWarning, self.tracker.registerScan,laser_data) # 第一次调用，期望抛出UserWarning
		assert_raises(UserWarning, self.tracker.registerScan,laser_data) # 第二次调用，仍然期望抛出UserWarning (因为所有数据都是NaN，可能无法找到有效目标)

		

	def test_real_real_min(self): # 定义一个测试方法，测试simpleTracker在存在多个最小值（一个真实，一个虚假/噪声）时能否找到真实最小值
		laser_data = makeLaserData() # 创建模拟激光数据
		laser_data.ranges[-1]=0.5 #real min # 将最后一个距离值设置为0.5 (标记为真实最小值)
		assert_raises(UserWarning, self.tracker.registerScan,laser_data) # 第一次调用，期望UserWarning
		laser_data.ranges[-1]=0.6 # 将最后一个距离值修改为0.6 (现在这个是期望被识别的真实最小值)
		laser_data.ranges[42]=0.1 #fake min # 将索引为42的距离值设置为0.1 (标记为虚假最小值，可能因为不稳定而被忽略)
		ang, dist = self.tracker.registerScan(laser_data) # 调用registerScan (此时tracker已处理过数据)
		assert dist == 0.6 # 断言找到的距离是0.6 (即忽略了更小的0.1，找到了0.6)
		#print("ang: {}, target: {}".format(ang, (laser_data.angle_min+ 23*laser_data.angle_increment)))	# (此行被注释掉) 打印角度和目标角度
		assert ang == laser_data.angle_min+ 68*laser_data.angle_increment # 断言找到的角度与索引68 (即ranges列表的最后一个元素) 对应的角度一致


class Test_PID: # 定义一个测试类，用于测试simplePID类 (假设simplePID在此上下文中可访问)
	def setUp(self): # 定义测试用例执行前的设置方法
		pass # 此处没有特定的设置操作

	def test_convergence(self): # 定义一个测试方法，测试PID控制器是否能收敛到目标值
		self.pid = simplePID([0,30], 0.8, 0.001, 0.0001) # 创建simplePID实例，目标值为[0, 30]，P,I,D增益分别为0.8, 0.001, 0.0001
		x =np.array([23, 12]) # 初始化当前值为[23, 12]
		for i in range(20): # 进行20次迭代更新
			update= self.pid.update(x) # 调用PID的update方法获取控制量
			print("added {} to current x {}".format(update, x)) # 打印控制量和当前值 (用于调试观察)
			x = x+update # 更新当前值：当前值 + 控制量
			time.sleep(0.1) # 暂停0.1秒，模拟离散时间系统的时间步进
		assert np.all(abs(x-[0,30])<=0.01) # 断言20次迭代后，当前值x与目标值[0,30]的差的绝对值都小于等于0.01，即已收敛

	def test_convergence_differentParamShape(self): # 定义一个测试方法，测试PID控制器在不同参数形状下的收敛性 (此测试与上一个几乎相同，可能用于验证构造函数对标量PID参数的处理)
		self.pid = simplePID([0,30],0.8, 0.001, 0.0001) # 创建simplePID实例，目标值为[0,30]，P,I,D增益为标量
		x =np.array([23, 12]) # 初始化当前值为[23, 12]
		for i in range(20): # 进行20次迭代更新
			update= self.pid.update(x) # 调用PID的update方法
			print("added {} to current x {}".format(update, x)) # 打印控制量和当前值
			x = x+update # 更新当前值
			time.sleep(0.1) # 暂停0.1秒
		assert np.all(abs(x-[0,30])<=0.01) # 断言收敛到目标值

	 
	def test_raises_unequal_param_shape_at_creation(self): # 定义一个测试方法，测试在创建simplePID实例时，如果P,I,D和target的形状不兼容是否会抛出TypeError
		assert_raises(TypeError, simplePID, [0,30],[0.8, 0.7, 0.1], 0.001, 0.0001) # P是3元素向量，target是2元素向量，应抛出TypeError
		assert_raises(TypeError, simplePID, [0,30],[0.8, 0.7], 0.001, 0.0001) # P是2元素向量，I,D是标量，target是2元素向量，这可能是允许的，取决于simplePID的实现 (如果允许标量广播，则不应抛异常)
		assert_raises(TypeError, simplePID, 0,[0.8, 0.7], 0.001, 0.0001) # target是标量，P是2元素向量，应抛出TypeError
		assert_raises(TypeError, simplePID, 0, [0.8, 0.7], [0.001, 0.001], [0.0001, 0,0001]) # target是标量，P,I,D是向量，应抛出TypeError (D的最后一个元素是0,0001，可能是笔误)
		_ =  simplePID([0,30],[0.8, 0.7], [0.001, 0.001], [0.0001, 0.0001]) # target, P, I, D 都是2元素向量，应成功创建
		_ =  simplePID([0,30],0.8, 0.001, 0.0001) # target是2元素向量，P,I,D是标量，应成功创建 (标量广播)
		_ =  simplePID(0,0.8, 0.001, 0.0001) # target, P, I, D 都是标量，应成功创建

	def test_raise_incompatable_input(self): # 定义一个测试方法，测试在调用PID的update方法时，如果输入current_value的形状与target不兼容是否会抛出TypeError
		self.pid = simplePID([0,30], 0.8, 0.001, 0.0001) # 创建PID实例，target是2元素向量
		_ = assert_raises(TypeError, self.pid.update, 3) # update的输入是标量3，与target形状不符，应抛出TypeError
		x =np.array([23, 12]) # 初始化当前值为[23, 12]
		for i in range(50): # 进行50次迭代更新 (比之前的测试迭代次数更多)
			update= self.pid.update(x) # 调用PID的update方法
			print("added {} to current x {}".format(update, x)) # 打印控制量和当前值
			x = x+update # 更新当前值
			time.sleep(0.1) # 暂停0.1秒
		assert np.all(abs(x-[0,30])<=0.001) # 断言50次迭代后，收敛到目标值，且精度更高 (误差小于等于0.001)

