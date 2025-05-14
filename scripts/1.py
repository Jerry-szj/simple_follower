#!/usr/bin/env python # 指定脚本的解释器为python
import rospy # 导入ROS的Python客户端库
from sensor_msgs.msg import LaserScan # 从sensor_msgs.msg模块导入LaserScan消息类型，用于处理激光雷达数据

def scan_callback(msg): # 定义一个回调函数，当接收到/scan话题的消息时被调用，参数msg为接收到的LaserScan消息
    # 创建一个新的LaserScan消息 # 原有注释：说明下面代码块的功能
    new_msg = LaserScan() # 创建一个新的LaserScan消息对象，用于存储过滤后的数据
    new_msg.header = msg.header # 将原始消息的header（包含时间戳、坐标系等信息）复制到新消息
    new_msg.angle_min = msg.angle_min # 复制最小扫描角度
    new_msg.angle_max = msg.angle_max # 复制最大扫描角度
    new_msg.angle_increment = msg.angle_increment # 复制角度增量（每两个扫描点之间的角度差）
    new_msg.time_increment = msg.time_increment # 复制时间增量（每两个扫描点之间的时间差）
    new_msg.scan_time = msg.scan_time # 复制完成一次完整扫描所需的时间
    new_msg.range_min = msg.range_min # 复制激光雷达的最小有效距离
    new_msg.range_max = msg.range_max # 复制激光雷达的最大有效距离

    # 过滤掉数据为0的点 # 原有注释：说明下面代码块的功能
    new_msg.ranges = [r for r in msg.ranges if r > 0] # 使用列表推导式过滤原始ranges数据，只保留大于0的距离值，并赋给新消息的ranges字段
    # 如果原始消息中包含intensities字段，并且其长度与ranges字段相同，才进行过滤
    if msg.intensities and len(msg.intensities) == len(msg.ranges):
        new_msg.intensities = [i for i, r in zip(msg.intensities, msg.ranges) if r > 0] # 使用列表推导式和zip函数同时遍历intensities和ranges，只保留对应range大于0的intensity值
    else:
        new_msg.intensities = [] # 如果原始消息没有intensities或长度不匹配，则新消息的intensities为空列表

    # 发布新的LaserScan消息 # 原有注释：说明下面代码的功能
    scan_pub.publish(new_msg) # 通过全局定义的scan_pub发布者发布过滤后的LaserScan消息

if __name__ == "__main__": # Python脚本的执行入口点，当脚本被直接运行时执行此块代码
    rospy.init_node("scan_filter") # 初始化ROS节点，节点名称为"scan_filter"
    scan_sub = rospy.Subscriber("/scan", LaserScan, scan_callback) # 创建一个ROS订阅者，订阅名为"/scan"的话题，消息类型为LaserScan，回调函数为scan_callback
    scan_pub = rospy.Publisher("/filtered_scan", LaserScan, queue_size=10) # 创建一个ROS发布者，发布到名为"/filtered_scan"的话题，消息类型为LaserScan，消息队列大小为10
    rospy.spin() # 进入ROS的主循环，使节点保持运行状态并处理回调函数，此调用会阻塞直到节点关闭

