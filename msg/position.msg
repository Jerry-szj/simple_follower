# 定义一个消息类型，用于表示检测到的目标或障碍物的位置信息

# float32 angleX
# 通常表示目标在水平方向上的角度（例如，相对于机器人前方的偏航角）。
# 单位通常是弧度 (radians)。
# 正值可能表示目标在右侧，负值在左侧，0表示在正前方，具体取决于坐标系定义。
float32 angleX

# float32 angleY
# 通常表示目标在垂直方向上的角度（例如，相对于机器人水平面的俯仰角）。
# 单位通常是弧度 (radians)。
# 这个字段在2D激光雷达跟随中可能不被使用或被赋予固定值，因为2D雷达通常只提供平面信息。
# 在某些脚本中，这个值被硬编码为42，可能表示无效或未使用。
float32 angleY

# float32 distance
# 表示机器人与目标之间的直线距离。
# 单位通常是米 (meters)。
float32 distance

