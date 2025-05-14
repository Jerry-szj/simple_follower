cmake_minimum_required(VERSION 2.8.3) # 指定CMake的最低版本要求
project(simple_follower) # 定义项目名称为simple_follower

## 查找catkin宏和库
## 如果使用了 COMPONENTS 列表，如 find_package(catkin REQUIRED COMPONENTS xyz)
## 也会查找其他的catkin包
find_package(catkin REQUIRED COMPONENTS # 查找catkin包，并指定依赖的组件
  roscpp # ROS C++客户端库
  rospy # ROS Python客户端库
  std_msgs # ROS标准消息类型
  sensor_msgs # ROS传感器消息类型
  geometry_msgs # ROS几何消息类型
  message_generation # 用于生成自定义消息的ROS包
  dynamic_reconfigure # 用于动态参数配置的ROS包
)

## 生成动态参数配置文件
## 列出所有需要通过 dynamic_reconfigure 生成的 .cfg 文件
generate_dynamic_reconfigure_options(
  cfg/arPID.cfg # arPID控制器的参数配置文件
  cfg/laser_params.cfg # 激光跟随相关的参数配置文件
  cfg/Params_color.cfg # 颜色检测相关的HSV参数配置文件
  cfg/Params_PID.cfg # 另一组PID控制器参数配置文件
)

## 系统依赖项通过CMake的约定查找
# find_package(Boost REQUIRED COMPONENTS system) # 如果需要Boost库，取消此行注释


## 如果包中有setup.py文件，则取消此宏的注释。此宏确保其中声明的模块和全局脚本得到安装
## 参考: http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## 声明ROS消息、服务和动作 ##
################################################

## 要在此包内声明和构建消息、服务或动作，请遵循以下步骤:
## * 假设 MSG_DEP_SET 是你在消息/服务/动作中使用的消息类型的包集合 (例如 std_msgs, actionlib_msgs, ...).
## * 在 package.xml 文件中:
##   * 为 "message_generation" 添加一个 build_depend 标签
##   * 为 MSG_DEP_SET 中的每个包添加一个 build_depend 和一个 run_depend 标签
##   * 如果 MSG_DEP_SET 不为空，以下依赖项已被引入，但为了确定性仍可声明:
##     * 为 "message_runtime" 添加一个 run_depend 标签
## * 在此文件 (CMakeLists.txt) 中:
##   * 将 "message_generation" 和 MSG_DEP_SET 中的每个包添加到 find_package(catkin REQUIRED COMPONENTS ...)
##   * 将 "message_runtime" 和 MSG_DEP_SET 中的每个包添加到 catkin_package(CATKIN_DEPENDS ...)
##   * 根据需要取消以下 add_*_files 部分的注释，并列出要处理的每个 .msg/.srv/.action 文件
##   * 取消以下 generate_messages 条目的注释
##   * 将 MSG_DEP_SET 中的每个包添加到 generate_messages(DEPENDENCIES ...)

## 在 'msg' 文件夹中生成消息
add_message_files( # 添加消息文件定义
  FILES
  position.msg # 自定义的position消息
)

## 在 'srv' 文件夹中生成服务 (当前未使用)
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## 在 'action' 文件夹中生成动作 (当前未使用)
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## 生成已添加的消息和服务，以及此处列出的任何依赖项
generate_messages( # 生成消息文件，并指定其依赖的消息包
  DEPENDENCIES
  std_msgs # 依赖std_msgs
  sensor_msgs # 依赖sensor_msgs
  geometry_msgs # 依赖geometry_msgs
)

################################################
## 声明ROS动态参数配置 ##
################################################

## 要在此包内声明和构建动态参数配置，请遵循以下步骤:
## * 在 package.xml 文件中:
##   * 为 "dynamic_reconfigure" 添加一个 build_depend 和一个 run_depend 标签
## * 在此文件 (CMakeLists.txt) 中:
##   * 将 "dynamic_reconfigure" 添加到 find_package(catkin REQUIRED COMPONENTS ...)
##   * 取消以下 "generate_dynamic_reconfigure_options" 部分的注释，并列出要处理的每个 .cfg 文件

## 在 'cfg' 文件夹中生成动态参数配置 (已在文件顶部调用)
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin 特定配置 ##
###################################
## catkin_package 宏为你的包生成cmake配置文件
## 声明要传递给依赖项目的内容
## INCLUDE_DIRS: 如果你的包包含头文件，则取消此注释
## LIBRARIES: 你在此项目中创建的、依赖项目也需要的库
## CATKIN_DEPENDS: 依赖项目也需要的catkin包
## DEPENDS: 此项目的、依赖项目也需要的系统依赖项
catkin_package(
#  INCLUDE_DIRS include # 如果有include目录需要导出，取消注释
#  LIBRARIES simple_follower # 如果有库需要导出，取消注释
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime # 声明此包的运行时catkin依赖项
#  DEPENDS system_lib # 如果有系统库依赖需要导出，取消注释
)

###########
## 构建 ##
###########

## 指定头文件的附加位置
## 你的包位置应列在其他位置之前
# include_directories(include) # 如果有自定义的include目录
include_directories( # 包含catkin提供的标准包含目录
  ${catkin_INCLUDE_DIRS}
)

## 声明一个C++库 (当前未使用)
# add_library(simple_follower
#   src/${PROJECT_NAME}/simple_follower.cpp
# )

## 添加库的cmake目标依赖项 (当前未使用)
## 例如，代码可能需要在库之前从消息生成或动态参数配置生成
# add_dependencies(simple_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 声明一个C++可执行文件 (当前未使用，具体的可执行文件在下面单独声明)
# add_executable(simple_follower_node src/simple_follower_node.cpp)

## 添加可执行文件的cmake目标依赖项 (当前未使用)
## 与上面的库相同
# add_dependencies(simple_follower_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 指定要链接到库或可执行文件目标的库 (当前未使用)
# target_link_libraries(simple_follower_node
#   ${catkin_LIBRARIES}
# )

#############
## 安装 ##
#############

# 所有安装目标都应使用catkin DESTINATION变量
# 参考: http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## 标记可执行脚本 (Python等) 进行安装 (当前未使用，Python脚本通常通过setup.py安装或直接作为节点类型使用)
## 与setup.py不同，你可以选择目标位置
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## 标记可执行文件和/或库进行安装 (当前未使用)
# install(TARGETS simple_follower simple_follower_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## 标记C++头文件进行安装 (当前未使用)
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## 添加C++可执行文件 avoidance
add_executable(avoidance src/avoidance.cpp) # 定义名为avoidance的可执行文件，源文件为src/avoidance.cpp
add_dependencies(avoidance ${${PROJECT_NAME}_EXPORTED_TARGETS}) # 添加依赖项，确保消息等在编译此可执行文件前已生成 (这里使用 ${PROJECT_NAME}_EXPORTED_TARGETS 可能不完全准确，通常是 ${catkin_EXPORTED_TARGETS} 或特定消息目标)
target_link_libraries(avoidance ${catkin_LIBRARIES}) # 链接avoidance可执行文件到catkin提供的库

## 添加C++可执行文件 avoidance_cx
add_executable(avoidance_cx src/avoidance_cx.cpp) # 定义名为avoidance_cx的可执行文件，源文件为src/avoidance_cx.cpp
add_dependencies(avoidance_cx ${${PROJECT_NAME}_EXPORTED_TARGETS}) # 添加依赖项
target_link_libraries(avoidance_cx ${catkin_LIBRARIES}) # 链接库


## 添加C++可执行文件 obs_avo
add_executable(obs_avo src/obs_avo.cpp) # 定义名为obs_avo的可执行文件，源文件为src/obs_avo.cpp
add_dependencies(obs_avo ${${PROJECT_NAME}_EXPORTED_TARGETS}) # 添加依赖项
target_link_libraries(obs_avo ${catkin_LIBRARIES}) # 链接库


## 标记其他文件进行安装 (例如launch和bag文件等) (当前未使用)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## 测试 ##
#############

## 添加基于gtest的C++测试目标并链接库 (当前未使用)
# catkin_add_gtest(${PROJECT_NAME}-test test/test_simple_follower.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## 添加要由python nosetests运行的文件夹 (当前未使用)
# catkin_add_nosetests(test)

