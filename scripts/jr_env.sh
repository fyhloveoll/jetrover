# JetRover 干净开发环境 (reuse vendor drivers, NOT vendor demos)
# 用法: source ~/jetrover_ws/jr_env.sh
# ---- 机型/传感器环境变量 (复刻自 vendor ~/ros2_ws/.typerc, 不依赖厂家 demo) ----
export need_compile=False
export LIDAR_TYPE=G4
export DEPTH_CAMERA_TYPE=Dabai
export MACHINE_TYPE=JetRover_Mecanum
export ROS_DOMAIN_ID=0

# ---- underlays: ROS + vendor 驱动/消息/第三方(只调用, 不修改) ----
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
source /home/ubuntu/third_party/third_party_ws/install/setup.bash
source /home/ubuntu/third_party/orbbec_ws/install/setup.bash

# ---- overlay: 我们自己的 workspace ----
if [ -f /home/ubuntu/jetrover_ws/install/setup.bash ]; then
  source /home/ubuntu/jetrover_ws/install/setup.bash
fi
