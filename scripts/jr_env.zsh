# JetRover 干净开发环境 (zsh 版,交互式 SSH 用这个;命令脚本用 jr_env.sh)
# 用法: source ~/jetrover_ws/jr_env.zsh
export need_compile=False
export LIDAR_TYPE=G4
export DEPTH_CAMERA_TYPE=Dabai
export MACHINE_TYPE=JetRover_Mecanum
export ROS_DOMAIN_ID=0

source /opt/ros/humble/setup.zsh
source /home/ubuntu/ros2_ws/install/local_setup.zsh
source /home/ubuntu/third_party/third_party_ws/install/local_setup.zsh
source /home/ubuntu/third_party/orbbec_ws/install/local_setup.zsh
[[ -f /home/ubuntu/jetrover_ws/install/local_setup.zsh ]] && source /home/ubuntu/jetrover_ws/install/local_setup.zsh
