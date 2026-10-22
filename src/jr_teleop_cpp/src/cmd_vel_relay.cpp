// rclcpp port of src/jr_teleop/jr_teleop/cmd_vel_relay.py
//
// Bridges /cmd_vel -> /controller/cmd_vel with a proportional speed cap:
// ONE scale factor for all three axes, so direction and curvature are preserved.
// Per-axis clamping distorts both (docs/engineering_decisions.md, D6).
// This is a safety net: with the planner's own limits set correctly it never triggers.
//
// Python original (for reference while porting):
//     lin = math.hypot(msg.linear.x, msg.linear.y)
//     s = 1.0
//     if lin > self.max_lin > 0:        s = min(s, self.max_lin / lin)      # C++: lin > max_lin_ && max_lin_ > 0
//     if abs(msg.angular.z) > self.max_ang > 0: s = min(s, self.max_ang / abs(msg.angular.z))
//     out.linear.x = msg.linear.x * s; out.linear.y = msg.linear.y * s; out.angular.z = msg.angular.z * s

#include <algorithm>   // std::min
#include <cmath>       // std::hypot, std::fabs
#include <memory>      // std::make_shared

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using Twist = geometry_msgs::msg::Twist;   // 类型别名：下面写 Twist 就等于写 geometry_msgs::msg::Twist

class CmdVelRelay : public rclcpp::Node {
public:
  CmdVelRelay() : Node("cmd_vel_relay") {
    // 1) 参数：和 Python 版的 JR_MAX_LIN / JR_MAX_ANG 对应，默认 0.15 m/s / 0.6 rad/s
    max_lin_ = declare_parameter<double>("max_lin", 0.15);
    max_ang_ = declare_parameter<double>("max_ang", 0.6);

    // 2) 发布者：话题 /controller/cmd_vel，队列深度 10
    pub_ = create_publisher<Twist>("/controller/cmd_vel", 10);

    // 3) 订阅者：话题 /cmd_vel，队列深度 10，回调是一个 lambda，捕获 this 以便调用 relay()
    // TODO(你写)：仿照上一行，用 create_subscription<Twist>(...) 创建 sub_，
    //   回调写成 [this](const Twist & in) { relay(in); }
    sub_ = ...;

    RCLCPP_INFO(get_logger(), "cmd_vel_relay up: max_lin=%.2f max_ang=%.2f", max_lin_, max_ang_);
  }

  // 析构：节点退出时发一条零速度，让车停下（Python 版 finally: publish(Twist())）
  ~CmdVelRelay() override {
    pub_->publish(Twist());
  }

private:
  // TODO(你写)：等比缩放。算出 s（三轴共用），构造 out，发布。
  //   提示：std::hypot(x, y) = sqrt(x²+y²)；std::fabs = 绝对值；std::min(a, b)
  //   Twist 的字段：in.linear.x  in.linear.y  in.angular.z（都是 double）
  void relay(const Twist & in) {
    Twist out;
    ...
    pub_->publish(out);
  }

  double max_lin_{0.15};
  double max_ang_{0.6};
  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::Subscription<Twist>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);                       // 启动 ROS2 客户端库（解析 --ros-args 等）
  rclcpp::spin(std::make_shared<CmdVelRelay>());  // 造节点，交给 executor 循环处理回调，直到 Ctrl+C
  rclcpp::shutdown();                             // 收尾；此时节点析构 → 发零速度
  return 0;
}
