#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <cmath>
#include <cstdlib>
#include <ctime>

class TurtleChaseNode : public rclcpp::Node
{
public:
  TurtleChaseNode() : Node("turtle_chase_node")
  {
    // 订阅两只乌龟的位置信息
    a_pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&TurtleChaseNode::aPoseCallback, this, std::placeholders::_1));
    b_pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle2/pose", 10, std::bind(&TurtleChaseNode::bPoseCallback, this, std::placeholders::_1));

    // 发布两只乌龟的速度控制命令
    a_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    b_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

    // 定时器控制B乌龟的随机运动
    b_move_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&TurtleChaseNode::moveBRandomly, this));

    // 初始化随机数种子
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    // 隐藏两只乌龟的轨迹
    hideTurtleTracks("/turtle1");
    hideTurtleTracks("/turtle2");
  }

private:
  turtlesim::msg::Pose a_pose_, b_pose_;

  // 订阅者和发布者
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr a_pose_subscriber_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr b_pose_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr a_velocity_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr b_velocity_publisher_;
  rclcpp::TimerBase::SharedPtr b_move_timer_;

  // A 乌龟的位置更新回调
  void aPoseCallback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    a_pose_ = *msg;
    chaseB();  // 每次位置更新后进行追逐
  }

  // B 乌龟的位置更新回调
  void bPoseCallback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    b_pose_ = *msg;
  }

  // A 乌龟追逐 B 乌龟
  void chaseB()
  {
    auto msg = geometry_msgs::msg::Twist();
    double dx = b_pose_.x - a_pose_.x;
    double dy = b_pose_.y - a_pose_.y;

    // 简单追逐逻辑：A 乌龟朝着 B 乌龟的方向移动
    msg.linear.x = std::sqrt(dx * dx + dy * dy) * 0.5;  // 调整速度
    msg.angular.z = std::atan2(dy, dx) - a_pose_.theta;
    a_velocity_publisher_->publish(msg);

    // 判断是否追逐到 B 乌龟
    if (std::sqrt(dx * dx + dy * dy) < 0.5)
    {
      teleportB();  // 追逐到后，传送 B 乌龟
    }
  }

  // 传送 B 乌龟到新位置
  void teleportB()
  {
    double new_x = 1.0 + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX / (10.0 - 1.0)));
    double new_y = 1.0 + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX / (10.0 - 1.0)));

    auto client = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle2/teleport_absolute");

    if (!client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for teleport service to become available...");
      return;
    }

    auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    request->x = new_x;
    request->y = new_y;
    request->theta = 0.0;
    client->async_send_request(request);
  }

  // 随机移动 B 乌龟
  void moveBRandomly()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 1.0 * ((std::rand() % 3) - 1); // 随机线速度 [-1, 0, 1]
    msg.angular.z = 1.0 * ((std::rand() % 3) - 1); // 随机角速度 [-1, 0, 1]
    
    // 墙壁规避逻辑
    if (b_pose_.x <= 1.0 || b_pose_.x >= 10.0)
    {
      msg.linear.x = -msg.linear.x;
    }
    if (b_pose_.y <= 1.0 || b_pose_.y >= 10.0)
    {
      msg.linear.x = -msg.linear.x;
    }

    b_velocity_publisher_->publish(msg);
  }

  // 隐藏乌龟的轨迹
  void hideTurtleTracks(const std::string & turtle_name)
  {
    auto client = this->create_client<turtlesim::srv::SetPen>(turtle_name + "/set_pen");

    if (!client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for set_pen service to become available...");
      return;
    }

    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = 0;
    request->g = 0;
    request->b = 0;
    request->width = 1;
    request->off = 1; // 关闭笔迹
    client->async_send_request(request);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleChaseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
