#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/spawn.hpp>
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
    // 随机生成初始位置
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    double a_x = 1.0 + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX / (10.0 - 1.0)));
    double a_y = 1.0 + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX / (10.0 - 1.0)));
    double b_x = 1.0 + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX / (10.0 - 1.0)));
    double b_y = 1.0 + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX / (10.0 - 1.0)));

    // 生成乌龟A和乌龟B
    //spawnTurtle("turtle1", a_x, a_y);
    spawnTurtle("turtle2", b_x, b_y);

    // 隐藏轨迹
    hideTurtleTracks("turtle1");
    hideTurtleTracks("turtle2");

    // 订阅乌龟A和B的位置
    a_pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10, std::bind(&TurtleChaseNode::aPoseCallback, this, std::placeholders::_1));
    b_pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle2/pose", 10, std::bind(&TurtleChaseNode::bPoseCallback, this, std::placeholders::_1));

    // 发布乌龟A和B的速度命令
    a_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    b_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

    // 定时器控制乌龟B的随机运动和追逐逻辑
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&TurtleChaseNode::updateMovement, this));
  }

private:
  turtlesim::msg::Pose a_pose_, b_pose_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr a_pose_subscriber_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr b_pose_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr a_velocity_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr b_velocity_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 更新乌龟的位置和追逐逻辑
  void updateMovement()
  {
    moveTurtleA();
    moveTurtleB();

    // 如果A追到了B，重新生成B
    double distance = std::sqrt(std::pow(a_pose_.x - b_pose_.x, 2) + std::pow(a_pose_.y - b_pose_.y, 2));
    if (distance < 0.5)
    {
      teleportB();
    }
  }

  // 控制乌龟A追逐乌龟B
  void moveTurtleA()
  {
    auto msg = geometry_msgs::msg::Twist();
    double dx = b_pose_.x - a_pose_.x;
    double dy = b_pose_.y - a_pose_.y;
    
    // A乌龟朝着B乌龟移动
    double angle_to_target = std::atan2(dy, dx);
    double angle_diff = angle_to_target - a_pose_.theta;
    
    msg.linear.x = 2.0;  // A乌龟的速度
    msg.angular.z = angle_diff;  // 转向B乌龟

    a_velocity_publisher_->publish(msg);
  }

  // 控制乌龟B随机运动
  void moveTurtleB()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 1.0;  // B乌龟匀速运动
    msg.angular.z = 0.2 * ((std::rand() % 3) - 1);  // 随机角速度， [-0.2, 0, 0.2]
    
    // 防止乌龟撞墙
    if (b_pose_.x <= 1.0 || b_pose_.x >= 10.0 || b_pose_.y <= 1.0 || b_pose_.y >= 10.0)
    {
      msg.angular.z = 1.0;  // 如果靠近墙壁，转向
    }

    b_velocity_publisher_->publish(msg);
  }

  // 生成乌龟
  void spawnTurtle(const std::string &turtle_name, double x, double y)
  {
    auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->x = x;
    request->y = y;
    request->theta = 0.0;
    request->name = turtle_name;
    
    if (!client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "Service /spawn not available");
      return;
    }

    auto future_result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Turtle %s spawned at (%.2f, %.2f)", turtle_name.c_str(), x, y);
    }
  }

  // 隐藏乌龟的运动轨迹
  void hideTurtleTracks(const std::string &turtle_name)
  {
    auto client = this->create_client<turtlesim::srv::SetPen>(turtle_name + "/set_pen");
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = 0;
    request->g = 0;
    request->b = 0;
    request->width = 1;
    request->off = 1;  // 关闭轨迹
    
    if (!client->wait_for_service(std::chrono::seconds(10)))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for set_pen service to become available...");
      return;
    }

    client->async_send_request(request);
  }

  // 传送B乌龟到新随机位置
  void teleportB()
  {
    auto client = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle2/teleport_absolute");

    if (!client->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for teleport service to become available...");
      return;
    }

    double new_x = 1.0 + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX / (10.0 - 1.0)));
    double new_y = 1.0 + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX / (10.0 - 1.0)));

    auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    request->x = new_x;
    request->y = new_y;
    request->theta = 0.0;
    
    client->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Turtle B teleported to (%.2f, %.2f)", new_x, new_y);
  }

  // A 乌龟的位置更新回调
  void aPoseCallback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    a_pose_ = *msg;
  }

  // B 乌龟的位置更新回调
  void bPoseCallback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    b_pose_ = *msg;
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
