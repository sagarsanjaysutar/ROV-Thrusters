#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

/**
 * @brief This class read keyboard inputs & publishes a Twist message.
 * Twist is represents linear & angular velocity; velocity which is needed to move ROV in various directions.
 */
class VelocityPublisher : public rclcpp::Node
{
public:
  VelocityPublisher() : Node("velocity_publisher")
  {
    // Publisher is defined with a topic name.
    m_velocityPublisher = this->create_publisher<geometry_msgs::msg::Twist>("/velocity", 10);

    RCLCPP_INFO(this->get_logger(), "Velocity Node started. Press WASD to move.");
  }

  /**
   * @brief Infinite loop that reads key presses & publishes
   */
  void run()
  {
    // Main loop that is reading the keyboard
    while (rclcpp::ok())
    {
      char key = getKey();
      auto msg = geometry_msgs::msg::Twist();

      switch (key)
      {
      case 'w':
      {
        // Surge forward
        msg.linear.x = 1;
        break;
      }
      case 's':
      {
        // Surge backward
        msg.linear.x = -1;
        break;
      }
      case 'a':
      {
        // Sway left
        msg.linear.y = 1;
        break;
      }
      case 'd':
      {
        // Sway right
        msg.linear.y = -1;
        break;
      }
      case 'q':
      {
        // Heave up
        msg.linear.z = 1;
        break;
      }
      case 'e':
      {
        // Heave down
        msg.linear.z = -1;
        break;
      }

      case 'j':
      {
        // Roll left
        msg.angular.x = 1;
        break;
      }
      case 'l':
      {
        // Roll right
        msg.angular.x = -1;
        break;
      }
      case 'i':
      {
        // Pitch up
        msg.angular.y = 1;
        break;
      }
      case 'k':
      {
        // Pitch down
        msg.angular.y = -1;
        break;
      }
      case 'u':
      {
        // Yaw left
        msg.angular.z = 1;
        break;
      }
      case 'o':
      {
        // Yaw right
        msg.angular.z = -1;
        break;
      }
      default:
        break;
      }

      m_velocityPublisher->publish(msg);
      RCLCPP_INFO(this->get_logger(),
                  "Cmd: linear[%.1f,%.1f,%.1f] angular[%.1f,%.1f,%.1f]",
                  msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z);
    }
  }

private:
  /**
   * @brief Return the pressed key on the keyboard.
   * @note This is boiler-plate code & it's directly picked up from internet
   */
  char getKey()
  {
    // Terminal input handling
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_velocityPublisher;
};

int main(int argc, char *argv[])
{
  // Initialises ROS 2
  rclcpp::init(argc, argv);

  // Starts data processing from the node.
  auto node = std::make_shared<VelocityPublisher>();
  node->run();

  rclcpp::shutdown();
  return 0;
}
