#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>

using namespace std;

/**
 * @brief Intrepets key presses & moves the ROV.
 * 
 * Refer readme for the mapping.
 *
 */
class ThrusterControl : public rclcpp::Node
{
public:
    ThrusterControl() : Node("mavros_bridge")
    {
        RCLCPP_INFO(this->get_logger(), "ThrusterControl Node starting...");

        // Publish thruster values to ArduSub endpoint
        m_thrusterPublisher = this->create_publisher<geometry_msgs::msg::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
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
                msg.linear.y = 1;
                break;
            }
            case 's':
            {
                // Surge backward
                msg.linear.y = -1;

                break;
            }
            case 'a':
            {
                // Sway right
                msg.linear.x = -1;
                break;
            }
            case 'd':
            {
                // Sway left
                msg.linear.x = 1;
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
            case ' ':
            {
                msg.linear.x = 0;
                msg.linear.y = 0;
                msg.linear.z = 0;
                msg.angular.x = 0;
                msg.angular.y = 0;
                msg.angular.z = 0;
            }
            default:
                break;
            }

            m_thrusterPublisher->publish(msg);
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

    // Send value to the ArduSub
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_thrusterPublisher;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThrusterControl>();
    // rclcpp::spin(node);
    node->run();
    rclcpp::shutdown();
    return 0;
}
