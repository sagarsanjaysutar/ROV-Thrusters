#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

using namespace std;

/**
 * @brief Intrepets key presses & moves the ROV.
 *
 * Run `ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14550@127.0.0.1:14555 -p target_system_id:=1 -p target_component_id:=1`
 * to establish link b/w ROS & ArduSub `sim_vehicle.py --console --map`
 *
 */
class ThrusterControl : public rclcpp::Node
{
public:
    ThrusterControl() : Node("mavros_bridge"), m_isArmed(false), m_isGuided(false)
    {
        RCLCPP_INFO(this->get_logger(), "ThrusterControl Node starting...");

        m_mavrosState = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state",
            10,
            std::bind(&ThrusterControl::readMavrosState, this, std::placeholders::_1));
        m_modeClient = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        m_armingClient = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

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
     * @brief Ensures that Sub is Guided & Armed mode. That's the only way it could be controlled.
     */
    void readMavrosState(const mavros_msgs::msg::State::SharedPtr state)
    {
        m_isArmed = state.get()->armed;
        m_isGuided = state.get()->guided;

        RCLCPP_INFO(this->get_logger(), "Mavros State Armed: %s", m_isArmed ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Mavros State Guided: %s", m_isGuided ? "true" : "false");

        if (!m_isArmed)
        {
            std::shared_ptr<mavros_msgs::srv::CommandBool::Request> armingRequest = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            armingRequest->value = true;
            m_armingClient->async_send_request(armingRequest);
        }

        if (!m_isGuided)
        {
            std::shared_ptr<mavros_msgs::srv::SetMode::Request> guidedRequest = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            guidedRequest->custom_mode = "GUIDED";
            m_modeClient->async_send_request(guidedRequest);
        }
    }

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

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr m_mavrosState;

    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr m_armingClient;
    bool m_isArmed;

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr m_modeClient;
    bool m_isGuided;

    // Send value to the ArduSub
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_thrusterPublisher;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThrusterControl>();

    // Run subscribers
    std::thread spin_thread([node]()
                            { rclcpp::spin(node); });

    // Read keypresses
    node->run();

    rclcpp::shutdown();
    return 0;
}
