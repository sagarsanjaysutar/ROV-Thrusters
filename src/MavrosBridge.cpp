#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

using namespace std;

class MavrosBridge : public rclcpp::Node
{
public:
    MavrosBridge() : Node("mavros_bridge")
    {
        RCLCPP_INFO(this->get_logger(), "MavrosBridge Node starting...");

        // Publish thruster values to ArduSub endpoint
        m_thrusterPublisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

        // Subscribe to the thruster values generated from keypress
        m_thrusterSubscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/thrusters",
            10,
            std::bind(&MavrosBridge::cmdCallback, this, std::placeholders::_1));
    }

private:
    void cmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr thrusterValues)
    {
        if (!thrusterValues->data.empty())
        {
            auto msg = geometry_msgs::msg::TwistStamped();
            msg.header.stamp = this->now();
            msg.header.frame_id = "map";
            msg.twist.linear.x = thrusterValues->data[0];
            msg.twist.linear.y = thrusterValues->data[1];
            msg.twist.linear.z = thrusterValues->data[2];
            msg.twist.angular.x = thrusterValues->data[3];
            msg.twist.angular.y = thrusterValues->data[4];
            msg.twist.angular.z = thrusterValues->data[5];
            m_thrusterPublisher->publish(msg);
        }
    }

    // Send value to the ArduSub
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_thrusterPublisher;

    // Frequency timer
    rclcpp::TimerBase::SharedPtr m_frequencyTimer;

    // Gets value from the keypress
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_thrusterSubscriber;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MavrosBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
