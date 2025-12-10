#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

/**
 * @brief This class subscribes to the velocity data interpreted from the key pressed
 * & returns an array of 6 values, each representing the individual Thruster's movement. *
 */
class ThrusterPublisher : public rclcpp::Node
{
public:
    ThrusterPublisher() : Node("thurster_publisher")
    {
        // Subscribe to the tele commmand publisher & bind it to a callback function that publishes thruster values.
        m_velocitySubscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "/velocity",
            10,
            std::bind(&ThrusterPublisher::cmdCallback, this, std::placeholders::_1));

        // Publish thruster values
        m_thrusterPublisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/thrusters", 10);

        RCLCPP_INFO(this->get_logger(), "Thruster Node started");
    }

private:
    // Converts the Twist object into 6 thurster array.
    std::array<double, 6> mix(geometry_msgs::msg::Twist &cmd)
    {

        double surge = cmd.linear.x;  // forward/backward
        double sway = cmd.linear.y;   // left/right
        double heave = cmd.linear.z;  // up/down
        double roll = cmd.angular.x;  // roll
        double pitch = cmd.angular.y; // pitch
        double yaw = cmd.angular.z;   // yaw

        // We assume a 6 thuster setup where 4 are horizontal & 2 are vertical. Below array is T1...T6 (Thruster 1.... Thruster 6)
        std::array<double, 6> thrusters = {0, 0, 0, 0, 0, 0};

        // T1: front‑left Thruster
        // T2: front‑right Thruster
        // T3: rear‑left Thruster
        // T4: rear‑right Thruster

        // Surge, Sway and Yaw movements can be performed by the 4 horizontal thruster.
        // Todo: Need to figure out when to add, when to subsract
        thrusters[0] = surge + sway - yaw;
        thrusters[1] = surge - sway + yaw;
        thrusters[2] = surge - sway - yaw;
        thrusters[3] = surge + sway + yaw;

        // Heave & Pitch movements can be performed by 2 vertical thuster
        thrusters[4] = heave + pitch - roll;
        thrusters[5] = heave - pitch + roll;

        // Normalize to [-1.0, 1.0]
        double max_val = 0.0;
        for (auto &t : thrusters)
        {
            max_val = std::max(max_val, std::abs(t));
        }
        if (max_val > 1.0)
        {
            for (auto &t : thrusters)
            {
                t /= max_val;
            }
        }

        return thrusters;
    }

    void cmdCallback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::array<double, 6> thrusters = mix(*msg);

        // Publish thruster values
        auto thruster_msg = std_msgs::msg::Float64MultiArray();
        thruster_msg.data = {thrusters[0], thrusters[1], thrusters[2],
                             thrusters[3], thrusters[4], thrusters[5]};
        m_thrusterPublisher->publish(thruster_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Thrusters: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                    thrusters[0], thrusters[1], thrusters[2],
                    thrusters[3], thrusters[4], thrusters[5]);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_velocitySubscriber;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_thrusterPublisher;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterPublisher>());
    rclcpp::shutdown();
    return 0;
}