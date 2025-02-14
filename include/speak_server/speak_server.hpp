#ifndef SPEAK_SERVER__SPEAK_SERVER_HPP_
#define SPEAK_SERVER__SPEAK_SERVER_HPP_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/empty.hpp>
#include <waypoint_function_msgs/srv/command.hpp>
#include <voicevox_ros2_msgs/msg/talk.hpp>

namespace waypoint_function
{
    class SpeakServer : public rclcpp::Node
    {
        public:
            explicit SpeakServer(const rclcpp::NodeOptions & options);

        private:
            void Callback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request,
                            std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response);
            void ServerApply();
        
            rclcpp::Client<waypoint_function_msgs::srv::Command>::SharedPtr apply_client_;
            rclcpp::Service<waypoint_function_msgs::srv::Command>::SharedPtr server_;
            rclcpp::Publisher<voicevox_ros2_msgs::msg::Talk>::SharedPtr voicevox_pub_;

            std::string COMMAND_HEADER = "speak";
            std::string SERVER_NAME = "speak_server";
    };
}

#endif  // SPEAK_SERVER__SPEAK_SERVER_HPP_