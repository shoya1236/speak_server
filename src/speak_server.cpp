
#include "speak_server/speak_server.hpp"

using namespace std::chrono_literals;

SpeakServer::SpeakServer(const rclcpp::NodeOptions &options) : Node("speak_server", options)
{
    // Publisher to send talk msg
    voicevox_pub_=create_publisher<voicevox_ros2_msgs::msg::Talk>("voicevox_ros2", rclcpp::QoS(10));

    // Subscriber to update when waypoint updated
    update_sub_ = create_subscription<example_interfaces::msg::Empty>("server_update", 1,
        bind(&SpeakServer::Update, this, std::placeholders::_1));
    
    // Create Server
    server_ = create_service<waypoint_function_msgs::srv::Command>(
        SERVER_NAME,
        std::bind(&SpeakServer::Callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Create Client for Server Apply
    apply_client_ = create_client<waypoint_function_msgs::srv::Command>("server_apply");
    // Apply Tihs Server to Host Server to create connection
    ServerApply();
}

void SpeakServer::Update(const example_interfaces::msg::Empty::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "SpeakServer Server Update.");
    /* write upate code when waypoint updated */
}

void SpeakServer::Callback(const std::shared_ptr<waypoint_function_msgs::srv::Command::Request> request, std::shared_ptr<waypoint_function_msgs::srv::Command::Response> response)
{
    RCLCPP_INFO(get_logger(), "SpeakServer Server Called.");

    // publish talk text
    voicevox_ros2_msgs::msg::Talk talk_mgg;
    talk_mgg.speaker_id = 1;
    talk_mgg.text = request->data[1];
    talk_mgg.queuing = false;
    voicevox_pub_->publish(talk_mgg);

    // Send Result Message 
    std::string result_msg;
    result_msg = "complete"; // Result Message Example 1
    response->message = SERVER_NAME + ":" + result_msg;
}

void SpeakServer::ServerApply()
{
    // Execute Server Apply
    while (!apply_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Service is not available. waiting...");
    }
    
    // Send Request to Host Server
    auto request = std::make_shared<waypoint_function_msgs::srv::Command::Request>();
    request->data.push_back(COMMAND_HEADER);
    request->data.push_back(SERVER_NAME);
    auto result = apply_client_->async_send_request(request);

    auto returnCode = rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), result);
    
    // Wait for Recieving Result
    if(returnCode == rclcpp::FutureReturnCode::SUCCESS){
        std::string msg = result.get()->message;
        if(msg[0] == 'S') RCLCPP_INFO(get_logger(), msg.c_str());
        else if (msg[0] == 'F') RCLCPP_ERROR(get_logger(), msg.c_str());
    }
    else RCLCPP_ERROR(get_logger(), "Server Apply Failed.");

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SpeakServer)
