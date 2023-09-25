#include "rclcpp/rclcpp.hpp"
#include "ros2neuro_recorder/Recorder.hpp"

int main(int argc, char** argv) {

    
    // ros initialization
    rclcpp::init(argc, argv);

    auto recorder = std::make_shared<rosneuro::Recorder>();

    if(recorder->run() == false)
        RCLCPP_ERROR(recorder->get_logger(), "Recorder interrupted while running");

    rclcpp::shutdown();
    return 0;
}
