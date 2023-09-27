#ifndef ROS2NEURO_RECORDER_HPP
#define ROS2NEURO_RECORDER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "ros2neuro_recorder/FactoryWriter.hpp"
#include "ros2neuro_data/NeuroData.hpp"
#include "ros2neuro_data/NeuroDataTools.hpp"

#include "ros2neuro_msgs/msg/neuro_event.hpp"
#include "ros2neuro_msgs/msg/neuro_frame.hpp"
#include "ros2neuro_msgs/msg/neuro_data_info.hpp"
#include "ros2neuro_msgs/srv/get_acquisition_info.hpp"

namespace ros2neuro {

class Recorder : public rclcpp::Node {
    public:
        Recorder(void);
        virtual ~Recorder(void);

        bool configure(void);

        bool run(void);


    public:
        enum {IS_IDLE, IS_WAITING, IS_STARTING, IS_READY, IS_QUIT};

    private:
        void on_received_data(const ros2neuro_msgs::msg::NeuroFrame& msg);
        void on_received_event(const ros2neuro_msgs::msg::NeuroEvent& msg);

        unsigned int on_writer_idle(void);
        unsigned int on_writer_waiting(void);
        unsigned int on_writer_starting(void);

        bool on_request_record(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                               const std::shared_ptr<std_srvs::srv::Empty::Response> res);
        bool on_request_quit(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                             const std::shared_ptr<std_srvs::srv::Empty::Response> res);

        std::string get_datetime(void);
        std::string get_filename(std::string subject, std::string modality, 
                                 std::string task, std::string extra);

    private:
        rclcpp::Subscription<ros2neuro_msgs::msg::NeuroFrame>::SharedPtr sub_data_;
        rclcpp::Subscription<ros2neuro_msgs::msg::NeuroEvent>::SharedPtr sub_evt_;    
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr    srv_record_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr    srv_quit_;
        rclcpp::Client<ros2neuro_msgs::srv::GetAcquisitionInfo>::SharedPtr srv_info_;
        std::string         topic_data_;
        std::string         topic_evt_;
        unsigned int        state_;

        bool                firstdata_;
        double              starttime_;

        FactoryWriter            factory_;
        std::unique_ptr<Writer>  writer_;

        std::string        filename_;
        bool            autostart_;
        bool            is_frame_set_;
        
        NeuroFrame                        frame_;
        ros2neuro_msgs::msg::NeuroFrame    msg_;

};

}


#endif
