#ifndef ROS2NEURO_RECORDER_CPP
#define ROS2NEURO_RECORDER_CPP

#include "ros2neuro_recorder/Recorder.hpp"

namespace rosneuro {

Recorder::Recorder(void) : Node("recorder") {
    this->topic_data_    = "/neurodata";
    this->topic_evt_     = "/events/bus";
    this->is_frame_set_  = false;
    this->autostart_     = false;
    this->firstdata_     = false;
    this->state_         = Recorder::IS_IDLE;

    // Retrieve filepath (default: $ROSNEURO_DATA | then: $HOME)
    const char* default_path;
    if( (default_path = std::getenv("ROSNEURO_DATA")) == nullptr) {
        default_path = std::getenv("HOME");
    }

    // declare all ros parameters
    this->declare_parameter("filepath", std::string(default_path));
    this->declare_parameter("filename", "");
    this->declare_parameter("protocol/subject", "");
    this->declare_parameter("protocol/modality", "");
    this->declare_parameter("protocol/task", "");
    this->declare_parameter("protocol/extra", "");
    this->declare_parameter("autostart", false);
}

Recorder::~Recorder(void) {

    this->writer_->close();
}

bool Recorder::configure(void) {

    unsigned int wrttype = WriterType::XDFWRT;
    
    std::string filepath;
    std::string subject;
    std::string task;
    std::string modality;
    std::string extra;
    std::string default_filename;

    this->writer_ = this->factory_.createWriter(&(this->frame_), wrttype);

    // Retrieve subject, task, modality, extra (if exist), filepath, filename, autostart
    subject = this->get_parameter("protocol/subject").as_string();
    modality = this->get_parameter("protocol/modality").as_string();
    task = this->get_parameter("protocol/task").as_string();
    extra = this->get_parameter("protocol/extra").as_string();
    filepath = this->get_parameter("filepath").as_string();

    // Create default filename
    default_filename = get_filename(subject, modality, task, extra);

    // Retrieve filename from param server (if exists)
    rclcpp::Parameter filename_param;
    if(this->get_parameter("filename").as_string().empty()){
        this->filename_ = default_filename;
    }else{
        this->filename_ = filename_param.as_string();
    }

    // Prepend filepath to filename    
    this->filename_ = filepath + "/" + this->filename_;
    RCLCPP_INFO(this->get_logger(), "Data will be recorded in: %s", this->filename_.c_str());
    
    this->autostart_ = this->get_parameter("autostart").as_bool();

    this->sub_data_ = this->create_subscription<ros2neuro_msgs::msg::NeuroFrame>(this->topic_data_, 1000, std::bind(&Recorder::on_received_data, this, std::placeholders::_1));
    this->sub_evt_ = this->create_subscription<ros2neuro_msgs::msg::NeuroEvent>(this->topic_evt_, 1000, std::bind(&Recorder::on_received_event, this, std::placeholders::_1));
    
    this->srv_record_ = this->create_service<std_srvs::srv::Empty>("recorder/record", std::bind(&Recorder::on_request_record, this, std::placeholders::_1, std::placeholders::_2));
    this->srv_quit_ = this->create_service<std_srvs::srv::Empty>("recorder/quit", std::bind(&Recorder::on_request_quit, this, std::placeholders::_1, std::placeholders::_2));

    this->srv_info_ = this->create_client<ros2neuro_msgs::srv::GetAcquisitionInfo>("acquisition/get_info");

    return true;
}

std::string Recorder::get_filename(std::string subject, std::string modality, 
                                   std::string task, std::string extra) {

    std::string osubject  = "UNKNOWN.";
    std::string ctime     = this->get_datetime() + ".";
    std::string omodality = "neuromodality.";
    std::string otask     = "neurotask.";
    std::string oextra;

    if(subject.empty() == false)
        osubject = subject + ".";

    if(modality.empty() == false)
        omodality = modality + ".";
    
    if(task.empty() == false)
        otask = task + ".";
    
    if(extra.empty() == false)
        oextra = extra + ".";

    return osubject + ctime + omodality + otask + oextra + "gdf";

}

std::string Recorder::get_datetime(void) {
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [80];

    time (&rawtime);
    timeinfo = localtime (&rawtime);

    strftime (buffer,80,"%Y%m%d.%H%M%S",timeinfo);

    return std::string(buffer);
}

void Recorder::on_received_data(const ros2neuro_msgs::msg::NeuroFrame& msg) {

    size_t gsize;
    int    wsize;

    gsize = msg.eeg.info.nsamples;

    if(this->state_ != Recorder::IS_READY) {
        return;
    }

    if(firstdata_ == false) {
        // Created by L.Tonin  <luca.tonin@epfl.ch> on 17/03/19 15:38:31    
        // Removing the framerate from starting time
        this->starttime_ = rclcpp::Clock{RCL_ROS_TIME}.now().seconds() - (float)msg.eeg.info.nsamples/(float)msg.sr;
        //this->starttime_ = msg.header.stamp.toSec();
        firstdata_ = true;
    }
    RCLCPP_WARN_ONCE(this->get_logger(), "First NeuroFrame received. The recording started. Message id number: %d", msg.neuroheader.seq);

    // Convert message frame to message data
    NeuroDataTools::ToNeuroFrame(msg, this->frame_);

    // Write data
    wsize = this->writer_->write(gsize);
    
    if(wsize == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed writing on file");
    } else if (wsize != gsize) {
        RCLCPP_WARN(this->get_logger(), "Not all data has been written");
    }
}

void Recorder::on_received_event(const ros2neuro_msgs::msg::NeuroEvent& msg) {

    double onset;

    if(this->state_ != Recorder::IS_READY) {
        return;
    }

    onset = msg.header.stamp.sec - this->starttime_;
    if(this->writer_->addEvent(msg.event, onset, msg.duration) == false) {
        RCLCPP_ERROR(this->get_logger(), "Cannot add event %d: %s", msg.event, strerror(errno)); 
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Added event %d at %fs (duration=%f)", msg.event, onset, msg.duration);
}

bool Recorder::run(void) {

    bool quit = false;
    rclcpp::Rate r(8192);

    // Configure Recorder
    this->configure();
    RCLCPP_INFO(this->get_logger(), "Recorder succesfully configured");

    RCLCPP_INFO(this->get_logger(), "Recorder started");
    while(rclcpp::ok() && quit == false) {
        
        rclcpp::spin_some(shared_from_this()); //ros::spinOnce();

        r.sleep();


        switch(this->state_) {
            case Recorder::IS_IDLE:
                this->state_ = this->on_writer_idle();
                break;
            case Recorder::IS_WAITING:
                this->state_ = this->on_writer_waiting();
                break;
            case Recorder::IS_STARTING:
                this->state_ = this->on_writer_starting();
                break;
            case Recorder::IS_READY:
                break;
            case Recorder::IS_QUIT:
                quit = true;
                break;
            default:
                break;
        }
    }
    RCLCPP_INFO(this->get_logger(), "Recorder closed");

    return true;
}

unsigned int Recorder::on_writer_idle(void) {
    
    if(this->autostart_ == false) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Writer is idle. Waiting for start");
        return Recorder::IS_IDLE;
    }

    RCLCPP_INFO(this->get_logger(), "Writer is waiting for data info from acquisition");
    return Recorder::IS_WAITING;
}

unsigned int Recorder::on_writer_waiting(void) {
    
    if (!srv_info_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_WARN(get_logger(), "Waiting for data info from acquisition...");
      return Recorder::IS_WAITING;
    }

    auto request = std::make_shared<ros2neuro_msgs::srv::GetAcquisitionInfo::Request>();
    auto result = srv_info_->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS){
        auto response = result.get();

        // Configure frame from service message
        if(NeuroDataTools::ConfigureNeuroFrame(response->frame, this->frame_) == false) {
            RCLCPP_ERROR(this->get_logger(), "Cannot configure frame");
            return Recorder::IS_QUIT;
        }

        // Reserve frame data
        this->frame_.eeg.reserve(this->frame_.eeg.nsamples(), this->frame_.eeg.nchannels());
        this->frame_.exg.reserve(this->frame_.exg.nsamples(), this->frame_.exg.nchannels());
        this->frame_.tri.reserve(this->frame_.tri.nsamples(), this->frame_.tri.nchannels());

        // Debug - Dump device configuration
        this->frame_.eeg.dump();
        this->frame_.exg.dump();
        this->frame_.tri.dump();

        this->is_frame_set_ = true;

        return Recorder::IS_STARTING;

        
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to call service get_acquisition_info");
        return Recorder::IS_QUIT;
    }

    
}

unsigned int Recorder::on_writer_starting(void) {

    if(this->state_ == Recorder::IS_READY)
        return Recorder::IS_READY;

    if(this->is_frame_set_ == false) {
        RCLCPP_WARN(this->get_logger(), "NeuroFrame is not set yet. Waiting for data configuration");
        return Recorder::IS_WAITING;
    }

    // Open writer
    if(this->writer_->open(this->filename_) == false) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open the file: %s", this->filename_.c_str());
        return Recorder::IS_QUIT;
    }
    RCLCPP_INFO(this->get_logger(), "File open: %s", this->filename_.c_str());

    // Setup Writer
    if(this->writer_->setup() == false) {
        RCLCPP_ERROR(this->get_logger(), "Cannot setup the writer");
        return Recorder::IS_QUIT;
    }
    RCLCPP_INFO(this->get_logger(), "Writer correctly setup");


    return Recorder::IS_READY;
}

bool Recorder::on_request_record(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                 const std::shared_ptr<std_srvs::srv::Empty::Response> res) {


    if(this->state_ == Recorder::IS_READY) {
        RCLCPP_INFO(this->get_logger(), "Recorder is already recording");
        return true;
    }

    if(this->state_ == Recorder::IS_WAITING) {
        RCLCPP_INFO(this->get_logger(), "Recorder is waiting for data info from acquisition");
        return true;
    }

    if(this->state_ == Recorder::IS_STARTING) {
        RCLCPP_INFO(this->get_logger(), "Recorder is already starting");
        return true;
    }

    RCLCPP_INFO(this->get_logger(), "Writer is waiting for data info from acquisition");
    this->state_ = Recorder::IS_WAITING;

    return true;
}

bool Recorder::on_request_quit(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                               const std::shared_ptr<std_srvs::srv::Empty::Response> res) {

    RCLCPP_WARN(this->get_logger(), "Requested recorder to quit");

    if(this->writer_->close() == false) {
        RCLCPP_ERROR(this->get_logger(), "Cannot close the recorder");
        this->state_ = Recorder::IS_QUIT;
    }
    RCLCPP_INFO(this->get_logger(), "Recorder correctly closed");
    this->state_ = Recorder::IS_QUIT;

    return true;
}


}


#endif
