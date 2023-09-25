#ifndef ROS2NEURO_RECORDER_WRITER_CPP
#define ROS2NEURO_RECORDER_WRITER_CPP

#include "ros2neuro_recorder/Writer.hpp"

namespace rosneuro {

Writer::Writer(NeuroFrame* frame) {
    this->name_   = "undefined";
    this->frame_  = frame;
}

Writer::~Writer(void) {
}

std::string Writer::getName(void) {
    return this->name_;
}

void Writer::who(void) {
    printf("[%s] - %s writer\n", this->getName().c_str(), this->getName().c_str());
}

}


#endif
