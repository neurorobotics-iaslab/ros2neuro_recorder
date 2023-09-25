#ifndef ROS2NEURO_RECORDER_DUMMYWRITER_CPP
#define ROS2NEURO_RECORDER_DUMMYWRITER_CPP

#include "ros2neuro_recorder/DummyWriter.hpp"

namespace rosneuro {

DummyWriter::DummyWriter(NeuroFrame* frame) : Writer(frame) {
    this->name_ = "dummywriter";
}

DummyWriter::~DummyWriter(void) {
}

bool DummyWriter::setup(void) {
    return true;
}

bool DummyWriter::open(const std::string& filename) {
    return true;
}

bool DummyWriter::close(void) {
    return true;
}

int DummyWriter::write(int nswrite) {
    return 0;
}

bool DummyWriter::addEvent(int event, double onset, double duration) {
    return true;
}

}


#endif
