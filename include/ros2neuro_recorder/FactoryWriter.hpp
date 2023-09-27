#ifndef ROS2NEURO_RECORDER_FACTORY_WRITER_HPP
#define ROS2NEURO_RECORDER_FACTORY_WRITER_HPP

#include <memory>
#include "ros2neuro_data/NeuroData.hpp"
#include "ros2neuro_recorder/Writer.hpp"
#include "ros2neuro_recorder/XDFWriter.hpp"
#include "ros2neuro_recorder/DummyWriter.hpp"

namespace ros2neuro {

enum WriterType {XDFWRT, DUMMYWRT};

class FactoryWriter {
    public:
        std::unique_ptr<Writer> createWriter(NeuroFrame* frame, unsigned int type = WriterType::XDFWRT);

};


}


#endif
