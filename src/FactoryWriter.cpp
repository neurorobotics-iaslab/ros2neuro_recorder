#ifndef ROS2NEURO_RECORDER_FACTORY_WRITER_CPP
#define ROS2NEURO_RECORDER_FACTORY_WRITER_CPP

#include "ros2neuro_recorder/FactoryWriter.hpp"

namespace ros2neuro {

std::unique_ptr<Writer> FactoryWriter::createWriter(NeuroFrame* frame, unsigned int type) {

    std::unique_ptr<Writer> wrt;
    switch(type) {
        case WriterType::XDFWRT:
            wrt = std::unique_ptr<XDFWriter>(new XDFWriter(frame));
            break;
        case WriterType::DUMMYWRT:
            wrt = std::unique_ptr<DummyWriter>(new DummyWriter(frame));
            break;
        default:
            printf("[FactoryWriter] - Unknown writer type required: %u\n", type);
            wrt = std::unique_ptr<Writer>(nullptr);
            break;
    }

    return wrt;
}

}

#endif
