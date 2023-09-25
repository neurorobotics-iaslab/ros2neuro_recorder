#ifndef ROS2NEURO_RECORDER_DUMMYWRITER_HPP
#define ROS2NEURO_RECORDER_DUMMYWRITER_HPP

#include "ros2neuro_recorder/Writer.hpp"
#include "ros2neuro_data/NeuroData.hpp"

namespace rosneuro {

class DummyWriter : public Writer {

    public:
        DummyWriter(NeuroFrame* frame);
        virtual ~DummyWriter(void);

        bool setup(void);
        bool open(const std::string& filename);
        bool close(void);
        int write(int nswrite);

        bool addEvent(int event, double onset, double duration);

};

}


#endif
