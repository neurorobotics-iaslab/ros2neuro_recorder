#ifndef ROS2NEURO_RECORDER_WRITER_HPP
#define ROS2NEURO_RECORDER_WRITER_HPP

#include <string>
#include <errno.h>
#include <string.h>
#include <iostream>
#include <cstring>

#include "ros2neuro_data/NeuroData.hpp"

namespace ros2neuro {


class Writer {

    public:
        Writer(NeuroFrame* frame);
        virtual ~Writer(void);

        virtual bool setup(void) = 0;
        virtual bool open(const std::string& filename) = 0;
        virtual bool close(void) = 0;
        virtual int write(int nswrite) = 0;


        virtual bool addEvent(int event, double onset, double duration) = 0;
        
        virtual std::string getName(void);
        virtual void who(void);
    
    protected:
        std::string    name_;
        NeuroFrame*    frame_;


};


}


#endif
