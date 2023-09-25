#ifndef ROS2NEURO_RECORDER_XDFWRITER_HPP
#define ROS2NEURO_RECORDER_XDFWRITER_HPP

#include <xdfio.h>
#include "ros2neuro_recorder/Writer.hpp"
#include "ros2neuro_data/NeuroData.hpp"

namespace rosneuro {

class XDFWriter : public Writer {

    public:
        XDFWriter(NeuroFrame* frame);
        virtual ~XDFWriter(void);

        bool setup(void);
        bool open(const std::string& filename);
        bool close(void);
        int write(int nswrite);

        bool addEvent(int event, double onset, double duration);

    private:
        bool setup_xdf_group(NeuroDataInfo* info, unsigned int index);
        xdffiletype get_filetype(const std::string& filename);

    protected:
        struct xdf*    file_;
        size_t*    strides_;
    
};

}


#endif
