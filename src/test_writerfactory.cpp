#include "ros2neuro_recorder/FactoryWriter.hpp"
#include <unistd.h>

using namespace rosneuro;

int main(int argc, char** argv) {

    rosneuro::NeuroFrame    frame;
    FactoryWriter factory;

    std::unique_ptr<Writer> xdfwrt   = factory.createWriter(&frame, WriterType::XDFWRT);
    std::unique_ptr<Writer> dummywrt = factory.createWriter(&frame, WriterType::DUMMYWRT);
    
    xdfwrt->who();
    dummywrt->who();

    if(xdfwrt->open("./test_file.gdf") == false)
        std::cerr<<"Open failed"<<std::endl;

    if(xdfwrt->setup() == false)
        std::cerr<<"Setup failed"<<std::endl;


    return 0;
}
