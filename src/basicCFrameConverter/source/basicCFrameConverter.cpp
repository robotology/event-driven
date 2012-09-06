#include "basicCFrameConverter.hpp"

using namespace yarp::os;
using namespace emorph::ebuffer;
basicCFrameConverter::basicCFrameConverter(unsigned int _h, unsigned int _w, unsigned int _acc, string _src, unsigned int _type, unsigned int _ori)
:converter(_h,_w, _acc, _src, _type, _ori)
{
    converter.start();
}
basicCFrameConverter::~basicCFrameConverter()
{
}
void basicCFrameConverter::onRead(eventBuffer& i_ub)
{
#ifdef _DEBUG
    std::cerr << "[basicCFrameConverter] Received packet => size=" << i_ub.get_sizeOfPacket() << std::endl;
#endif
    converter.setBuffer(i_ub.get_packet(), i_ub.get_sizeOfPacket());
}
