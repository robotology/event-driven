#ifndef VECTORPP_H
#define VECTORPP_H

#include <yarp/sig/Vector.h>

#ifdef __cplusplus
extern "C" {
#endif

namespace yarp
{
    namespace vectorpp
    {
        yarp::sig::Vector operator-(yarp::sig::Vector&);
    }
}

#ifdef __cplusplus
}
#endif

#endif //VECTORPP_H
