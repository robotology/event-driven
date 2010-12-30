#include "vectorpp.h"

using namespace yarp::sig;
Vector yarp::vectorpp::operator-(Vector& in)
{
    Vector out(in.size());
    for(int i=0; i<in.size(); i++)
        out(i)=-in(i);
    return out;
}
