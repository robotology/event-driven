#ifndef MATRIXPP_H
#define MATRIXPP_H

#include <cmath>

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

//#ifdef __cplusplus
//extern "C" {
//#endif

namespace yarp
{
    namespace matrixpp
    {
        yarp::sig::Matrix square(yarp::sig::Matrix);
        yarp::sig::Matrix exp(yarp::sig::Matrix);

        yarp::sig::Matrix operator/(yarp::sig::Matrix&, yarp::sig::Matrix&);
        yarp::sig::Matrix operator/(yarp::sig::Matrix&, double&);
        yarp::sig::Matrix operator/(yarp::sig::Matrix&, int&);
        yarp::sig::Matrix operator%(yarp::sig::Matrix&, yarp::sig::Matrix&);
        yarp::sig::Matrix operator-(yarp::sig::Matrix&);
        yarp::sig::Matrix operator>=(yarp::sig::Matrix&, double&);
        yarp::sig::Matrix operator>=(yarp::sig::Matrix&, int&);
        yarp::sig::Matrix operator!=(yarp::sig::Matrix&, double&);

        void upSubMat(yarp::sig::Matrix&, yarp::sig::Matrix&, int, int, int, int);
        yarp::sig::Vector mat2vec(yarp::sig::Matrix&);
    }
}

//#ifdef __cplusplus
//}
//#endif

#endif //MATRIXPP_H
