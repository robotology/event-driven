#include "tsOptFlow.hpp"

using namespace std; 
using namespace emorph::ebuffer;
using namespace yarp::sig;
using namespace yarp::os;
tsOptFlow::tsOptFlow(uint &_h, uint &_w, string &_src, uint &_type, uint &_acc, uint &_bin, double &_th, uint &_nn, uint &_ssz, uint &_tsval, double &_a, double &_td, int &_pol, uint &_ori, bool &_save, yarp::os::BufferedPort<VelocityBuffer>* _port)
{
    vxMat = new Matrix(_h, _w);
    vyMat = new Matrix(_h, _w);
    mutex = new Semaphore();
    velBuf= new VelocityBuffer();

    _pol=1;
    tsofThreadPos=new tsOptFlowThread(_h, _w, _src, _type, _acc, _bin, _th, _nn, _ssz, _tsval, _a, _td, _pol, _ori, _save, vxMat, vyMat, mutex, velBuf);
    tsofThreadPos->start();
    _pol=-1;
    tsofThreadNeg=new tsOptFlowThread(_h, _w, _src, _type, _acc, _bin, _th, _nn, _ssz, _tsval, _a, _td, _pol, _ori, _save, vxMat, vyMat, mutex, velBuf);
    tsofThreadNeg->start();

    sendvelbuf=new sendVelBuf(velBuf, mutex, _port, _acc);
    sendvelbuf->start();
}

tsOptFlow::~tsOptFlow()
{
    tsofThreadPos->stop();
    delete tsofThreadPos;
    tsofThreadNeg->stop();
    delete tsofThreadNeg;
}

void tsOptFlow::onRead(eventBuffer& _buf)
{
#ifdef _DEBUG
    cout << "[tsOptFlow] Buffer to forward received" << endl;
#endif
    tsofThreadPos->setBuffer(_buf.get_packet(), _buf.get_sizeOfPacket());
    tsofThreadNeg->setBuffer(_buf.get_packet(), _buf.get_sizeOfPacket());
}

