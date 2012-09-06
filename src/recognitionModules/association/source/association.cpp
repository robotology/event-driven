#include "association.hpp"

using namespace emorph::ebuffer;
association::association(string _src, unsigned int _type, string _eye, string _features, unsigned int _szSpace, unsigned int _szTemp, double _eigenvalSim, double _eigenvecSim, unsigned int _dim, bool _learn, yarp::os::BufferedPort<emorph::ehist::eventHistBuffer>* _port)
{
    associaterThread=new associationThread(_src, _type, _eye, _features, _szSpace, _szTemp, _eigenvalSim, _eigenvecSim, _dim, _learn, _port);
    associaterThread->start();
}

association::~association()
{
    associaterThread->stop();
    delete associaterThread;
}

void association::onRead(eventBuffer &_buf)
{
    //associaterThread->setBuffer(_buf.get_packet(), _buf.get_sizeOfPacket());
    associaterThread->setBuffer(_buf);
}
