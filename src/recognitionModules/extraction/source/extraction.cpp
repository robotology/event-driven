#include "extraction.hpp"

using namespace emorph::ebuffer;
extraction::extraction(std::string _src, unsigned int _type, std::string _eye, unsigned int _szSpace, unsigned int _szTemp, double _eigenvalSim, double _eigenvecSim, unsigned int _dim)
{
    extracterThread=new extractionThread(_src, _type, _eye, _szSpace, _szTemp, _eigenvalSim, _eigenvecSim, _dim);
    extracterThread->start();
}

extraction::~extraction()
{
    extracterThread->stop();
    delete extracterThread;
}

void extraction::onRead(eventBuffer& _buf)
{
    extracterThread->setBuffer(_buf.get_packet(), _buf.get_sizeOfPacket());
}
