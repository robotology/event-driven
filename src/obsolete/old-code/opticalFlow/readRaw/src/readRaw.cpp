#include "readRaw.h"

using namespace std;
using namespace yarp::os;
readRaw::readRaw()
{
//    void** argELK= new void*[4];
//    int sMapX   = 128;    argELK[0]= &sMapX;
//    int sMapY   = 128;    argELK[1]= &sMapY;
//    int neighLR = 2;      argELK[2]= &neighLR;
//    int stdDev  = 0.2;    argELK[3]= &stdDev;
//
//    objFlow = eLucasKanade(4, argELK);

//    void** argSYN= new void*[12];
//                                                    argSYN[0]= &sMapX;
//                                                    argSYN[1]= &sMapY;
//                                                    argSYN[2]= &neighLR;
//        stdDev                  = 0.8;              argSYN[3]= &stdDev;
//    int threshold               = 5;                argSYN[4]= &threshold;
//    int alpha                   = 5;                argSYN[5]= &threshold;
//    int tauC                    = 5;                argSYN[6]= &tauC;
//    int tauD                    = 10;               argSYN[7]= &tauD;
//    int accTime                 = 5000;             argSYN[8]= &accTime;
////    void (*fn2call)(int, void**)= &objFlow.fnCall;  argSYN[9]= reinterpret_cast<void*>(fn2call);
//    int nApF                    = 4;                argSYN[10]= &nApF;
//    int selection               = 0x00000033;       argSYN[11]= &selection;

//    objSynapse = synapse(12, argSYN);

//    objUnmask = unmask(&objSynapse.fnCall);
}
readRaw::~readRaw()
{
}
void readRaw::onRead(eventBuffer& i_ub)
{
//    cout << "Receive events" << endl
//         << "Size of the packet: " << i_ub.get_sizeOfPacket() << endl;
    objUnmask.unmaskData(i_ub.get_packet(), i_ub.get_sizeOfPacket(), 4);
//    cout << "unmaskData called ..." << endl;
}
