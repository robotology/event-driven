
#include <yarp/os/all.h>
#include <iostream>
#include <iomanip>
#include <string>

#include <iCub/eventCodec.h>

using namespace std;
using namespace yarp::os;
using namespace emorph::ecodec;

void printPacket(const Bottle &packets)
{
    for (int i=0; i<packets.size(); i++)
    {
        cout<<hex<<setw(8)<<setfill('0')
            <<packets.get(i).asInt()<<" ";
    }
    cout<<endl;
}


int main()
{
    // define events to be sent
    TimeStamp ts;
    ts.setStamp(1235423);
    cout<<ts.getContent().toString().c_str()<<endl;

    AddressEvent ae;
    ae.setChannel(0);
    ae.setPolarity(1);
    ae.setX(121);
    ae.setY(78);
    cout<<ae.getContent().toString().c_str()<<endl;

    AddressEvent3D ae3d;
    ae3d.setDisparity(45);
    ae3d.setPolarity(0);
    ae3d.setX(98);
    ae3d.setY(53);
    cout<<ae3d.getContent().toString().c_str()<<endl;

    AddressEventFeatures aef;
    aef.setChannel(1);
    aef.setPolarity(0);
    aef.setX(99);
    aef.setY(77);
    aef.setOrientation(56);
    aef.setXFlow(21);
    aef.setYFlow(19);    
    cout<<aef.getContent().toString().c_str()<<endl;

    AddressEvent3DFeatures ae3df;
    ae3df.setDisparity(201);
    ae3df.setPolarity(1);
    ae3df.setX(87);
    ae3df.setY(12);
    ae3df.setOrientation(3);
    ae3df.setXFlow(65);
    ae3df.setYFlow(90);    
    cout<<ae3df.getContent().toString().c_str()<<endl;

    // encode events within packets
    Bottle packets;
    packets.append(ts.encode());
    packets.append(ae.encode());
    packets.append(ae3d.encode());
    packets.append(aef.encode());
    packets.append(ae3df.encode());
    printPacket(packets);

    // network comes into play here
    // ...

    // receive the packets and
    // decode events
    eEventQueue q;
    if (eEvent::decode(packets,q))
    {
        for (size_t i=0; i<q.size(); i++)
            cout<<q[i]->getContent().toString().c_str()<<endl;
    }
    else
        cout<<"unrecognized packets!"<<endl;

    // whenever q is destructed the memory
    // allocated for the decoded events will
    // be automatically released!

    return 0;
}


