/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <yarp/os/all.h>
#include <iostream>
#include <iomanip>
#include <string>

#include "eventCodec.h"

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

    ClusterEvent cle;
    cle.setChannel(1);
    cle.setNumAE(34);
    cle.setXCog(11);
    cle.setYCog(87);
    cout<<cle.getContent().toString().c_str()<<endl;

    ClusterEventFeatures1 clef1;
    clef1.setChannel(0);
    clef1.setNumAE(3746323);
    clef1.setXCog(24);
    clef1.setYCog(73);
    cout<<clef1.getContent().toString().c_str()<<endl;

    ClusterEventFeatures2 clef2;
    clef2.setChannel(1);
    clef2.setNumAE(785675486);
    clef2.setXCog(25);
    clef2.setYCog(62);
    clef2.setShapeType(43);
    clef2.setXSize(67);
    clef2.setYSize(59);
    cout<<clef2.getContent().toString().c_str()<<endl;

    ClusterEventFeatures3 clef3;
    clef3.setChannel(0);
    clef3.setNumAE(9434223);
    clef3.setXCog(24);
    clef3.setYCog(67);
    clef3.setShapeType(41);
    clef3.setShapeProb(44);
    clef3.setXSize(67);
    clef3.setYSize(59);
    clef3.setXVel(68);
    clef3.setYVel(43);
    cout<<clef3.getContent().toString().c_str()<<endl;

    cout<<endl;

    // encode events within packets
    Bottle packets;
    packets.append(ts.encode());
    packets.append(ae.encode());
    packets.append(ae3d.encode());
    packets.append(aef.encode());
    packets.append(ae3df.encode());
    packets.append(cle.encode());
    packets.append(clef1.encode());
    packets.append(clef2.encode());
    packets.append(clef3.encode());
    printPacket(packets);

    // network comes into play here
    // ...
    cout<<endl;

    // receive the packets and
    // decode events
    eEventQueue q;
    double t0=Time::now();
    bool ok=eEvent::decode(packets,q);
    double t1=Time::now();

    if (ok)
    {
        for (size_t i=0; i<q.size(); i++)
            cout<<q[i]->getContent().toString().c_str()<<endl;
        cout<<"decoded in "<<t1-t0<<" [s]"<<endl;
    }
    else
        cout<<"unrecognized packets!"<<endl;

    // whenever q is destructed the memory
    // allocated for the decoded events will
    // be automatically released!

    return 0;
}


