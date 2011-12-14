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
#include <typeinfo>

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
    cout<<dec<<endl;
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
    clef2.setNumAE(7856754);
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

    ClusterEvent3D cle3d;
    cle3d.setChannel(0);
    cle3d.setDisparity(67);
    cle3d.setXCog(13);
    cle3d.setYCog(91);    
    cout<<cle3d.getContent().toString().c_str()<<endl;

    ClusterEvent3DFeatures1 cle3df1;
    cle3df1.setChannel(1);
    cle3df1.setDisparity(68);
    cle3df1.setNumAE(5767433);
    cle3df1.setXCog(24);
    cle3df1.setYCog(73);
    cout<<cle3df1.getContent().toString().c_str()<<endl;

    ClusterEvent3DFeatures2 cle3df2;
    cle3df2.setChannel(0);
    cle3df2.setDisparity(69);
    cle3df2.setNumAE(7856754);
    cle3df2.setXCog(51);
    cle3df2.setYCog(59);
    cle3df2.setShapeType(44);
    cle3df2.setXSize(12);
    cle3df2.setYSize(33);
    cout<<cle3df2.getContent().toString().c_str()<<endl;

    ClusterEvent3DFeatures3 cle3df3;
    cle3df3.setChannel(1);
    cle3df3.setDisparity(70);
    cle3df3.setNumAE(263543);
    cle3df3.setXCog(26);
    cle3df3.setYCog(73);
    cle3df3.setShapeType(74);
    cle3df3.setShapeProb(89);
    cle3df3.setXSize(12);
    cle3df3.setYSize(9);
    cle3df3.setXVel(78);
    cle3df3.setYVel(43);
    cout<<cle3df3.getContent().toString().c_str()<<endl;

    // keep trace of sent events
    // but set ownership to false since
    // events have been defined statically
    eEventQueue txQueue(false);
    txQueue.push_back(&ts);
    txQueue.push_back(&ae);
    txQueue.push_back(&ae3d);
    txQueue.push_back(&aef);
    txQueue.push_back(&ae3df);
    txQueue.push_back(&cle);
    txQueue.push_back(&clef1);
    txQueue.push_back(&clef2);
    txQueue.push_back(&clef3);
    txQueue.push_back(&cle3d);
    txQueue.push_back(&cle3df1);
    txQueue.push_back(&cle3df2);
    txQueue.push_back(&cle3df3);

    cout<<endl;

    // encode events within packets
    Bottle packets;
    for (size_t i=0; i<txQueue.size(); i++)
        packets.append(txQueue[i]->encode());
    printPacket(packets);

    // network comes into play here
    // ...
    cout<<endl;

    // receive the packets and
    // decode events
    eEventQueue rxQueue;    // the ownership is true by default
    double t0=Time::now();
    bool ok=eEvent::decode(packets,rxQueue);
    double t1=Time::now();

    // insert mismatches deliberately
    // to check the functionality of operator==()
    for (size_t i=0; i<rxQueue.size(); i++)
    {
        // to identify the type of the packet
        // user can rely on the getType() method
        if (rxQueue[i]->getType()=="AE")
        {
            AddressEvent* ptr=dynamic_cast<AddressEvent*>(rxQueue[i]);
            if (ptr!=NULL)
            {
                int xOld=ptr->getX();
                int xNew=xOld+1;
                ptr->setX(xNew);

                cout<<"packet #"<<i<<": changed x from "
                    <<xOld<<" to "<<xNew<<endl;
                cout<<endl;
            }
        }
    }

    if (ok)
    {
        for (size_t i=0; i<rxQueue.size(); i++)
            cout<<rxQueue[i]->getContent().toString().c_str()<<endl;
        cout<<"decoded in "<<t1-t0<<" [s]"<<endl;        

        // ensure the equality
        cout<<endl;
        cout<<"#"<<txQueue.size()<<" packets sent"<<endl;
        cout<<"#"<<rxQueue.size()<<" packets received"<<endl;
        cout<<"testing equalities ..."<<endl;

        size_t len=std::min(txQueue.size(),rxQueue.size());
        for (size_t i=0; i<len; i++)
            cout<<"#"<<i<<": "<<(*(txQueue[i])==*(rxQueue[i])?"ok":"no!")<<endl;
    }
    else
        cout<<"unrecognized packets!"<<endl;

    // whenever rxQueue is disposed the
    // memory allocated for events is
    // automatically released!

    return 0;
}


