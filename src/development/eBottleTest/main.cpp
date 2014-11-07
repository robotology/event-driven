
#include <iostream>
#include <yarp/os/all.h>
#include <iCub/emorph/eBottle.h>
#include <iCub/emorph/eCodec.h>


int main(int argc, char * argv[]) {





    //test eBottle
    eBottle bb;
    emorph::ecodec2::AddressEvent ae;

    std::cout << "Adding Events:" << std::endl;

    ae.setChannel(1);
    ae.setPolarity(1);
    ae.setX(20);
    ae.setY(30);
    ae.setStamp(44);
    bb.addEvent(ae);

    std::cout << ae.getContent().toString() << std::endl;


    ae.setChannel(0);
    ae.setPolarity(0);
    ae.setX(11);
    ae.setY(98);
    ae.setStamp(1000);
    bb.addEvent(ae);

    std::cout << ae.getContent().toString() << std::endl;


    ae.setChannel(1);
    ae.setPolarity(0);
    ae.setX(126);
    ae.setY(9);
    ae.setStamp(12345678);
    bb.addEvent(ae);

    std::cout << ae.getContent().toString() << std::endl;

    std::cout << "Contents of eBottle" << std::endl;
    std::cout << bb.toString() << std::endl;

    emorph::ecodec2::eEventQueue q;

    //get all the events of type given in arg1
    bb.get(ae, q);


    std::cout << "Contents retrieved from eBottle" << std::endl;
    emorph::ecodec2::AddressEvent *aeout;
    emorph::ecodec2::eEventQueue::iterator qi;
    for(qi = q.begin(); qi != q.end(); qi++) {
        aeout = dynamic_cast<emorph::ecodec2::AddressEvent*>(*qi);
        std::cout << aeout->getContent().toString() << std::endl;
    }

    std::cout << "Testing Done" << std::endl;

    return 0;


}
