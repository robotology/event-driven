
#include <iostream>
#include <yarp/os/all.h>
#include <iCub/emorph/all.h>


int main(int argc, char * argv[]) {

    //test eBottle
    emorph::eBottle bb;
    emorph::AddressEvent ae;
    emorph::ClusterEvent cle;
    emorph::eEventQueue q;
    emorph::eEventQueue::iterator qi;

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

    cle.setChannel(0);
    cle.setId(223);
    cle.setStamp(632);
    cle.setXCog(33);
    cle.setYCog(9999);
    bb.addEvent(cle);

    std::cout << cle.getContent().toString() << std::endl;

    cle.setChannel(1);
    cle.setId(234);
    cle.setStamp(7980);
    cle.setXCog(789);
    cle.setYCog(2);
    bb.addEvent(cle);

    std::cout << cle.getContent().toString() << std::endl;

    ae.setChannel(1);
    ae.setPolarity(0);
    ae.setX(55);
    ae.setY(56);
    ae.setStamp(20);
    bb.addEvent(ae);

    std::cout << ae.getContent().toString() << std::endl;

    std::cout << "Contents of eBottle" << std::endl;
    std::cout << bb.toString() << std::endl;



    //get all the events of type given in arg1
    std::cout << "Contents retrieved from eBottle by type" << std::endl;

    std::cout << "Address Events" << std::endl;
    bb.get<emorph::AddressEvent>(q);  
    for(qi = q.begin(); qi != q.end(); qi++) {
        std::cout << (*qi)->getContent().toString() << std::endl;
        //to use functions in the address event cast it to the correct type
        //and then go for it
        emorph::AddressEvent * aep = dynamic_cast<emorph::AddressEvent *>
                (*qi);
        aep->getX();
    }
    q.clear();

    std::cout << "Cluster Events" << std::endl;
    bb.get<emorph::ClusterEvent>(q);
    for(qi = q.begin(); qi != q.end(); qi++) {
        std::cout << (*qi)->getContent().toString() << std::endl;
    }
    q.clear();

    std::cout << "Just Address Events Sorted" << std::endl;
    bb.getSorted<emorph::AddressEvent>(q);
    for(qi = q.begin(); qi != q.end(); qi++) {
        std::cout << (*qi)->getContent().toString() << std::endl;
    }
    q.clear();

    std::cout << "Sorted Contents Returned From Bottle" << std::endl;
    bb.getAllSorted(q);
    for(qi = q.begin(); qi != q.end(); qi++) {
        std::cout << (*qi)->getContent().toString() << std::endl;
    }

    std::cout << "Testing Done" << std::endl;

    return 0;

}
