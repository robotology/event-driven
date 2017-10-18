#include "vSkinInterface.h"

/*////////////////////////////////////////////////////////////////////////////*/
// SKININTERFACE
/*////////////////////////////////////////////////////////////////////////////*/


bool skinInterface::open(std::string name)
{
    if(!inputPort.open(name + "/vBottle:i"))
        return false;
    if(!outputPort.open(name + "/vBottle:o"))
        return false;
    if(!scopePort.open(name + "/scope:o"))
        return false;

    return true;
}

void skinInterface::onStop()
{
    inputPort.close();
    outputPort.close();
    scopePort.close();
    inputPort.releaseDataLock();
}

void skinInterface::run()
{
    yarp::os::Stamp ystamp;

    //event output data
    vBottleMimic eventBottle;
    eventBottle.setHeader(AE::tag);

    while(true) {

        //int firstEv = 0, lastEv = 0, firstRaw = 0 , lastRaw = 0;
        //double dt;
        int countEv = 0, countRaw = 0;
        vQueue qsending;

        ev::vQueue *q = 0;
        while(!q && !isStopping()) {
            q = inputPort.getNextQ(ystamp);
        }
        if(isStopping()) return;


        for(unsigned int i = 0; i < q->size(); i++) {

            AE *v = read_as<AE>((*q)[i]);
            bool vType = v->type;
            if (vType == 0){
                qsending.push_back((*q)[i]);
//                if (firstEv == 0)
//                {
//                    firstEv = v->stamp;
//                }
//                lastEv = v->stamp;
                countEv++;

            } else {
//                if (firstRaw == 0)
//                {
//                    firstRaw = v->stamp;
//                }
//                lastRaw = v->stamp;
                countRaw++;
            }

        }

        //write to our scope
        static double pscopetime = yarp::os::Time::now();
        if(scopePort.getOutputCount()) {

            double dt = yarp::os::Time::now() - pscopetime;
            pscopetime += dt;

            yarp::os::Bottle &scopedata = scopePort.prepare();
            scopedata.clear();
            scopedata.addDouble(countEv / dt);
            scopedata.addDouble(countRaw / dt);

            scopePort.write();

        }

        if(qsending.size()) {
            ystamp.update();
            eventBottle.setInternalData(qsending);
            outputPort.setEnvelope(ystamp);
            outputPort.write(eventBottle);
        }

        inputPort.scrapQ();

    }



}

