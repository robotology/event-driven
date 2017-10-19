#include "vSkinInterface.h"

/*////////////////////////////////////////////////////////////////////////////*/
// SKININTERFACE
/*////////////////////////////////////////////////////////////////////////////*/


bool skinInterface::open(std::string name)
{
    if(!inputPort.open(name + "/vBottle:i"))
        return false;
    if(!outEvPort.open(name + "/vBottleEv:o"))
        return false;
    if(!outRawPort.open(name + "/vBottleRaw:o"))
        return false;
    if(!scopePort.open(name + "/scope:o"))
        return false;

    return true;
}

void skinInterface::onStop()
{
    inputPort.close();
    outEvPort.close();
    outRawPort.close();
    scopePort.close();
    inputPort.releaseDataLock();
}

void skinInterface::run()
{
    yarp::os::Stamp ystamp;

    //event output data
    vBottleMimic eventBottle;
    eventBottle.setHeader(AE::tag);

    vBottleMimic rawBottle;
    rawBottle.setHeader(AE::tag);

    while(true) {

        //int firstEv = 0, lastEv = 0, firstRaw = 0 , lastRaw = 0;
        //double dt;
        int countEv = 0, countRaw = 0;
        vQueue qsend_ev, qsend_raw;

        ev::vQueue *q = 0;
        while(!q && !isStopping()) {
            q = inputPort.getNextQ(ystamp);
        }
        if(isStopping()) return;


        for(unsigned int i = 0; i < q->size(); i++) {

            AE *v = read_as<AE>((*q)[i]);
            bool vType = v->type;
            if (vType == 0){
                qsend_ev.push_back((*q)[i]);
//                if (firstEv == 0)
//                {
//                    firstEv = v->stamp;
//                }
//                lastEv = v->stamp;
                countEv++;

            } else {
                qsend_raw.push_back((*q)[i]);
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

        if(qsend_ev.size()) {
            ystamp.update();
            eventBottle.setInternalData(qsend_ev);
            outEvPort.setEnvelope(ystamp);
            outEvPort.write(eventBottle);
        }
        if(qsend_raw.size()) {
            ystamp.update();
            rawBottle.setInternalData(qsend_raw);
            outRawPort.setEnvelope(ystamp);
            outRawPort.write(rawBottle);
        }

        inputPort.scrapQ();

    }



}

