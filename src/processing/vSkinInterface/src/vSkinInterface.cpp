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
        
        int firstEv = 0, lastEv = 0, firstRaw = 0 , lastRaw = 0;
        double dt;
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
                if (firstEv == 0)
                {
                    firstEv = v->stamp;
                }
                lastEv = v->stamp;
                countEv++;
                
            } else {
                if (firstRaw == 0)
                {
                    firstRaw = v->stamp;
                }
                lastRaw = v->stamp;
                countRaw++;
            }
            
        }
        if(qsending.size()) {
            ystamp.update();
            eventBottle.setInternalData(qsending);
            outputPort.setEnvelope(ystamp);
            outputPort.write(eventBottle);
        }
        //write to our scope
        //static double pscopetime = yarp::os::Time::now();
        //static double ratetime = yarp::os::Time::now();
        if(scopePort.getOutputCount()) {

            //static int countscope1 = 1;
            //static int countscope2 = 1;
            static double val1 = 0;//-ev::vtsHelper::max_stamp;
            static double val2 = 0;//-ev::vtsHelper::max_stamp;
       
            double rateEv = 0, rateRaw = 0;
            
            if (firstEv != lastEv){
                
                dt = lastEv-firstEv;
                if (dt < 0)
                {
                    dt +=vtsHelper::max_stamp;
                }
                dt *= vtsHelper::tsscaler;
                
                rateEv = countEv/dt;
                
                std::cout << "# Ev: " << countEv << "rate Ev: "<< rateEv << std::endl;
                //val1 += rateEv;
                val1 = rateEv;
                //countscope1++;
            }
            if (firstRaw != lastRaw){
                dt = lastRaw-firstRaw;
                if (dt < 0)
                {
                    dt +=vtsHelper::max_stamp;
                }
                dt *= vtsHelper::tsscaler;
                
                rateRaw = countRaw/dt;
                val2 = rateRaw;
                //countscope2++;
            }
        

            //double scopedt = yarp::os::Time::now() - pscopetime;
            //if((scopedt > 0.05 || scopedt < 0) && countscope1 > 3 && countscope2 > 3) {
            //if(countscope1 > 0 && countscope2 > 0) {
            //        pscopetime += scopedt;

                yarp::os::Bottle &scopedata = scopePort.prepare();
                scopedata.clear();
                //scopedata.addDouble(val1/countscope1);
                //scopedata.addDouble(val2/countscope2);
                scopedata.addDouble(val1);
                scopedata.addDouble(val2);
            
                //val1 = 0;//-ev::vtsHelper::max_stamp;
                //val2 = 0;//-ev::vtsHelper::max_stamp;
                
                //countscope1 = 0;
                //countscope2 = 0;
                scopePort.write();
            //}
        }
        
    }
    
    inputPort.scrapQ();
    
}

