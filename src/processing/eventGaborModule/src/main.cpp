#include <cstdlib>
#include <iostream>
#include <time.h>
#include <vector>
#include <sstream>
#include <math.h>
#include <tr1/memory>

#include <vector>

#include "receptiveField.h"
#include "yarpDefault.h"
#include "neuronsLIF.h"

using namespace std;
using namespace yarp::os;
using namespace emorph;

#define THRESHOLD 15.0

int main(int argc, char *argv[])
{
    bool run = true;
    bool verbose = false;

    int numNeurons = 0;

    ResourceFinder rf;
    rf.configure(argc, argv);

    int X;
    int Y;
    
//    string weightsFile = rf.find("file").asString();
    string inputPortName = rf.find("inputPort").asString();
    string imagePortName = rf.find("imagePort").asString();
    string outputPrt = rf.find("outputPort").asString();    
    string sourceEventsPort = rf.find("sourcePort").asString();

    double sampleBy = rf.find("sampleBy").asDouble();

    if(sampleBy == 0)
        sampleBy = 1;

    X = rf.find("X").asInt();
    Y = rf.find("Y").asInt();
 
    X = X/sampleBy + (64 - 128/(2*sampleBy));
    Y = Y/sampleBy + (64 - 128/(2*sampleBy));


    if(inputPortName == "")
        inputPortName = "/inputEvents";

    if(imagePortName == "")
        imagePortName = "/imageRF";

    if(outputPrt == "")
        outputPrt = "/outputState";

    if(X == 0)
         X = 64;
     
    if(Y == 0)
        Y = 80;

    if(sourceEventsPort == "")
        sourceEventsPort = "/aexGrabber/vBottle:o";
    

//********* TO DO : Need too remove this to a for loop. Vector push_back not working for some reason.

    vector<receptiveField*> receptiveFields;
    string weightsFile;
    //    receptiveFields.resize(24);
 
    receptiveField rf1;
    weightsFile = "/home/icub/curvedRFs/cRF1.txt" ;
    rf1.setWeightsFile(weightsFile.c_str());
    rf1.setWeights();
//        cout << "Receptive Field " << r << " is valid? " <<  rf1.isValid() << endl;
    receptiveFields.push_back(&rf1);

    vector<neuronLIF*> neurons;

    neuronLIF neuron1;
    neuron1.setNeuronId("neuron1");
    neuron1.setDebug(false);
    neuron1.save2File(false);
    neuron1.setNeuronCenter(X, Y);
    neurons.push_back(&neuron1);

    numNeurons = neurons.size();

    yarpDefault network(inputPortName, imagePortName, outputPrt, sourceEventsPort);

    yarp::os::Bottle outBottle;
    network.output = outBottle;
        

    vBottle *tempData;
    vector <double> potentials;    
    potentials.resize(numNeurons);

    vector <double> spikes;
    spikes.resize(numNeurons);

    double maxPotential = THRESHOLD;
    int indexMax = -1;


	receptiveField *rField0 = receptiveFields.at(0);
	
	
//    rf1.multiplyWeights(0.5);
    while(run)
    {

        spikes.clear();
        spikes.resize(numNeurons);

        //read events from vBottle
        tempData = network.inputPort.read();

        //create event queue
        emorph::vQueue q;
        //create queue iterator
        emorph::vQueue::iterator qi;

        tempData->getAll(q);

        Bottle outB2;
        Bottle &outB = network.outputNeuronStatePort.prepare();
        outB.clear();

        double currentToUpdate = 0;

        if(verbose) //debugging
            cout << "Processing " << q.size()<< " events." << endl;

        double sumCurrent = 0;
        for (int rfUpdate = 0; rfUpdate < numNeurons; rfUpdate++)
        {
            receptiveField *rField = receptiveFields.at(rfUpdate);
            neuronLIF *neuron = neurons.at(rfUpdate);
            //neuron->resetNeuron();

            for (qi = q.begin(); qi != q.end(); qi++)
            {



                unsigned int ev_t = (*qi)->getStamp();

                AddressEvent *aep = (*qi)->getAs<AddressEvent>();
                int posX    = aep->getX();
                int posY    = aep->getY(); posY = 127 - posY;
                int pol     = aep->getPolarity();
                int channel = aep->getChannel();


                if(verbose)
                {
                    cout << "[main]: Updating neuron: " << rfUpdate << " from total " << numNeurons << "." << endl;
                    cout << "[main]: Updating neuron with ID: " << neuron->getNeuronId() << endl;
                }

                if ((posX < neuron->getCenterX()+16) && (posX > neuron->getCenterX()-16) && (posY < neuron->getCenterY()+16) && (posY > neuron->getCenterY()-16) && channel == 0 && pol == 1)
                {

                    double toUpdateX = posX - (neuron->getCenterX() - 16);
                    double toUpdateY = posY - (neuron->getCenterY() - 16);

                    currentToUpdate = rField->getWeightAt(toUpdateX, toUpdateY)* (pol*2-1); //change pol from 0:1 to -1:1
                    sumCurrent += currentToUpdate;


                    neuron->updateNeuron(currentToUpdate);

                    potentials.at(rfUpdate) = neuron->getPotential();

                    //yarp::sig::PixelRgb& pixel = network.imageToWrite.pixel(posX, posY);
                    //pixel.r = 0;
                    //pixel.g = 100+100*rField1->getWeightAt(toUpdateX, toUpdateY)*(pol*2-1);
                    //pixel.b = 100+100*rField0->getWeightAt(toUpdateX, toUpdateY)*(pol*2-1);

                    // */

                }
                network.imagePort.write(network.imageToWrite);
            }//end of bottle events

            outB2.addDouble(neuron->getPotential());
            //outB2.addDouble(spikes.at(rfUpdate));

            if(potentials.at(rfUpdate) > maxPotential)
            {
                indexMax = rfUpdate;
                maxPotential = potentials.at(rfUpdate);
            }

            ///* add a small square at gabor center location
            ///
                                  for (int x = -2; x < 2; x++)
                                  {
                                      for (int y = -2; y < 2; y++)
                                      {
                                          yarp::sig::PixelRgb& pixel = network.imageToWrite.pixel(X+x, Y+y);
                                          pixel.r = 255;
                                          pixel.g = 0;
                                          pixel.b = 0;
                                       }
                                  }
                                  network.imagePort.write(network.imageToWrite);
              // */
        } // end of update of all neurons

        for (int rfUpdate = 0; rfUpdate < numNeurons; rfUpdate++)
        {
            neuronLIF *neuron = neurons.at(rfUpdate);
            cout << "RF" << rfUpdate << ": " << neuron->getNumSpikes() << " : " << neuron->getPotential() << " " ;
            //cout << spikes.at(rfUpdate) <<  " " << neuron->getNumSpikes() << " ";
        }
        cout << endl;

        //Required when using competition among filters
        if(indexMax > -1)
        {

            receptiveField *rfToShow = receptiveFields.at(indexMax);

            // Add receptive field to image

         //   /*
                                  for (int x = 0; x < 32; x++)
                                  {
                                      for (int y = 0; y < 32; y++)
                                      {
                                          yarp::sig::PixelRgb& pixel = network.imageToWrite.pixel(X-16+x, Y-16+y);
                                                    //network.imageToWrite.pixel(64-x+16, 64-y+16) = 127+neuron1.getPotential()*rf1.getWeightAt(x, y);

                                              pixel.r = 127+100*(rfToShow->getWeightAt(x,y)); //abs(int(50.0*neuron->getPotential()*rf1.getWeightAt(x, y)));
                                              pixel.g = 127+100*(rfToShow->getWeightAt(x,y)); //abs(int(50.0*neuron->getPotential()*rf1.getWeightAt(x, y)));
                                              pixel.b = 127+100*(rfToShow->getWeightAt(x,y)); //abs(int(50.0*neuron->getPotential()*rf1.getWeightAt(x, y)));

                                      }
                                  }
              // */
            network.imagePort.write(network.imageToWrite);
            indexMax = -1;
            maxPotential = THRESHOLD;
            // Reset all neurons if one reaches threshold
            for (int rfUpdate = 0; rfUpdate < numNeurons; rfUpdate++)
            {
                neuronLIF *neuron = neurons.at(rfUpdate);
                neuron->resetNeuron();
            }
        }
        else
        {
            // Clear image at receptive field

            /*
             *                     for (int x = 0; x < 32; x++)
             *                     {
             *                         for (int y = 0; y < 32; y++)
             *                         {
             *                             yarp::sig::PixelRgb& pixel = network.imageToWrite.pixel(X-16+x, Y-16+y);
             *                                       //network.imageToWrite.pixel(64-x+16, 64-y+16) = 127+neuron1.getPotential()*rf1.getWeightAt(x, y);
             *                             pixel.r = 0;
             *                             pixel.g = 0;
             *                             pixel.b = 0;
             *                         }
             *                     }
             *                     network.imagePort.write(network.imageToWrite);
             *
             *
             *              // */
            //                    maxPotential = maxPotential*0.95;

            for (int rfUpdate = 0; rfUpdate < numNeurons; rfUpdate++)
            {
                neuronLIF *neuron = neurons.at(rfUpdate);
                //neuron->resetNeuron();
            }
        }

        // *    /

        outB = outB2;
        if(network.outputNeuronStatePort.getOutputCount())
        {
            network.outputNeuronStatePort.write(); //write the potential on port
        }
        //			}
        outB2.clear();
    }
    //    system("PAUSE");

    return EXIT_SUCCESS;
}

