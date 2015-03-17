#include "yarpDefault.h"

using namespace emorph;
using namespace yarp::os;
using namespace yarp::sig;


yarpDefault::yarpDefault()
{
    Network yarp;

    inputPort.open("/inputEvents");   

    std::cout << "Connecting to input port " << yarp.connect("/vDownSampleFour/vBottle:o", "/inputEvents", "tcp") << std::endl;
    
    
    imageToWrite.resize(128, 128);

    for (int i=0; i < 128; i++)
    {
            for (int j=0; j < 128; j++)    
            {
                PixelRgb& pixel = imageToWrite.pixel(i, j);
                pixel.r = 0.5;
                pixel.g = 0.5;
                pixel.b = 0.5;
            }
    }

    imagePort.open("/imageRF");
    outputNeuronStatePort.open("/outputState");

}

yarpDefault::yarpDefault(std::string inputPortName, std::string imagePortName, std::string outputNeuronState, std::string sourcePortName)
{
    Network yarp;

    inputPort.open(inputPortName.c_str());   

    std::cout << "Connecting to input port " << yarp.connect(sourcePortName, inputPortName.c_str(), "udp") << std::endl;

    imageToWrite.resize(128, 128);

    for (int i=0; i < 128; i++)
    {
            for (int j=0; j < 128; j++)    
            {
                PixelRgb& pixel = imageToWrite.pixel(i, j);
                pixel.r = 0.5;
                pixel.g = 0.5;
                pixel.b = 0.5;
            }
    }

    imagePort.open(imagePortName.c_str());
    
    outputNeuronStatePort.open(outputNeuronState.c_str());

}

yarpDefault::~yarpDefault()
{
    inputPort.close();
    imagePort.close();
}
