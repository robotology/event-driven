#include "vDownSamplingProcess.h"
#include <stdio.h>

downSamplingProcessor::downSamplingProcessor()
{

    std::fprintf(stdout, "[downSamplingProcessor]:: Creating object.\n");

    numRow = 128;
    numCol = 128;

    std::fprintf(stdout, "  [downSamplingProcessor]:: Resizing vectors.\n");

//    centersX.clear();
//    centersY.clear();
//    weights.clear();
//    originalImage.clear();
//    downSampledImage.clear();


    centersX.resize(numRow*numCol);
    centersY.resize(numRow*numCol);
    weights.resize(numRow*numCol);

    originalEvents.resize(numRow*numCol);
    downSampledEvents.resize(numRow*numCol);

    originalImage.resize(numRow*numCol);
    downSampledImage.resize(numRow*numCol);


    std::fprintf(stdout, "  [downSamplingProcessor]:: Resizing vectors.\n");

    windowSize = 2;

    sampleBy = 2;

//    numCenters = initializeCenters();

    std::fprintf(stdout, "  [downSamplingProcessor]:: Initialize weights.\n");
    //addWeights(1/sampleBy);
}

downSamplingProcessor::~downSamplingProcessor()
{
    centersX.clear();
    centersY.clear();
    weights.clear();
    originalImage.clear();
    downSampledImage.clear();
}



void downSamplingProcessor::addWeights(double w)
{

    for (int i=0; i < numRow; i++)
    {
        for (int j = 0; j < numCol; j++)
        {
            weights.at(numRow*i + j) = w;
            originalImage.at(numRow*i+j) = 0;
            downSampledImage.at(numRow*i+j) = 0;
        }
    }
}

int downSamplingProcessor::initializeCenters()
{
    int c=0;

    for (int i=0; i < numRow; i+=windowSize)
    {
        for (int j = 0; j < numCol; j+=windowSize)
        {
            centersX.at(c) = i;
            centersY.at(c) = j;
            c++;
        }
    }
    return c;
}

int downSamplingProcessor::downSampling(int x, int y, int pol)
{

    //std::fprintf(stdout, "  [downSamplingProcessor downSampling]:: Updating location %d, %d with polarity %d.\n", x, y, pol);



    originalImage.at( x*numRow + y ) = originalImage.at( x*numRow + y ) + pol;

    downSampledImage.at( x*numRow + y ) = downSampledImage.at( x*numRow + y ) + pol*weights.at(x*numRow + y);

    originalEvents.at( x*numRow + y ) = originalEvents.at( x*numRow + y ) + pol;
    downSampledEvents.at( x*numRow + y ) = downSampledEvents.at( x*numRow + y ) + pol*weights.at(x*numRow + y);

    if(downSampledEvents.at(x*numRow+y) >= 1)
    {
        downSampledEvents.at(x*numRow+y) = 0;
        return 1;
    }
    else if (downSampledEvents.at(x*numRow+y) <= -1)
    {
        downSampledEvents.at(x*numRow+y) = 0;
        return -1;
    }
    else
    {
        return 0;
    }

}

bool downSamplingProcessor::setSamplingFactor(double factor)
{
    sampleBy = factor;
}

double downSamplingProcessor::getSamplingFactor()
{
    return sampleBy;
}

//emorph::vEvent& downSamplingProcessor::myFunc(emorph::vEvent &event)
//{
//    return event;
//}


