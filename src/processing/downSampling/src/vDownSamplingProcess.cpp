#include "vDownSamplingProcess.h"

downSamplingProcessor::downSamplingProcessor()
{

    numRow = 128;
    numCol = 128;


    centerX.resize(numRow*numCol);
    centerY.resize(numRow*numCol);
    weights.resize(numRow*numCol);

    originalImage.resize(numRow*numCol);
    downSampledImage.resize(numRow/2*numCol/2);

    centerX.clear();
    centerY.clear();
    weights.clear();
    originalImage.clear();
    downSampledImage.clear();

    addWeights(0.25);
}

downSamplingProcessor::~downSamplingProcessor()
{
    centerX.clear();
    centerY.clear();
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
            weights.at(numRow*i + numCol) = w;
        }
    }
}


void downSamplingProcessor::downSampling(int x, int y, int pol)
{
    originalImage.at( x*numRow + y ) = originalImage.at( x*numRow + y ) + pol;
    downSampledImage.at( x*numRow + y ) = downSampledImage.at( x*numRow + y ) + pol*weights.at(x*numRow + y);

}


//emorph::vEvent& downSamplingProcessor::myFunc(emorph::vEvent &event)
//{
//    return event;
//}


