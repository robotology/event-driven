#include <iCub/emorph/all.h>
#include <vector>

class downSamplingProcessor
{
public:

    downSamplingProcessor();
    virtual ~downSamplingProcessor();
    //emorph::vEvent& myFunc(emorph::vEvent &event);

    void addWeights(double);
    int downSampling(int, int, int);
    int initializeCenters();
    bool setSamplingFactor(double);
    double getSamplingFactor();

private:

    int numCenters;
    int windowSize;
    int numRow;
    int numCol;

    double sampleBy;

    std::vector<double> originalImage;
    std::vector<double> downSampledImage;

    std::vector<double> centersX;
    std::vector<double> centersY;
    std::vector<double> weights;

    std::vector<double> originalEvents;
    std::vector<double> downSampledEvents;

};
