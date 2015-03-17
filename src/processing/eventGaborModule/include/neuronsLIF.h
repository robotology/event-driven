
#include <string>
#include <fstream>

class neuronLIF
{
      public:
             neuronLIF();
             virtual ~neuronLIF();
             neuronLIF(neuronLIF&);

             bool updateNeuron(double curr);
             bool updateNeuron(double curr, double delT);
             bool updateNeuron(double curr, unsigned int timeNow);
             bool updateNeuron();
             
             void setInputCurrent();
             bool setInputCurrent(double curr);

             void setNeuronCenter(double u, double v);
             
             bool setThresholdPotential(double thershold);
             double getPotential();
             
             void save2File(bool);
             void writeToFile();
             
             unsigned int getTimeStamp();
             void setDebug(bool);
             
             bool setNeuronId(std::string);
             std::string getNeuronId()
             {
                 return neuronId;
             };

             void resetNeuron();

             double getCenterX();
             double getCenterY();

             unsigned int getNumSpikes();
             
      private:
              // Init neuron related variables
              double restingPotential;
	          double potential;
	          double thresholdPotential;
              double centerX;
              double centerY;

              
              unsigned int lastTimeStamp;
	          unsigned int lastUpdateTime;
	          unsigned int timeFromLastSpike;
	          unsigned int totalSpikes;
	          unsigned int initTimeStamp;
	          double inputCurrent;
	          double membraneTimeConstant;
              // Init display parameters
	          bool display;
	          // Init file save parameters	
              bool updated;
              bool save2file;
              std::string fileName;              
              std::string neuronId;
              
              bool debug;
              std::ofstream saveFid;
             
};
	
