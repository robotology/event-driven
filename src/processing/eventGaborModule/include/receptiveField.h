#ifndef RECEPTIVEFIELD_H
#define RECEPTIVEFIELD_H
 
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <fstream>
 
class receptiveField
{
    public:
        receptiveField();
        receptiveField(std::string);
        virtual ~receptiveField();
         
        bool setFieldSize(int, int);
        bool setWeightsFile(std::string);
         
         
        int getFieldSizeX();
        int getFieldSizeY();
         
        double getWeightAt(int, int);
        bool setWeights();
         
        bool normalizeWeights();
        bool multiplyWeights(double);
//        bool scaleUpWeights();
         
        bool isValid();
        void setVerbose(bool);

        double getCenterX();
        double getCenterY();    

    protected:
        double inhibitor; 
 
    private:
        int fieldSizeX;
        int fieldSizeY;
    
        yarp::sig::Vector weightVector;
        bool valid;
        std::string weightsFile;
        int verbose;
        std::ifstream fid;
};
 
#endif // RECEPTIVEFIELD_H

