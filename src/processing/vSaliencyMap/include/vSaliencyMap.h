/*
 * Copyright (C) 2011 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Chiara Bartolozzi
 * email:  chiara.bartolozzi@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __VSALMAP__
#define __VSALMAP__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/emorph/all.h>
#include <iCub/emorph/vtsHelper.h>
#include <math.h>


/* ------------------------------------------------------------------------------ */
/* vFeatureMapProcessor -- buffered port that reads the output of the vFeatureMap */
/* ------------------------------------------------------------------------------ */

class vFeatureMapProcessor : public yarp::os::BufferedPort<emorph::vBottle>
{
private:
    
    bool strictness;
    //bool timestampUpdate;
    
    int retinalSize;                    // dimension of the retina device
    int saliencySize;                   // dimension of the saliency map
    
    double maxLeft,minLeft;
    double maxRight, minRight;
    
    double* featureMapLeft;                // map of the feature for the left image; always 128 x 128
    double* featureMapRight;               // map of the feature for the right image; always 128 x 128
    unsigned long* timestampMapLeft;       //
    unsigned long* timestampMapRight;      //

    /**
     * @brief function that given a train of events represent them in the spatial domaain
     @param q reference to the queue of events
     */
    void spatialSelection(emorph::vQueue *q);
    
    /**
     * function that reduces the response using a function of difference in timestamp
     */
    void forgettingMemory();
    
public:
    
    vFeatureMapProcessor();
    ~vFeatureMapProcessor();
    
    bool    open(const std::string moduleName, bool strictness = false);
    void    close();
    void    interrupt();
    
    //this is the entry point to your main functionality
    void    onRead(emorph::vBottle &bot);
    void copyFeatureMapLeft(double *pointer);
    void copyTimestampMapLeft(unsigned long *pointer);
    //unsigned long getLastTimestamp() {return last_ts;};
    
    
};



/* ----------------------------------------------------------------------------------------------- */
/* vSaliencyMapManager -- RateThread for reading the feature maps from fmProcessor buffered ports  */
/* it creates the saliency map as weighted sum of the feature maps and generates a saccade command */
/* ----------------------------------------------------------------------------------------------- */

class vSaliencyMapManager : public yarp::os::RateThread
{
private:

    bool strictness;

    int count;                          // loop counter of the thread
    int iCount;                         // counter of attentional shifts
    //u64 Tnow;
    unsigned long precl;
    unsigned long lc;
    unsigned long lcprev;
    unsigned long rcprev;
    unsigned long rc;
    
    double forgettingFactor;            // forgetting factor subtracted to pixel of feature map [0.0,1.0]
    double microseconds;
    double microsecondsPrev;
    int countStop;                      // counter of equal timestamp
    int countDivider;                   // divider of the count
    int countCommands;                  // counter of sent commands;
    int retinalSize;                    // dimension of the retina device
    int width, height;                  // dimension of the extended input image (extending)
    int height_orig, width_orig;        // original dimension of the input and output images
    int synchPeriod;                    // synchronization period between events and viewer
    int responseGradient;               // responseGradient parameter
    
    std::string name;                   // rootname of all the ports opened by this thread
   
    //bool verb;
    bool synchronised;                  // flag to check whether the microsecond counter has been synchronised
    //bool greaterHalf;                   // indicates whether the counter has passed the half of the range
    //bool idle;                          // controls idle mode
    //bool firstRun;                      // flag that check whether the run is a first useful run
    //bool logPolar;                      // flag that indicates whether the viewer represent logpolar information
    //bool stereo;                        // flag that indicates whether the synchronization is stereo
    //bool asvFlag, dvsFlag;              // flag for operating mode
    bool tristate;                      // option that represent the image with three baselines
    unsigned long minCount;             // minimum timestamp allowed for the current frame
    unsigned long maxCount;             // maximum timestamp allowed for the current frame
    unsigned long minCountRight;
    unsigned long maxCountRight;
    unsigned long* lasttimestamp;       // timestamp of the last event represented
    
    double maxDistance;                  // distance from the center of the WTA
    double startTimer;
    double interTimer;
    double endTimer;
    yarp::os::Semaphore mutex;           // semaphore thar regulates the access to the buffer resource
    clock_t endTime,startTime;
    long T1,T2;
    
    //unmask* unmask_events;               // object that unmask events
    char* bufferRead;                    // buffer of events read from the port
    char* bufferCopy;                    // local copy of the events read
    FILE* fout;                          // file for temporarely savings of events
    FILE* fstore;                        // file for the saving of wta position with timestamps
    FILE* istore;                        // file for the saving of ini
    FILE* raw;                           // file dumper for debug
    
    double* saliencyMapLeft;             // saliencyMap of the left camera
    double* saliencyMapRight;            // saliencyMap of the right camera
    
    double* featureMap41Left;            // 1 feature map from the type 4 (left):
    double* featureMap41Right;           // 1 feature map from the type 4 (right);
    double* featureMap42Left;            // 2 feature map from the type 4 (left):
    double* featureMap42Right;           // 2 feature map from the type 4 (right);
    double* featureMap43Left;            // 3 feature map from the type 4 (left):
    double* featureMap43Right;           // 3 feature map from the type 4 (right);
    double* featureMapA1Left;            // 1 feature map from the type A (left);
    double* featureMapA1Right;           // 1 feature map from the type A (right);
    double* featureMapA2Left;            // 2 feature map from the type A (left);
    double* featureMapA2Right;           // 2 feature map from the type A (right);
    
    unsigned char* saliencyMap;          // saliencyMap collection of responses in different feature maps
    int* featureMap;                     // map of the feature;
    
    unsigned long* timestampMapLeft;     // timestamp reference for the map of the feature
    unsigned long* timestampMap41Left;   // pointer to the copy of the timestamp map (41-Left)
    unsigned long* timestampMap41Right;  // pointer to the copy of the timestamp map (41-Right)
    unsigned long* timestampMap42Left;   // pointer to the copy of the timestamp map (42-Left)
    unsigned long* timestampMap42Right;  // pointer to the copy of the timestamp map (42-Right)
    unsigned long* timestampMap43Left;   // pointer to the copy of the timestamp map (43-Left)
    unsigned long* timestampMap43Right;  // pointer to the copy of the timestamp map (43-Right)
    unsigned long* timestampMapA1Left;   // pointer to the copy of the timestamp map (A1-Left)
    unsigned long* timestampMapA1Right;  // pointer to the copy of the timestamp map (A1-Right)
    unsigned long* timestampMapA2Left;   // pointer to the copy of the timestamp map (A2-Left)
    unsigned long* timestampMapA2Right;  // pointer to the copy of the timestamp map (A2-Right)
    //AER_struct* unmaskedEvents;          // trained of unmasked events
    
    yarp::sig::ImageOf<yarp::sig::PixelRgb>*  imageLeft;                                  //image representing the signal on the leftcamera
    yarp::sig::ImageOf<yarp::sig::PixelRgb>*  imageRight;                                 //image representing the signal on the right camera
    yarp::sig::ImageOf<yarp::sig::PixelMono>* imageLeftBW;                               //image representing the signal on the leftcamera
    yarp::sig::ImageOf<yarp::sig::PixelMono>* imageRightBW;                              //image
    
    //plotterThread* pThread;                                              // plotterThread for the trasformation of the event in images
    
    /* ----- Ports ------ */
    
    yarp::os::BufferedPort<yarp::os::Bottle > outputCmdPort;             // port that is dedicated to sending the typology of the gaze behaviour and some params
    yarp::os::BufferedPort<emorph::vBottle> smEventsPort;                // port that sends bottles of saliency map events
    
    /* ----- Buffered port threads that take each feature map output from ------ */
    /* vFeatureMap and compute the analog feature maps as input of the           */
    /* saliency map                                                              */
    
    vFeatureMapProcessor* bptA1;                                        // processor thread of the bottle whole retina events 1
    vFeatureMapProcessor* bptA2;                                        // processor thread of the bottle whole retina events 2
    vFeatureMapProcessor* bpt41;                                        // processor thread of the bottle feature map 1
    vFeatureMapProcessor* bpt42;                                        // processor thread of the bottle feature map 2
    vFeatureMapProcessor* bpt43;                                        // processor thread of the bottle feature map 3
    
    bool plotLatency;
    double timeStart, timeStop;                                          // variables that measures the computation load
    FILE* latencyFile;                                                   // file where all the latency measurements are saved
    
public:

    vSaliencyMapManager();
    ~vSaliencyMapManager();
    
    bool threadInit();
    void threadRelease();

    void    interrupt();
    void    run();

    /**
     * @brief returns a mono image of the output of the dvs camera (either left or right)
     * @param pixelMono reference to the image contains the counts of events
     * @param minCount reference to the min timestamp in the frame
     * @param maxCount reference to the max timestamp in the frame
     * @param camera reference to the camera the image belongs LEFT 1, RIGHT 1
     */
    void getMonoImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>* image, unsigned long minCount,unsigned long maxCount, bool camera);

    
    /**
     * function that reduces the response using a function of difference in timestamp
     */
    void setName(std::string str){this->name=str;};
    void setStrictness(bool strictness){this->strictness=strictness;};
    //std::string getName(const char* p);

    /**
     * function that sets the width and the height of the images based on the dimension of the input image
     * @param width width of the input image
     * @return height height of the input image
     */
    void resizeAll(int width, int height);


    
};

/* ----------------------------------------------------------------------------------------------- */
/* vSaliencyMapModule -- RF module for rpc calls, it start the vSaliencyMapManager rateThread      */
/* ----------------------------------------------------------------------------------------------- */

class vSaliencyMapModule : public yarp::os::RFModule
{

    //the event bottle input and output handler
    vSaliencyMapManager      *smManager;
    yarp::os::Port          handlerPort;                 // a port to handle messages


public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

};


#endif
//empty line to make gcc happy
