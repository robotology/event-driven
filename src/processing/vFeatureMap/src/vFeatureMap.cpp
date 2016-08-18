/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#include "vFeatureMap.h"


#define X_SHIFT         1
#define Y_SHIFT         8
#define Y_MASK          0x00007F00
#define X_MASK          0x000000FE


inline int convertChar2Dec(char value) {
    if (value > 60)
        return value - 97 + 10;
    else
        return value - 48;
}


/**********************************************************/
bool vFeatureMapModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vFeatureMap")).asString();
    yarp::os::RFModule::setName(moduleName.c_str());

    bool strict = rf.check("strict") &&
            rf.check("strict", yarp::os::Value(true)).asBool();
    
    /* name of the file where the feature map is stored */
    std::string mapName =
            rf.check("mode", yarp::os::Value("intensity")).asString();
    mapName += ".txt";
    std::string mapNameComplete = rf.findFile(mapName.c_str());
    
    /* attach a port of the same name as the module (prefixed with a /) to the module 
     so that messages received from the port are redirected to the respond method */
    
    std::string handlerPortName =  "/";
    handlerPortName += getName();         // use getName() rather than a literal
    
    if (!handlerPort.open(handlerPortName.c_str())) {
        std::cout << getName() << ": Unable to open port " << handlerPortName << std::endl;
        return false;
    }
    
    attach(handlerPort);                  // attach to port

    /* set parameters */
    int retSize = rf.check("retSize", yarp::os::Value(128)).asInt();
    int featSize = rf.check("featSize", yarp::os::Value(128)).asInt();
    int thrOn = rf.check("thrOn", yarp::os::Value(127)).asInt();
    int thrOff = rf.check("thrOff", yarp::os::Value(10)).asInt();
    int constLeak = rf.check("constLeak", yarp::os::Value(1)).asInt();
    
    /* create the thread and pass pointers to the module parameters */
    fmmanager = new vFeatureMapManager(retSize, featSize, thrOn, thrOff, constLeak);
    fmmanager->setMapURL(mapNameComplete);

    return fmmanager->open(moduleName, strict);

}

/**********************************************************/
bool vFeatureMapModule::interruptModule()
{
    fmmanager->interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vFeatureMapModule::close()
{
    fmmanager->close();
    delete fmmanager;
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vFeatureMapModule::updateModule()
{
    return true;
}

/**********************************************************/
double vFeatureMapModule::getPeriod()
{
    return 1;
}
/**********************************************************/
bool vFeatureMapModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply) {
    std::string helpMessage =  std::string(getName().c_str()) +
    " commands are: \n" +
    "help \n" +
    "quit \n";
    
    reply.clear();
    
    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString()=="help") {
        std::cout << helpMessage;
        reply.addString("ok");
    }
    return true;
}

/******************************************************************************/
//vFeatureMapManager
/******************************************************************************/
vFeatureMapManager::vFeatureMapManager(int retSize, int featSize, int thrOn, int thrOff, int constLeak)
{
    this->retSize = retSize;
    this->featSize = featSize;
    this->scaleFactor = retSize/featSize;
    this->thrOn = thrOn;
    this->thrOff = thrOff;
    this->constLeak = constLeak;
    
    devianceFea  = 50;
    devianceFeaSurround  = 20;
    
    //for speed we predefine the memory for some matricies
    //sobelx = yarp::sig::Matrix(sobelsize, sobelsize);
    //sobely = yarp::sig::Matrix(sobelsize, sobelsize);

}
/**********************************************************/
bool vFeatureMapManager::open(const std::string moduleName, bool strictness)
{
    this->strictness = strictness;
    if(strictness) {
        std::cout << "Setting " << moduleName << " to strict" << std::endl;
        this->setStrict();
    }
    this->useCallback();

    // why is the input port treated differently???? both in open and close
    std::string inPortName = "/" + moduleName + "/vBottle:i";
    bool check1 = BufferedPort<emorph::vBottle>::open(inPortName);

    if(strictness) outPort.setStrict();
    std::string outPortName = "/" + moduleName + "/vBottle:o";
    bool check2 = outPort.open(outPortName);

    outPortName = "/" + moduleName + "/feaMapLeftOn:o";
    bool check3 = outFeaLeftOnPort.open(outPortName);

    outPortName = "/" + moduleName + "/feaMapLeftOff:o";
    bool check4 = outFeaLeftOffPort.open(outPortName);
    
    outPortName = "/" + moduleName + "/feaMapRightOn:o";
    bool check5 = true;//outFeaRightOnPort.open(outPortName);
    
    outPortName = "/" + moduleName + "/feaMapRightOff:o";
    bool check6 = true;//outFeaRightOffPort.open(outPortName);

    std::cout << "opened ports: " << std::endl << "vBottle:i " << check1  << std::endl << "vBottle:o "<< check2  << std::endl<< "/feaMapLeftOn:o " << check3  << std::endl<< "/feaMapLeftOff:o " << check4  << std::endl << "/feaMapRightOn:o " << check5  << std::endl << "/feaMapRightOff:o " << check6 << std::endl;
    
    
    /* from Rea's code: create and initialise LUT to -1 */
    std::cout << "allocating memory for the LUT " << std::endl;
    lut = new int[retSize * retSize * 5]; // number of feature maps = 5
    // ---- but if there's one eventFeature Extractor for each feature map, then the lut should only be 128x128 ----- //
    std::cout << "initialising memory for the LUT " << std::endl;
    
    for(int i = 0; i < retSize * retSize * 5; i++) {
        lut[i] = -1;
        //std::cout << "i: " << i << " lut: "<< lut[i] << std::endl;
        
    }
    printf("successfully initialised memory for LUT \n");

    /* opening the file of the mapping and creating the LUT */
    
    fout        = fopen ("lut.txt","w+");
    //fdebug      = fopen ("./eventFeatureExtractor.dumpSet.txt","w+");
    
    pFile       = fopen (mapURL.c_str(),"rb");
    if (pFile == NULL) {
        printf("file of mapping was not found. The module terminates \n");
        return false;
    }
    else {
        long lSize;
        size_t result;
        // obtain file size:
        fseek (pFile , 0 , SEEK_END);
        lSize = ftell (pFile);
        printf("dimension of the file %lu \n", lSize);
        rewind(pFile);
        // saving into the buffer
        char * buffer;
        buffer = (char*) malloc (sizeof(char) * lSize);

        printf("The file was correctly opened \n");
        result = fread (buffer,1,lSize,pFile);
        printf(" Saved the data of the file into the buffer lSize:%lu \n", lSize);
        // checking the content of the buffer
        long word = 0;
        long input = -1, output = -1;
        short x, y;
        //int countMap = 0;
        
        //lSize
        for (int i = 0; i < lSize; i++) {
            
            int value = convertChar2Dec(*buffer);
            //looking for EOL
            if(*buffer == 10) {
                //sac words
                if(word < 500000) {
                    word -= 262144;
                }
                else if(word < 1000000) {
                    word -= 524288;
                }
                else {
                    word -= 1114112;
                }
                
                x = word & 0x001F;
                y = word >> 5;
                
                //std::cout << "sac output: " << word << " ---> " << x << ", " << y << std::endl;
                output = y * 32 + x;
                word = 0;
            }
            //looking for space
            else if(*buffer == 32)  {
                //angel words
                if(word < 500000) {
                    word -= 262144;
                }
                else if(word < 1000000) {
                    word -= 524288;
                }
                else {
                    word -= 1114112;
                }
                
                x = (word & X_MASK) >> X_SHIFT;
                y = (word & Y_MASK) >> Y_SHIFT;
                //std::cout << "angel input: " << word << " ---> " << x << ", " << y << std::endl;
                
                input = y * 128 + x;
                word = 0;
            }
            else{
                //std::cout << "other: " << value << " ---> " << word << std::endl;
                word = word << 4;
                word += value;
            }
            buffer++;
            if((input != -1) && (output!=-1)) {
                
                int inputy  = input / 128;
                int inputx  = input - inputy * 128;
                int outputy = output / 32;
                int outputx = output - outputy * 32;
                
                bool continueSaving = true;
                
                // any input coordinate can point up to 5 output coordinates
                int i = 0;
                while(continueSaving) {
                    if(lut[input + i * retSize * retSize] != -1) {
                        if(lut[input + i * retSize * retSize ]!= output) {
                            i++;
                            if (i>= 5) {
                                continueSaving = false;
                            }
                        }
                        else {
                            continueSaving = false;
                        }
                    }
                    else {
                        //saving
                        lut[input + i * retSize * retSize] = output;
                        //printf("lut : %ld-->%ld (%d) \n", input, output, i);
                        fprintf(fout," %ld %ld > %d %d > %d %d   \n",input, output, inputy, inputx,  outputy, outputx);
                        input  = -1;
                        output = -1;
                        continueSaving = false;
//                        countMap++;
                    }
                } //end of while
            }
        }
//        printf("counted the number of mapping %d \n", countMap);
        
    }

    //    leftInputImage         = new ImageOf<PixelMono>;
    //leftOutputImage        = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
    
    //leftFeaOutputImage     = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
    leftFeaOutputImageOn   = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
    leftFeaOutputImageOff  = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
    rightFeaOutputImageOn   = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
    rightFeaOutputImageOff  = new yarp::sig::ImageOf<yarp::sig::PixelMono>;

    
    //    leftInputImage       ->resize(retSize,retSize);
    //leftOutputImage      ->resize(retSize,retSize);

    //leftFeaOutputImage   ->resize(featSize,featSize);
    leftFeaOutputImageOn ->resize(featSize,featSize);
    leftFeaOutputImageOff->resize(featSize,featSize);
    rightFeaOutputImageOn ->resize(featSize,featSize);
    rightFeaOutputImageOff->resize(featSize,featSize);
    
    //    int padding = leftInputImage->getPadding();
    //int padding = leftOutputImage->getPadding(); // check what does padding do!!!!!! it was from input image applied to output image (they have same size though)
    
    //initialisation of the memory image
    //   unsigned char* pLeft     = leftInputImage->getRawImage();
    //unsigned char* pLeftOut  = leftOutputImage->getRawImage();
 
    // assign 127 to all the location in image plane
    //int rowsize = leftOutputImage->getRowSize();
    /*
    for(int r = 0 ; r < retSize ; r++){
        for(int c = 0 ; c < retSize ; c++){
            //            *pLeft  = 127; pLeft++;
            *pLeftOut = 127; pLeftOut++;
            //            *pRightOut = 127; pRightOut++;
        }
        //        pLeft +=  padding;
        pLeftOut += padding;
        //       pRightOut += padding;
    } */
    
    // the feature maps are defined in the interval [0,255]. 127 value is baseline
    //unsigned char* pFeaOut     = leftFeaOutputImage->getRawImage();
    //    unsigned char* pFeaRightOut    = rightFeaOutputImage->getRawImage();
    //memset(pFeaOut,     127, featSize * featSize * sizeof(unsigned char));
    //    memset(pFeaRightOut,    127, featSize * featSize * sizeof(unsigned char));
    
    // the feature maps are definde in the interval [0,255]. 0 value is the baseline
    unsigned char* pFeaLeftOutOn   = leftFeaOutputImageOn->getRawImage();
    unsigned char* pFeaLeftOutOff  = leftFeaOutputImageOff->getRawImage();
    memset(pFeaLeftOutOn,   127, featSize * featSize * sizeof(unsigned char));
    memset(pFeaLeftOutOff,  127, featSize * featSize * sizeof(unsigned char));
    unsigned char* pFeaRightOutOn   = leftFeaOutputImageOn->getRawImage();
    unsigned char* pFeaRightOutOff  = leftFeaOutputImageOff->getRawImage();
    memset(pFeaRightOutOn,   127, featSize * featSize * sizeof(unsigned char));
    memset(pFeaRightOutOff,  127, featSize * featSize * sizeof(unsigned char));
    
    //rowSize  = leftOutputImage->getRowSize();
    rowSizeFea  = leftFeaOutputImageOn->getRowSize();

    
    std::cout << "initialisation correctly ended" << std::endl;
    
    return check1 && check2 && check3 && check4 && check5 && check6;

}

/**********************************************************/
void vFeatureMapManager::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();
    outFeaLeftOnPort.close();
    outFeaLeftOffPort.close();
    outFeaRightOnPort.close();
    outFeaRightOffPort.close();
    
    free(leftFeaOutputImageOn);
    free(leftFeaOutputImageOff);
    free(rightFeaOutputImageOn);
    free(rightFeaOutputImageOff);

    /* closing the file */
    if(lut!=NULL) {
        delete[] lut;
    }
    
    fclose (pFile);
    fclose (fout);

}

/**********************************************************/
void vFeatureMapManager::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();
    outFeaLeftOnPort.interrupt();
    outFeaLeftOffPort.interrupt();
    outFeaRightOnPort.interrupt();
    outFeaRightOffPort.interrupt();
}

/**********************************************************/
void vFeatureMapManager::onRead(emorph::vBottle &bot)
{
    //int detectedCorners = 0;

    /*prepare output vBottle with AEs */
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle.clear();
    yarp::os::Stamp st;
    this->getEnvelope(st); outPort.setEnvelope(st);

    /* get the event queue in the vBottle bot */
    emorph::vQueue q = bot.get<emorph::AddressEvent>();
    q.sort(true);

    for(emorph::vQueue::iterator qi = q.begin(); qi != q.end(); qi++)
    {
        emorph::AddressEvent *aep = (*qi)->getAs<emorph::AddressEvent>();
        if(!aep) continue;
        
        updateFeatureMap(aep, &outBottle);
        
    }
    
    
    // --- constant leak of feature maps --- //
    unsigned char* pFeaOn, * pFeaOff;
    
    pFeaOn  = leftFeaOutputImageOn->getRawImage();
    pFeaOff = leftFeaOutputImageOff->getRawImage();
    leakFeatureMap(pFeaOn);
    leakFeatureMap(pFeaOff);
    
    pFeaOn  = rightFeaOutputImageOn->getRawImage();
    pFeaOff = rightFeaOutputImageOff->getRawImage();
    leakFeatureMap(pFeaOn);
    leakFeatureMap(pFeaOff);

    // --- writing vBottle on buffered output port
    if (strictness) outPort.writeStrict();
    else outPort.write();
    
    // --- writing images of feature maps on output port
    if(outFeaLeftOnPort.getOutputCount()) {
        std::cout << "sending left images"<< std::endl;
        outFeaLeftOnPort.prepare()  = *leftFeaOutputImageOn;
        outFeaLeftOnPort.write();
    }
    if(outFeaLeftOffPort.getOutputCount()) {
        std::cout << "sending left images"<< std::endl;
        outFeaLeftOffPort.prepare()  = *leftFeaOutputImageOff;
        outFeaLeftOffPort.write();
    }
    if(outFeaRightOnPort.getOutputCount()) {
        outFeaRightOnPort.prepare() = *rightFeaOutputImageOn;
        outFeaRightOnPort.write();
    }
    if(outFeaRightOffPort.getOutputCount()) {
        outFeaRightOffPort.prepare() = *rightFeaOutputImageOff;
        outFeaRightOffPort.write();
    }
}

/**********************************************************/
void vFeatureMapManager::updateFeatureMap(emorph::AddressEvent *aep, emorph::vBottle *outBottle)
{
    // unmask event: get x, y, pol, channel
    int cartY     = aep->getX();
    int cartX     = aep->getY();
    int camera    = aep->getChannel();
    int polarity  = aep->getPolarity();
    
    unsigned char* pFeaOn, * pFeaOff;
    
    /**********************************************************************/
    // can I move this outside the for loop?
    if(camera == 0)
    {   // LEFT CAMERA
        //unsigned char* pFea    = leftFeaOutputImage->getRawImage();
        std::cout << "left event unmasked" << std::endl;

        pFeaOn  = leftFeaOutputImageOn->getRawImage();
        pFeaOff = leftFeaOutputImageOff->getRawImage();
    } else if (camera == 1)
    {   //right camera
        std::cout << "right event unmasked" << std::endl;

        pFeaOn  = rightFeaOutputImageOn->getRawImage();
        pFeaOff = rightFeaOutputImageOff->getRawImage();
    }
    /**********************************************************************/
    
    for (int i = 0; i< 5 ; i++) {
        std::cout << "scan LUT" << std::endl;

        int pos = lut[i * retSize * retSize +  cartY * retSize + cartX];
        if(pos == -1) {// EVENT NOT MAPPED in the LUT :
            std::cout << "event not in LUT" << std::endl;

            // if the event is not mapped this reduces the response of the receptive field
            
            // position of the event in the feature map space
            int xevent = cartX / scaleFactor;
            int yevent = (retSize - cartY) / scaleFactor;
            int posFeaImage     = yevent * rowSizeFea + xevent ;
            
            // depressing the feature map in the location
            
            if(polarity>0){
                // positive event in the surround of center-on
                if(pFeaOn[posFeaImage] >=  devianceFeaSurround) {
                    pFeaOn[posFeaImage]  -= devianceFeaSurround;
                }
                else {
                    pFeaOn[posFeaImage]  = 0;
                }
                // positive event in the surround of center-off
                if(pFeaOff[posFeaImage] <= 255 - devianceFeaSurround) {
                    pFeaOff[posFeaImage] += devianceFeaSurround;
                }
                else {
                    pFeaOff[posFeaImage]  = 255;
                }
            }
            else{
                // negative event in the surround of center-on
                if(pFeaOn[posFeaImage] <= 255 - devianceFeaSurround) {
                    pFeaOn[posFeaImage]  += devianceFeaSurround;
                }
                else {
                    pFeaOn[posFeaImage]  = 255;
                }
                // negative event in the surround of center-off
                if(pFeaOff[posFeaImage] >=  devianceFeaSurround) {
                    pFeaOff[posFeaImage] -= devianceFeaSurround;
                }
                else {
                    pFeaOff[posFeaImage]  = 0;
                }
            }
        }
        else {
            std::cout << "event in LUT" << std::endl;

            // position of the event in the feature map space
            int yevent_tmp      = pos / featSize;
            int yevent          = featSize - yevent_tmp;
            int xevent_tmp      = pos - yevent_tmp * featSize;
            int xevent          = xevent_tmp;
            int polevent        = polarity;
            //int cameraevent     = 1;
            //unsigned long blob  = 0;
            int posFeaImage     = yevent * rowSizeFea + xevent ;
            // ----------------------------------------------------
            
            /**********************************************************************/
            //converting the event in a blob // chiara?????????????????????????????
            //unmask_events.maskEvent(xevent, yevent, polevent, cameraevent, blob);
            /**********************************************************************/
            
            //printf("Given pos %d extracts x=%d and y=%d pol=%d blob=%08x evPU = %08x \n", pos, xevent, yevent, pol, (unsigned int) blob, (unsigned int)ts);
            
            //------------  updating the feature map (image) -------------------
            if(polevent > 0) {
                // positive event in the center of center-on cell
                if(pFeaOn[posFeaImage] <= 255 - devianceFea) {
                    pFeaOn[posFeaImage] += devianceFea ;
                }
                else {
                    pFeaOn[posFeaImage] = 255;
                }
                
                // positive event in the center of center-off cell
                if(pFeaOff[posFeaImage] >= devianceFea) {
                    pFeaOff[posFeaImage] -= devianceFea ;
                }
                else {
                    pFeaOff[posFeaImage] = 0;
                }
                
            }
            else {
                // negative event in the center of center-on cell
                if(pFeaOn[posFeaImage] >= devianceFea) {
                    pFeaOn[posFeaImage] -= devianceFea ;
                }
                else {
                    pFeaOn[posFeaImage] = 0;
                }
                
                // negative event in the center of center-off cell
                if(pFeaOff[posFeaImage] <= 255 - devianceFea) {
                    pFeaOff[posFeaImage] += devianceFea ;
                }
                else {
                    pFeaOff[posFeaImage] = 255;
                }
            }
            
            //-------------------- adding the event to the output vBottle if it passes thresholds -------//
            if(pFeaOn[posFeaImage] > thrOn) {
                std::cout << "adding On event to vBottle" << std::endl;

                emorph::AddressEvent ae = *aep;
                ae.setPolarity(1);
                ae.setX(xevent);
                ae.setY(yevent);
                
                outBottle->addEvent(ae);
                
            }
            else if(pFeaOff[posFeaImage] < thrOff) {
                std::cout << "adding Off event to vBottle" << std::endl;
                emorph::AddressEvent ae = *aep;
                ae.setPolarity(0);
                ae.setX(xevent);
                ae.setY(yevent);
                
                outBottle->addEvent(ae);
                
            }
        }
    }
}

void vFeatureMapManager::leakFeatureMap(unsigned char* pFea)
{
    // in the original code, the leak of the On and Off feature maps was the same, however, it looks like the Off feature map is inverted with respect to the On feature map (e.g. at rest Off feat map is = 255, incresing saliency goes towards 0) -- in this case we ahve to setup a different leak
    // we should put a time dependent leak.....
    std::cout << "feature map leak" << std::endl;
    for(int row =0; row < featSize; row++)
    {
        for (int col = 0; col< featSize; col++)
        {
            if(*pFea >= constLeak)
            {
                *pFea -= constLeak;
            }
            else
            {
                *pFea = 0;
            }
        }
        pFea++;
    }
}

//empty line to make gcc happy
