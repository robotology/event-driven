#include "iCub/eFramer.h"

namespace emorph {
/*////////////////////////////////////////////////////////////////////////////*/
//eFrame
/*////////////////////////////////////////////////////////////////////////////*/
///
/// \brief eFrame::eFrame
/// \param retinaWidth
/// \param retinaHeight
/// \param windowWidth
/// \param windowHeight
///
///
eFrame::eFrame(int retinaWidth, int retinaHeight)
{

    publishWidth = retinaWidth;
    publishHeight = retinaHeight;

    rawImage = cv::Mat(retinaHeight, retinaWidth, CV_8U);
    clear();

}

void eFrame::setPublishSize(int width, int height)
{
    publishWidth = width;
    publishHeight = height;
}


///
/// \brief eFrame::publish
/// \return
///
yarp::sig::ImageOf<yarp::sig::PixelMono> eFrame::publish() {

    //create the output yarp-style image
    yarp::sig::ImageOf<yarp::sig::PixelMono> yarpImage;
    //yarpImage.resize(publishWidth, publishHeight);

    //create the output openCV-style image and point it to the yarp image
    //cv::Mat publishImage((IplImage *) yarpImage.getIplImage(), false);

    //publish the raw image to the publish container image  (yarp-style
    //shoule be update also
    cv::Mat publishImage(publishWidth, publishHeight, CV_8U);
    cv::resize(rawImage, publishImage, publishImage.size());

    cv::imshow("CV DEBUG WINDOW 1", publishImage);
    cv::waitKey(1);

    //return our published yarp-style image
    return yarpImage;

}

///
/// \brief eFrame::clear
///
void eFrame::clear()
{
    rawImage.setTo(0);
}

/*////////////////////////////////////////////////////////////////////////////*/
//eAddressFrame
/*////////////////////////////////////////////////////////////////////////////*/

///
/// \brief eAddressFrame::addEvent
/// \param event
///
void eAddressFrame::addEvent(emorph::eEvent &event)
{

    emorph::AddressEvent *aep = event.getAs<emorph::AddressEvent>();
    if(aep) {
        rawImage.at<char>(aep->getX(), aep->getY()) = 255;
    }

}

/*////////////////////////////////////////////////////////////////////////////*/
//eFramerProcess
/*////////////////////////////////////////////////////////////////////////////*/
eFramerProcess::~eFramerProcess()
{
    delete eImage;
}

///
/// \brief eFramerProcess::eFramerProcess
///
eFramerProcess::eFramerProcess( const std::string &moduleName )
{

    portName = moduleName;
    eImage = 0;

    current_period = 0;
    period = 100;

    //yarpImage = imgWriter.prepare(); //check this works as it is a return by ref

    //yarpImage.resize(256, 256);
    eImage = new eAddressFrame(128, 128);
    eImage->setPublishSize(256, 256);

}

bool eFramerProcess::open()
{
    this->useCallback();

    std::cout << "Opening BufferedPort::eFramerProcess" << std::endl;
    std::string name = "/" + portName + "/eBottle:i";
    bool r = BufferedPort<emorph::eBottle>::open(name.c_str());

    //r = r && imgWriter.open(std::string(portName + "yarpImage:o").c_str());

    return r;

}

void eFramerProcess::close()
{
    std::cout << "Closing BufferedPort::eFramerProcess" << std::endl;
    BufferedPort<emorph::eBottle>::close();
}



void eFramerProcess::interrupt()
{
   BufferedPort<eBottle>::interrupt();
}

void eFramerProcess::setWindowSize(int width, int height)
{
    if (eImage) {
        eImage->setPublishSize(width, height);
    }
}

void eFramerProcess::onRead(emorph::eBottle &incoming)
{




    emorph::eEventQueue q;
    emorph::eEventQueue::iterator qi;
    incoming.getAllSorted(q);

    int lPeriod;
    int lcPeriod;
    int lStamp;

    /*
     * the problem with the current method is that it will hang on the last
     * frame if no new events are read as frame publoishing is triggered by
     * events.
     */

    //std::cout << "New Bottle: (" << q.size() <<  ")" << ": " <<
    //             q.front()->getStamp() << std::endl;

    if(!eImage) return;
    for(qi = q.begin(); qi != q.end(); qi++) {

        lPeriod = period;
        lcPeriod = current_period;
        lStamp = (*qi)->getStamp();

        if((*qi)->getStamp() > current_period + 2 * period)
        {
           std::cout << "Problem here: ";


        }


        //publish at a set rate
        if((*qi)->getStamp() > current_period) {

            yarpImage = eImage->publish();
            //imgWriter.write();
            eImage->clear();
            //current_period = (*qi)->getStamp() + period;
            current_period = current_period + period;
        }

        //continue adding all events
        eImage->addEvent(**qi);
    }
    std::cout << lStamp << " " << lcPeriod << std::endl;


}


/*////////////////////////////////////////////////////////////////////////////*/
//eFramerModule
/*////////////////////////////////////////////////////////////////////////////*/
bool eFramerModule::configure(yarp::os::ResourceFinder &rf)
{
    //read in config file
    moduleName = "eFramerModule"; //update this from rf

    setName(moduleName.c_str());

    //set up robot etc.

    //set up specific parameters
    eframer = new eFramerProcess(moduleName);

    eframer->open();

    eframer->setPeriodMS(10000);  //update this from rf
    eframer->setWindowSize(256, 256);  //update this from rf


    return true;

}

bool eFramerModule::interruptModule()
{
    eframer->interrupt();
    return true;
}

bool eFramerModule::close()
{
    eframer->close();
    delete eframer;

    return true;
}

bool eFramerModule::respond(const yarp::os::Bottle& command,
                            yarp::os::Bottle& reply)
{
    //add respond messages
    return true;
}

bool eFramerModule::updateModule()
{
    //perhaps do a publish here depending on type of image out.
    eframer->
    return true;
}

double eFramerModule::getPeriod()
{
    return 1.0;
}

} //namespace emorph
