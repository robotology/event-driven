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
    yarpImage.resize(publishWidth, publishHeight);

    //create the output openCV-style image and point it to the yarp image
    cv::Mat publishImage((IplImage *) yarpImage.getIplImage(), false);

    //publish the raw image to the publish container image  (yarp-style
    //shoule be update also
    cv::resize(rawImage, publishImage, publishImage.size());

    cv::imshow("CV DEBUG WINDOW 1", publishImage);
    cv::waitKey();

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
///
/// \brief eFramerProcess::eFramerProcess
///
eFramerProcess::eFramerProcess()
{

    yarpImage = imgWriter.prepare(); //check this works as it is a return by ref
    yarpImage.resize(256, 256);
    eImage = new eAddressFrame(128, 128);
    eImage->setPublishSize(256, 256);

}

eFramerProcess::~eFramerProcess()
{

}

void eFramerProcess::onRead(emorph::eBottle &datum)
{
    emorph::eEventQueue q;
    emorph::eEventQueue::iterator qi;
    datum.getAllSorted(q);

    /*
     * the problem with the current method is that it will hang on the last
     * frame if no new events are read as frame publoishing is triggered by
     * events.
     */

    for(qi = q.begin(); qi != q.end(); qi++) {

        //publish at a set rate
        if((*qi)->getStamp() > current_period) {
            yarpImage = eImage->publish();
            imgWriter.write();
            eImage->clear();
            current_period = (*qi)->getStamp() + period;
        }

        //continue adding all events
        eImage->addEvent(**qi);
    }


}


/*////////////////////////////////////////////////////////////////////////////*/
//eFramerModule
/*////////////////////////////////////////////////////////////////////////////*/
bool eFramerModule::configure(yarp::os::ResourceFinder &rf)
{
    //read in config file

    //set up robot etc.

    //set up specific parameters
    eframer.setPeriodMS(500);
    eframer.setWindowSize(256, 256);
    //set the port name for eframer
    eframer.open();


}

bool eFramerModule::interruptModule()
{
    eframer.interrupt();
    return true;
}

bool eFramerModule::close()
{
    eframer.close();
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
    return true;
}

} //namespace emorph
