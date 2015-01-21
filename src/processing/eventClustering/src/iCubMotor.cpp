//##############################################################################################################################################################################################################//
//Aquila - An Open-Source GPU-Accelerated Toolkit for Cognitive and Neuro-Robotics Research																														//
//																																																				//
//Copyright (c) <2012>, Anthony Morse                                                                                                                                                                                //
//All rights reserved.																																															//
//																																																				//
//Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:																//
//																																																				//
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.																				//
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.	//
//																																																				//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR	//
//A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT	//
//LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR	//
//TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.																//
//                                                                                                                                                                                                              //
//The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted                                                                                  //
//as representing official policies,either expressed or implied, of the FreeBSD Project.                                                                                                                        //
//##############################################################################################################################################################################################################//

#include <string>
#include "iCubMotor.h"
#include <yarp/os/Network.h>

YARP_DECLARE_DEVICES(icubmod)

namespace aquilacubmotor
{
/*!
 * \brief Default constructor. Sets the defaults that would have been
    in standard Aquila config.ini for iCubMotor
 */
ICubMotor::ICubMotor()
{

  simulationMode = true;
  cartesianMode = false;

}

/*!
 * \brief Alternative constructor to set parameters on creation
 */
ICubMotor::ICubMotor(bool simMode, bool cartMode)
{

  simulationMode = simMode;
  cartesianMode = cartMode;

  

 // start the cartesian controller thread

 if(cartesianMode == true){

    //cartMod.setSimulationMode(simMode);

    // we need to initialize the drivers list 
    YARP_REGISTER_DEVICES(icubmod)
    //cartMod.runModule(rf);
    thr=new CubCartThread(CTRL_THREAD_PER,simMode);
    thr->start();
    yarp::os::Time::delay(2);

 }

}

/*!
 * \brief Destructor.
 */
ICubMotor::~ICubMotor()
{
    clean();
}

/*!
 * \brief Opens image ports.
 */
bool ICubMotor::openPorts(std::string portPrefix)
{
    //open your ports here
    pointPortName = portPrefix + "/point:o";
    return (pointPort.open(pointPortName.c_str()));

}
/*!
 * \brief Connects ports.
 */
void ICubMotor::connectPorts()
{

    // if we have enabled cartMode, check that
    // the /cart port is actually available
    // and if it is connect /point:o to /cart

    // if it's not available give a warning
    // message to user and disable cart mode

    if (cartesianMode == true){
      NetworkBase netbase;
      ConstString portName = "/cart";

      printf("Checking port %s\n", portName.c_str());

      if (netbase.exists(portName, true) == true){
         printf("/cart port exists\n");
           Network::connect(pointPortName.c_str(), "/cart");

      } else {
         printf("WARNING: /cart port does not exist, disabling cartesian mode\n");
         cartesianMode = false;
      }
    }


}

/*!
 * \brief Disconnects ports.
 */
void ICubMotor::disconnectPorts()
{

    if (cartesianMode == true){

       Network::disconnect(pointPortName.c_str(), "/cart");

    }
}


/*!
 * \brief Thread loop.
 */
void ICubMotor::run()
{
    running = true;

    head = new SimplePart;
    head->initialised = false;
    head->num_clients = 0;

/*    
	leftLeg = new SimplePart;
    leftLeg->initialised = false;
    leftLeg->num_clients = 0;
    rightLeg = new SimplePart;
    rightLeg->initialised = false;
    rightLeg->num_clients = 0;
    torso = new SimplePart;
    torso->initialised = false;
    torso->num_clients = 0;
    leftArm = new SimplePart;
    leftArm->initialised = false;
    leftArm->num_clients = 0;
    rightArm = new SimplePart;
    rightArm->initialised = false;
    rightArm->num_clients = 0;
*/
    initHead(simulationMode);
//    initleftLeg(simulationMode);
//    initrightLeg(simulationMode);
//    initTorso(simulationMode);
    if (cartesianMode == true){
       initImpedance(simulationMode);
    }

    yarp::os::Time::delay(1);

    double speedX, speedY;
    double delayForMovement;
    char message[100];
    moving = false;
    int torsoCount = 0;
    bool armHome = false;
    readyToMove = true;
    cartMove = false;
    tracking = false;
    pickUp = false;
    trackingMotion = false;
    standing = true;

    while(running)
    {
        // If we are not in the middle of a cartesian move
        // get ready for other movements
        if(!cartMove)
        {
 
            tracking = false;
            head->pos->getRefSpeed(0, &speedY);
            head->pos->getRefSpeed(2, &speedX);
            delayForMovement = ((max(speedX, speedY) / 100) - 0.05) * 1.5;
            yarp::os::Time::delay(delayForMovement);

            if(moving == true)
            {
                readyToMove = false;
                moving = false;
            }
            else
            {
                readyToMove = true;
            }
            // if we are enabled for a cartesian move allow arms and torso
            // to move a bit 
            if (cartesianMode == true){
              if(torsoCount > 10)
              {
                  moveTorso();
                  torsoCount = 0;
                  moveArms();
                  armHome = true;
              }
              else torsoCount ++;
            }

        }
        else
        { 
            // we are doing a cartesian move so inhibit other movements
            readyToMove = false;
            headTracking();
            yarp::os::Time::delay(0.1);
        }
    }

    closeHead();
    closeLeftLeg();
    closeRightLeg();
    closeTorso();

    if (cartesianMode == true){
       closeImpedance();
    }

}

void ICubMotor::point()
{
    cartMove = true;            //block other movement

    yarp::os::Time::delay(1);

    Bottle &b = pointPort.prepare();
    b.clear();
    int headPos = head->encoders->data()[2] - head->encoders->data()[4];
    double headPosY = double(head->encoders->data()[0] + head->encoders->data()[3]) / 2;

    headPosY = headPosY -50.0;

    double distance = -0.50 - ((double(headPosY) + 16) * 0.02);
    if(distance > -0.3) distance = -0.3;
    if(distance < -0.5) distance = -0.5;

    if(headPos > -3)
    {
       b.addString("left");        
       b.addDouble(distance);  //distance in front of iCub
       b.addDouble((float)-headPos/100); //x axis
       b.addDouble(0.00);  //height of table
       pointPort.write();
       if(distance > -0.4)
       {
          pointLeftHand();
       }
    }
    else
    {
       b.addString("right");
       b.addDouble(distance);  //distance in front of iCub
       b.addDouble((float)-(headPos+10)/100); //x axis
       b.addDouble(0.00);  //height of table
       pointPort.write();
       if(distance > -0.4)
       {
          pointRightHand();
       }

    }

    yarp::os::Time::delay(2);

    armHome = false;
    Bottle &c = pointPort.prepare();
    c.clear();
    c.addString("return arms");
    pointPort.write();

    moveTorso();
    moveArms();

    yarp::os::Time::delay(2);

    cartMove = false;
}
void ICubMotor::reachPickUp()
{
    pickUp = true;
    reach();
}

void ICubMotor::reach()
{
    cartMove = true;            //block other movement

    yarp::os::Time::delay(1);

    Bottle &b = pointPort.prepare();
    b.clear();
    int headPos = head->encoders->data()[2] - head->encoders->data()[4];
    double headPosY = double(head->encoders->data()[0] + head->encoders->data()[3]) / 2;

    double distance = -0.50 - ((double(headPosY) + 16) * 0.02);
    if(distance > -0.3) distance = -0.3;
    if(distance < -0.5) distance = -0.5;

    if(headPos > -3)
    {
       b.addString("left");        
       b.addDouble(distance);  //distance in front of iCub
       b.addDouble((float)-headPos/100); //x axis
       b.addDouble(0.00);  //height of table
       pointPort.write();
       if(pickUp)
       {
           if(distance > -0.4)
           {
               shutLeftHand(distance, headPos);
           }
           pickUp = false;
       }
    }
    else
    {
       b.addString("right");
       b.addDouble(distance);  //distance in front of iCub
       b.addDouble((float)-(headPos+10)/100); //x axis
       b.addDouble(0.00);  //height of table
       pointPort.write();
       if(pickUp)
       {
           if(distance > -0.4)
           {
               shutRightHand(distance, headPos);
           }
           pickUp = false;
       }
    }

    yarp::os::Time::delay(5);

    armHome = false;
    Bottle &c = pointPort.prepare();
    c.clear();
    c.addString("return arms");
    pointPort.write();

    moveTorso();
    moveArms();

    yarp::os::Time::delay(5);

    cartMove = false;
}

/**
*@brief     makes small random movement in the Torso, also returns iCub to a neutral position (following a cartesian reach)
*/
void ICubMotor::moveTorso()
{
    torso->pos->setRefSpeed(0,10);
    torso->pos->setRefSpeed(1,2);
    torso->pos->setRefSpeed(2,10);
    torso->command->data()[0] = 0; // + ((int)rand()%3 -1);
    torso->command->data()[1] = 0 + ((int)rand()%3 -1);
    torso->command->data()[2] = 8; // + ((int)rand()%3 -1);
    torso->pos->positionMove(torso->command->data());
}

/**
*@brief     makes small random movement in the arms, also returns iCub to a neutral position (following a cartesian reach)
*/
void ICubMotor::moveArms()
{
    leftArm->command->data()[0] = -15;
    leftArm->command->data()[1] = 40;
    leftArm->command->data()[2] = -10;
    leftArm->command->data()[3] = 80;
    leftArm->command->data()[4] = 0;
    leftArm->command->data()[5] = 0;
    leftArm->command->data()[6] = 0;
    leftArm->command->data()[7] = 38;
    leftArm->command->data()[8] = 10;
    leftArm->command->data()[9] = 1;
    leftArm->command->data()[10] = 0;
    leftArm->command->data()[11] = 3;
    leftArm->command->data()[12] = 0;
    leftArm->command->data()[13] = 0;
    leftArm->command->data()[14] = 0;
    leftArm->command->data()[15] = 0;

    rightArm->command->data()[0] = -15;
    rightArm->command->data()[1] = 40;
    rightArm->command->data()[2] = -10;
    rightArm->command->data()[3] = 70;
    rightArm->command->data()[4] = 0;
    rightArm->command->data()[5] = 0;
    rightArm->command->data()[6] = 0;
    rightArm->command->data()[7] = 38;
    rightArm->command->data()[8] = 10;
    rightArm->command->data()[9] = 1;
    rightArm->command->data()[10] = 0;
    rightArm->command->data()[11] = 3;
    rightArm->command->data()[12] = 0;
    rightArm->command->data()[13] = 0;
    rightArm->command->data()[14] = 0;
    rightArm->command->data()[15] = 0;

    //set ref speeds for hand
    for(int i=7; i<15; i++)
    {
        leftArm->pos->setRefSpeed(i, 50);
        rightArm->pos->setRefSpeed(i, 50);
    }

    leftArm->pos->positionMove(leftArm->command->data());
    rightArm->pos->positionMove(rightArm->command->data());
}

/**
*@brief     shut the left hand to pick something up
*/
void ICubMotor::shutLeftHand(double distance, double headPos)
{
    //***********************************************lower the left hand
    yarp::os::Time::delay(5);
    Bottle &c = pointPort.prepare();
    c.clear();
    c.addString("left");
    c.addDouble(distance);  //distance in front of iCub
    c.addDouble((float)-headPos/100); //x axis
    c.addDouble(-0.09);  //height of table

    pointPort.write();

    //************************************************close the hand
    leftArm->encs->getEncoders(leftArm->encoders->data());
    //set ref speeds for hand
    for(int i=7; i<14; i++)
    {
        leftArm->pos->setRefSpeed(i, 50);
    }
    leftArm->pos->setRefSpeed(15, 80);
    //set position for closed hand
    for(int i=0; i<15; i++)
    {
        leftArm->command->data()[i] = leftArm->encoders->data()[i];
    }
    //close the hand
    leftArm->command->data()[7] = 27;
    leftArm->command->data()[8] = 60;
    leftArm->command->data()[9] = 30;
    leftArm->command->data()[10] = 30;
    leftArm->command->data()[11] = 36;
    leftArm->command->data()[12] = 82;
    leftArm->command->data()[13] = 36;
    leftArm->command->data()[14] = 82;
    leftArm->command->data()[15] = 144;

    leftArm->pos->positionMove(leftArm->command->data());

    yarp::os::Time::delay(2);

    //*************************************************lift the hand up again
    Bottle &d = pointPort.prepare();
    d.clear();
    d.addString("left");
    d.addDouble(distance);  //distance in front of iCub
    d.addDouble((float)-headPos/100); //x axis
    d.addDouble(0.05);  //height of table

    pointPort.write();
}

/**
*@brief     shut the right hand to pick something up
*/
void ICubMotor::shutRightHand(double distance, double headPos)
{
    //***********************************************lower the right hand
    yarp::os::Time::delay(5);
    Bottle &c = pointPort.prepare();
    c.clear();
    c.addString("right");
    c.addDouble(distance);  //distance in front of iCub
    c.addDouble((float)-(headPos+10)/100); //x axis
    c.addDouble(-0.09);  //height of table

    pointPort.write();

    //************************************************close the hand
    rightArm->encs->getEncoders(rightArm->encoders->data());
    //set ref speeds for hand
    for(int i=7; i<14; i++)
    {
        rightArm->pos->setRefSpeed(i, 50);
    }
    rightArm->pos->setRefSpeed(15, 80);
    //set position for closed hand
    for(int i=0; i<15; i++)
    {
        rightArm->command->data()[i] = rightArm->encoders->data()[i];
    }
    //close the hand
    rightArm->command->data()[7] = 27;
    rightArm->command->data()[8] = 60;
    rightArm->command->data()[9] = 30;
    rightArm->command->data()[10] = 30;
    rightArm->command->data()[11] = 36;
    rightArm->command->data()[12] = 82;
    rightArm->command->data()[13] = 36;
    rightArm->command->data()[14] = 82;
    rightArm->command->data()[15] = 144;

    rightArm->pos->positionMove(rightArm->command->data());

    yarp::os::Time::delay(2);

    //*************************************************lift the hand up again
    Bottle &d = pointPort.prepare();
    d.clear();
    d.addString("right");
    d.addDouble(distance);  //distance in front of iCub
    d.addDouble((float)-(headPos+10)/100); //x axis
    d.addDouble(0.05);  //height of table

    pointPort.write();
}

/**
*@brief     point with the left hand
*/
void ICubMotor::pointLeftHand()
{


    leftArm->encs->getEncoders(leftArm->encoders->data());
    //set ref speeds for hand
    for(int i=7; i<14; i++)
    {
        leftArm->pos->setRefSpeed(i, 50);
    }
    leftArm->pos->setRefSpeed(15, 80);
    //set position for closed hand
    for(int i=0; i<15; i++)
    {
        leftArm->command->data()[i] = leftArm->encoders->data()[i];
    }
    //make pointing movement
    leftArm->command->data()[7] = 27;
    leftArm->command->data()[8] = 60;
    leftArm->command->data()[9] = 50;
    leftArm->command->data()[10] = 50;
    leftArm->command->data()[11] = 2;
    leftArm->command->data()[12] = 30;
    leftArm->command->data()[13] = 50;
    leftArm->command->data()[14] = 90;
    leftArm->command->data()[15] = 177;

    leftArm->pos->positionMove(leftArm->command->data());

    yarp::os::Time::delay(2);

}

/**
*@brief     point with the right hand
*/
void ICubMotor::pointRightHand()
{

    rightArm->encs->getEncoders(rightArm->encoders->data());
    //set ref speeds for hand
    for(int i=7; i<14; i++)
    {
        rightArm->pos->setRefSpeed(i, 50);
    }
    rightArm->pos->setRefSpeed(15, 80);
    //set position for closed hand
    for(int i=0; i<15; i++)
    {
        rightArm->command->data()[i] = rightArm->encoders->data()[i];
    }
    //make pointing movement
    rightArm->command->data()[7] = 27;
    rightArm->command->data()[8] = 60;
    rightArm->command->data()[9] = 50;
    rightArm->command->data()[10] = 50;
    rightArm->command->data()[11] = 2;
    rightArm->command->data()[12] = 30;
    rightArm->command->data()[13] = 50;
    rightArm->command->data()[14] = 90;
    rightArm->command->data()[15] = 177;

    rightArm->pos->positionMove(rightArm->command->data());

    yarp::os::Time::delay(2);

}

/**
*@brief     get the head to track torso movements
*/
void ICubMotor::headTracking()
{

    head->encs->getEncoders(head->encoders->data());
    torso->encs->getEncoders(torso->encoders->data());

    if(tracking)
    {
        head->pos->setRefSpeed(2, 20); //head x

        float torsoMoveX = -(prevTorso[0] - torso->encoders->data()[0]);
        float torsoMoveY = -(prevTorso[2] - torso->encoders->data()[2]);
        float headMoveX = prevHead[2] - head->encoders->data()[2];
        float headMoveY = prevHead[0] - head->encoders->data()[0];

        head->vel->velocityMove(2, (torsoMoveX - headMoveX) * 5 );      //X - axis
        head->vel->velocityMove(0, (torsoMoveY - headMoveY) * 5 );      //Y - axis
    }
    else tracking = true;

    prevTorso[0] = torso->encoders->data()[0];
    prevTorso[2] = torso->encoders->data()[2];
    prevHead[2] = head->encoders->data()[2];
    prevHead[0] = head->encoders->data()[0];
}

/**
*@brief     moves the head to a specified target in image co-ordinates
*@param[in] target_x    - The point in the image where you want to look
*@param[in] target_y    - The point in the image where you want to look
*/
void ICubMotor::moveHead(float target_x, float target_y)
{
    if((readyToMove) && (!cartMove))
    {

        int imageWidth = 128;
        int imageHeight = 128;
        int targetSpeedX, targetSpeedY;

        //modify the position relative to the center of the image
        target_x -= 64; //imageWidth / 2;  // center of image should mean no motion
        target_y -= 70; //imageHeight / 2;

        // Convert the pixel position into the joint angle change requirted to reach that posotion
        // divide by the max value and Multiply by the range of the motor. On the Plymouth robot this is as follows:
        // j0: -40 to 30 (head vertical axis)
        // j1: tilt not used
        // j2: -55 to 55 (head horizontal axis)
        // j3: -35 to 15 (eyes vertical axis)
        // j4: -50 to 52 (eyes horizontal axis)
        // j5: vergence not used, vergence should be fixed for disparity based depth measures
        target_x = (target_x / (imageWidth / 2)) * 50;
        target_y = (target_y / (imageHeight / 2)) * 40;


        //head
        // get the current position
        head->encs->getEncoders(head->encoders->data());
        // save this so we can restore this position
        head->encs->getEncoders(head->initpos->data());
        printf("%f %f\n", head->initpos->data()[0],head->initpos->data()[2]);
        target_x = -target_x + head->encoders->data()[2];	//take into account the current facing direction of the head
        target_y = -target_y + head->encoders->data()[0];
        if(target_x > 30) target_x = 30;
        if(target_x < -30) target_x = -30;
        if(target_y > 20) target_y = 20;
        if(target_y < -20) target_y = -20;

        //eyes
        float target_ey = target_y;//(target_y - 10) * 0.5;
        float target_ex = target_x;// * 0.5; //(target_x - 20) * 1.4;
        if(target_ex > 30) target_ex = 30;
        if(target_ex < -30) target_ex = -30;
        if(target_ey > 17) target_ey = 17;
        if(target_ey < -25) target_ey = -25;

        for(int i=0;i<head->num_joints;i++) head->command->data()[i] = 0.0;

  /*      if(target_x > head->encoders->data()[2]+5 || target_x < head->encoders->data()[2]-5 || target_y > head->encoders->data()[0]+5 || target_y < head->encoders->data()[0]-5)
        {
            head->command->data()[2] = int(target_x);	//move head
            head->command->data()[0] = int(target_y);
            //Set speed of movement according to the distance of the movement (faster for larger moves)
            targetSpeedX = 3*getDifference(int(target_x), int(head->encoders->data()[2]));
            targetSpeedY = 2*getDifference(int(target_y), int(head->encoders->data()[0]));
            if(targetSpeedX > MAX_REAL_HEAD_VELOCITY) targetSpeedX = MAX_REAL_HEAD_VELOCITY;
            if(targetSpeedY > MAX_REAL_HEAD_VELOCITY) targetSpeedY = MAX_REAL_HEAD_VELOCITY;
            head->pos->setRefSpeed(0, targetSpeedY); //head y
            head->pos->setRefSpeed(2, targetSpeedX); //head x
        }
        else
        {
*/
            head->command->data()[2] = target_ex; //head->encoders->data()[2];
            head->command->data()[0] = target_ey; //head->encoders->data()[0];
  //      }


        head->pos->positionMove(head->command->data());
        moving = true;
        readyToMove = false;
        Time::delay(4);
	
        head->command->data()[2] = 0;
        head->command->data()[0] = 0;
        head->pos->positionMove(head->command->data());
        moving = false;
        readyToMove = true;

    } // ready to move?
}


/**
*@brief     moves the eyes to a specified target in image co-ordinates
*@param[in] target_x    - The point in the image where you want to look
*@param[in] target_y    - The point in the image where you want to look
*/
void ICubMotor::moveEyes(float target_x, float target_y)
{
    printf("All right\n");
    if((readyToMove) && (!cartMove))
    {

        int imageWidth = 128;
        int imageHeight = 128;
        int targetSpeedX, targetSpeedY;

        //modify the position relative to the center of the image
        target_x -= imageWidth / 2;  // center of image should mean no motion
        target_y -= 70; //imageHeight / 2;

        // Convert the pixel position into the joint angle change requirted to reach that posotion
        // divide by the max value and Multiply by the range of the motor. On the Plymouth robot this is as follows:
        // j0: -40 to 30 (head vertical axis)
        // j1: tilt not used
        // j2: -55 to 55 (head horizontal axis)
        // j3: -35 to 15 (eyes vertical axis)
        // j4: -50 to 52 (eyes horizontal axis)
        // j5: vergence not used, vergence should be fixed for disparity based depth measures
        target_x = (target_x / (imageWidth / 2)) * 30;
        target_y = (target_y / (imageHeight / 2)) * 26;


        //head
        // get the current position
        head->encs->getEncoders(head->encoders->data());
        // save this so we can restore this position
        head->encs->getEncoders(head->initpos->data());
        printf("%f %f\n", head->initpos->data()[3],head->initpos->data()[4]);
        target_x = -target_x + head->encoders->data()[4];	//take into account the current facing direction of the head
        target_y = -target_y + head->encoders->data()[3];
	printf("%f %f\n", target_x, target_y);	
        if(target_x > 30) target_x = 30;
        if(target_x < -30) target_x = -30;
        if(target_y > 30) target_y = 30;
        if(target_y < -30) target_y = -30;

        //eyes
        float target_ey = target_y;//(target_y - 10) * 0.5;
        float target_ex = target_x;// * 0.5; //(target_x - 20) * 1.4;
        if(target_ex > 30) target_ex = 30;
        if(target_ex < -30) target_ex = -30;
        if(target_ey > 30) target_ey = 30;
        if(target_ey < -30) target_ey = -30;

        for(int i=0;i<head->num_joints;i++) head->command->data()[i] = 0.0;

//        if(target_x > head->encoders->data()[4]+5 || target_x < head->encoders->data()[4]-5 || target_y > head->encoders->data()[3]+5 || target_y < head->encoders->data()[3]-5)
  //      {
    //        head->command->data()[4] = int(target_x);	//move head
      //      head->command->data()[3] = int(target_y);
            //Set speed of movement according to the distance of the movement (faster for larger moves)
 //           targetSpeedX = 3*getDifference(int(target_x), int(head->encoders->data()[4]));
  //          targetSpeedY = 2*getDifference(int(target_y), int(head->encoders->data()[3]));
 //           if(targetSpeedX > MAX_REAL_HEAD_VELOCITY) targetSpeedX = MAX_REAL_HEAD_VELOCITY;
  //          if(targetSpeedY > MAX_REAL_HEAD_VELOCITY) targetSpeedY = MAX_REAL_HEAD_VELOCITY;
  //          head->pos->setRefSpeed(3, targetSpeedY); //head y
  //          head->pos->setRefSpeed(4, targetSpeedX); //head x
  //      }
  //      else
  //      {
            head->command->data()[4] = -target_ex/2;// head->encoders->data()[4];
            head->command->data()[3] = target_ey/2; //head->encoders->data()[3];
            head->command->data()[2] = target_ex;// head->encoders->data()[4];
            head->command->data()[0] = target_ey; //head->encoders->data()[3];

		
  //      }


        head->pos->positionMove(head->command->data());
//	printf("Moving??? %d\n", head->enc->checkMotionDone());
        moving = true;
        readyToMove = false;
        Time::delay(2);

       head->command->data()[4] = 0;
       head->command->data()[3] = 0;
       head->command->data()[2] = 0;//-target_ey;// head->encoders->data()[4];
       head->command->data()[0] = -10;//target_ex; //head->encoders->data()[3];

   //     Time::delay(2);
       head->pos->positionMove(head->command->data());
       
	 moving = false;
        readyToMove = true;

    } // ready to move?
}


/**
*@brief     resets head back to start position
*/
void ICubMotor::resetHead()
{
    if((readyToMove) && (!cartMove))
    {
       head->pos->positionMove(head->initpos->data());
       moving = true;
       readyToMove = false;
        Time::delay(2);
        moving = false;
        readyToMove = true;
    }
}

void ICubMotor::lookDown()
{
    head->pos->setRefSpeed(2, 20); //head x
    head->pos->setRefSpeed(0, 20); //head y

    for(int i=0;i<head->num_joints;i++) head->command->data()[i] = 0.0;
    head->command->data()[0] = -15; // y head
    head->command->data()[3] = -15; // y eyes
    head->command->data()[2] = 0;   // x head
    head->command->data()[4] = 0;   // x eyes
    head->pos->positionMove(head->command->data());
    yarp::os::Time::delay(0.5);
}

void ICubMotor::sitDown()
{
    if(standing)
    {
        for(int i=0;i<leftLeg->num_joints;i++) leftLeg->command->data()[i] = 0.0;
        leftLeg->command->data()[0] = 45;   //hip (leg up down)
        leftLeg->command->data()[3] = -70;  //knee
        for(int i=0;i<rightLeg->num_joints;i++) rightLeg->command->data()[i] = 0.0;
        rightLeg->command->data()[0] = 45;   //hip (leg up down)
        rightLeg->command->data()[3] = -70;  //knee
        standing = false;
    }
    else
    {
        for(int i=0;i<leftLeg->num_joints;i++) leftLeg->command->data()[i] = 0.0;
        for(int i=0;i<rightLeg->num_joints;i++) rightLeg->command->data()[i] = 0.0;
        standing = true;
    }

    leftLeg->pos->positionMove(leftLeg->command->data());
    rightLeg->pos->positionMove(rightLeg->command->data());
    yarp::os::Time::delay(0.5);
}

void ICubMotor::lookLeft()
{
/*    head->pos->setRefSpeed(2, 20); //head x
    head->pos->setRefSpeed(0, 20); //head y

    for(int i=0;i<head->num_joints;i++) head->command->data()[i] = 0.0;
    head->command->data()[0] = -15;
    head->command->data()[3] = -15;
    head->command->data()[2] = 5;  // x head
    head->command->data()[4] = -5;   // x eyes
    head->pos->positionMove(head->command->data());
    yarp::os::Time::delay(1.5); */

    lookDown();
}

void ICubMotor::lookRight()
{
/*    head->pos->setRefSpeed(2, 20); //head x
    head->pos->setRefSpeed(0, 20); //head y

    for(int i=0;i<head->num_joints;i++) head->command->data()[i] = 0.0;
    head->command->data()[0] = -15;
    head->command->data()[3] = -15;
    head->command->data()[2] = -8;  // x head
    head->command->data()[4] = 8;   // x eyes
    head->pos->positionMove(head->command->data());
    yarp::os::Time::delay(1.5); */

    lookDown();
}

void ICubMotor::trackMotion()
{
 /*   //for(int i=0;i<head->num_joints;i++) head->command->data()[i] = 0.0;
    //head->pos->positionMove(head->command->data());
    if(!trackingMotion)
    {
        yarp::os::Network::disconnect("/era/0/move:o", "/iCubMotor/0:i");
        yarp::os::Network::connect("/tracker/0:o", "/iCubMotor/0:i");
        trackingMotion = true;
    }
    yarp::os::Time::delay(0.5); */
}

/**
*@brief     TO BE ADDED BY TONY
*@param[in] a   - TO BE ADDED BY TONY
*@param[in] b   - TO BE ADDED BY TONY
*/
int ICubMotor::getDifference(int a, int b)
{
        if(a > b) return a-b;
        else return b-a;
}

/**
*@brief     initialises head
*/
void ICubMotor::initHead(bool simulation)
{
    if(!head->initialised)
    {

        printf("initialising head\n");

        Property options;
        options.put("device", "remote_controlboard");
        options.put("local", "/aquila/head");
        if(simulation)
        {
            options.put("remote", "/icubSim/head");
            head->max_velocity = MAX_SIM_HEAD_VELOCITY;
        }
        else
        {
            options.put("remote", "/icub/head");
            head->max_velocity = MAX_REAL_HEAD_VELOCITY;
        }

        head->device = new PolyDriver(options);

        if (!head->device->isValid())         printf("iCubMotor: head device is not available\n");
        if (!head->device->view(head->lim))   printf("iCubMotor: head IControlLimits interface is not available\n");
        if (!head->device->view(head->pos))   printf("iCubMotor: head IPositionControl interface is not available\n");
        if (!head->device->view(head->encs))  printf("iCubMotor: head IEncoders interface is not available\n");
        if (!head->device->view(head->vel))   printf("iCubMotor: head IVelocityControl interface is not available\n");
        if (!head->device->view(head->mode))  printf("iCubMotor: head IControlMode interface is not available\n");

        head->encoders = new yarp::sig::Vector;
        head->initpos = new yarp::sig::Vector;
        head->command  = new yarp::sig::Vector;
        head->velocity = new yarp::sig::Vector;

        head->pos->getAxes(&head->num_joints);
        head->encoders->resize(head->num_joints);
        head->initpos->resize(head->num_joints);
        head->command->resize(head->num_joints);
        head->velocity->resize(head->num_joints);

        //allocate memory and set velocities and positions to 0
        head->velocities = new double[head->num_joints];
        head->positions  = new double[head->num_joints];
        head->min_limit  = new double[head->num_joints];
        head->max_limit  = new double[head->num_joints];
        for(int i=0;i<head->num_joints;i++)
        {
            head->positions[i] = 0.0;
            head->positions[i] = 0.0;
            head->lim->getLimits(i,&head->min_limit[i],&head->max_limit[i]);
        }
        head->initialised = true;

        //set reference speeds for the eyes...
        head->pos->setRefSpeed(3,90);
        head->pos->setRefSpeed(4,90);
    }
    else printf("iCubMotor: head connection success (was already connected)\n");
    head->num_clients++;
}

/**
*@brief     initialises rightLeg
*/
void ICubMotor::initrightLeg(bool simulation)
{
    if(!rightLeg->initialised)
    {
        printf("initialising right leg\n");

        Property options;
        options.put("device", "remote_controlboard");
        options.put("local", "/aquila/right_leg");
        if(simulation)
        {
            options.put("remote", "/icubSim/right_leg");
            rightLeg->max_velocity = MAX_SIM_LEG_VELOCITY;
        }
        else
        {
            options.put("remote", "/icub/right_leg");
            rightLeg->max_velocity = MAX_REAL_LEG_VELOCITY;
        }

        rightLeg->device = new PolyDriver(options);

        if (!rightLeg->device->isValid())         printf("iCubMotor: rightLeg device is not available\n");
        if (!rightLeg->device->view(rightLeg->lim))   printf("iCubMotor: rightLeg IControlLimits interface is not available\n");
        if (!rightLeg->device->view(rightLeg->pos))   printf("iCubMotor: rightLeg IPositionControl interface is not available\n");
        if (!rightLeg->device->view(rightLeg->encs))  printf("iCubMotor: rightLeg IEncoders interface is not available\n");
        if (!rightLeg->device->view(rightLeg->vel))   printf("iCubMotor: rightLeg interface is not available\n");
        if (!rightLeg->device->view(rightLeg->mode))  printf("iCubMotor: rightLeg IControlMode interface is not available\n");

        rightLeg->encoders = new yarp::sig::Vector;
        rightLeg->command  = new yarp::sig::Vector;
        rightLeg->velocity = new yarp::sig::Vector;

        rightLeg->pos->getAxes(&rightLeg->num_joints);
        rightLeg->encoders->resize(rightLeg->num_joints);
        rightLeg->command->resize(rightLeg->num_joints);
        rightLeg->velocity->resize(rightLeg->num_joints);

        //allocate memory and set velocities and positions to 0
        rightLeg->velocities = new double[rightLeg->num_joints];
        rightLeg->positions  = new double[rightLeg->num_joints];
        rightLeg->min_limit  = new double[rightLeg->num_joints];
        rightLeg->max_limit  = new double[rightLeg->num_joints];
        for(int i=0;i<rightLeg->num_joints;i++)
        {
            rightLeg->positions[i] = 0.0;
            rightLeg->positions[i] = 0.0;
            rightLeg->lim->getLimits(i,&rightLeg->min_limit[i],&rightLeg->max_limit[i]);
        }
        rightLeg->initialised = true;

        //set reference speeds for the knees...
        rightLeg->pos->setRefSpeed(3,20);
    }
    else printf("iCubMotor: rightLeg connection success (was already connected)\n");
    rightLeg->num_clients++;
}

/**
*@brief     initialises leftLeg
*/
void ICubMotor::initleftLeg(bool simulation)
{
    if(!leftLeg->initialised)
    {
        printf("initialising left leg\n");

        Property options;
        options.put("device", "remote_controlboard");
        options.put("local", "/aquila/left_leg");
        if(simulation)
        {
            options.put("remote", "/icubSim/left_leg");
            leftLeg->max_velocity = MAX_SIM_LEG_VELOCITY;
        }
        else
        {
            options.put("remote", "/icub/left_leg");
            leftLeg->max_velocity = MAX_REAL_LEG_VELOCITY;
        }

        leftLeg->device = new PolyDriver(options);

        if (!leftLeg->device->isValid())         printf("iCubMotor: leftLeg device is not available\n");
        if (!leftLeg->device->view(leftLeg->lim))   printf("iCubMotor: leftLeg IControlLimits interface is not available\n");
        if (!leftLeg->device->view(leftLeg->pos))   printf("iCubMotor: leftLeg IPositionControl interface is not available\n");
        if (!leftLeg->device->view(leftLeg->encs))  printf("iCubMotor: leftLeg IEncoders interface is not available\n");
        if (!leftLeg->device->view(leftLeg->vel))   printf("iCubMotor: leftLeg IVelocityControl interface is not available\n");
        if (!leftLeg->device->view(leftLeg->mode))  printf("iCubMotor: leftLeg IControlMode interface is not available\n");

        leftLeg->encoders = new yarp::sig::Vector;
        leftLeg->command  = new yarp::sig::Vector;
        leftLeg->velocity = new yarp::sig::Vector;

        leftLeg->pos->getAxes(&leftLeg->num_joints);
        leftLeg->encoders->resize(leftLeg->num_joints);
        leftLeg->command->resize(leftLeg->num_joints);
        leftLeg->velocity->resize(leftLeg->num_joints);

        //allocate memory and set velocities and positions to 0
        leftLeg->velocities = new double[leftLeg->num_joints];
        leftLeg->positions  = new double[leftLeg->num_joints];
        leftLeg->min_limit  = new double[leftLeg->num_joints];
        leftLeg->max_limit  = new double[leftLeg->num_joints];
        for(int i=0;i<leftLeg->num_joints;i++)
        {
            leftLeg->positions[i] = 0.0;
            leftLeg->positions[i] = 0.0;
            leftLeg->lim->getLimits(i,&leftLeg->min_limit[i],&leftLeg->max_limit[i]);
        }
        leftLeg->initialised = true;

        leftLeg->pos->setRefSpeed(3,20);
    }
    else printf("iCubMotor: leftLeg connection success (was already connected)\n");
    leftLeg->num_clients++;
}

/**
*@brief     initialises torso
*/
void ICubMotor::initTorso(bool simulation)
{
    if(!torso->initialised)
    {
        printf("initialising torso\n");

        Property options;
        options.put("device", "remote_controlboard");
        options.put("local", "/aquila/torso");
        if(simulation)
        {
            options.put("remote", "/icubSim/torso");
        }
        else
        {
            options.put("remote", "/icub/torso");
        }

        torso->max_velocity = MAX_TORSO_VELOCITY;
        torso->device = new PolyDriver(options);

        if (!torso->device->isValid())         printf("iCubMotor: torso device is not available\n");
        if (!torso->device->view(torso->lim))   printf("iCubMotor: torso IControlLimits interface is not available\n");
        if (!torso->device->view(torso->pos))   printf("iCubMotor: torso IPositionControl interface is not available\n");
        if (!torso->device->view(torso->encs))  printf("iCubMotor: torso IEncoders interface is not available\n");
        if (!torso->device->view(torso->vel))   printf("iCubMotor: torso IVelocityControl interface is not available\n");
        if (!torso->device->view(torso->mode))  printf("iCubMotor: torso IControlMode interface is not available\n");

        torso->encoders = new yarp::sig::Vector;
        torso->command  = new yarp::sig::Vector;

        torso->pos->getAxes(&torso->num_joints);
        torso->encoders->resize(torso->num_joints);
        torso->command->resize(torso->num_joints);

        //allocate memory and set velocities and positions to 0
        torso->velocities = new double[torso->num_joints];
        torso->positions  = new double[torso->num_joints];
        torso->min_limit  = new double[torso->num_joints];
        torso->max_limit  = new double[torso->num_joints];
        for(int i=0;i<torso->num_joints;i++)
        {
            torso->positions[i] = 0.0;
            torso->positions[i] = 0.0;
            torso->lim->getLimits(i,&torso->min_limit[i],&torso->max_limit[i]);
        }
        torso->initialised = true;
    }
    else printf("iCubMotor: torso connection success\n");
    torso->num_clients++;
}

/**
*@brief     initialises left and right arms and sets up impedance mode
*/
int ICubMotor::initImpedance(bool simulation)
{
    if(simulation)
    {
        printf("IMPEDANCE MODE OFF this is a simulation, impedance only works on the real robot\n");
    }
    else
    {
        printf("switching body to impedance mode\n");

        Property optionsLeft;
        optionsLeft.put("device", "remote_controlboard");
        optionsLeft.put("local", "/aquila/left_arm");
        optionsLeft.put("remote", "/icub/left_arm");

        Property optionsRight;
        optionsRight.put("device", "remote_controlboard");
        optionsRight.put("local", "/aquila/right_arm");
        optionsRight.put("remote", "/icub/right_arm");

        // create a device
        leftArm->device = new PolyDriver(optionsLeft);
        rightArm->device = new PolyDriver(optionsRight);

        if (!leftArm->device->isValid())            printf("iCubMotor: leftArm device is not available\n");
        if (!leftArm->device->view(leftArm->lim))   printf("iCubMotor: leftArm IControlLimits interface is not available\n");
        if (!leftArm->device->view(leftArm->pos))   printf("iCubMotor: leftArm IPositionControl interface is not available\n");
        if (!leftArm->device->view(leftArm->encs))  printf("iCubMotor: leftArm IEncoders interface is not available\n");
        if (!leftArm->device->view(leftArm->vel))   printf("iCubMotor: leftArm IVelocityControl interface is not available\n");
        if (!leftArm->device->view(leftArm->mode))  printf("iCubMotor: leftArm IControlMode interface is not available\n");
        if (!leftArm->device->view(leftArm->iimp))  printf("iCubMotor: leftArm IImpedanceControl interface is not available\n");
        if (!leftArm->device->view(leftArm->itrq))  printf("iCubMotor: leftArm ITorqueControl interface is not available\n");
        if (!rightArm->device->isValid())             printf("iCubMotor: rightArm device is not available\n");
        if (!rightArm->device->view(rightArm->lim))   printf("iCubMotor: rightArm IControlLimits interface is not available\n");
        if (!rightArm->device->view(rightArm->pos))   printf("iCubMotor: rightArm IPositionControl interface is not available\n");
        if (!rightArm->device->view(rightArm->encs))  printf("iCubMotor: rightArm IEncoders interface is not available\n");
        if (!rightArm->device->view(rightArm->vel))   printf("iCubMotor: rightArm IVelocityControl interface is not available\n");
        if (!rightArm->device->view(rightArm->mode))  printf("iCubMotor: rightArm IControlMode interface is not available\n");
        if (!rightArm->device->view(rightArm->iimp))  printf("iCubMotor: rightArm IImpedanceControl interface is not available\n");
        if (!rightArm->device->view(rightArm->itrq))  printf("iCubMotor: rightArm ITorqueControl interface is not available\n");

        leftArm->encoders = new yarp::sig::Vector;
        leftArm->command  = new yarp::sig::Vector;
        leftArm->torques  = new yarp::sig::Vector;
        rightArm->encoders = new yarp::sig::Vector;
        rightArm->command  = new yarp::sig::Vector;
        rightArm->torques  = new yarp::sig::Vector;

        leftArm->pos->getAxes(&leftArm->num_joints);
        leftArm->encoders->resize(leftArm->num_joints);
        leftArm->command->resize(leftArm->num_joints);

        rightArm->pos->getAxes(&rightArm->num_joints);
        rightArm->encoders->resize(rightArm->num_joints);
        rightArm->command->resize(rightArm->num_joints);

        for (int i = 0; i < 4; i++)
        {
            //SET THE IMPEDANCE:
            //0.111 is the stiffness coefficient. units:   Nm/deg
            //0.014 is the damping coefficient. units:     Nm/(deg/s)
            //0 is the additional torque offset
            //WARNING: playing with this value may lead to undamped oscillatory behaviours.
            //when you raise the stiffness, you should increase the damping coefficient accordingly.
            leftArm->iimp->setImpedance(i, 0.111, 0.014);
            rightArm->iimp->setImpedance(i, 0.111, 0.014);
        }

        for (int i = 0; i < 4; i++)
        {
            //leftArm->mode->setImpedancePositionMode(i);
            //rightArm->mode->setImpedancePositionMode(i);
            leftArm->mode->setImpedanceVelocityMode(i);
            rightArm->mode->setImpedanceVelocityMode(i);
        }

        leftArm->initialised = true;
        rightArm->initialised = true;
        leftArm->num_clients++;
        rightArm->num_clients++;

        printf("impedance mode ON\n");
    }


    yarp::os::Time::delay(1);
}


/**
*@brief     de-initialises head
*/
void ICubMotor::closeHead()
{
    if(head->initialised)
    {
        if(head->num_clients == 1)
        {
            printf("iCubMotor: removing last client connection to head...\n");

            head->device->close();

            delete   head->device;
            delete   head->encoders;
            delete   head->command;
            delete[] head->velocities;
            delete[] head->positions;
            delete[] head->min_limit;
            delete[] head->max_limit;

            head->device     = NULL;
            head->encoders   = NULL;
            head->command    = NULL;
            head->lim        = NULL;
            head->pos        = NULL;
            head->encs       = NULL;
            head->vel        = NULL;
            head->mode       = NULL;

            head->num_clients--;
            head->initialised = false;
        }
        else
        {
            if(head->num_clients-1>1) printf("iCubMotor: removing client connection to head, more to go...\n");
            else                      printf("iCubMotor: removing client connection to head, one client is still connected...\n");
            head->num_clients--;
        }
    }
    else printf("iCubMotor: closing call ignored, head was not initialised...\n");
}

/**
*@brief     de-initialises leftLeg
*/
void ICubMotor::closeLeftLeg()
{
    if(leftLeg->initialised)
    {
        if(leftLeg->num_clients == 1)
        {
            printf("iCubMotor: removing last client connection to leftLeg...\n");

            leftLeg->device->close();

            delete   leftLeg->device;
            delete   leftLeg->encoders;
            delete   leftLeg->command;
            delete[] leftLeg->velocities;
            delete[] leftLeg->positions;
            delete[] leftLeg->min_limit;
            delete[] leftLeg->max_limit;

            leftLeg->device     = NULL;
            leftLeg->encoders   = NULL;
            leftLeg->command    = NULL;
            leftLeg->lim        = NULL;
            leftLeg->pos        = NULL;
            leftLeg->encs       = NULL;
            leftLeg->vel        = NULL;
            leftLeg->mode       = NULL;

            leftLeg->num_clients--;
            leftLeg->initialised = false;
        }
        else
        {
            if(leftLeg->num_clients-1>1) printf("iCubMotor: removing client connection to leftLeg, more to go...\n");
            else                      printf("iCubMotor: removing client connection to leftLeg, one client is still connected...\n");
            leftLeg->num_clients--;
        }
    }
    else printf("iCubMotor: closing call ignored, leftLeg was not initialised...\n");
}

/**
*@brief     de-initialises rightLeg
*/
void ICubMotor::closeRightLeg()
{
    if(rightLeg->initialised)
    {
        if(rightLeg->num_clients == 1)
        {
            printf("iCubMotor: removing last client connection to rightLeg...\n");

            rightLeg->device->close();

            delete   rightLeg->device;
            delete   rightLeg->encoders;
            delete   rightLeg->command;
            delete[] rightLeg->velocities;
            delete[] rightLeg->positions;
            delete[] rightLeg->min_limit;
            delete[] rightLeg->max_limit;

            rightLeg->device     = NULL;
            rightLeg->encoders   = NULL;
            rightLeg->command    = NULL;
            rightLeg->lim        = NULL;
            rightLeg->pos        = NULL;
            rightLeg->encs       = NULL;
            rightLeg->vel        = NULL;
            rightLeg->mode       = NULL;

            rightLeg->num_clients--;
            rightLeg->initialised = false;
        }
        else
        {
            if(rightLeg->num_clients-1>1) printf("iCubMotor: removing client connection to rightLeg, more to go...\n");
            else                      printf("iCubMotor: removing client connection to rightLeg, one client is still connected...\n");
            rightLeg->num_clients--;
        }
    }
    else printf("iCubMotor: closing call ignored, rightLeg was not initialised...\n");
}
/**
*@brief     de-initialises torso
*/
void ICubMotor::closeTorso()
{
    if(torso->initialised)
    {
        if(torso->num_clients == 1)
        {
            printf("iCubMotor: removing last client connection to torso...\n");

            torso->device->close();

            delete   torso->device;
            delete   torso->encoders;
            delete   torso->command;
            delete[] torso->velocities;
            delete[] torso->positions;
            delete[] torso->min_limit;
            delete[] torso->max_limit;

            torso->device     = NULL;
            torso->encoders   = NULL;
            torso->command    = NULL;
            torso->lim        = NULL;
            torso->pos        = NULL;
            torso->encs       = NULL;
            torso->vel        = NULL;
            torso->mode       = NULL;

            torso->num_clients--;
            torso->initialised = false;
        }
        else
        {
            if(torso->num_clients-1>1) printf("iCubMotor: removing client connection to torso, more to go...\n");
            else                      printf("iCubMotor: removing client connection to torso, one client is still connected...\n");
            torso->num_clients--;
        }
    }
    else printf("iCubMotor: closing call ignored, torso was not initialised...\n");
}


void ICubMotor::closeImpedance()
{
    if(leftArm->initialised)
    {
        printf("switching off impedance mode left...\n");
        for (int i = 0; i < 4; i++)
        {
            leftArm->mode->setPositionMode(i);
        }

        printf("iCubMotor: removing last client connection to leftArm...\n");
        leftArm->device->close();

        delete   leftArm->device;
        delete   leftArm->encoders;
        delete   leftArm->command;
        delete   leftArm->torques;

        leftArm->num_clients--;
        leftArm->initialised = false;

    }
    else printf("iCubMotor: closing call ignored, leftArm was not initialised...\n");

    if(rightArm->initialised)
    {
        printf("switching off impedance mode right...\n");
        for (int i = 0; i < 4; i++)
        {
            rightArm->mode->setPositionMode(i);
        }

        printf("iCubMotor: removing last client connection to rightArm...\n");
        rightArm->device->close();

        delete   rightArm->device;
        delete   rightArm->encoders;
        delete   rightArm->command;
        delete   rightArm->torques;

        rightArm->num_clients--;
        rightArm->initialised = false;
    }
    else printf("iCubMotor: closing call ignored, rightArm was not initialised...\n");
}


/*!
 * \brief Stops the thread.
 */
void ICubMotor::stop()
{
    printf("Stopping.............\n");
    // stop the cartesian controller thread
    //if(cartesianMode == true){
     //cartMod.close();
      //thr->stop();
      //delete thr;
    //}
    running  = false;
}

/*!
 * \brief Clean up.
 */
void ICubMotor::clean()
{
    printf("Cleaning up.............\n");
    pointPort.interrupt();
    disconnectPorts();
    pointPort.close();
}

/*!
 * \brief Sets simulation mode.
 */
void ICubMotor::setSimulationMode(bool simON)
{
    simulationMode = simON;
}


/*!
 * \brief Gets simulation mode.
 */
bool ICubMotor::getSimulationMode()
{
    return simulationMode;
}


/*!
 * \brief Sets cartesian mode.

 */
void ICubMotor::setCartesianMode(bool cartON)
{
    cartesianMode = cartON;
}


/*!
 * \brief Gets cartesian mode.
 */
bool ICubMotor::getCartesianMode()
{
    return cartesianMode;
}

void CubCartThread::cartesianEventCallback()
   {
        fprintf(stdout,"20%% of trajectory attained\n");
   }


   CubCartThread::CubCartThread(const double period, bool simMode) : RateThread(int(period*1000.0))
   {
        // we wanna raise an event each time the arm is at 20%
        // of the trajectory (or 70% far from the target)
        cartesianEventParameters.type="motion-ongoing";
        cartesianEventParameters.motionOngoingCheckPoint=0.2;
        simulationMode = simMode;
   }

   bool CubCartThread::threadInit()
    {
        inputPort.open("/cart");

        // open a client interface to connect to the cartesian server of the simulator
        // we suppose that:
        //
        // 1 - the iCub simulator is running
        //     (launch iCub_SIM)
        //
        // 2 - the cartesian server is running
        //     (launch simCartesianControl)
        //     
        // 3 - the cartesian solver for the left arm is running too
        //     (launch iKinCartesianSolver --context simCartesianControl/conf --part left_arm)
        //

        Property Loption("(device cartesiancontrollerclient)");
        Property Roption("(device cartesiancontrollerclient)");

        if(simulationMode == true){ 

           printf("Simulation mode on\n");
 
           Loption.put("remote","/icubSim/cartesianController/left_arm");
           Loption.put("local","/cartesian_client/left_arm");

           Roption.put("remote","/icubSim/cartesianController/right_arm");
           Roption.put("local","/cartesian_client/right_arm");

        } else {

           printf("Simulation mode off\n");

           Loption.put("remote","/icub/cartesianController/left_arm");
           Loption.put("local","/cartesian_client/left_arm");

           Roption.put("remote","/icub/cartesianController/right_arm");
           Roption.put("local","/cartesian_client/right_arm");
        }



        if (!Lclient.open(Loption) || !Rclient.open(Roption))
            return false;

        // open the view
        Lclient.view(icartL);
        Rclient.view(icartR);

        // latch the controller context in order to preserve
        // it after closing the module
        // the context contains the dofs status, the tracking mode,
        // the resting positions, the limits and so on.
        icartL->storeContext(&startup_context_left_id);
		icartR->storeContext(&startup_context_right_id);
		
        // set trajectory time
        icartL->setTrajTime(2.0);
        icartR->setTrajTime(2.0);

        // get the torso dofs
        Vector newDof, curDof;
        icartL->getDOF(curDof);
        newDof=curDof;

        // enable the torso yaw and pitch
        // disable the torso roll
        newDof[0]=1;    //forward back
        newDof[1]=0;    //side to side tilt!
        newDof[2]=1;    //turn

        // impose some restriction on the torso pitch
        limitTorsoPitch();

        // send the request for dofs reconfiguration
        icartL->setDOF(newDof,curDof);
        icartR->setDOF(newDof,curDof);

        // print out some info about the controller
        Bottle info;
        icartL->getInfo(info);
        fprintf(stdout,"info = %s\n",info.toString().c_str());
        icartR->getInfo(info);
        fprintf(stdout,"info = %s\n",info.toString().c_str());   

        // register the event, attaching the callback
        icartL->registerEvent(*this);
		icartR->registerEvent(*this);
		
        Lxd.resize(3);
        Rxd.resize(3);
        Lod.resize(4);
        Rod.resize(4);

        //some default "safe" position
        Lxd[0] = -0.3;
        Lxd[1] = -0.1;
        Lxd[2] = 0.1;
        Rxd[0] = -0.3;
        Rxd[1] = -0.1;
        Rxd[2] = 0.1;
        
        
        // we keep the orientation of the left arm constant:
        // we want the middle finger to point forward (end-effector x-axis)
        // with the palm turned down (end-effector y-axis points leftward);
        // to achieve that it is enough to rotate the root frame of pi around z-axis
        Lod[0]=0.0; Lod[1]=0.0; Lod[2]=1.0; Lod[3]=M_PI;    //plam down
        Rod[0]=0.0; Rod[1]=1.0; Rod[2]=0.0; Rod[3]=M_PI;    //palm down

        return true;
    }

    void CubCartThread::afterStart(bool s)
    {
        if (s)
            fprintf(stdout,"Thread started successfully\n");
        else
            fprintf(stdout,"Thread did not start\n");

        t=t0=t1=Time::now();
    }

    void CubCartThread::run()
    {
        t=Time::now();

        getTarget();
        //generateTarget();

        // go to the target :)
        // (in streaming)
        //icartL->goToPose(xd,od);
        // some verbosity
        printStatus();
    }

    void CubCartThread::threadRelease()
    {    
        // we require an immediate stop
        // before closing the client for safety reason
        icartL->stopControl();
		icartR->stopControl();
		
        // it's a good rule to restore the controller
        // context as it was before opening the module
        icartL->restoreContext(startup_context_left_id);
		icartR->restoreContext(startup_context_right_id);
        Lclient.close();
        Rclient.close();
    }


    void CubCartThread::getTarget()
    {   
        Bottle command;

        printf("current command: left %f %f %f, right %f %f %f\n", Lxd[0], Lxd[1], Lxd[2], Rxd[0], Rxd[1], Rxd[2]);

        if(inputPort.read(command))
        {
            if(command.get(0).asString()=="left")
            {
                icartR->stopControl();  //stop the right arm
                printf("recieved command: left %f %f %f\n", command.get(1).asDouble(), command.get(2).asDouble(), command.get(3).asDouble());

                Lxd[0] = command.get(1).asDouble() - 0.065;    //left arm negative in front
                Lxd[1] = command.get(2).asDouble() - 0.065;    //left arm negative on correct side of body (x)
                Lxd[2] = command.get(3).asDouble();    //positive from waist up

				// sanity check the position before moving there
                if(Lxd[0] < -0.5) Lxd[0] = -0.5;
				if(Lxd[0] > -0.1) Lxd[0] = -0.1;
				if(Lxd[1] < -0.4) Lxd[1] = -0.4;
                if(Lxd[1] > 0.1) Lxd[1] = 0.1;
				if(Lxd[2] < -0.2) Lxd[2] = -0.2;
				if(Lxd[2] > 0.4) Lxd[2] = 0.4;

                icartL->goToPose(Lxd,Lod);
                Time::delay(3);              
                icartL->stopControl();  //stop the left arm
            }
            else if(command.get(0).asString()=="right")
            {
                icartL->stopControl();  //stop the left arm
                printf("recieved command: right %f %f %f\n", command.get(1).asDouble(), command.get(2).asDouble(), command.get(3).asDouble());

                Rxd[0] = command.get(1).asDouble() - 0.04;    //right arm negative in front
                Rxd[1] = command.get(2).asDouble() + 0.05;    //right arm positive on correct side of body
                Rxd[2] = command.get(3).asDouble();    //positive from waist up

				// sanity check the position before moving there
                if(Rxd[0] < -0.5) Rxd[0] = -0.5;
				if(Rxd[0] > -0.1) Rxd[0] = -0.1;
				if(Rxd[1] > 0.4) Rxd[1] = 0.4;
				if(Rxd[1] < 0.0) Rxd[1] = 0.0;
				if(Rxd[2] < -0.2) Rxd[2] = -0.2;
				if(Rxd[2] > 0.4) Rxd[2] = 0.4;

                icartR->goToPose(Rxd,Rod);
                Time::delay(3);
                icartR->stopControl();  //stop the right arm
            }
        }
    }


    void CubCartThread::generateTarget()
    {   
        // translational target part: a circular trajectory
        // in the yz plane centered in [-0.3,-0.1,0.1] with radius=0.1 m
        // and frequency 0.1 Hz
        
       // xd[0]=-0.3;
       // xd[1]=-0.1+0.1*cos(2.0*M_PI*0.1*(t-t0));
       // xd[2]=+0.1+0.1*sin(2.0*M_PI*0.1*(t-t0));
                 
        // we keep the orientation of the left arm constant:
        // we want the middle finger to point forward (end-effector x-axis)
        // with the palm turned down (end-effector y-axis points leftward);
        // to achieve that it is enough to rotate the root frame of pi around z-axis
       // od[0]=0.0; od[1]=0.0; od[2]=1.0; od[3]=M_PI;
    }

    double CubCartThread::norm(const Vector &v)
    {
        return sqrt(dot(v,v));
    }

    void CubCartThread::limitTorsoPitch()
    {
        int axis=0; // pitch joint
        double min, max;

        // sometimes it may be helpful to reduce
        // the range of variability of the joints;
        // for example here we don't want the torso
        // to lean out more than 30 degrees forward

        // we keep the lower limit
        icartL->getLimits(axis,&min,&max);
        icartL->setLimits(axis,min,MAX_TORSO_PITCH);
    }

    void CubCartThread::printStatus()
    {        
        if (t-t1>=PRINT_STATUS_PER)
        {
            Vector x,o,xdhat,odhat,qdhat;

            // we get the current arm pose in the
            // operational space
            icartL->getPose(x,o);

            // we get the final destination of the arm
            // as found by the solver: it differs a bit
            // from the desired pose according to the tolerances
            icartL->getDesired(xdhat,odhat,qdhat);

            double e_x=norm(xdhat-x);
            double e_o=norm(odhat-o);

            fprintf(stdout,"+++++++++\n");
            fprintf(stdout,"xd          [m] = %s\n",Lxd.toString().c_str());
            fprintf(stdout,"xdhat       [m] = %s\n",xdhat.toString().c_str());
            fprintf(stdout,"x           [m] = %s\n",x.toString().c_str());
            fprintf(stdout,"od        [rad] = %s\n",Lod.toString().c_str());
            fprintf(stdout,"odhat     [rad] = %s\n",odhat.toString().c_str());
            fprintf(stdout,"o         [rad] = %s\n",o.toString().c_str());
            fprintf(stdout,"norm(e_x)   [m] = %g\n",e_x);
            fprintf(stdout,"norm(e_o) [rad] = %g\n",e_o);
            fprintf(stdout,"---------\n\n");

            fprintf(stdout,"xd          [m] = %s\n",Rxd.toString().c_str());
            fprintf(stdout,"xdhat       [m] = %s\n",xdhat.toString().c_str());
            fprintf(stdout,"x           [m] = %s\n",x.toString().c_str());
            fprintf(stdout,"od        [rad] = %s\n",Rod.toString().c_str());
            fprintf(stdout,"odhat     [rad] = %s\n",odhat.toString().c_str());
            fprintf(stdout,"o         [rad] = %s\n",o.toString().c_str());
            fprintf(stdout,"norm(e_x)   [m] = %g\n",e_x);
            fprintf(stdout,"norm(e_o) [rad] = %g\n",e_o);
            fprintf(stdout,"---------\n\n");

            t1=t;
        }
    }

    bool CubCart::configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        thr=new CubCartThread(CTRL_THREAD_PER,simulationMode);
        if (!thr->start())
        {
            delete thr;
            return false;
        }

        return true;
    }

    bool CubCart::close()
    {
        thr->stop();
        delete thr;

        return true;
    }

    double CubCart::getPeriod()
    { 
       return 1.0;
    }
    bool   CubCart::updateModule() 
    { 
       return true; 
    }
    void CubCart::setSimulationMode(bool simMode){

       simulationMode = simMode;

    }


}

