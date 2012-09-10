// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff Ugo Pattacini
 * email:  vadim.tikhanoff@iit.it
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

#include <sstream>
#include <stdio.h>
#include <yarp/math/Rand.h>
#include <yarp/math/Math.h>
#include "iCub/module.h"

#include <gsl/gsl_math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

#define RET_INVALID     -1

#define CMD_ON                  VOCAB2('o','n')

#define CMD_OFF                 VOCAB3('o','f','f')
#define CMD_TAP                 VOCAB3('t','a','p')
#define CMD_KPUSH_TRAD          VOCAB3('k','p','t')           // push in karmaMotor retina position on the traditionalCameras
#define CMD_KPUSH_EVENT         VOCAB3('k','p','e')           // push in karmaMotor retina position on the eventCameras

#define CMD_TRAIN               VOCAB4('t','r','a','i')
#define CMD_EXECUTE             VOCAB4('e','x','e','c')
#define CMD_HELP                VOCAB4('h','e','l','p')
#define CMD_HOME                VOCAB4('h','o','m','e')
#define CMD_PUSH                VOCAB4('p','u','s','h')
#define CMD_DUMP                VOCAB4('d','u','m','p')



/**********************************************************/
bool Manager::configure(ResourceFinder &rf)
{
    name=rf.find("name").asString().c_str();

    //incoming
    pointedLoc.open(     ("/"+name+"/point:i").c_str());
    blobExtractor.open(  ("/"+name+"/blobs:i").c_str());

    //outgoing
    iolStateMachine.open(("/"+name+"/iolState:o").c_str());    

    //rpc 
    rpcMIL.open(         ("/"+name+"/mil:rpc").c_str());          //rpc client to query mil classifications
    rpcHuman.open(       ("/"+name+"/human:rpc").c_str());        //rpc server to interact with the italkManager
    rpcMotorAre.open(    ("/"+name+"/are:rpc").c_str());          //rpc server to query ARE
    rpcMotorKarma.open(  ("/"+name+"/karma:rpc").c_str());        //rpc server to query Karma

    rpcTransTrad.open(   ("/"+name+"/transTrad:rpc").c_str());    //rpc server to query the transTrad
    rpcTransEvent.open(  ("/"+name+"/transEvent:rpc").c_str());   //rpc server to query the transEvent
    rpcVelExtract.open(  ("/"+name+"/velExtract:rpc").c_str());   //rpc server to query the velocityExtractor
    rpcAttention.open(   ("/"+name+"/attention:rpc").c_str());    //rpc server to query the velocityExtractor
    rpcGrabber.open(     ("/"+name+"/grabber:rpc").c_str());      //rpc server to activate and deactivate the grabber
                          
    Rand::init();

    randActions[0] = 0.0;
    randActions[1] = 30.0;
    randActions[2] = 150.0;
    randActions[3] = 180.0;
    randActions[4] = 225.0;
    randActions[5] = 315.0;
    
    pointGood=false;
    init=false;
    //attach(rpcHuman);
    return true;
}

/**********************************************************/
bool Manager::interruptModule()
{
    blobExtractor.interrupt();
    rpcHuman.interrupt();
    rpcMotorAre.interrupt();
    rpcMotorKarma.interrupt();
    pointedLoc.interrupt();
    iolStateMachine.interrupt();
    rpcMIL.interrupt();
    rpcTransTrad.interrupt();
    rpcTransEvent.interrupt();
    rpcVelExtract.interrupt();
    rpcAttention.interrupt();
    rpcGrabber.interrupt();
    return true;
}
/**********************************************************/
bool Manager::close()
{
    blobExtractor.close();
    rpcHuman.close();
    rpcMotorAre.close();
    rpcMotorKarma.close();
    pointedLoc.close();
    iolStateMachine.close();
    rpcMIL.close();
    rpcTransTrad.close();
    rpcTransEvent.close();
    rpcVelExtract.close();
    rpcAttention.close();
    rpcGrabber.close();

    return true;
}
/**********************************************************/
int Manager::processHumanCmd(const Bottle &cmd, Bottle &b)
{
    int ret=Vocab::encode(cmd.get(0).asString().c_str());
    b.clear();
    if (cmd.size()>1)
    {
        if (cmd.get(1).isList())
            b=*cmd.get(1).asList();
        else
            b=cmd.tail();
    }
    return ret;
}
/**********************************************************/
bool Manager::updateModule()
{
    if (isStopping())
        return false;
        
    while (!init)
    {   
        Time::delay(5.0);
        fprintf(stdout, "waiting for connection from iolStateMachineHandler... \n");
        if (iolStateMachine.getOutputCount() > 0 )
        {
            fprintf(stdout, "sending home \n");
            Bottle cmdIol, replyIol;
            cmdIol.addString("home");
            iolStateMachine.write(cmdIol,replyIol);
            fprintf(stdout,"%s\n", replyIol.toString().c_str());
            fprintf(stdout,"stopping attention...\n");
            cmdIol.clear();
            replyIol.clear();
            cmdIol.addString("attention");
            cmdIol.addString("stop");
            iolStateMachine.write(cmdIol,replyIol);
            fprintf(stdout,"%s\n", replyIol.toString().c_str());
            init = true;
            fprintf(stdout, "have successfully initialize it all\n");
        }
    }
    
    Bottle cmd, val, reply;
    rpcHuman.read(cmd, true);
    if (cmd != NULL)
    {
        int rxCmd=processHumanCmd(cmd,val);

        if (rxCmd==Vocab::encode("train"))
        {
            obj=cmd.get(1).asString().c_str();
            
            if (cmd.size() > 2)
            {
                fprintf(stdout,"with user data\n");
                userTheta=cmd.get(2).asDouble();
            }
            else 
            {
               fprintf(stdout,"no user data\n");
               userTheta = -1.0;
            }
            //pointGood = pointedLoc.getLoc(pointLocation);
            //Time::delay(1.5);
            executeOnLoc(true); //execute whatever is needed

            reply.addString("ack");
            rpcHuman.reply(reply);
            pointGood = false;
        }
        if (rxCmd==Vocab::encode("test"))
        {
            obj=cmd.get(1).asString().c_str();
            
            //pointGood = pointedLoc.getLoc(pointLocation);
            //Time::delay(1.5);
            executeOnLoc(false);
            reply.addString("ack");
            rpcHuman.reply(reply);
            pointGood = false;
        }
        
        //*************************************************************************
        switch(rxCmd) {
        case CMD_HELP: {
            reply.addString("HELP \n");
            reply.addString("train \n");
            reply.addString("test \n");
            reply.addString("tap \n");
            reply.addString("push (u v)");
            reply.addString("kpt (u v)");
            reply.addString("===== \n");
            rpcHuman.reply(reply);
        }break;
        case CMD_HOME:{
            printf("home command received \n");
            goHome();
        }break;
        case CMD_TAP: {
            reply.addString("tap command \n");
            
            rpcHuman.reply(reply);
        }break;
        case CMD_PUSH: {
            reply.addString("push command \n");
            if(cmd.size() > 1 ){
                Bottle* opt = cmd.get(1).asList();
                fprintf(stdout,"option of the command push  \n");
            }
            rpcHuman.reply(reply);
        }break;
        case CMD_KPUSH_TRAD: {
            reply.addString("karma push command for traditional camera: ");
            if(cmd.size() > 1 ){
                Bottle* opt = cmd.get(1).asList();
                fprintf(stdout,"option of the command push %s \n", opt->toString().c_str());
                int u = opt->get(0).asInt();
                int v = opt->get(1).asInt();
                double x,y,z;
                pushTraditional(u,v,x,y,z);
                reply.addDouble(x);
                reply.addDouble(y);
                reply.addDouble(z);
                
            }
            else {
                reply.addString("UNSUCCESS \n");
            }
            rpcHuman.reply(reply);
        }break;
        defaut: {
            }
        }
        
    }
    Bottle result;
    result.clear();

    return true;
}

void Manager::goHome()
{
    fprintf(stdout,"goHome command executing ...... ");
    Bottle cmdAre, replyAre;
    cmdAre.clear();
    replyAre.clear();
    cmdAre.addString("home");
    cmdAre.addString("all");
    rpcMotorAre.write(cmdAre,replyAre);
    fprintf(stdout,"gone home %s:\n",replyAre.toString().c_str()); 
}
/****************************************************************************************************************/


void Manager::pushTraditional(int u, int v, double& x, double& y, double& z){
    fprintf(stdout, "push retina position of traditional cameras %d %d \n", u, v);
    if(rpcTransTrad.getOutputCount()) {
        Bottle cmdPushTrad, replyPushTrad;
        cmdPushTrad.clear(); replyPushTrad.clear();
        cmdPushTrad.addString("home");
        iolStateMachine.write(cmdPushTrad,replyPushTrad);
        fprintf(stdout,"%s\n", replyPushTrad.toString().c_str());
        fprintf(stdout,"getting the 3d position ... \n");
        cmdPushTrad.clear();
        replyPushTrad.clear();
        cmdPushTrad.addString("get3d");
        cmdPushTrad.addInt(u);
        cmdPushTrad.addInt(v);
        cmdPushTrad.addString("left");
        rpcTransTrad.write(cmdPushTrad,replyPushTrad);
        fprintf(stdout,"reply:%s \n", replyPushTrad.toString().c_str());
        x = replyPushTrad.get(0).asDouble();
        y = replyPushTrad.get(1).asDouble();
        z = replyPushTrad.get(2).asDouble();
        
        x = -0.35;
        y = 0.05;
        z = -0.20;
        
        if(rpcMotorKarma.getOutputCount() && (rpcMotorAre.getOutputCount()) ) {
            // sending fix command to the actionRenderingEngine "look (x y z) fixate"
            Bottle areTarget; areTarget.clear();
            Bottle areMotor,areReply;
            areMotor.clear(); areReply.clear();
            areMotor.addString("look");
            areTarget = areMotor.addList();
            areTarget.addDouble(x);
            areTarget.addDouble(y);
            areTarget.addDouble(z);
            areMotor.addString("fixate");
            rpcMotorAre.write(areMotor,areReply);
            fprintf(stdout,"fication is %s:\n",areReply.toString().c_str());
            
            // sending command to dump event
            if(rpcGrabber.getOutputCount()){
                Bottle grabberRequest,grabberReply;
                grabberRequest.clear(); grabberReply.clear();
                grabberRequest.addVocab(CMD_DUMP);
                grabberRequest.addVocab(CMD_ON); 
                rpcGrabber.write(grabberRequest,grabberReply);
            }
            else {
                fprintf(stdout, "connection to the grabber missing \n");
            }
            
            // sending push command to the karma
            double actionOrient = 0.0;
            double offset       = 0.06;
            fprintf(stdout,"Will now send to karmaMotor:\n");
            Bottle karmaMotor,karmaReply;
            karmaMotor.clear(); karmaReply.clear();
            karmaMotor.addDouble(x);
            karmaMotor.addDouble(y);
            karmaMotor.addDouble(z + 0.05);
            karmaMotor.addDouble(actionOrient);
            karmaMotor.addDouble( offset );// + 0.06 );

            fprintf(stdout,"%s\n",karmaMotor.toString().c_str());
            rpcMotorKarma.write(karmaMotor, karmaReply);
            fprintf(stdout,"action is %s:\n",karmaReply.toString().c_str());
            
            //temporal delay
            Time::delay(1000);

            // sending command to dump event
            if(rpcGrabber.getOutputCount()){
                Bottle grabberRequest,grabberReply;
                grabberRequest.clear(); grabberReply.clear();
                grabberRequest.addVocab(CMD_DUMP);
                grabberRequest.addVocab(CMD_OFF); 
                rpcGrabber.write(grabberRequest,grabberReply);
            }
            else {
                fprintf(stdout, "connection to the grabber missing \n");
            }
            
            goHome();
        }
        else {
            fprintf(stdout,"Either the ActionRenderingEngine or the KARMA motor missing in connections \n");
        }
        
    }
    else {
        fprintf(stdout, "interface with traslator traditional camera not active \n");
    }
}


/****************************************************************************************************************/
int Manager::executeOnLoc(bool shouldTrain)
{

    fprintf(stdout, "\n\n\n****************************************************************************\n\n" );
    Bottle blobs;
    blobs.clear();
    // grab the blobs
    blobs=getBlobs();
    // failure handling

    if (blobs.size()==0)
        return RET_INVALID;
    
    if (blobs.size()<2)
    {
        pointLocation = getBlobCOG(blobs,0);
        fprintf (stdout,"point is %d %d \n", pointLocation.x, pointLocation.y);
        pointGood = true;
    }else
    {
        fprintf (stdout,"I see more than two blobs\n");
        pointGood = false;
    }
    if (pointGood)
    {
        Bottle closestBlob;
        mutexResources.wait();
        closestBlob=findClosestBlob(blobs,pointLocation);
        mutexResources.post();
        
        CvPoint cog;
        cog.x = closestBlob.get(0).asInt();
        cog.y = closestBlob.get(1).asInt();
        
        int index = 0;
        index = closestBlob.get(2).asInt();

        fprintf(stdout, "closest blob is x:%d y:%d with index %d\n\n",cog.x, cog.y, index);
        
        //classify the blob
        Bottle mil;
        mil=classify(blobs, index);
        //get the type of the blob
        Bottle type;
        type=getType(&mil, index);

        double actionOrient = 0.0;
        int guessAction=(int)Rand::scalar(0,randActions.size());
        fprintf(stdout, "the guess action is %d chosen from size %d\n", guessAction, randActions.size());
        
        double orient = 0.0;
        // get the orientation of the blob
        orient = closestBlob.get(3).asDouble();
        
        Vector initPos;
        double finalOrient;
        if (get3DPosition(cog,initPos))
        {
            Bottle results;
            double offset = 0.0;
            results = getOffset(closestBlob, actionOrient, initPos);
            offset = results.get(0).asDouble();
            finalOrient = results.get(1).asDouble();
            
            fprintf(stdout,"Will now send to karmaMotor:\n");
            Bottle karmaMotor,KarmaReply;
            karmaMotor.addDouble(initPos[0]);
            karmaMotor.addDouble(initPos[1]);
            karmaMotor.addDouble(initPos[2] + 0.05);
            karmaMotor.addDouble(actionOrient);
            karmaMotor.addDouble( offset );// + 0.06 );

            fprintf(stdout,"%s\n",karmaMotor.toString().c_str());
            rpcMotorKarma.write(karmaMotor, KarmaReply);
            fprintf(stdout,"action is %s:\n",KarmaReply.toString().c_str());

            goHome();
        }
    }
    return 0;
}
/**********************************************************/
double Manager::wrapAng ( const double ang )
{
    if ((ang<0.0) || (ang>=360.0))
    {
        double theta=(M_PI/180.0)*ang;
        theta=(180.0/M_PI)*atan2(sin(theta),cos(theta));
        if (theta<0.0)
            theta+=360.0;
        return theta;
    }
    else
        return ang;
}
/**********************************************************/
Bottle Manager::getOffset( Bottle &closestBlob, double actionOrient, Vector &initPos )
{
    
    double orient = 0.0;
    orient = closestBlob.get(3).asDouble();
    
    int axe1 = 0;
    int axe2 = 0;
    CvPoint cog;
    cog.x = closestBlob.get(0).asInt();
    cog.y = closestBlob.get(1).asInt();
    
    axe1 = closestBlob.get(4).asInt();
    axe2 = closestBlob.get(5).asInt();
    double finalOrient=actionOrient;
    fprintf(stdout ,"INITIAL orientation is: %lf \n", orient);
    if (abs(axe1-axe2) < 5)
    {
        fprintf(stdout,"seem a round object to me...sending theta2\n");
    }
    else
    {
        fprintf(stdout,"sending theta2 - theta1 <-> axis diff is %d\n", abs(axe1-axe2));
        finalOrient -= orient;
    }

    CvPoint pxls;
    Vector offPos;
    fprintf(stdout,"The 3Dpos is %lf %lf %lf \n",initPos[0], initPos[1], initPos[2]);
    
    double alpha = finalOrient * M_PI/180; //radians
    pxls.x = int(cog.x + (axe1 * 1.2 ) * cos(alpha));
    pxls.y = int(cog.y - (axe2 * 1.2 ) * sin(alpha));
    
    get3DPosition(pxls,offPos);
    fprintf(stdout,"The 3Dpos off point is  %lf %lf %lf \n",offPos[0], offPos[1], offPos[2]);
    double offset= norm (initPos-offPos);
    fprintf(stdout,"The offset is %lf \n",offset);
    
    Bottle ret;
    ret.addDouble(offset);
    ret.addDouble(finalOrient);
    return ret;
}

/**********************************************************/
Bottle Manager::classify(const Bottle &blobs, int index)
{
    //grab resources
    mutexResources.wait();
    Bottle mils;
    mils.clear();

    Bottle gotMils;
    gotMils.clear();

    Bottle cmd,reply;
    cmd.addVocab(Vocab::encode("classify"));

    Bottle &options=cmd.addList();
    ostringstream tag;
    tag<<"blob_"<<index;
    Bottle &item=options.addList();
    item.addString(tag.str().c_str());
    item.addList()=*blobs.get(index).asList();

    //printf("Sending classification request: %s\n",cmd.toString().c_str());
    rpcMIL.write(cmd,reply);
    printf("Received reply: %s\n",reply.toString().c_str());
    mutexResources.post();
    //Bottle &toReturn = gotMils.addList();
    
    if (reply!=NULL)
    {
        CvPoint cog;
        cog=getBlobCOG(blobs,index);
        Bottle &tmpLine = gotMils.addList();
        Bottle &tmpMils = tmpLine.addList();
        tmpMils.addDouble(cog.x);
        tmpMils.addDouble(cog.y);
        tmpLine.addList()=*reply.get(0).asList()->get(1).asList();
    }
    mils.clear();
    mils.addList()=gotMils;
    //release resources
    return mils;
}
/**********************************************************/
Bottle Manager::getType(const yarp::os::Bottle *scores, int index)
{
    ostringstream tag;
    tag<<"blob_"<<index;
    if (scores!=NULL)
    {
        double max_score=0.0;
        string max_label="unknown";
        if (scores->get(0).asList()->size() > 0)
        {
            Bottle *tmp_scores=scores->get(0).asList()->get(0).asList()->get(1).asList();
            
            //fprintf(stdout,"bottle is: %s \n",tmp_scores->toString().c_str());
               
            for(int s=0; s<tmp_scores->size(); s++)
            {
                if(tmp_scores->get(s).asList()->get(1).asDouble()>max_score)
                {
                    max_score=tmp_scores->get(s).asList()->get(1).asDouble();
                    max_label=tmp_scores->get(s).asList()->get(0).asString().c_str();
                }
            }
        }
        //fprintf(stdout, "the prob is %lf ", scores->get(0).asList()->get(j).asList()->get(1).asList()->get(1).asDouble());
        //fprintf(stdout,"\n");
        tag.str("");
        tag.clear();
        tag<<max_label;
        Bottle type;
        type.clear();
        type.addString(max_label.c_str());
        return type;
    }
    Bottle type;
    type.clear();
    type.addString("error");
    return type;
}
/**********************************************************/
double Manager::getPeriod()
{
    return 0.1;
}

/**********************************************************/
bool Manager::get3DPosition(const CvPoint &point, Vector &x)
{
    Bottle cmdMotor,replyMotor;
    cmdMotor.addVocab(Vocab::encode("get"));
    cmdMotor.addVocab(Vocab::encode("s2c"));
    Bottle &options=cmdMotor.addList();
    //options.addString(camera.c_str());
    options.addInt(point.x);
    options.addInt(point.y);
    printf("Sending motor query: %s\n",cmdMotor.toString().c_str());
    rpcMotorAre.write(cmdMotor,replyMotor);
    printf("Received blob cartesian coordinates: %s\n",replyMotor.toString().c_str());

    if (replyMotor.size()>=3)
    {   
        x.resize(3);
        x[0]=replyMotor.get(0).asDouble();
        x[1]=replyMotor.get(1).asDouble();
        x[2]=replyMotor.get(2).asDouble();
        return true;
    }
    else
        return false;
}
/**********************************************************/
Bottle Manager::getBlobs()
{
    // grab resources
    mutexResources.wait();

    if (Bottle *pBlobs=blobExtractor.read(false))
    {
        lastBlobs=*pBlobs;
        printf("Received blobs list: %s\n",lastBlobs.toString().c_str());

        if (lastBlobs.size()==1)
        {
            if (lastBlobs.get(0).asVocab()==Vocab::encode("empty"))
                lastBlobs.clear();
        }
    }  
    // release resources
    mutexResources.post();

    return lastBlobs;
}
/**********************************************************/
CvPoint Manager::getBlobCOG(const Bottle &blobs, const int i)
{
    CvPoint cog=cvPoint(RET_INVALID,RET_INVALID);
    if ((i>=0) && (i<blobs.size()))
    {
        CvPoint tl,br;
        Bottle *item=blobs.get(i).asList();
        if (item==NULL)
            return cog;

        tl.x=(int)item->get(0).asDouble();
        tl.y=(int)item->get(1).asDouble();
        br.x=(int)item->get(2).asDouble();
        br.y=(int)item->get(3).asDouble();

        cog.x=(tl.x+br.x)>>1;
        cog.y=(tl.y+br.y)>>1;
    }
    return cog;
}
/**********************************************************/
Bottle Manager::findClosestBlob(const Bottle &blobs, const CvPoint &loc)
{
    int ret=RET_INVALID;
    double min_d2=1e9;
    Bottle pointReturn;
    pointReturn.clear();
    for (int i=0; i<blobs.size(); i++)
    {
        CvPoint cog=getBlobCOG(blobs,i);
        if ((cog.x==RET_INVALID) || (cog.y==RET_INVALID))
            continue;

        double dx=loc.x-cog.x;
        double dy=loc.y-cog.y;
        double d2=dx*dx+dy*dy;

        if (d2<min_d2)
        {
            min_d2=d2;
            ret=i;
        }
    }
    CvPoint cog=getBlobCOG( blobs, ret );
    pointReturn.addDouble(cog.x);   //cog x
    pointReturn.addDouble(cog.y);   //cog y
    pointReturn.addInt(ret);        //index of blob
    Bottle *item=blobs.get(ret).asList();
    double orient = (double)item->get(4).asDouble();
    int axe1 = (int)item->get(5).asInt();
    int axe2 = (int)item->get(6).asInt();
    pointReturn.addDouble(orient);
    pointReturn.addInt(axe1);
    pointReturn.addInt(axe2);
    return pointReturn;
}
