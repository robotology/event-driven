// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org
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

/**
 * @file zynqGrabberModule.cpp
 * @brief Implementation of the zynqGrabberModule (see header file).
 */

#include <iCub/zynqGrabberModule.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;


//#define MAGIC_NUM 100
//#define SP2NEU_VERSION         _IOR (MAGIC_NUM,  7, void *)
//#define SP2NEU_TIMESTAMP       _IOR (MAGIC_NUM,  8, void *)
//#define SP2NEU_GEN_REG         _IOWR(MAGIC_NUM,  6, void *)
//#define SP2NEU_SET_LOC_LBCK    _IOW (MAGIC_NUM, 10, void *)
//#define SP2NEU_SET_REM_LBCK    _IOW (MAGIC_NUM, 11, void *)
//#define SP2NEU_SET_FAR_LBCK    _IOW (MAGIC_NUM, 12, void *)
//
//
//#define CTRL_REG     0x00
//#define RXDATA_REG   0x08
//#define RXTIME_REG   0x0C
//#define TXDATA_REG   0x10
//#define DMA_REG      0x14
//#define RAWI_REG     0x18
//#define IRQ_REG      0x1C
//#define MASK_REG     0x20
//#define STMP_REG     0x28
//#define ID_REG       0x5c
//
//// CTRL register bit field
////#define CTRL_ENABLEIP 0x00000001
//#define CTRL_ENABLEINTERRUPT 0x00000004
//#define CTRL_FLUSHFIFO       0x00000010
//#define CTRL_ENABLE_REM_LBCK 0x01000000
//#define CTRL_ENABLE_LOC_LBCK 0x02000000
//#define CTRL_ENABLE_FAR_LBCK 0x04000000
//
//// INterrupt Mask register bit field
//#define MSK_RXBUF_EMPTY  0x00000001
//#define MSK_RXBUF_AEMPTY 0x00000002
//#define MSK_RXBUF_FULL   0x00000004
//#define MSK_TXBUF_EMPTY  0x00000008
//#define MSK_TXBUF_AFULL  0x00000010
//#define MSK_TXBUF_FULL   0x00000020
//#define MSK_TIMEWRAPPING 0x00000080
//#define MSK_RXBUF_READY  0x00000100
//#define MSK_RX_NOT_EMPTY 0x00000200
//#define MSK_TX_DUMPMODE  0x00001000
//#define MSK_RX_PAR_ERR   0x00002000
//#define MSK_RX_MOD_ERR   0x00004000


/*
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the
 *  equivalent of the "open" method.
 */

bool zynqGrabberModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */
    printf("configure in zynqGrabberModule \n");
    
    //printf("moduleName  %s \n", moduleName.c_str());
    moduleName            = rf.check("name",
                                     Value("/zynqGrabber"),
                                     "module name (string)").asString();
    printf("extracted the module name \n");
    
    
    /*
     * before continuing, set the module name before getting any other parameters,
     * specifically the port names which are dependent on the module name
     */
    printf("setting the module rootname \n");
    setName(moduleName.c_str());
    
    /*
     * get the robot name which will form the stem of the robot ports names
     * and append the specific part and device required
     */
    robotName             = rf.check("robot",
                                     Value("icub"),
                                     "Robot name (string)").asString();
    robotPortName         = "/" + robotName + "/zynq";
    
    /*
     * get the device name which will be used to read events
     */
    deviceName             = rf.check("deviceName",
                                      Value("/dev/spinn2neu"),
                                      "Device name (string)").asString();
    //devicePortName         =  deviceName ;
    
    //setDeviceName(deviceName);
    //    printf("trying to connect to the device %s \n",deviceName.c_str());
    //
    //    if (openDevice()==false){
    //        fprintf(stdout,"error opening the device\n");
    //        return false;
    //    }
    
    
    
    /*
     * attach a port of the same name as the module (prefixed with a /) to the module
     * so that messages received from the port are redirected to the respond method
     */
    handlerPortName =  "";
    handlerPortName += getName();         // use getName() rather than a literal
    
    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;
        return false;
    }
    
    attach(handlerPort);                  // attach to port
    
    //bool _save = false;
    std::string deviceNum = "0";
    
    /*
     * get the file name of binaries when the biases are read from this file
     */
    binaryName             = rf.check("file",
                                      Value("none"),
                                      "filename of the binary (string)").asString();
    printf("trying to read %s  for biases \n",binaryName.c_str());
    binaryNameComplete = rf.findFile(binaryName.c_str());
    
    /*
     * get the file name of binaries when the biases are read from this file
     */
    dumpNameComplete = "";
    dumpName             = rf.check("dumpFile",
                                    Value("none"),
                                    "filename of the binary (string)").asString();
    printf("trying to save events in %s  \n",dumpName.c_str());
    dumpNameComplete = rf.findFile(dumpName.c_str());
    
    // class manageDevice
    devManager = new deviceManager(deviceName);
    
    // open device
    printf("trying to connect to the device %s \n",deviceName.c_str());
    bool success = devManager->openDevice();

    if (success == false){
        fprintf(stdout,"error opening the device\n");
        return false;
    }
    
    
    //open rateThread device2yarp
    
    D2Y = new device2yarp();
    D2Y->attachDeviceManager(devManager);
    if(!D2Y->threadInit(moduleName)) {
        //could not start the thread
        return false;
    }
    D2Y->start();
    
    
    //open bufferedPort yarp2device
    
    Y2D.attachDeviceManager(devManager);
    if(!Y2D.open(moduleName))
    {
        std::cerr << " : Unable to open ports" << std::endl;
        return false;
    }
    
    closing = false;
    return true ;
}

bool zynqGrabberModule::interruptModule() {
    handlerPort.interrupt();
    Y2D.interrupt();
    // D2Y ???
    return true;
}

bool zynqGrabberModule::close() {
    
    closing = true;
    
    handlerPort.close();        // rpc of the RF module
    Y2D.close();
    D2Y->stop();                // bufferedport from yarp to device
    //D2Y->threadRelease();       // ratethread from device to yarp
    
    devManager->closeDevice();  // device
    /* stop the thread */
    return true;
}

bool zynqGrabberModule::respond(const Bottle& command, Bottle& reply) {
    bool ok = false;
    bool rec = false; // is the command recognized?
    string helpMessage =  string(getName().c_str()) +
    " commands are: \n" +
    "help \n" +
    "quit \n" +
    "set thr <n> ... set the threshold \n" +
    "(where <n> is an integer number) \n";
    
    reply.clear();
    
    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
    
    mutex.wait();
    switch (command.get(0).asVocab()) {
        case COMMAND_VOCAB_HELP:
            rec = true;
        {
            reply.addString("many");
            reply.addString("help");
            
            reply.addString("");
            
            ok = true;
        }
            break;
        case COMMAND_VOCAB_SUSPEND:
            rec = true;
        {
            D2Y->suspend();
            ok = true;
        }
            break;
        case COMMAND_VOCAB_RESUME:
            rec = true;
        {
            D2Y->resume();
            ok = true;
        }
            break;
        case COMMAND_VOCAB_SETBIAS:
            rec = true;
        {
            string biasName = command.get(1).asString();
            double biasValue = command.get(2).asDouble();
            int channel = command.get(3).asInt();
            
            // setBias function
            // biasManager.setBias(biasName, biasValue, channel);
            ok = true;
        }
            break;
        case COMMAND_VOCAB_PROG:
            rec= true;
        {
            int channel = command.get(1).asInt();
            
            // progBias function
            // biasManager.progBias(channel);
            ok = true;
            
        }
            break;
    }
    
    mutex.post();
    
    if (!rec)
        ok = RFModule::respond(command,reply);
    
    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);
    
    return ok;
    
    return true;
}

/* Called periodically every getPeriod() seconds */
bool zynqGrabberModule::updateModule() {
    
    return !closing;
    
    return true;
}

double zynqGrabberModule::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 1.0;
}


////----------------------------------------------------------------------------------------------------
//// functions for device opening
////----------------------------------------------------------------------------------------------------
//void zynqGrabberModule::setDeviceName(string deviceName) {
//    printf("saving portDevice \n");
//    portDeviceName=deviceName;
//}
//
//void zynqGrabberModule::closeDevice(){
//    ::close(devDesc);
//    fprintf(stdout, "closing device %s \n",portDeviceName.c_str());
//}
//
//bool zynqGrabberModule::openDevice(){
//    //opening the device
//    cout <<"name of the file buffer:" <<portDeviceName.c_str()<< endl;
//    devDesc = open(portDeviceName.c_str(), O_RDWR | O_NONBLOCK);
//    if (devDesc < 0) {
//        printf("Cannot open device file: %s \n",portDeviceName.c_str());
//        return false;
//    }
//
//    //initialization for writing to device
//    unsigned long version;
//    unsigned char hw_major,hw_minor;
//    char          stringa[4];
//    int i;
//    unsigned int  tmp_reg;
//
//    ioctl(devDesc, SP2NEU_VERSION, &version);
//
//    hw_major = (version & 0xF0) >> 4;
//    hw_minor = (version & 0x0F);
//    stringa[3]=0;
//
//    for (i=0; i<3; i++) {
//        stringa[i] = (version&0xFF000000) >> 24;
//        version = version << 8;
//    }
//    fprintf(stderr, "\r\nIdentified: %s version %d.%d\r\n\r\n", stringa, hw_major, hw_minor);
//
//    // Write the WrapTimeStamp register with any value if you want to clear it
//    //write_generic_sp2neu_reg(fp,STMP_REG,0);
//    fprintf(stderr, "Times wrapping counter: %d\n", read_generic_sp2neu_reg(devDesc, STMP_REG));
//
//    // Enable Time wrapping interrupt
//    write_generic_sp2neu_reg(devDesc, MASK_REG, MSK_TIMEWRAPPING | MSK_TX_DUMPMODE | MSK_RX_PAR_ERR | MSK_RX_MOD_ERR);
//
//    // Flush FIFOs
//    tmp_reg = read_generic_sp2neu_reg(devDesc, CTRL_REG);
//    write_generic_sp2neu_reg(devDesc, CTRL_REG, tmp_reg | CTRL_FLUSHFIFO); // | CTRL_ENABLEIP);
//
//    // Start IP in LoopBack
//    tmp_reg = read_generic_sp2neu_reg(devDesc, CTRL_REG);
//    write_generic_sp2neu_reg(devDesc, CTRL_REG, tmp_reg | (CTRL_ENABLEINTERRUPT | CTRL_ENABLE_FAR_LBCK));
//    //    write_generic_sp2neu_reg(devDesc, CTRL_REG, tmp_reg | CTRL_ENABLE_FAR_LBCK);
//
//    tmp_reg = read_generic_sp2neu_reg(devDesc, CTRL_REG);
//    write_generic_sp2neu_reg(devDesc, CTRL_REG, tmp_reg | CTRL_ENABLE_FAR_LBCK);
//
//
//
//
//    return true;
//}
//

//void zynqGrabberModule::write_generic_sp2neu_reg (int devDesc, unsigned int offset, unsigned int data) {
//    sp2neu_gen_reg_t reg;
//
//    reg.rw = 1;
//    reg.data = data;
//    reg.offset = offset;
//    ioctl(devDesc, SP2NEU_GEN_REG, &reg);
//}
//
//
//unsigned int zynqGrabberModule::read_generic_sp2neu_reg (int devDesc, unsigned int offset) {
//    sp2neu_gen_reg_t reg;
//
//    reg.rw = 0;
//    reg.offset = offset;
//    ioctl(devDesc, SP2NEU_GEN_REG, &reg);
//
//    return reg.data;
//}
//
//
//void zynqGrabberModule::usage (void) {
//    fprintf (stderr, "%s <even number of data to transfer>\n", __FILE__);
//    //exit(1);
//}
//

