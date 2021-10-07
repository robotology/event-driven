#pragma once
#include "visionController.h"
#include "yarp/os/all.h"

class visCtrlATIS1 : public visCtrlInterface
{
private:
    //const static int VSCTRL_BG_CNFG_ADDR = 0x20;
    bool setLatchAtEnd(bool enable);
    bool setShiftCount(uint8_t shiftCount);
    bool setDefaultRegisterValues();
    bool updateBiases(yarp::os::Bottle &bias_list, bool voltage_biases = true);
    bool activateAPSshutter();


public:
    visCtrlATIS1(int fd, channel_name channel) : visCtrlInterface(fd, channel) {};
    bool activate(bool activate = true) override;
    bool configure(yarp::os::ResourceFinder rf) override;

};