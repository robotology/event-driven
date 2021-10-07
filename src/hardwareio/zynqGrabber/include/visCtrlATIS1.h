#pragma once
#include "visionController.h"

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
    bool configure(yarp::os::ResourceFinder rf) override
    {
        activate(false);
        setDefaultRegisterValues();
        std::string bias_group_name = channel == LEFT ? "ATIS1_BIAS_LEFT"
                                                      : "ATIS1_BIAS_RIGHT";
        if(rf.check(bias_group_name)) {
            updateBiases(rf.findGroup(bias_group_name));
        }

        //we could set APS on here
        //we could look to use current-based biases
        //however these two options never worked and need to be checked if
        //they were done correctly in anycase.

        return true;
    }

};