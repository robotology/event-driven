#pragma once
#include "visionController.h"

class visCtrlATIS3 : public visCtrlInterface
{
private:
    int readSisleyRegister(uint32_t sisley_reg_address, uint32_t *sisley_data);
    int writeSisleyRegister(uint32_t sisley_reg_address, uint32_t sisley_data);
    bool enableHSSAER();
    bool enableGTP();
    bool setROI(int x, int y, int width, int height);
    int setROIAXIS(int start, int size, xory_t coord, tdorem_t type);
    bool sisleySetup();
    bool updateBiases(yarp::os::Bottle &bias);
    void printSensitivyBiases();
    void setSensitivityBiases(int sensitivity);

public:
    visCtrlATIS3(int fd, channel_name channel) : visCtrlInterface(fd, channel) {};
    bool activate(bool activate = true) override;
    bool configure(yarp::os::ResourceFinder rf) override;

};