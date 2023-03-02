#include "visCtrlATIS1.h"
#include "visionController.h"
#include <yarp/os/all.h>

bool visCtrlATIS1::configure(yarp::os::ResourceFinder rf) {
    yInfo() << "Turning off camera";
    activate(false);
    yInfo() << "Setting register values";
    setDefaultRegisterValues();
    std::string bias_group_name = channel == LEFT ? "ATIS1_BIAS_LEFT"
                                                  : "ATIS1_BIAS_RIGHT";
    if (rf.check(bias_group_name)) {
        yInfo() << "Programming biases:" << bias_group_name;
        updateBiases(rf.findGroup(bias_group_name));
    } else {
        yError() << "No biases found:" << bias_group_name;
    }

    //we could set APS on here
    //we could look to use current-based biases
    //however these two options never worked and need to be checked if
    //they were done correctly in anycase.

    return true;
}

bool visCtrlATIS1::activate(bool activate) 
{
    channelSelect(fd, channel);
    unsigned int config_reg;

    //get current config state
    if(i2cRead(fd, VSCTRL_BG_CNFG_ADDR, (unsigned char *)&config_reg, 
            sizeof(config_reg)) != sizeof(config_reg))
        return false;

    //alter the correct bit
    if(activate)
        config_reg &= ~BG_PWRDWN_MSK;
    else
        config_reg |= BG_PWRDWN_MSK;

    //rewrite the new config status
    return i2cWrite(fd, VSCTRL_BG_CNFG_ADDR, (unsigned char *)(&config_reg), 
            sizeof(config_reg)) == sizeof(config_reg);
}
bool visCtrlATIS1::setDefaultRegisterValues() 
{
    channelSelect(fd, channel);
    unsigned char valReg[4];

    // --- configure BG Timings --- //
    valReg[0] = BG_LAT;  // Latch Active Time
    valReg[1] = BG_LS;  // Latch Setup
    valReg[2] = BG_CAT;  // Clock Active Time
    valReg[3] = BG_SHT;  // Setup Hold Time
    if(i2cWrite(fd, VSCTRL_BG_TIMINGS_ADDR, valReg, 4) < 0) return false;

    // --- configure BG Levels --- //
    valReg[0] = BG_CNFG;   // BGtype = 1 (ATIS), BG overwrite = 1, CK active level = 1, LATCH active level = 1
    valReg[1] = 0x00;   // reserved
    valReg[2] = BG_LATEND_SHCNT;   // LatchOut@end = 1, ShiftCount = 32
    valReg[3] = BG_ROI;   // Choose if setting ROI or setting BG (0 -> BG, 1 -> ROI)
    if(i2cWrite(fd, VSCTRL_BG_CNFG_ADDR, valReg, 4) < 0) return false;

    // --- configure BG Prescaler --- //
    for (int i = 0; i < 4; i++)
        valReg[i]  = (BG_PRESC >> (i*8)) & 0xFF;
    if(i2cWrite(fd, VSCTRL_BG_PRESC_ADDR, valReg, 4) < 0) return false;

    // --- configure Source Config --- //
    valReg[0] = AER_LVL;   // AER Ack and Req Levels (Ack active low, Req active high)
    valReg[1] = ACK_SET_DEL;   // Ack Set Delay 20 ns
    valReg[2] = ACK_SAM_DEL;   // Ack Sample Delay 30 ns
    valReg[3] = ACK_REL_DEL;   // Ack Release Delay 50ns
    if(i2cWrite(fd, VSCTRL_SRC_CNFG_ADDR, valReg, 4) < 0) return false;

    // --- configure Source Destination Control --- //
    valReg[0] =  TD_APS_CTRL;  // TD loopback = 0, TD EN =1, APS loppback = 0, APS EN = 1, flush fifo = 0, ignore FIFO Full = 0
    // if(aps) {
    //     std::cout << "APS events enabled";
    //     valReg[0] |= APS_CTRL;
    // }
    valReg[1] = SRC_CTRL;   // Flush FIFOs = 0, Ignore FIFO Full = 0, PAER En = 0, SAER En = 1, GTP En = 0, Sel DEST = 01 (HSSAER)
    valReg[2] = 0;      // reserved
    valReg[3] = 0;      // reserved
    if(i2cWrite(fd, VSCTRL_SRC_DST_CTRL_ADDR, valReg, 4) < 0) return false;

    // --- configure HSSAER --- //
// --- this should be done only if we use HSSAER (with ATIS, SKIN, but not with SpiNNaker nor DVS)
    valReg[0] =  VSCTRL_ENABLE_ALLCHANNELS;     // enable ch0, ch1, ch2
    valReg[1] = 0;          // reserved
    valReg[2] = 0;          // reserved
    valReg[3] = 0;          // reserved
    if(i2cWrite(fd, VSCTRL_HSSAER_CNFG_ADDR, valReg, 4) < 0) return false;

    // --- configure GPO register --- //
    valReg[0] = 0x00;// 0x2;
    valReg[1] = 0x00; //0x4;
    valReg[2] = 0x00;
    valReg[3] = 0x00;
    if(i2cWrite(fd, VSCTRL_GPO_ADDR, valReg, 4) < 0) return false;

    //read and set biases

    return true;
}

bool visCtrlATIS1::setLatchAtEnd(bool enable){

    channelSelect(fd, channel);
    unsigned int val;

    if(i2cRead(fd, VSCTRL_BG_CNFG_ADDR, (unsigned char *)&val, sizeof(val)) < 0) return false;

    if (enable == true) {
        val |= BG_LATOUTEND_MSK;
    } else {
        val &= ~BG_LATOUTEND_MSK;
    }

    if(i2cWrite(fd, VSCTRL_BG_CNFG_ADDR, (unsigned char *)(&val), sizeof(int)) < 0)
        return false;

    return true;
}

bool visCtrlATIS1::setShiftCount(uint8_t shiftCount){
    
    channelSelect(fd, channel);
    unsigned int val;

    if(i2cRead(fd, VSCTRL_BG_CNFG_ADDR, (unsigned char *)&val, sizeof(val)) < 0)
        return false;

    val = (val & ~BG_SHIFT_COUNT_MSK) | ((shiftCount << 16) & BG_SHIFT_COUNT_MSK);
    if(i2cWrite(fd, VSCTRL_BG_CNFG_ADDR, (unsigned char *)(&val), sizeof(int)) < 0)
        return false;

    return true;
}

bool visCtrlATIS1::updateBiases(yarp::os::Bottle &bias_list, bool voltage_biases) {

    channelSelect(fd, channel);
    clearStatusReg(fd);
    activate(false);

    // send the first 4 bits (disabling the Latch)
    if(!setLatchAtEnd(false)) return false;
    if(!setShiftCount(ATIS_PDSHIFT)) return false;

    unsigned int pds = ATIS_PDSTRENGTH; //paddrivestrength
    if(i2cWrite(fd, VSCTRL_BG_DATA_ADDR, (unsigned char *)&pds, sizeof(pds)) < 0)
        return false;

    // set the number of bits in each bias (ATIS is 32, DVS is 24)
    if(!setShiftCount(ATIS_BIASSHIFT))
        return false;

    yInfo() << "Programming " << bias_list.size() - 1 << " biases:";
    yInfo() << "== Bias Hex Stream ==";
    printf("0x%02X\n", 3);
    double vref, voltage;
    int header;
    size_t i;
    for(i = 1; i < bias_list.size(); i++) {
        
        //for the very last element set the latch at end to true again
        if(i == bias_list.size() - 1) 
            if(!setLatchAtEnd(true)) 
                return false;
        //turn the bias value into the appropriate hex
        yarp::os::Bottle *biasdata = bias_list.get(i).asList();
        vref = biasdata->get(1).asInt32();
        header = biasdata->get(2).asInt32();
        voltage = biasdata->get(3).asInt32();
        unsigned int biasVal = 0;
        if(!voltage_biases)
            biasVal = voltage;
         else
            biasVal = 255 * (voltage / vref);
        biasVal += header << 21;
        printf("0x%08X ", biasVal);
        if(i2cWrite(fd, VSCTRL_BG_DATA_ADDR, (unsigned char *)&biasVal, sizeof(biasVal)) != sizeof(biasVal))
            return false;
    }
    printf("\n");

    // //set the latch true for the last bias
    // if(!setLatchAtEnd(true)) return false;
    // yarp::os::Bottle *biasdata = bias_list.get(i).asList();
    // vref = biasdata->get(1).asInt32();
    // header = biasdata->get(2).asInt32();
    // voltage = biasdata->get(3).asInt32();
    // unsigned int biasVal = 0;
    // if(!voltage_biases)
    //     biasVal = voltage;
    // else
    //     biasVal = 255 * (voltage / vref);
    // biasVal += header << 21;
    // //std::cout << biasdata->get(0).asString() << " " << biasVal << std::endl;
    // if(i2cWrite(fd, VSCTRL_BG_DATA_ADDR, (unsigned char *)&biasVal, sizeof(biasVal)) != sizeof(biasVal))
    //     return false;
    
    yarp::os::Time::delay(0.05);
    if(!checkBiasDone(fd)) {
        yarp::os::Time::delay(1.0);
        if(!checkBiasDone(fd))
            yError() << "Bias done flag not set!";
    }
    if(checkCRCError(fd))
        yError() << "CRC error occured during bias program";

    clearStatusReg(fd);

    return true;
}

bool visCtrlATIS1::activateAPSshutter()
{
    unsigned char valReg[4];
    valReg[0] = 0x00;
    valReg[1] = 0x20;
    valReg[2] = 0x00;
    valReg[3] = 0x00;
    if(i2cWrite(fd, VSCTRL_GPO_ADDR, valReg, 4) < 0)
        return false;
    return true;
}
