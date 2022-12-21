/*
 *   Copyright (C) 2017 Event-driven Perception for Robotics
 *   Author: arren.glover@iit.it
 *           chiara.bartolozzi@iit.it
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "hpuInterface.h"
#include "deviceRegisters.h"

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <iostream>
#include <iomanip>
#include <cstring>

using namespace yarp::os;
using namespace ev;
using std::string;
using std::vector;

/******************************************************************************/
//device2yarp
/******************************************************************************/
device2yarp::device2yarp()
{
    fd = -1;
    max_dma_pool_size = 0;
    max_packet_size = 0;
}

void device2yarp::configure(string module_name, int fd, unsigned int pool_size,
                       unsigned int packet_size, bool record_mode)
{
    this->fd = fd;

    if(pool_size > packet_size) {
        packet_size = pool_size;
        yWarning() << "Setting packet_size to pool_size:" << packet_size;
    }
    this->max_packet_size = packet_size;
    this->max_dma_pool_size = pool_size;
    this->port_name = module_name + "/AE:o";
    this->skin_port_name = module_name + "/skin:o";
    
    // min_packet_duration = 0.0 by default
    if(record_mode) 
    {
        min_packet_duration = 0.004; // 4 ms
        yInfo() << "RECORD MODE [ON]: packets minimum 4ms";
    }
}

void device2yarp::yarpOpen()
{
    if(AE_output_port.isClosed()) {
        if(!Network::checkNetwork(1.0)) {
            yInfo() << "D2Y: YARP network not available";
        } else {
            if(AE_output_port.open(port_name))
                yInfo() << "D2Y: opened" << port_name;
            else
                yInfo() << "D2Y: cannot open" << port_name;

            if(skin_output_port.open(skin_port_name))
                yInfo() << "D2Y: opened" << skin_port_name;
            else
                yInfo() << "D2Y: cannot open" << skin_port_name;
        }
    }
}

void  device2yarp::run() {

    if(fd < 0) {
        yError() << "HPU reading device not open";
        return;
    }  

    unsigned int event_count = 0;
    unsigned int packet_count = 0;

    double tic = yarp::os::Time::now();

    while(!isStopping()) {

        ev::packet<encoded> packet;
        ev::packet<encoded>& AE_packet = AE_output_port.prepare();
        ev::packet<encoded>& skin_packet = skin_output_port.prepare();
        packet.clear();
        packet.size(max_packet_size / sizeof(AE) + (max_packet_size % sizeof(AE) ? 1 : 0));
        AE_packet.clear();
        AE_packet.size(max_packet_size / sizeof(AE) + (max_packet_size % sizeof(AE) ? 1 : 0));
        skin_packet.clear();
        skin_packet.size(max_packet_size / sizeof(AE) + (max_packet_size % sizeof(AE) ? 1 : 0));

        bool iswriting = true;
        double time_accum = 0.0;
        while(iswriting || time_accum < min_packet_duration) 
        {
            int r = packet.singleDeviceRead(fd);
            iswriting = AE_output_port.isWriting();
            time_accum = yarp::os::Time::now() - tic;
            if(r == 0) break; //the packet is full, we cannot read anymore data!
        }
        tic += time_accum;

        for (auto & p: packet){
            if (IS_SKIN(p.data)){
                skin_packet.push_back(p); // TODO add skin.process from vPreProcess
                continue;
            }
            AE_packet.push_back(p);
        }

        AE_packet.duration(time_accum);
        skin_packet.duration(time_accum);
        

        if(AE_packet.size() == 0) {
            yError() << "0 size packet?";
            AE_output_port.unprepare();
            continue;
        }
        if(skin_packet.size() == 0) {
            //yError() << "0 size packet?";
            skin_output_port.unprepare();
            //continue;
        }

        yarp_stamp.update();
        AE_packet.envelope() = yarp_stamp;
        //skin_packet.envelope() = yarp_stamp;
        AE_output_port.write();
        //skin_output_port.write();

        //report statistics to yarp
        event_count += AE_packet.size();
        event_count += skin_packet.size();
        packet_count++;
        static double prev_ts = yarp::os::Time::now();
        double update_period = yarp::os::Time::now() - prev_ts;
        
        if(update_period > 5.0) {

            yInfo() << "[READ ]"
                    << (int)(event_count/(1000.0*update_period))
                    << "k events/s over"
                    << packet_count
                    << "packets";

            prev_ts += update_period;
            event_count = 0;
            packet_count = 0;
        }
    }

}

void device2yarp::onStop()
{
    AE_output_port.close();
    skin_output_port.close();
}

/******************************************************************************/
//yarp2device
/******************************************************************************/
yarp2device::yarp2device()
{
    fd = -1;
}

void yarp2device::configure(std::string module_name, int fd)
{
    this->fd = fd;
    port_name = module_name + "/AE:i";
}

void yarp2device::yarpOpen()
{
    if(input_port.isClosed()) {
        if(!Network::checkNetwork(1.0)) {
            yInfo() << "D2Y: YARP network not available";
        } else {
            if(input_port.open(port_name))
                yInfo() << "D2Y: opened" << port_name;
            else
                yInfo() << "D2Y: cannot open" << port_name;
        }
    }
}

void yarp2device::onStop()
{
    input_port.interrupt();
    input_port.close();
}

void yarp2device::run()
{
    int total_events = 0;

    while(true) {

        if(input_port.isClosed()) {
            Time::delay(1.0);
            continue;
        }

        const packet<AE>* packet = input_port.read(true); //blocking read
        if(!packet) return; //when interrupt is called returns null

        int written = packet->pushToDevice(fd);
        total_events += written / sizeof(AE);

        static double previous_time = yarp::os::Time::now();
        double dt = yarp::os::Time::now() - previous_time;
        if(dt > 5.0) {

            hpu_regs_t hpu_regs = {0x18, 0, 0};
            if (-1 == ioctl(fd, HPU_GEN_REG, &hpu_regs)){
                yWarning() << "Couldn't read dump status";
            }

            if(hpu_regs.data & 0x00100000) {
                yInfo() << "[DUMP ] " << (int)(0.001 * total_events / dt) << " k events/s ("
                    << input_port.getPendingReads() << " delayed packets)";
            } else {
                yInfo() << "[WRITE] " << (int)(0.001 * total_events / dt) << " k events/s ("
                    << input_port.getPendingReads() << " delayed packets)";
            }

            total_events = 0;
            previous_time += dt;
        }

    }


}

/******************************************************************************/
//hpuInterface
/******************************************************************************/
hpuInterface::hpuInterface()
{
    fd = -1;
    read_thread_open = false;
    write_thread_open = false;
}

bool hpuInterface::configureDevice(string device_name, bool spinnaker, bool loopback, bool gtp)
{
    //open the device
    fd = open(device_name.c_str(), O_RDWR);
    if(fd < 0) {
        fd = open(device_name.c_str(), O_RDONLY | O_NONBLOCK);
        if(fd < 0) {
            yError() << "Could not open" << device_name << " device";
            return false;
        } else {
            yWarning() << device_name << "only opened in read-only, "
                       "non-blocking mode";
        }
    }


    //READ ID
    unsigned int version = 0;
    if(ioctl(fd, HPU_VERSION, &version) < 0)
        { yError() << "Could not read version"; return false; }

    char version_word[5];
    version_word[0] = (char)(version >> 24);
    version_word[1] = (char)(version >> 16);
    version_word[2] = (char)(version >> 8);
    version_word[3] = '-';
    version_word[4] = '\0';
    yInfo() << "ID and Version " << version_word
            << (int)((version >> 4) & 0xF) << "." << (int)((version >> 0) & 0xF);

    //32 bit timestamp
    uint32_t timestampswitch = 1;
    if(ioctl(fd, HPU_TS_MODE, &timestampswitch) < 0)
        { yError() << "Could not write timestamp mode"; return false; }

    uint32_t dma_latency = 1;
    if(ioctl(fd, HPU_AXIS_LATENCY, &dma_latency) < 0)
        { yError() << "Could not write dma latency"; return false; }

    uint32_t minimum_packet = 8; //bytes (not events)
    if(ioctl(fd, HPU_SET_BLK_RX_THR, &minimum_packet) < 0)
        { yError() << "Could not write BLK_RX_THR"; return false; }

    //read the pool size
    if(ioctl(fd, HPU_GET_RX_PS, &pool_size) < 0)
        { yError() << "Could not read pool size"; return false; }
    if(pool_size < 0 || pool_size > 32768) {
        yWarning() << "Pool size invalid (" << pool_size << "). Setting to ("
                     "4096)";
        pool_size = 4096;

    }

    unsigned int pool_count;
    if(ioctl(fd, HPU_GET_RX_PN, &pool_count) < 0)
        { yError() << "Could not read pool count"; return false; }

    hpu_interface_cfg_t trans_config;
    hpu_rx_interface_ioctl_t rx_config;
    if(gtp)
        trans_config = {{0, 0, 0, 0}, 1, 0, 0};
    else
        trans_config = {{1, 1, 1, 1}, 0, 0, 0};
        
    rx_config = {INTERFACE_EYE_R, {{0, 0, 0, 0}, 1, 0, 0}};
    if(ioctl(fd, HPU_RX_INTERFACE, &rx_config) < 0)
        { yError() << "Could not write EYE_R config"; return false; }

    rx_config = {INTERFACE_EYE_L, {{0, 0, 0, 0}, 1, 0, 0}};
    if(ioctl(fd, HPU_RX_INTERFACE, &rx_config) < 0)
        { yError() << "Could not write EYE_L config"; return false; }

    rx_config = {INTERFACE_AUX, {{0, 0, 0, 0}, 0, 0, 0}};
    if(ioctl(fd, HPU_RX_INTERFACE, &rx_config) < 0)
        { yError() << "Could not write AUX config"; return false; }

    hpu_tx_interface_ioctl_t tx_config = {{{0, 0, 0, 0}, 0, 0, 0}, ROUTE_FIXED};
    if(ioctl(fd, HPU_TX_INTERFACE, &tx_config) < 0)
        { yError() << "Could not write hpu tx config"; return false; }

    //READ CTRL_REG status
#if ENABLE_TS
    unsigned int ts_flag = 1;
    yInfo() << "ON: individual event timestamps";
#else
    unsigned int ts_flag = 0;
    yInfo() << "OFF: individual event timestamps";
#endif
    if(ioctl(fd, HPU_SET_RX_TS_EN, &ts_flag) < 0)
        { yError() << "Could not enable/disable timestamps (RX)"; return false; }
    if(ioctl(fd, HPU_SET_TX_TS_EN, &ts_flag) < 0)
        { yError() << "Could not enable/disable timestamps (TX)"; return false; }

    hpu_regs_t hpu_regs = {0, 0, 0};
    ioctl(fd, HPU_GEN_REG, &hpu_regs);
    std::stringstream ss;
    ss << "CTRL_REG: " << "0x" << std::hex << std::setw(8)
       << std::setfill('0') << hpu_regs.data << std::endl;
    yInfo() << ss.str();

    if(spinnaker) {

        yInfo() << "Configuring SpiNNaker";

        hpu_tx_interface_ioctl_t tx_config = {{{0, 0, 0, 0}, 0, 0, 1}, ROUTE_FIXED};
        if(ioctl(fd, HPU_TX_INTERFACE, &tx_config) < 0)
            { yError() << "Could not write hpu tx config"; return false; }

        spinn_keys_t ss_keys = {0x80000000, 0x40000000};
        ioctl(fd, HPU_SET_SPINN_KEYS, &ss_keys);

        spinn_keys_enable_t ss_policy = {0, 0, 1};
        ioctl(fd, HPU_SPINN_KEYS_EN, &ss_policy);

        hpu_regs_t hpu_regs;

        //SPNN_TX_MASK_REG
        unsigned int tx_mask = 0x00FFFFFF;
        ioctl(fd, HPU_SPINN_TX_MASK, &tx_mask);

        //SPNN_RX_MASK_REG
        unsigned int rx_mask = 0x00FFFFFF;
        ioctl(fd, HPU_SPINN_TX_MASK, &rx_mask);

        //ENABLE loopback
        spinn_loop_t lbmode = LOOP_NONE;
        if(loopback) {
            yWarning() << "SpiNNaker in Loopback mode";
            lbmode = LOOP_LSPINN;
        }
        ioctl(fd, HPU_SET_LOOPBACK, &lbmode);

        //SET TX synch
        hpu_regs = {0x44, 0, 0};
        ioctl(fd, HPU_GEN_REG, &hpu_regs); //read
        hpu_regs.data |= 2 << 12; //TX timing mode [0 1 2]
        //hpu_regs.data |= 0 << 14; //forces trigger [0 1]
        //hpu_regs.data |= 1 << 15; //enable resync
        hpu_regs.data |= 0x9 << 16; //resynch frequency
        hpu_regs.data |= 1 << 20; //differential mask size (1 = 20 bits)
        std::cout << "TX status: ";
        std::cout << "0x" << std::hex << std::setw(8)
                  << std::setfill('0') << hpu_regs.data << std::endl;
        hpu_regs.rw = 1;
        ioctl(fd, HPU_GEN_REG, &hpu_regs); //write

        //READ CTRL_REG status
        hpu_regs = {0, 0, 0};
        ioctl(fd, HPU_GEN_REG, &hpu_regs);
        std::cout << "CTRL_REG: ";
        std::cout << "0x" << std::hex << std::setw(8)
                  << std::setfill('0') << hpu_regs.data << std::endl;

        //READ Raw Status Register
        hpu_regs = {0x18, 0, 0};
        ioctl(fd, HPU_GEN_REG, &hpu_regs);
        std::cout << "Raw Status: ";
        std::cout << "0x" << std::hex << std::setw(8)
                  << std::setfill('0') << hpu_regs.data << std::endl;
    }

    yInfo() << "DMA pool size:" << pool_size;
    yInfo() << "DMA pool count:" << pool_count;
    yInfo() << "DMA latency:" << 1 << "ms";
    yInfo() << "Mimumum driver read:" << 8 << "bytes";

    return true;
}

bool hpuInterface::openReadPort(string module_name, unsigned int packet_size, bool record_mode)
{
    if(fd < 0)
        return false;
    D2Y.configure(module_name, fd, pool_size, packet_size, record_mode);
    yInfo() << "Maximum packet size:" << packet_size;
    read_thread_open = true;
    D2Y.yarpOpen();
    return true;
}

void hpuInterface::tryconnectToYARP()
{
    if(read_thread_open)
        D2Y.yarpOpen();
    if(write_thread_open)
        Y2D.yarpOpen();
}

bool hpuInterface::openWritePort(string module_name)
{
    if(fd < 0)
        return false;
    Y2D.configure(module_name, fd);
    write_thread_open = true;
    Y2D.yarpOpen();
    return true;
}


void hpuInterface::start()
{
    if(read_thread_open)
        D2Y.start();
    if(write_thread_open)
        Y2D.start();
}

void hpuInterface::stop()
{
    if(read_thread_open) {
        D2Y.stop();
        read_thread_open = false;
    }
    if(write_thread_open) {
        Y2D.stop();
        write_thread_open = false;
    }

    //READ Raw Status Register
    hpu_regs_t hpu_regs = {0x18, 0, 0};
    if (-1 == ioctl(fd, HPU_GEN_REG, &hpu_regs)){
        yError() << "Error: cannot read Raw Status";
    }

    std::cout << "Raw Status: ";
    std::cout << std::hex << hpu_regs.data << std::endl;

    close(fd); fd = -1;
}
