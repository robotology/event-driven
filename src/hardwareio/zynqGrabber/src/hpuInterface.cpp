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

/******************************************************************************/
//device2yarp
/******************************************************************************/

device2yarp::device2yarp()
{
    fd = -1;
    max_dma_pool_size = 0;
    max_packet_size = 0;
}

bool device2yarp::open(string module_name, int fd, unsigned int pool_size,
                       unsigned int packet_size)
{
    this->fd = fd;

    if(pool_size > packet_size) {
        packet_size = pool_size;
        yWarning() << "Setting packet_size to pool_size:" << packet_size;
    }
    this->max_packet_size = packet_size;
    this->max_dma_pool_size = pool_size;

    data.resize(max_packet_size);

    return output_port.open(module_name + "/AE:o");
}

void  device2yarp::run() {

    if(fd < 0) {
        yError() << "HPU reading device not open";
        return;
    }

    vPortableInterface external_storage;
    external_storage.setHeader(AE::tag);

    unsigned int event_count = 0;
    unsigned int prev_ts = 0;

    while(!isStopping()) {

        int r = max_dma_pool_size;
        unsigned int n_bytes_read = 0;
        while(r >= (int)max_dma_pool_size && n_bytes_read < max_packet_size) {
            r = read(fd, data.data() + n_bytes_read, max_packet_size - n_bytes_read);
            if(r < 0)
                yInfo() << "[READ ]" << std::strerror(errno);
            else
                n_bytes_read += r;
        }

        if(n_bytes_read == 0) continue;
        
        unsigned int first_ts = *(unsigned int *)data.data();
        if(prev_ts > first_ts)
            yWarning() << prev_ts << "->" << first_ts;
        prev_ts = first_ts;
        

        external_storage.setExternalData((const char *)data.data(), n_bytes_read);
        yarp_stamp.update();
        output_port.setEnvelope(yarp_stamp);
        output_port.write(external_storage);

        event_count += n_bytes_read / 8;

        static double prev_ts = yarp::os::Time::now();
        double update_period = yarp::os::Time::now() - prev_ts;
        if(update_period > 5.0) {

            yInfo() << "[READ ]"
                    << (int)(event_count/(1000.0*update_period))
                    << "kV/s";

            prev_ts += update_period;
            event_count = 0;
        }
    }

}

void device2yarp::onStop()
{
    output_port.close();
}

/******************************************************************************/
//yarp2device
/******************************************************************************/
yarp2device::yarp2device()
{
    fd = -1;
    total_events = 0;
}

bool yarp2device::open(std::string module_name, int fd)
{
    this->fd = fd;
    input_port.setStrict();
    return input_port.open(module_name + "/AE:i");
}

void yarp2device::onStop()
{
    input_port.interrupt();
    input_port.close();
}

void yarp2device::run()
{

    while(true) {

        Bottle *yarp_bottle = input_port.read(true); //blocking read
        //std::cout << ".";
        if(!yarp_bottle) return; //when interrupt is called returns null

        Bottle *data_bottle = yarp_bottle->get(1).asList();

        size_t data_size = data_bottle->size();
        if(data_size > data_copy.size())
            data_copy.resize(data_size, 0);

        //copy to internal data (needed to modify the contents)
        //we can remove data_copy and just use data_bottle when the
        //mask is done in the FPGA
        for(size_t i = 0; i < data_size; i += 1)
            data_copy[i] = data_bottle->get(i).asInt();


        //move to bytes space
        char * buffer = (char *)data_copy.data();
        size_t bytes_to_write = data_size * sizeof(int);
        size_t written = 0;
        while(written < bytes_to_write) {

            int ret = write(fd, buffer + written, bytes_to_write - written);

            if(ret > 0) { //success!
                written += ret;
            } else if(ret < 0 && errno != EAGAIN) { //error!
                perror("Error writing to device: ");
                return;
            }
        }

        total_events += written / (2.0 * sizeof(int));

        static double previous_time = yarp::os::Time::now();
        double dt = yarp::os::Time::now() - previous_time;
        if(dt > 5.0) {

            hpu_regs_t hpu_regs = {0x18, 0, 0};
            if (-1 == ioctl(fd, HPU_GEN_REG, &hpu_regs)){
                yWarning() << "Couldn't read dump status";
            }

            if(hpu_regs.data & 0x00100000) {
                yInfo() << "[DUMP ] " << (int)(0.001 * total_events / dt) << " kV/s ("
                    << input_port.getPendingReads() << " delayed packets)";
            } else {
                yInfo() << "[WRITE] " << (int)(0.001 * total_events / dt) << " kV/s ("
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

bool hpuInterface::configureDevice(string device_name, bool spinnaker, bool loopback)
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
    ioctl(fd, HPU_VERSION, &version);
    char version_word[4];
    version_word[0] = (char)(version >> 24);
    version_word[1] = (char)(version >> 16);
    version_word[2] = (char)(version >> 8);
    version_word[3] = '-';
    yInfo() << "ID and Version " << version_word
            << (int)((version >> 4) & 0xF) << "." << (int)((version >> 0) & 0xF);

    //32 bit timestamp
    uint32_t timestampswitch = 1;
    ioctl(fd, HPU_TS_MODE, &timestampswitch);

    uint32_t dma_latency = 1;
    ioctl(fd, HPU_AXIS_LATENCY, &dma_latency);

    uint32_t minimum_packet = 8; //bytes (not events)
    ioctl(fd, HPU_SET_BLK_RX_THR, &minimum_packet);

    //read the pool size
    ioctl(fd, HPU_GET_RX_PS, &pool_size);
    if(pool_size < 0 || pool_size > 32768) {
        yWarning() << "Pool size invalid (" << pool_size << "). Setting to ("
                     "4096)";
        pool_size = 4096;

    }

    unsigned int pool_count;
    ioctl(fd, HPU_GET_RX_PN, &pool_count);

    if(!spinnaker) {

        yInfo() << "Configuring Cameras/Skin";

        hpu_tx_interface_ioctl_t tx_config = {{{0, 0, 0, 0}, 0, 0, 0}, ROUTE_FIXED};
        ioctl(fd, HPU_TX_INTERFACE, &tx_config);

        hpu_rx_interface_ioctl_t rx_config;
        rx_config = {INTERFACE_EYE_R, {{1, 1, 1, 1}, 0, 0, 0}};
        ioctl(fd, HPU_RX_INTERFACE, &rx_config);

        rx_config = {INTERFACE_EYE_L, {{1, 1, 1, 1}, 0, 0, 0}};
        ioctl(fd, HPU_RX_INTERFACE, &rx_config);

        rx_config = {INTERFACE_AUX, {{1, 1, 1, 1}, 0, 0, 0}};
        ioctl(fd, HPU_RX_INTERFACE, &rx_config);

    } else {

        yInfo() << "Configuring SpiNNaker";

        hpu_tx_interface_ioctl_t tx_config = {{{0, 0, 0, 0}, 0, 0, 1}, ROUTE_FIXED};
        ioctl(fd, HPU_TX_INTERFACE, &tx_config);

        hpu_rx_interface_ioctl_t rx_config = {INTERFACE_AUX, {{0, 0, 0, 0}, 0, 0, 1}};
        ioctl(fd, HPU_RX_INTERFACE, &rx_config);

        //unsigned int ss_protection = 1;
        //ioctl(fd, HPU_SPINN_KEYS_EN, &ss_protection);

        //unsigned int dump_off = 1;
        //ioctl(fd, HPU_SPINN_DUMPOFF, &dump_off);

        spinn_keys_t ss_keys = {0x80000000, 0x40000000};
        ioctl(fd, HPU_SET_SPINN_KEYS, &ss_keys);

        spinn_keys_enable_t ss_policy = {0, 0, 1};
        //spinn_start_stop_policy_t ss_mode = KEY_ENABLE;
        ioctl(fd, HPU_SPINN_KEYS_EN, &ss_policy);

        hpu_regs_t hpu_regs;

        //SPNN_TX_MASK_REG
        unsigned int tx_mask = 0x00FFFFFF;
        ioctl(fd, HPU_SPINN_TX_MASK, &tx_mask);
        //hpu_regs = {0x88, 1, 0x00FFFFFF};
        //ioctl(fd, HPU_GEN_REG, &hpu_regs);

        //SPNN_RX_MASK_REG
        unsigned int rx_mask = 0x00FFFFFF;
        ioctl(fd, HPU_SPINN_TX_MASK, &rx_mask);
        //hpu_regs = {0x8C, 1, 0x00FFFFFF};
        //ioctl(fd, HPU_GEN_REG, &hpu_regs);

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

bool hpuInterface::openReadPort(string module_name, unsigned int packet_size)
{
    if(fd < 0 || !D2Y.open(module_name, fd, pool_size, packet_size))
        return false;

    yInfo() << "Maximum packet size:" << packet_size;
    read_thread_open = true;
    return true;
}

bool hpuInterface::openWritePort(string module_name)
{
    if(fd < 0 || !Y2D.open(module_name, fd))
        return false;

    write_thread_open = true;
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
