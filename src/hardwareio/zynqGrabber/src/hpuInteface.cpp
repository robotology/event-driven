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

/******************************************************************************/
//vDevReadBuffer
/******************************************************************************/

vDevReadBuffer::vDevReadBuffer(int fd, unsigned int read_size, unsigned int buffer_size)
{
    //parameters
    this->fd = fd;
    this->buffer_size = buffer_size;
    this->read_size = read_size;

    buffer1.resize(buffer_size);
    buffer2.resize(buffer_size);
    discard_buffer.resize(read_size);

    read_buffer = &buffer1;
    access_buffer = &buffer2;

    read_count = 0;
    loss_count = 0;

    bufferedreadwaiting = false;

    yInfo() << "buffer size: " << buffer1.size();

}


void vDevReadBuffer::run()
{

    signal.check();

    while(!isStopping()) {

        safety.wait();

        int r = 0;
        if(read_count < buffer_size) {
            //we read and fill up the buffer
            r = read(fd, read_buffer->data() + read_count,
                     std::min(buffer_size - read_count, read_size));
            if(r > 0) read_count += r;
        } else {
            //we have reached maximum software buffer - read from the HW but
            //just discard the result.
            r = read(fd, discard_buffer.data(), read_size);
            if(r > 0) loss_count += r;
        }
        //std::cout<< " bytes read "<< r <<std::endl;

        if(r < 0 && errno != EAGAIN) {
            perror("Error reading events: ");
        }

        safety.post();
        if(bufferedreadwaiting) {
            //the other thread is read to read
            signal.wait(); //wait for it to do the read
        }

    }


}


std::vector<unsigned char>& vDevReadBuffer::getBuffer(unsigned int &nBytesRead, unsigned int &nBytesLost)
{

    if(!this->isRunning()) {
        //direct read
        nBytesLost = 0; nBytesRead = 0;
        int r = read(fd, read_buffer->data(), read_size);
        if(r > 0) nBytesRead = r;
        if(r < 0 && errno != EAGAIN) {
            perror("Error reading events: ");
        }
        return *read_buffer;
    } else {

        //safely copy the data into the accessBuffer and reset the readCount
        bufferedreadwaiting = true;
        safety.wait();
        bufferedreadwaiting = false;

        //switch the buffer the read into
        std::vector<unsigned char> *temp;
        temp = read_buffer;
        read_buffer = access_buffer;
        access_buffer = temp;

        //reset the filling position
        nBytesRead = read_count;
        nBytesLost = loss_count;
        read_count = 0;
        loss_count = 0;

        //send the correct signals to restart the grabbing thread
        safety.post();
        signal.check();
        signal.post(); //tell the other thread we are done

        return *access_buffer;
    }

}

/******************************************************************************/
//device2yarp
/******************************************************************************/

device2yarp::device2yarp()
{
    countAEs = 0;
    countLoss = 0;
    prevAEs = 0;
    device_reader = 0;
    direct_read = false;
}

bool device2yarp::open(string module_name, int fd, unsigned int read_size,
                       bool direct_read, unsigned int packet_size,
                       unsigned int internal_storage_size)
{
    if(direct_read)
        device_reader = new vDevReadBuffer(fd, packet_size, internal_storage_size);
    else
        device_reader = new vDevReadBuffer(fd, read_size, internal_storage_size);

    this->direct_read = direct_read;
    this->packet_size = packet_size;

    return output_port.open(module_name + "/AE:o");
}

void device2yarp::afterStart(bool success)
{
    if(success && !direct_read)
        device_reader->start();
}

void  device2yarp::run() {

    vGenPortInterface external_storage;
    external_storage.setHeader(AE::tag);
    yInfo() << "packet size: " << packet_size;

    while(!isStopping()) {

        //get the data from the device read thread
        unsigned int nBytesRead, nBytesLost;
        std::vector<unsigned char> &data = device_reader->getBuffer(nBytesRead, nBytesLost);
        countAEs += nBytesRead / 8;
        countLoss += nBytesLost / 8;
        if (!output_port.getOutputCount() || nBytesRead <= 8) continue;

        unsigned int i = 0;
        while((i+1) * packet_size < nBytesRead) {

            external_storage.setExternalData((const char *)data.data() +
                                             i * packet_size, packet_size);
            yarp_stamp.update();
            output_port.setEnvelope(yarp_stamp);
            output_port.write(external_storage);

            i++;
        }

        external_storage.setExternalData((const char *)data.data() +
                                         i * packet_size,
                                         nBytesRead - i * packet_size);
        yarp_stamp.update();
        output_port.setEnvelope(yarp_stamp);
        output_port.write(external_storage);

        //display an output to let everyone know we are still working.
        double update_period = yarp::os::Time::now() - prevTS;
        if(update_period > 1.0) {

            yInfo() << "[READ] "
                    << (int)((countAEs - prevAEs)/(1000.0*update_period))
                    << "kV/s";
            if(countLoss > 0) {
                yWarning() << "[LOST] "
                           << (int)(countLoss/(1000.0*update_period))
                           << "kV/s";
                countLoss = 0;
            }
            prevTS += update_period;
            prevAEs = countAEs;
        }

    }

}

void device2yarp::onStop()
{
    device_reader->stop();
    output_port.close();
}

void device2yarp::threadRelease()
{
    delete device_reader;
    device_reader = 0;
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

    yInfo() << "Begin reading from device ... ";

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
        for(size_t i = 1; i < data_size; i += 2) {
            data_copy[i] = data_bottle->get(i).asInt();
        }

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
        if(dt > 3.0) {

            struct { unsigned int reg_offset; char rw; unsigned int data;} hpu_regs = {0x18, 0, 0};
            if (-1 == ioctl(fd, HPU_GEN_REG, &hpu_regs)){
                yWarning() << "Couldn't read dump status";
            }

            if(hpu_regs.data & 0x00010000)
                yInfo() << "[DUMP] " << 0.000032 * total_events / dt << " mbps ("
                    << input_port.getPendingReads() <<
                    ") .Example:" << data_copy[2] << data_copy[3];
            else
                yInfo() << "[WRITE] " << 0.000032 * total_events / dt << " mbps ("
                    << input_port.getPendingReads() <<
                    ") .Example:" << data_copy[2] << data_copy[3];

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

    std::cout << "ID and Version";
    char *c = (char *)&(version);
    int major = ((int)(*(c+3)))>>4;
    int minor = ((int)(*(c+3)))&0xF;
    std::cout << *c << *(c+1) << *(c+2) << major << "." << minor << std::endl;

    //32 bit timestamp
    unsigned int timestampswitch = 1;
    ioctl(fd, HPU_TS_MODE, &timestampswitch);

    //read the pool size
    ioctl(fd, HPU_GET_RX_PS, &pool_size);
    if(pool_size < 0 || pool_size > 32768) {
        yWarning() << "Pool size invalid (" << pool_size << "). Setting to ("
                     "4096)";
        pool_size = 4096;

    }
    yInfo() << "pool size: " << pool_size;


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

        unsigned int ss_protection = 1;
        ioctl(fd, HPU_SPINN_KEYS_EN, &ss_protection);

        spinn_keys_t ss_keys = {0x80000000, 0x40000000};
        ioctl(fd, HPU_SET_SPINN_KEYS, &ss_keys);

        //---------
        hpu_regs.reg_offset = 0x88; //SPNN_TX_MASK_REG
        hpu_regs.rw = 1;
        hpu_regs.data = 0x00FFFFFF;
        if (-1 == ioctl(fd, HPU_GEN_REG, &hpu_regs)){
            yError() << "Error: could not set a register";
            close(fd); fd = -1;
            return false;
        }

        //---------
        hpu_regs.reg_offset = 0x8C; //SPNN_RX_MASK_REG
        hpu_regs.rw = 1;
        hpu_regs.data = 0x00FFFFFF;
        if (-1 == ioctl(fd, HPU_GEN_REG, &hpu_regs)){
            yError() << "Error: could not set a register";
            close(fd); fd = -1;
            return false;
        }

        //---------
        //ENABLE loopback
        spinn_loop_t lbmode = LOOP_NONE;
        if(loopback) {
            yWarning() << "SpiNNaker in Loopback mode";
            lbmode = LOOP_LSPINN;
        }
        ioctl(fd, HPU_SET_LOOPBACK, &lbmode);

        //READ CTRL_REG status
        hpu_regs.reg_offset = CTRL_REG;
        hpu_regs.rw = 0;
        hpu_regs.data = 0;
        if (-1 == ioctl(fd, HPU_GEN_REG, &hpu_regs)){
            yError() << "Error: cannot read CTRL_REG";
            close(fd); fd = -1;
            return false;
        }
        std::cout << "CTRL_REG: 0x" << std::hex << std::setw(8)
                  << std::setfill('0') << hpu_regs.data << std::endl;

        //READ IRQ status
        hpu_regs.reg_offset = IRQ_REG;
        hpu_regs.rw = 0;
        hpu_regs.data = 0;
        if (-1 == ioctl(fd, HPU_GEN_REG, &hpu_regs)){
            yError() << "Error: cannot read IRQ status";
            close(fd); fd = -1;
            return false;
        }

        std::cout << "IRQ status register: ";
        std::cout << std::hex << hpu_regs.data << std::endl;

        //READ IP configuration
        hpu_regs.reg_offset = IP_CFNG_REG;
        hpu_regs.rw = 0;
        hpu_regs.data = 0;
        if (-1 == ioctl(fd, HPU_GEN_REG, &hpu_regs)){
            yError() << "Error: cannot read IP configuration";
            close(fd); fd = -1;
            return false;
        }

        std::cout << "IP configuration: ";
        std::cout << std::hex << hpu_regs.data << std::endl;

        //READ Raw Status Register
        hpu_regs.reg_offset = 0x18;
        hpu_regs.rw = 0;
        hpu_regs.data = 0;
        if (-1 == ioctl(fd, HPU_GEN_REG, &hpu_regs)){
            yError() << "Error: cannot read IP configuration";
            close(fd); fd = -1;
            return false;
        }

        std::cout << "Raw Status: ";
        std::cout << std::hex << hpu_regs.data << std::endl;


    }


    return true;
}

bool hpuInterface::openReadPort(string module_name, bool direct_read,
                                unsigned int packet_size,
                                unsigned int maximum_internal_memory)
{
    if(fd < 0 || !D2Y.open(module_name, fd, pool_size, direct_read, packet_size,
                           maximum_internal_memory))
        return false;

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
    hpu_regs.reg_offset = 0x18;
    hpu_regs.rw = 0;
    hpu_regs.data = 0;
    if (-1 == ioctl(fd, HPU_GEN_REG, &hpu_regs)){
        yError() << "Error: cannot read Raw Status";
    }

    std::cout << "Raw Status: ";
    std::cout << std::hex << hpu_regs.data << std::endl;

    close(fd); fd = -1;
}
