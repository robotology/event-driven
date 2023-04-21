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

#ifndef __EVENTDRIVENYARPINTERFACE__
#define __EVENTDRIVENYARPINTERFACE__

#include <yarp/os/all.h>
#include <event-driven/core.h>
#include <thread>

typedef enum {
    INTERFACE_EYE_R,
    INTERFACE_EYE_L,
    INTERFACE_AUX
} hpu_interface_t;

typedef struct {
    int hssaer[4];
    int gtp;
    int paer;
    int spinn;
} hpu_interface_cfg_t;

typedef struct {
    hpu_interface_t interface;
    hpu_interface_cfg_t cfg;
} hpu_rx_interface_ioctl_t;


typedef struct {
    uint32_t start;
    uint32_t stop;
} spinn_keys_t;

typedef struct {
    int enable_l;
    int enable_r;
    int enable_aux;
} spinn_keys_enable_t;

typedef enum {
    LOOP_NONE,
    LOOP_LNEAR,
    LOOP_LSPINN,
} spinn_loop_t;

typedef enum {
    ROUTE_FIXED,
    ROUTE_MSG,
} hpu_tx_route_t;

typedef struct {
    hpu_interface_cfg_t cfg;
    hpu_tx_route_t route;
} hpu_tx_interface_ioctl_t;

typedef struct {

    uint32_t reg_offset;
    char rw;
    uint32_t data;

} hpu_regs_t;

enum rx_err { ko_err = 0, rx_err, to_err, of_err, nomeaning_err };

typedef struct {
    enum rx_err err;
    uint8_t cnt_val;
} aux_cnt_t;

typedef enum {
    MASK_20BIT,
    MASK_24BIT,
    MASK_28BIT,
    MASK_32BIT,
} hpu_timestamp_mask_t;

typedef enum {
    TIMINGMODE_DELTA,
    TIMINGMODE_ASAP,
    TIMINGMODE_ABS,
} hpu_tx_timing_mode_t;

typedef enum {
    TIME_1mS,
    TIME_5mS,
    TIME_10mS,
    TIME_50mS,
    TIME_100mS,
    TIME_500mS,
    TIME_1000mS,
    TIME_2500mS,
    TIME_5000mS,
    TIME_10S,
    TIME_25S,
    TIME_50S,
    TIME_100S,
    TIME_250S,
    TIME_500S,
    TIME_DISABLE,
} hpu_tx_resync_time_t;

typedef enum {
    EMPTY,
    ALMOST_EMPTY,
    FULL,
    ALMOST_FULL,
    NOT_EMPTY
} fifo_status_t;

typedef struct {
    fifo_status_t rx_fifo_status;
    fifo_status_t tx_fifo_status;
    int rx_buffer_ready;
    int lrx_paer_fifo_full;
    int rrx_paer_fifo_full;
    int auxrx_paer_fifo_full;
    int rx_fifo_over_threshold;
    int global_rx_err_ko;
    int global_rx_err_tx;
    int global_rx_err_to;
    int global_rx_err_of;
    int tx_spinn_dump;
    int lspinn_parity_err;
    int rspinn_parity_err;
    int auxspinn_parity_err;
    int lspinn_rx_err;
    int rspinn_rx_err;
    int auxspinn_rx_err;
} hpu_hw_status_t;

typedef enum {
    TD,
    EM,
} tdorem_t;

typedef enum {
    X,
    Y,
} xory_t;

#define MAGIC_NUM 0
#define HPU_READ_TS         _IOR (MAGIC_NUM,  1, unsigned int *)
#define HPU_CLEAR_TS        _IOW (MAGIC_NUM,  2, unsigned int *)
#define HPU_VERSION         _IOR (MAGIC_NUM,  3, unsigned int *)
#define HPU_TS_MODE         _IOW (MAGIC_NUM,  7, unsigned int *)
#define HPU_GEN_REG         _IOWR(MAGIC_NUM,  8, hpu_regs_t *)
#define HPU_GET_RX_PS       _IOR (MAGIC_NUM,  9, unsigned int *)

#define HPU_SET_AUX_THRS    _IOW (MAGIC_NUM, 10, aux_cnt_t *)
#define HPU_GET_AUX_THRS    _IOR (MAGIC_NUM, 11, unsigned int *)
#define HPU_GET_AUX_CNT0    _IOR (MAGIC_NUM, 12, unsigned int *)
#define HPU_GET_AUX_CNT1    _IOR (MAGIC_NUM, 13, unsigned int *)
#define HPU_GET_AUX_CNT2    _IOR (MAGIC_NUM, 14, unsigned int *)
#define HPU_GET_AUX_CNT3    _IOR (MAGIC_NUM, 15, unsigned int *)

#define HPU_GET_LOSTCNT     _IOR (MAGIC_NUM, 16, unsigned int *)
#define HPU_SET_LOOPBACK    _IOW (MAGIC_NUM, 18, spinn_loop_t *)
#define HPU_GET_TX_PS       _IOR (MAGIC_NUM, 20, unsigned int *)
#define HPU_SET_BLK_TX_THR  _IOW (MAGIC_NUM, 21, unsigned int *)
#define HPU_SET_BLK_RX_THR  _IOW (MAGIC_NUM, 22, unsigned int *)
#define HPU_SET_SPINN_KEYS  _IOW (MAGIC_NUM, 23, spinn_keys_t *)
//#define HPU_SPINN_KEYS_EN   _IOW (MAGIC_NUM, 24, unsigned int *)
#define HPU_SPINN_STA_STO   _IOR (MAGIC_NUM, 25, unsigned int *)
#define HPU_RX_INTERFACE    _IOW (MAGIC_NUM, 26, hpu_rx_interface_ioctl_t *)
#define HPU_TX_INTERFACE    _IOW (MAGIC_NUM, 27, hpu_tx_interface_ioctl_t *)
#define HPU_AXIS_LATENCY    _IOW (MAGIC_NUM, 28, unsigned int *)

#define HPU_GET_RX_PN       _IOR (MAGIC_NUM, 29, unsigned int *)
//#define HPU_SPINN_ST_SP     _IOW (MAGIC_NUM, 30, spinn_start_stop_policy_t *)
#define HPU_TS_MASK         _IOW (MAGIC_NUM, 31, hpu_timestamp_mask_t *)
#define HPU_TX_TIMING_MODE  _IOW (MAGIC_NUM, 32, hpu_tx_timing_mode_t *)
#define HPU_SET_TX_RESYNC   _IOW (MAGIC_NUM, 33, hpu_tx_resync_time_t *)
#define HPU_RESET_TX_RESYNC _IOW (MAGIC_NUM, 34, unsigned int *)
#define HPU_FORCE_TX_RESYNC _IOW (MAGIC_NUM, 35, unsigned int *)
#define HPU_SPINN_TX_MASK   _IOW (MAGIC_NUM, 36, unsigned int *)
#define HPU_SPINN_RX_MASK   _IOW (MAGIC_NUM, 37, unsigned int *)
#define HPU_GET_HW_STATUS   _IOR (MAGIC_NUM, 38, hpu_hw_status_t *)
#define HPU_SPINN_KEYS_EN   _IOW (MAGIC_NUM, 39, spinn_keys_enable_t *)
#define HPU_SET_RX_TS_EN    _IOW (MAGIC_NUM, 40, unsigned int *)
#define HPU_SET_TX_TS_EN    _IOW (MAGIC_NUM, 41, unsigned int *)

/******************************************************************************/
//device2yarp
/******************************************************************************/
class device2yarp : public yarp::os::Thread {

private:

    //data buffer thread
    int fd; 
    ev::BufferedPort<ev::AE> output_port;
    yarp::os::Stamp yarp_stamp;

    //parameters
    unsigned int max_dma_pool_size;
    unsigned int max_packet_size;
    std::string port_name;
    double min_packet_duration{0.0};

public:

    device2yarp();
    void configure(std::string module_name, int fd, unsigned int pool_size,
              unsigned int packet_size, bool record_mode);
    void yarpOpen();
    void run();
    void onStop();

};

/******************************************************************************/
//yarp2device
/******************************************************************************/
class yarp2device : public yarp::os::Thread
{
protected:

    int fd;
    ev::BufferedPort<ev::AE> input_port;
    std::string port_name;

public:

    yarp2device();
    void configure(std::string module_name, int fd);
    void yarpOpen();
    void run();
    void onStop();


};

/******************************************************************************/
//hpuInterface
/******************************************************************************/
class hpuInterface {

private:

    int fd{-1};
    device2yarp D2Y;
    yarp2device Y2D;

    //DEVICE TO YARP
    ev::BufferedPort<ev::AE> output_port;
    yarp::os::Stamp yarp_stamp;
    unsigned int max_dma_pool_size;
    unsigned int max_packet_size;
    std::string port_name;
    double min_packet_duration{0.0};
    std::thread y2d_thread;
    void d2y();

    //YARP TO DEVICE
    std::string port_name;
    ev::BufferedPort<ev::AE> input_port;
    std::thread d2y_thread;
    void y2d();

    int pool_size;
    bool read_thread_open{false};
    bool write_thread_open{false};

public:
 bool configureDevice(std::string device_name, bool spinnaker = false,
                      bool loopback = false, bool gtp = true) {
     // open the device
     fd = open(device_name.c_str(), O_RDWR);
     if (fd < 0) {
         fd = open(device_name.c_str(), O_RDONLY | O_NONBLOCK);
         if (fd < 0) {
             yError() << "Could not open" << device_name << " device";
             return false;
         } else {
             yWarning() << device_name << "only opened in read-only, "
                                          "non-blocking mode";
         }
     }

     // READ ID
     unsigned int version = 0;
     if (ioctl(fd, HPU_VERSION, &version) < 0) {
         yError() << "Could not read version";
         return false;
     }

     char version_word[5];
     version_word[0] = (char)(version >> 24);
     version_word[1] = (char)(version >> 16);
     version_word[2] = (char)(version >> 8);
     version_word[3] = '-';
     version_word[4] = '\0';
     yInfo() << "ID and Version " << version_word
             << (int)((version >> 4) & 0xF) << "." << (int)((version >> 0) & 0xF);

     // 32 bit timestamp
     uint32_t timestampswitch = 1;
     if (ioctl(fd, HPU_TS_MODE, &timestampswitch) < 0) {
         yError() << "Could not write timestamp mode";
         return false;
     }

     uint32_t dma_latency = 1;
     if (ioctl(fd, HPU_AXIS_LATENCY, &dma_latency) < 0) {
         yError() << "Could not write dma latency";
         return false;
     }

     uint32_t minimum_packet = 8;  // bytes (not events)
     if (ioctl(fd, HPU_SET_BLK_RX_THR, &minimum_packet) < 0) {
         yError() << "Could not write BLK_RX_THR";
         return false;
     }

     // read the pool size
     if (ioctl(fd, HPU_GET_RX_PS, &pool_size) < 0) {
         yError() << "Could not read pool size";
         return false;
     }
     if (pool_size < 0 || pool_size > 32768) {
         yWarning() << "Pool size invalid (" << pool_size << "). Setting to ("
                                                             "4096)";
         pool_size = 4096;
     }

     unsigned int pool_count;
     if (ioctl(fd, HPU_GET_RX_PN, &pool_count) < 0) {
         yError() << "Could not read pool count";
         return false;
     }

     hpu_interface_cfg_t trans_config;
     hpu_rx_interface_ioctl_t rx_config;
     if (gtp)
         trans_config = {{0, 0, 0, 0}, 1, 0, 0};
     else
         trans_config = {{1, 1, 1, 1}, 0, 0, 0};

     rx_config = {INTERFACE_EYE_R, {{0, 0, 0, 0}, 1, 0, 0}};
     if (ioctl(fd, HPU_RX_INTERFACE, &rx_config) < 0) {
         yError() << "Could not write EYE_R config";
         return false;
     }

     rx_config = {INTERFACE_EYE_L, {{0, 0, 0, 0}, 1, 0, 0}};
     if (ioctl(fd, HPU_RX_INTERFACE, &rx_config) < 0) {
         yError() << "Could not write EYE_L config";
         return false;
     }

     rx_config = {INTERFACE_AUX, {{0, 0, 0, 0}, 0, 0, 0}};
     if (ioctl(fd, HPU_RX_INTERFACE, &rx_config) < 0) {
         yError() << "Could not write AUX config";
         return false;
     }

     hpu_tx_interface_ioctl_t tx_config = {{{0, 0, 0, 0}, 0, 0, 0}, ROUTE_FIXED};
     if (ioctl(fd, HPU_TX_INTERFACE, &tx_config) < 0) {
         yError() << "Could not write hpu tx config";
         return false;
     }

     // READ CTRL_REG status
#if ENABLE_TS
     unsigned int ts_flag = 1;
     yInfo() << "ON: individual event timestamps";
#else
     unsigned int ts_flag = 0;
     yInfo() << "OFF: individual event timestamps";
#endif
     if (ioctl(fd, HPU_SET_RX_TS_EN, &ts_flag) < 0) {
         yError() << "Could not enable/disable timestamps (RX)";
         return false;
     }
     if (ioctl(fd, HPU_SET_TX_TS_EN, &ts_flag) < 0) {
         yError() << "Could not enable/disable timestamps (TX)";
         return false;
     }

     hpu_regs_t hpu_regs = {0, 0, 0};
     ioctl(fd, HPU_GEN_REG, &hpu_regs);
     std::stringstream ss;
     ss << "CTRL_REG: "
        << "0x" << std::hex << std::setw(8)
        << std::setfill('0') << hpu_regs.data << std::endl;
     yInfo() << ss.str();

     if (spinnaker) {
         yInfo() << "Configuring SpiNNaker";

         hpu_tx_interface_ioctl_t tx_config = {{{0, 0, 0, 0}, 0, 0, 1}, ROUTE_FIXED};
         if (ioctl(fd, HPU_TX_INTERFACE, &tx_config) < 0) {
             yError() << "Could not write hpu tx config";
             return false;
         }

         spinn_keys_t ss_keys = {0x80000000, 0x40000000};
         ioctl(fd, HPU_SET_SPINN_KEYS, &ss_keys);

         spinn_keys_enable_t ss_policy = {0, 0, 1};
         ioctl(fd, HPU_SPINN_KEYS_EN, &ss_policy);

         hpu_regs_t hpu_regs;

         // SPNN_TX_MASK_REG
         unsigned int tx_mask = 0x00FFFFFF;
         ioctl(fd, HPU_SPINN_TX_MASK, &tx_mask);

         // SPNN_RX_MASK_REG
         unsigned int rx_mask = 0x00FFFFFF;
         ioctl(fd, HPU_SPINN_TX_MASK, &rx_mask);

         // ENABLE loopback
         spinn_loop_t lbmode = LOOP_NONE;
         if (loopback) {
             yWarning() << "SpiNNaker in Loopback mode";
             lbmode = LOOP_LSPINN;
         }
         ioctl(fd, HPU_SET_LOOPBACK, &lbmode);

         // SET TX synch
         hpu_regs = {0x44, 0, 0};
         ioctl(fd, HPU_GEN_REG, &hpu_regs);  // read
         hpu_regs.data |= 2 << 12;           // TX timing mode [0 1 2]
         // hpu_regs.data |= 0 << 14; //forces trigger [0 1]
         // hpu_regs.data |= 1 << 15; //enable resync
         hpu_regs.data |= 0x9 << 16;  // resynch frequency
         hpu_regs.data |= 1 << 20;    // differential mask size (1 = 20 bits)
         std::cout << "TX status: ";
         std::cout << "0x" << std::hex << std::setw(8)
                   << std::setfill('0') << hpu_regs.data << std::endl;
         hpu_regs.rw = 1;
         ioctl(fd, HPU_GEN_REG, &hpu_regs);  // write

         // READ CTRL_REG status
         hpu_regs = {0, 0, 0};
         ioctl(fd, HPU_GEN_REG, &hpu_regs);
         std::cout << "CTRL_REG: ";
         std::cout << "0x" << std::hex << std::setw(8)
                   << std::setfill('0') << hpu_regs.data << std::endl;

         // READ Raw Status Register
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

 bool openReadPort(std::string module_name, unsigned int packet_size, bool record_mode = false) {
     if (fd < 0)
         return false;
     D2Y.configure(module_name, fd, pool_size, packet_size, record_mode);
     yInfo() << "Maximum packet size:" << packet_size;
     read_thread_open = true;
     D2Y.yarpOpen();
     return true;
 }

 bool openWritePort(std::string module_name) {
     if (fd < 0)
         return false;
     Y2D.configure(module_name, fd);
     write_thread_open = true;
     Y2D.yarpOpen();
     return true;
 }

 void tryconnectToYARP() {
     if (read_thread_open)
         D2Y.yarpOpen();
     if (write_thread_open)
         Y2D.yarpOpen();
 }

 void start() {
     if (read_thread_open)
         D2Y.start();
     if (write_thread_open)
         Y2D.start();
 }

 void stop() {
     output_port.close();
     input_port.interrupt();
     input_port.close();

     // READ Raw Status Register
     hpu_regs_t hpu_regs = {0x18, 0, 0};
     if (-1 == ioctl(fd, HPU_GEN_REG, &hpu_regs)) {
         yError() << "Error: cannot read Raw Status";
     }

     std::cout << "Raw Status: ";
     std::cout << std::hex << hpu_regs.data << std::endl;

     close(fd);
     fd = -1;
 }
};

#endif
