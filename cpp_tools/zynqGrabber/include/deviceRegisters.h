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

#ifndef _VDEVICEREGISTERS_
#define _VDEVICEREGISTERS_

#include <cstdint>


#define LOW8(x) x&0xFF
#define HIGH8(x) (x&0xFF00)>>8
#define FIXED_UINT(x) (uint32_t)(x*65536)
#define UNSIGN_BITS(x) *(reinterpret_cast<uint32_t *>(&x))
//#define UNSIGN_BITS(x) (uint32_t)(*((uint32_t *)(&x)))

 // da mettere su zynq
//#define I2C_SLAVE 0x00 // da togliere su zynq

// vsctrl
#define I2C_ADDRESS_LEFT		 0x10
#define I2C_ADDRESS_RIGHT		 0x11
// aux i2c -- skin and other sensors (e.g. accelerometers)
#define I2C_ADDRESS_AUX          0x1e

#define TIMESTAMP32BIT          1
#define TIMESTAMP24BIT          0

#define AUTOINCR      0x80

// --- addresses of the registers --- //
#define VSCTRL_INFO_ADDR         0x00
#define VSCTRL_STATUS_ADDR       0x04
#define VSCTRL_RSVD_08_ADDR      0x08
#define VSCTRL_SRC_CNFG_ADDR     0x0C
#define VSCTRL_SRC_DST_CTRL_ADDR 0x10
#define VSCTRL_PAER_CNFG_ADDR    0x14
#define VSCTRL_HSSAER_CNFG_ADDR  0x18
#define VSCTRL_GTP_CNFG_ADDR     0x1C
// Addresses 0x20, 0x24 and 0x28 change meaning from ATIS gen1 to gen3
//ATIS gen1
#define VSCTRL_BG_CNFG_ADDR      0x20
#define VSCTRL_BG_PRESC_ADDR     0x24
#define VSCTRL_BG_TIMINGS_ADDR   0x28
//ATIS gen3
#define VSCTRL_SISLEY_LDO_RSTN_REG 0x20
#define VSCTRL_SISLEY_LDO_RSTN_REG1 0x21
#define VSCTRL_SISLEY_DATA_REG     0x28
#define VSCTRL_SISLEY_ADDRESS_REG  0x24


#define VSCTRL_BG_EXP_CRC_ADDR   0x2C
#define VSCTRL_BG_DATA_ADDR      0x30
#define VSCTRL_RSVD_34_ADDR      0x34
#define VSCTRL_GPO_ADDR          0x38
#define VSCTRL_GPI_ADDR          0x3C


#define VSCTRL_DSTCTRL_REG         0x11
#define VSCTRL_G3_SYNSTHDATARATE_REG 0x12




// --- default values for ATIS chip --- //

// --- register VSCTRL_SRC_CNFG_ADDR --- //
#define ACK_REL_DEL              0x3 //0x05 // 50ns (one tick is 10ns)
#define ACK_SAM_DEL              0x1 //0x03 // 30n
#define ACK_SET_DEL              0x0 //0x02 // 20n
#define AER_LVL                  0x15 // overwrite = 1, ack active low, req active high (ATIS default)
// --- register VSCTRL_SRC_DST_CTRL_ADDR --- //
#define TD_APS_CTRL              0x02 // TD loopback = 0, TD EN =1, APS loppback = 0, APS EN = 1, flush fifo = 0, ignore FIFO Full = 0
#define TD_CTRL                  0x02
#define APS_CTRL                 0x08
#define SRC_CTRL                 0x12
// --- register VSCTRL_HSSAER_CNFG_ADDR --- //
#define CH_SAER_EN                    0x07 // enable ch0, ch1, ch2
// --- register VSCTRL_HSSAER_CNFG_ADDR --- //
#define TX_PAER_CFG                    0x03 // enable ch0, ch1, ch2
// --- register VSCTRL_BG_CNFG_ADDR --- //
#define BG_CNFG                 0x39 // BGtype = 1 (ATIS), BG overwrite = 1, CK active level = 1, LATCH active level = 1
#define BG_LATEND_SHCNT         0x20 // LatchOut@end = 1, ShiftCount = 32
#define BG_ROI                  0x00 // Choose if setting ROI or setting BG (0 -> BG, 1 -> ROI)

// --- register VSCTRL_BG_PRESC_ADDR --- //
#define BG_PRESC                 24 // 24x10ns diventa il periodo del clock
#define BG_LAT                   3  // Latch Active Time
#define BG_LS                    4  // Latch Setup
#define BG_CAT                   2  // Clock Active Time
#define BG_SHT                   1  // Setup Hold Time

// --- paddrivestrength --- //
#define ATIS_PDSHIFT 4
#define ATIS_PDSTRENGTH 3
#define ATIS_BIASSHIFT 32

// --- default values for ATIS Gen3 chip --- //

#define GEN3_RIGHT 					0x11
#define VSCTRL_ENABLE_GEN3 			0x08
#define VSCTRL_ENABLE_GEN3_TESTMODE	0x10
#define VSCTRL_ENABLE_CHANNEL0      0x01
#define VSCTRL_ENABLE_CHANNEL1      0x02
#define VSCTRL_ENABLE_CHANNEL2      0x04
#define VSCTRL_ENABLE_CHANNEL3      0x08
#define VSCTRL_ENABLE_ALLCHANNELS   (VSCTRL_ENABLE_CHANNEL0 | VSCTRL_ENABLE_CHANNEL1 | \
                                     VSCTRL_ENABLE_CHANNEL2 | VSCTRL_ENABLE_CHANNEL3 )
#define VSCTRL_DESTINATION_PAER     0x0
#define VSCTRL_DESTINATION_HSSAER   0x1
#define VSCTRL_DESTINATION_GTP      0x2
#define VSCTRL_ENABLETXON_PAER      0x1
#define VSCTRL_ENABLETXON_HSSAER    0x2
#define VSCTRL_ENABLETXON_GTP       0x4
#define VSCTRL_FLUSH_FIFO           (0x1<<6)
#define VSCTRL_ENABLE_CLK           0x1
#define VSCTRL_DISABLE_CLK          0x0
#define VSCTRL_ENABLE_VDDA          0x1
#define VSCTRL_ENABLE_VDDC          0x2
#define VSCTRL_ENABLE_VDDD          0x4
#define VSCTRL_DISABLE_TDRSTN       0x40
#define VSCTRL_DISABLE_TPRSTN       0x20
#define VSCTRL_DISABLE_MRRSTN       0x10

#define READ_SISLEY_REG  (0<<31)
#define WRITE_SISLEY_REG (1<<31)

#define I2C_AUTOTINCRREGS   0x80

#define SISLEY_GLOBAL_CTRL_REG     0x00
#define SISLEY_ROI_CTRL_REG        0x04
#define SISLEY_READOUT_CTRL_REG    0x08
#define SISLEY_TEST_BUS_CTRL_REG   0x0C
#define SISLEY_CLK_SYNC_CTRL_REG   0x10
#define SISLEY_CHIP_ID_REG         0x18
#define SISLEY_SPARE_CTRL_REG	   0x1C

#define BIAS_LATCHOUT_OR_PU        0x100
#define BIAS_REQX_OR_PU            0x104
#define BIAS_REQ_PUX               0x108
#define BIAS_REQ_PUY               0x10C
#define BIAS_DEL_REQX_OR           0x110
#define BIAS_SENDREQ_PDX           0x114
#define BIAS_SENDREQ_PDY           0x118
#define BIAS_DEL_ACK_ARRAY         0x11C
#define BIAS_DEL_TIMEOUT           0x120
#define BIAS_INV                   0x124
#define BIAS_REFR                  0x128
#define BIAS_CLK                   0x12C
#define BIAS_TAIL                  0x130
#define BIAS_OUT                   0x134
#define BIAS_HYST                  0x138
#define BIAS_VREFL                 0x13C
#define BIAS_VREFH                 0x140
#define BIAS_CAS                   0x144
#define BIAS_DIFF_OFF              0x148
#define BIAS_DIFF_ON               0x14C
#define BIAS_DIFF                  0x150
#define BIAS_FO                    0x154
#define BIAS_PR                    0x158
#define BIAS_BULK                  0x15C
#define BIAS_OVERFLOW              0x160
#define BIAS_VDD_HPF               0x164
#define BIAS_BUF                   0x168

#define SISLEY_TD_ROI_X_OFFSET	   0x200
#define SISLEY_TD_ROI_Y_OFFSET	   0x300
#define SISLEY_EM_ROI_X_OFFSET	   0x400
#define SISLEY_EM_ROI_Y_OFFSET	   0x500

#define SISLEY_CAM_X_SIZE          640
#define SISLEY_CAM_Y_SIZE          480

#define COUPLES32BIT            80000
#define DIFF_TIMESTAMP_SYSCLK   8

// --- register VSCTRL_STATUS --- //

// --- masks --- //
#define ST_TD_FIFO_FULL_MSK             0x01
#define ST_APS_FIFO_FULL_MSK            0x02
#define ST_CRC_ERR_MSK                  0x10
#define ST_BIAS_DONE_MSK                0x20
#define ST_I2C_TIMEOUT_MSK              0x80

// --- BG masks --- //
#define BG_PWRDWN_MSK                   0x10000000 // powerdown chip mask
#define BG_SHIFT_COUNT_MSK              0x003F0000 // mask of bg shift count
//#define BG_SHIFT_AUTORST_MSK          0x40 // mask of bg shift count
#define BG_LATOUTEND_MSK                0x00800000 // mask of bg shift count
//#define BG_ROISEL_MSK                 0x20
#define BG_VAL_MSK                      0x1FFFFF // mask of 21 bit for bg value


// --- addresses of the registers --- //
#define SKCTRL_EN_ADDR                  0x00
#define SKCTRL_GEN_SELECT               0x04
#define SKCTRL_DUMMY_PERIOD_ADDR        0x08
#define SKCTRL_DUMMY_CFG_ADDR           0x0C
#define SKCTRL_DUMMY_BOUND_ADDR         0x10
#define SKCTRL_DUMMY_INC_ADDR           0x14
#define SKCTRL_RES_TO_ADDR              0x18
#define SKCTRL_EG_PARAM1_ADDR           0x1C
#define SKCTRL_EG_PARAM2_ADDR           0x20
#define SKCTRL_EG_PARAM3_ADDR           0x24
#define SKCTRL_EG_PARAM4_ADDR           0x28
#define SKCTRL_I2C_ACQ_SOFT_RST_ADDR    0x34 // write only
#define SKCTRL_EG_FILTER_ADDR           0x38 // read only
#define SKCTRL_STATUS_ADDR              0x3A // read only
#define SKCTRL_VERSION_MIN              0x3C
#define SKCTRL_VERSION_MAJ              0x3D

// -- addresses of the registers bits/bytes for the skin -- //

// register SKCTRL_EN_ADDR                  0x00 // DUMMY_GEN_SEL[2:0]|DUMMY_ACQ_EN|FORCE_CALIB|reserved|reserved|I2C_ACQ_EN;
#define I2C_ACQ_EN                      0x01 // bit0 -- I2C_ACQ_EN;
#define FORCE_CALIB_EN                  0x08 // bit3 -- FORCE_CALIB;
#define DUMMY_ACQ_EN                    0x10 // bit4 -- DUMMY_ACQ_EN;
#define DUMMY_SEL                       0xE0 // bit5:7 -- DUMMY_GEN_SEL[2:0];

#define ASR_FILTER_TYPE                 0x80 // bit7 -- Resampling/evgen filter type (0: allow only, 1: deny)
#define ASR_FILTER_EN                   0x40 // bit6 -- Enable resampling/evgen filter;
#define EVGEN_NTHR_EN                   0x08 // bit3 -- Enable noise threshold for event generation;
#define PREPROC_SAMPLES                 0x04 // bit2 -- Preproc samples type (1: 16b, 0: 8b);
#define PREPROC_EVGEN                   0x02 // bit1 -- Preproc evgen type (1: 16b, 0: 8b);
#define DRIFT_COMP_EN                   0x01 // bit0 -- Drift comp enable;

#define SAMPLES_TX_MODE                 0x80 // bit 7 -- Samples TX mode(1: 16 bits transmission with dual 8b, 0: 8 bits)
#define SAMPLES_RSHIFT                  0x70 // bit4:6 -- only if Sample_tx_mode = 0 -> transmit 8 bits: define rshift with saturation
#define SAMPLES_RSHIFT_SHIFT            0x04 // shift the value we want to set in the SAMPLES_RSHIFT to the right position
#define SAMPLES_SEL                     0x08 // bit3 -- Samples source (0: pre-proc, 1: post-preproc);
#define AUX_TX_EN                       0x04 // bit2 -- Enable transmission of AS-AER events on AUX channel
#define SAMPLES_TX_EN                   0x02 // bit1 -- Samples transmission enable
#define EVENTS_TX_EN                    0x01 // bit0 -- Events transmission enable

// default values
//#define EG_CFG = ASR_FILTER_TYPE|ASR_FILTER_EN|EVGEN_NTHR_EN|PREPROC_SAMPLES|PREPROC_EVGEN|DRIFT_COMP_EN;
#define SAMPLES_RSHIFT_DEFAULT          2
// register SKCTRL_DUMMY_PERIOD_ADDR        0x04
#define EV_GEN_SELECT_DEFAULT 0x01
#define DUMMY_PERIOD_DEFAULT                          0x60
// register SKCTRL_DUMMY_CFG_ADDR           0x08
#define DUMMY_CALIB_DEFAULT                        0x00
#define DUMMY_ADDR_DEFAULT                        0x00
// register SKCTRL_DUMMY_BOUND_ADDR         0x0C
#define DUMMY_UP_BOUND_DEFAULT                     0xFF
#define DUMMY_LOW_BOUND_DEFAULT                    0x00

// register SKCTRL_DUMMY_INC_ADDR           0x10
#define DUMMY_INC_DEFAULT                          0x01
#define DUMMY_DECR_DEFAULT                         0x01
// register SKCTRL_RES_TO_ADDR              0x14
#define RESAMPLING_TIMEOUT_DEFAULT                   0x32
// register SKCTRL_EG_UPTHR_ADDR            0x18
#define EG_UP_THR_DEFAULT                       0.1
// register SKCTRL_EG_DWTHR_ADDR            0x1C
#define EG_DWN_THR_DEFAULT                      0.1
// register SKCTRL_EG_NOISE_RISE_THR_ADDR      0x20
#define EG_NOISE_RISE_THR_DEFAULT               12.0 // 32b unsigned fixed point (16b fractional); default value approx. 12
// register SKCTRL_EG_NOISE_FALL_THR_ADDR      0x24
#define EG_NOISE_FALL_THR_DEFAULT               12.0 // 32b unsigned fixed point (16b fractional); default value approx. 12
// register SKCTRL_I2C_ACQ_SOFT_RST_ADDR    0x34
#define I2C_ACQ_SOFT_RST_DEFAULT                0x00 // write only value???

//-- SKCTRL_STATUS_ADDR -- masks --//
// 32 bits register -- Arren approved ;-)
#define SKCTRL_EDMTB_SKIN_TYPE_MSK          0x00000001
#define SKCTRL_TX_KEEPALIVE_EN_MSK          0x00000080
#define SKCTRL_I2C_CFG_TABLE_LEN_MSK        0x00000300
#define SKCTRL_I2C_CFG_FILTER_TAPS_MSK      0x00000C00
#define SKCTRL_I2C_CFG_SCL_FREQ_MSK         0x00003000
#define SKCTRL_I2C_CFG_SDA_N_MSK            0x0000C000
#define SKCTRL_MINOR_MSK                    0x00FF0000
#define SKCTRL_MAJOR_MSK                    0xFF000000

#define EV_GEN_1 0x00
#define EV_GEN_2 0x01
#define EV_GEN_NEURAL 0x02

#define EV_MASK_SA1 0x01
#define EV_MASK_RA1 0x02
#define EV_MASK_RA2 0x04

#define EV_GEN_SA1 0x02
#define EV_GEN_RA1 0x03
#define EV_GEN_RA2 0x04


//HPU IOCTL STRUCTS/ENUMS

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

// HPU CORE IOCTLS

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

#endif
