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


#define LOW8(x) x&0xFF
#define HIGH8(x) (x&0xFF00)>>8
#define FIXED_UINT(x) (unsigned int)(x*65536)


 // da mettere su zynq
//#define I2C_SLAVE 0x00 // da togliere su zynq

// vsctrl
#define I2C_ADDRESS_LEFT		 0x10
#define I2C_ADDRESS_RIGHT		 0x11
// aux i2c -- skin and other sensors (e.g. accelerometers)
#define I2C_ADDRESS_AUX          0x1e

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
#define VSCTRL_BG_CNFG_ADDR      0x20
#define VSCTRL_BG_PRESC_ADDR     0x24
#define VSCTRL_BG_TIMINGS_ADDR   0x28
#define VSCTRL_BG_EXP_CRC_ADDR   0x2C
#define VSCTRL_BG_DATA_ADDR      0x30
#define VSCTRL_RSVD_34_ADDR      0x34
#define VSCTRL_GPO_ADDR          0x38
#define VSCTRL_GPI_ADDR          0x3C

// --- default values for ATIS chip --- //

// --- register VSCTRL_SRC_CNFG_ADDR --- //
#define ACK_REL_DEL              0x3 //0x05 // 50ns (one tick is 10ns)
#define ACK_SAM_DEL              0x1 //0x03 // 30n
#define ACK_SET_DEL              0x0 //0x02 // 20n
#define AER_LVL                  0x15 // overwrite = 1, ack active low, req active high (ATIS default)
// --- register VSCTRL_SRC_DST_CTRL_ADDR --- //
#define TD_APS_CTRL              0x02 // TD loopback = 0, TD EN =1, APS loppback = 0, APS EN = 1, flush fifo = 0, ignore FIFO Full = 0
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
#define SKCTRL_DUMMY_PERIOD_ADDR        0x04
#define SKCTRL_DUMMY_CFG_ADDR           0x08
#define SKCTRL_DUMMY_BOUND_ADDR         0x0C
#define SKCTRL_DUMMY_INC_ADDR           0x10
#define SKCTRL_RES_TO_ADDR              0x14
#define SKCTRL_EG_UPTHR_ADDR            0x18
#define SKCTRL_EG_DWTHR_ADDR            0x1C
#define SKCTRL_EG_NOISE_RISE_THR_ADDR   0x20
#define SKCTRL_EG_NOISE_FALL_THR_ADDR   0x24
#define SKCTRL_I2C_ACQ_SOFT_RST_ADDR    0x34 // write only
#define SKCTRL_EG_FILTER_ADDR           0x38 // read only
#define SKCTRL_STATUS_ADDR              0x3A // read only

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
#define EG_UP_THR_DEFAULT                       0x00001999 // 32b unsigned fixed point (16b fractional); default value approx. 0.1
// register SKCTRL_EG_DWTHR_ADDR            0x1C
#define EG_DWN_THR_DEFAULT                      0x00001999 // 32b unsigned fixed point (16b fractional); default value approx. 0.1
// register SKCTRL_EG_NOISE_RISE_THR_ADDR      0x20
#define EG_NOISE_RISE_THR_DEFAULT               0x000C0000 // 32b unsigned fixed point (16b fractional); default value approx. 0.1
// register SKCTRL_EG_NOISE_FALL_THR_ADDR      0x24
#define EG_NOISE_FALL_THR_DEFAULT               0x000C0000 // 32b unsigned fixed point (16b fractional); default value approx. 0.1
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




// hpu_core & spinn2neu

#define MAGIC_NUM 0
#define IOC_READ_TS         _IOR (MAGIC_NUM,  1, void *)
#define IOC_CLEAR_TS        _IOW (MAGIC_NUM,  2, void *)
#define AER_VERSION         _IOR (MAGIC_NUM,  3, void *)
#define IOC_SET_TS_TYPE     _IOW (MAGIC_NUM,  7, void *)
//#define AER_TIMESTAMP       _IOR (MAGIC_NUM,  8, void *)
#define AER_GEN_REG         _IOWR(MAGIC_NUM,  8, void *)
#define IOC_GET_PS          _IOR (MAGIC_NUM,  9, void *)
#define AER_SET_LOC_LBCK    _IOW (MAGIC_NUM, 10, void *)
#define AER_SET_REM_LBCK    _IOW (MAGIC_NUM, 11, void *)
#define AER_SET_FAR_LBCK    _IOW (MAGIC_NUM, 12, void *)

#define CTRL_REG         0x00
#define RXDATA_REG       0x08
#define RXTIME_REG       0x0C
#define TXDATA_REG       0x10
#define DMA_REG          0x14
#define RAWI_REG         0x18
#define IRQ_REG          0x1C
#define MASK_REG         0x20
#define STMP_REG         0x28
#define RX_CTRL_REG      0x40
#define TX_CTRL_REG      0x44
#define IP_CFNG_REG      0x50
#define ID_REG           0x5C
#define AUX_RX_CTRL_REG  0x60

// CTRL register bit field
//#define CTRL_ENABLEIP         0x00000001
#define CTRL_ENABLEINTERRUPT    0x00000004
#define CTRL_FLUSHFIFO          0x00000010
#define CTRL_ENABLE_REM_LBCK    0x01000000
#define CTRL_ENABLE_LOC_LBCK    0x02000000
#define CTRL_ENABLE_FAR_LBCK    0x04000000
#define CTRL_32BITCLOCK         0x00008000
#define AUX_RX_ENABLE_SKIN      0x00000F01
#define RX_REG_ENABLE_CAMERAS   0x07010701

// INterrupt Mask register bit field
#define MSK_RXBUF_EMPTY     0x00000001
#define MSK_RXBUF_AEMPTY    0x00000002
#define MSK_RXBUF_FULL      0x00000004
#define MSK_TXBUF_EMPTY     0x00000008
#define MSK_TXBUF_AFULL     0x00000010
#define MSK_TXBUF_FULL      0x00000020
#define MSK_TIMEWRAPPING    0x00000080
#define MSK_RXBUF_READY     0x00000100
#define MSK_RX_NOT_EMPTY    0x00000200
#define MSK_TX_DUMPMODE     0x00001000
#define MSK_RX_PAER_ERR     0x00007000 // fifo full from left, right, aux
#define MSK_RX_MOD_ERR      0x00004000
//#define MASK_RX_EMPTY    0x01
//#define MASK_RX_FULL     0x04
#define MSK_RX_CTRL_REG      0xF070F07
#define MSK_AUX_RX_CTRL_REG  0x0000F07


#endif
