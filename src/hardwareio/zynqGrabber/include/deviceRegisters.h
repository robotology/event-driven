/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author: arren.glover@iit.it, chiara.bartolozzi@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef _VDEVICEREGISTERS_
#define _VDEVICEREGISTERS_

 // da mettere su zynq
//#define I2C_SLAVE 0x00 // da togliere su zynq

// vsctrl

#define I2C_ADDRESS_LEFT		 0x11
#define I2C_ADDRESS_RIGHT		 0x10
//#define I2C_ADDRESS_VSCTRL       0x11
//#define I2C_ADDRESS              I2C_ADDRESS_VSCTRL

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
#define BG_PWRDWN_MSK            0x10000000 // powerdown chip mask
#define BG_SHIFT_COUNT_MSK           0x003F0000 // mask of bg shift count
//#define BG_SHIFT_AUTORST_MSK         0x40 // mask of bg shift count
#define BG_LATOUTEND_MSK             0x00800000 // mask of bg shift count
//#define BG_ROISEL_MSK                0x20
#define BG_VAL_MSK                    0x1FFFFF // mask of 21 bit for bg value

// hpu_core & spinn2neu

#define MAGIC_NUM 0
#define AER_VERSION         _IOR (MAGIC_NUM,  3, void *)
#define AER_TIMESTAMP       _IOR (MAGIC_NUM,  8, void *)
#define AER_GEN_REG         _IOWR(MAGIC_NUM,  6, void *)
#define AER_SET_LOC_LBCK    _IOW (MAGIC_NUM, 10, void *)
#define AER_SET_REM_LBCK    _IOW (MAGIC_NUM, 11, void *)
#define AER_SET_FAR_LBCK    _IOW (MAGIC_NUM, 12, void *)

#define CTRL_REG     0x00
#define RXDATA_REG   0x08
#define RXTIME_REG   0x0C
#define TXDATA_REG   0x10
#define DMA_REG      0x14
#define RAWI_REG     0x18
#define IRQ_REG      0x1C
#define MASK_REG     0x20
#define STMP_REG     0x28
#define ID_REG       0x5C

// CTRL register bit field
//#define CTRL_ENABLEIP 0x00000001
#define CTRL_ENABLEINTERRUPT 0x00000004
#define CTRL_FLUSHFIFO       0x00000010
#define CTRL_ENABLE_REM_LBCK 0x01000000
#define CTRL_ENABLE_LOC_LBCK 0x02000000
#define CTRL_ENABLE_FAR_LBCK 0x04000000

// INterrupt Mask register bit field
#define MSK_RXBUF_EMPTY  0x00000001
#define MSK_RXBUF_AEMPTY 0x00000002
#define MSK_RXBUF_FULL   0x00000004
#define MSK_TXBUF_EMPTY  0x00000008
#define MSK_TXBUF_AFULL  0x00000010
#define MSK_TXBUF_FULL   0x00000020
#define MSK_TIMEWRAPPING 0x00000080
#define MSK_RXBUF_READY  0x00000100
#define MSK_RX_NOT_EMPTY 0x00000200
#define MSK_TX_DUMPMODE  0x00001000
#define MSK_RX_PAER_ERR   0x00007000 // fifo full from left, right, aux
#define MSK_RX_MOD_ERR   0x00004000
//#define MASK_RX_EMPTY    0x01
//#define MASK_RX_FULL     0x04


#endif
