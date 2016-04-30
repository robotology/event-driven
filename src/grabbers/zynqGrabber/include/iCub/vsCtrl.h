//
//  vsCtrl.h
//  eMorph
//
//  Created by Chiara Bartolozzi on 27/08/15.
//
//

#ifndef eMorph_vsCtrl_h
#define eMorph_vsCtrl_h

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>


#define MAX_BUF_SIZE 16777216
// vsctrl

#define VSCTRL_MAGIC_NUM 101

#define VSCTRL_GEN_REG_ACCESS    _IOWR(VSCTRL_MAGIC_NUM,  5, void *)
#define VSCTRL_GET_FPGAREL       _IOR (VSCTRL_MAGIC_NUM,  6, unsigned int *)
#define VSCTRL_GET_INFO          _IOR (VSCTRL_MAGIC_NUM,  7, unsigned int *)
#define VSCTRL_GET_STATUS        _IOR (VSCTRL_MAGIC_NUM,  8, unsigned int *)
#define VSCTRL_INIT_DEV          _IOW (VSCTRL_MAGIC_NUM,  9, unsigned int)
#define VSCTRL_RESET_ARRAY       _IOW (VSCTRL_MAGIC_NUM, 10, unsigned int)
#define VSCTRL_SET_PWRDWN        _IOW (VSCTRL_MAGIC_NUM, 11, unsigned int)
#define VSCTRL_SET_BIASGEN       _IOW (VSCTRL_MAGIC_NUM, 12, unsigned int)
#define VSCTRL_SET_ROIGEN        _IOW (VSCTRL_MAGIC_NUM, 13, unsigned int)
#define VSCTRL_SET_GPO           _IOW (VSCTRL_MAGIC_NUM, 14, unsigned int)
#define VSCTRL_GET_GPI           _IOR (VSCTRL_MAGIC_NUM, 15, unsigned int *)
#define VSCTRL_SET_AER_TIMINGS   _IOW (VSCTRL_MAGIC_NUM, 16, void *)
#define VSCTRL_SET_BG_TIMINGS    _IOW (VSCTRL_MAGIC_NUM, 17, void *)
#define VSCTRL_GET_AER_TIMINGS   _IOR (VSCTRL_MAGIC_NUM, 18, void *)
#define VSCTRL_GET_BG_TIMINGS    _IOR (VSCTRL_MAGIC_NUM, 19, void *)
#define VSCTRL_CLR_STATUS        _IOW (VSCTRL_MAGIC_NUM, 20, unsigned int)

#define VSCTRL_MAX_BUF_SIZE 16777216

#define CHIP_DVS   0
#define CHIP_ATIS  1
#define CHIP_AUTO -1

#define VSCTRL_IOC_READ  0
#define VSCTRL_IOC_WRITE 1

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

// hpu_core & spinn2neu

#define MAGIC_NUM 100
#define AER_VERSION         _IOR (MAGIC_NUM,  7, void *)
#define AER_TIMESTAMP       _IOR (MAGIC_NUM,  8, void *)
#define AER_GEN_REG         _IOWR(MAGIC_NUM,  6, void *)
#define AER_SET_LOC_LBCK    _IOW (MAGIC_NUM, 10, void *)
#define AER_SET_REM_LBCK    _IOW (MAGIC_NUM, 11, void *)
#define AER_SET_FAR_LBCK    _IOW (MAGIC_NUM, 12, void *)

#define AER_MAX_BUF_SIZE 16777216

#define CTRL_REG     0x00
#define RXDATA_REG   0x08
#define RXTIME_REG   0x0C
#define TXDATA_REG   0x10
#define DMA_REG      0x14
#define RAWI_REG     0x18
#define IRQ_REG      0x1C
#define MASK_REG     0x20
#define STMP_REG     0x28
#define ID_REG       0x5c

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
#define MSK_RX_PAR_ERR   0x00002000
#define MSK_RX_MOD_ERR   0x00004000
//#define MASK_RX_EMPTY    0x01
//#define MASK_RX_FULL     0x04



typedef union vsctrl_ioctl_arg_t {
    struct {
        uint8_t addr;
        char rw;
        uint32_t data;
    } regs;
    struct {
        uint32_t prescaler_value;
        uint8_t setup_hold_time;
        uint8_t clock_active_time;
        uint8_t latch_setup_time;
        uint8_t latch_active_time;
    } bg_timings;
    struct {
        uint8_t cfg_ack_rel_delay;
        uint8_t cfg_sample_delay;
        uint8_t cfg_ack_set_delay;
    } aer_timings;
} vsctrl_ioctl_arg_t;


typedef struct aerGenReg {
    unsigned int offset;
    char         rw;
    unsigned int data;
} aerGenReg_t;

typedef struct fpgaStatus {
    bool crcErr;
    bool biasDone;
    bool i2cTimeout;
    bool apsFifoFull;
    bool tdFifoFull;
} fpgaStatus_t;


#endif
