/* IOCTL handling */
#ifndef IIT_I2C_VSCTRL_IOCTL_H
#define IIT_I2C_VSCTRL_IOCTL_H

#define IOC_MAGIC_NUMBER 101

#define IOC_GEN_REG_ACCESS    _IOWR(IOC_MAGIC_NUMBER,  5, unsigned int *)
#define IOC_GET_FPGAREL       _IOR (IOC_MAGIC_NUMBER,  6, unsigned int *)
#define IOC_GET_INFO          _IOR (IOC_MAGIC_NUMBER,  7, unsigned int *)
#define IOC_GET_STATUS        _IOR (IOC_MAGIC_NUMBER,  8, unsigned int *)
#define IOC_INIT_DEV          _IOW (IOC_MAGIC_NUMBER,  9, unsigned int)
#define IOC_RESET_ARRAY       _IOW (IOC_MAGIC_NUMBER, 10, unsigned int)
#define IOC_SET_PWRDWN        _IOW (IOC_MAGIC_NUMBER, 11, unsigned int)
#define IOC_SET_BIASGEN       _IOW (IOC_MAGIC_NUMBER, 12, unsigned int)
#define IOC_SET_ROIGEN        _IOW (IOC_MAGIC_NUMBER, 13, unsigned int)
#define IOC_SET_GPO           _IOW (IOC_MAGIC_NUMBER, 14, unsigned int)
#define IOC_GET_GPI           _IOR (IOC_MAGIC_NUMBER, 15, unsigned int *)
#define IOC_SET_AER_TIMINGS   _IOW (IOC_MAGIC_NUMBER, 16, unsigned int *)
#define IOC_SET_BG_TIMINGS    _IOW (IOC_MAGIC_NUMBER, 17, unsigned int *)
#define IOC_GET_AER_TIMINGS   _IOR (IOC_MAGIC_NUMBER, 18, unsigned int *)
#define IOC_GET_BG_TIMINGS    _IOR (IOC_MAGIC_NUMBER, 19, unsigned int *)
#define IOC_CLR_STATUS        _IOW (IOC_MAGIC_NUMBER, 20, unsigned int)


/*
typedef struct icub_vsctrl_regs_t {
    u8 reg_addr;
    char rw;
    u32 data;
} icub_vsctrl_regs_t;
*/


typedef union icub_vsctrl_ioctl_arg_t {
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
} icub_vsctrl_ioctl_arg_t;

#endif  // IIT_I2C_VSCTRL_IOCTL_H


