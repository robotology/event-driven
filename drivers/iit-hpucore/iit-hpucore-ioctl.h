/* IOCTL handling */
#ifndef IIT_HPUCORE_IOCTL_H
#define IIT_HPUCORE_IOCTL_H

#define IOC_MAGIC_NUMBER 100

#define IOC_RESET                _IOW (IOC_MAGIC_NUMBER,  5, unsigned int *)
#define IOC_GEN_REG              _IOWR(IOC_MAGIC_NUMBER,  6, unsigned int *)
#define IOC_GET_VERSION          _IOR (IOC_MAGIC_NUMBER,  7, unsigned int *)
#define IOC_GET_TIMESTAMP        _IOR (IOC_MAGIC_NUMBER,  8, unsigned int *)
#define IOC_CLEAR_TIMESTAMP      _IOW (IOC_MAGIC_NUMBER,  9, unsigned int *)
#define IOC_SET_LOCNEAR_LB       _IOW (IOC_MAGIC_NUMBER, 10, unsigned int)
#define IOC_SET_LOCFAR_LB        _IOW (IOC_MAGIC_NUMBER, 11, unsigned int)
#define IOC_SET_REMOTE_LB        _IOW (IOC_MAGIC_NUMBER, 12, unsigned int)
//#define IOC_GET_TIMESTAMPER      _IOR (IOC_MAGIC_NUMBER, 13, unsigned int)

typedef struct hpucore_regs {
    u32 reg_offset;
    char rw;
    u32 data;
} hpucore_regs_t;

#endif
