
//--------------------------
// I2C register addresses
//--------------------------
#define C_INFO_ADDR32             0
#define C_FPGAREL_ADDR            0
#define C_FPGACFG_ADDR            1
#define C_INFO2_ADDR              2
#define C_INFO3_ADDR              3

#define C_STATUS_ADDR32           4
#define C_STAT_GEN_ADDR           4
#define C_STAT_PAER_ADDR          5
#define C_STAT_HSSAER_ADDR        6
#define C_STAT_GTP_ADDR           7

#define C_UNUSED_8_ADDR           8
#define C_UNUSED_9_ADDR           9
#define C_UNUSED_10_ADDR         10
#define C_UNUSED_11_ADDR         11

#define C_SRC_CFG_ADDR32         12
#define C_SRCCNFG_ADDR           12
#define C_SRCTIM1_ADDR           13
#define C_SRCTIM2_ADDR           14
#define C_SRCTIM3_ADDR           15

#define C_SRC_CTL_ADDR32         16
#define C_SRCCTRL_ADDR           16
#define C_DSTCTRL_ADDR           17
#define C_UNUSED_18_ADDR         18
#define C_UNUSED_19_ADDR         19

#define C_PAER_CFG_ADDR32        20
#define C_PAERCNFG0_ADDR         20
#define C_PAERCNFG1_ADDR         21
#define C_PAERCNFG2_ADDR         22
#define C_PAERCNFG3_ADDR         23

#define C_SAER_CFG_ADDR32        24
#define C_HSSAERCNFG0_ADDR       24
#define C_HSSAERCNFG1_ADDR       25
#define C_HSSAERCNFG2_ADDR       26
#define C_HSSAERCNFG3_ADDR       27

#define C_GTP_CFG_ADDR32         28
#define C_GTPCNFG0_ADDR          28
#define C_GTPCNFG1_ADDR          29
#define C_GTPCNFG2_ADDR          30
#define C_GTPCNFG3_ADDR          31

#define C_BG_CFG_ADDR32          32
#define C_BGCNFG0_ADDR           32
#define C_BGCNFG1_ADDR           33
#define C_BGCTRL0_ADDR           34
#define C_BGCTRL1_ADDR           35

#define C_BG_PRESC_ADDR32        36
#define C_BGPRESC0_ADDR          36
#define C_BGPRESC1_ADDR          37
#define C_BGPRESC2_ADDR          38
#define C_BGPRESC3_ADDR          39

#define C_BG_TIM_ADDR32          40
#define C_BGTIM0_ADDR            40
#define C_BGTIM1_ADDR            41
#define C_BGTIM2_ADDR            42
#define C_BGTIM3_ADDR            43

#define C_BG_CRC_ADDR32          44
#define C_BGCRC0_ADDR            44
#define C_BGCRC1_ADDR            45
#define C_BGCRC2_ADDR            46
#define C_BGCRC3_ADDR            47

#define C_BG_DATA_ADDR32         48
#define C_BGDATA0_ADDR           48
#define C_BGDATA1_ADDR           49
#define C_BGDATA2_ADDR           50
#define C_BGDATA3_ADDR           51

#define C_UNUSED_52_ADDR         52
#define C_UNUSED_53_ADDR         53
#define C_UNUSED_54_ADDR         54
#define C_UNUSED_55_ADDR         55

#define C_OUT_SDBUS_ADDR32       56
#define C_OUTSDBUS0_ADDR         56
#define C_OUTSDBUS1_ADDR         57
#define C_OUTSDBUS2_ADDR         58
#define C_OUTSDBUS3_ADDR         59

#define C_IN_SDBUS_ADDR32        60
#define C_INSDBUS0_ADDR          60
#define C_INSDBUS1_ADDR          61
#define C_INSDBUS2_ADDR          62
#define C_INSDBUS3_ADDR          63

//--------------------------
// Bias Generator CRC polynomial
//--------------------------
#define C_BG_CRC_POLY            0x04C11DB7

//--------------------------
// Chip type ID
//--------------------------
#define C_CHIP_AUTO              -1
#define C_CHIP_DVS                0
#define C_CHIP_ATIS               1
#define C_CHIP_RSVD2              2
#define C_CHIP_RSVD3              3
#define C_CHIP_RSVD4              4
#define C_CHIP_RSVD5              5
#define C_CHIP_RSVD6              6
#define C_CHIP_RSVD7              7

//--------------------------
// register Bit mask
//--------------------------
#define C_BG_CRCCLR_MSK        0x01 
#define C_BG_CRCEN_MSK         0x02
#define C_BG_PWRDWN_MSK        0x10 
#define C_BG_BIASROI_MSK       0x20

#define C_BG_SHFTCNT_MSK       0x3F
#define C_BG_AUTORST_MSK       0x40
#define C_BG_LTCHOUT_MSK       0x80

#define C_ARRAY_RST_MSK        0x01

