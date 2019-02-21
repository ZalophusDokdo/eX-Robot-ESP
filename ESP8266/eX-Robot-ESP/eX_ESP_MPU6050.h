#ifndef EX_ESP_MPU6050_H
#define EX_ESP_MPU6050_H

#include "eX_ESP_I2C.h"

class Quaternion {
    public:
        float w;
        float x;
        float y;
        float z;
        
        Quaternion() {
            w = 1.0f;
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
        }
        
        Quaternion(float nw, float nx, float ny, float nz) {
            w = nw;
            x = nx;
            y = ny;
            z = nz;
        }

        Quaternion getProduct(Quaternion q) {
            // Quaternion multiplication is defined by:
            //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
            //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
            //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
            //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
            return Quaternion(
                w*q.w - x*q.x - y*q.y - z*q.z,   // new w
                w*q.x + x*q.w + y*q.z - z*q.y,   // new x
                w*q.y - x*q.z + y*q.w + z*q.x,   // new y
                w*q.z + x*q.y - y*q.x + z*q.w);  // new z
        }

        Quaternion getConjugate() {
            return Quaternion(w, -x, -y, -z);
        }
        
        float getMagnitude() {
            return sqrt(w*w + x*x + y*y + z*z);
        }
        
        void normalize() {
            float m = getMagnitude();
            w /= m;
            x /= m;
            y /= m;
            z /= m;
        }
        
        Quaternion getNormalized() {
            Quaternion r(w, x, y, z);
            r.normalize();
            return r;
        }
};

Quaternion        q;

#define MPU6050_ADDRESS_AD0_LOW     0x68   // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69   // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

//uint8_t   devAddr = MPU6050_DEFAULT_ADDRESS;
uint8_t   buffer[14];
uint8_t  *dmpPacketBuffer;
uint16_t  dmpPacketSize;
uint8_t   mpuIntStatus;
uint16_t  fifoCount;                      // count of all bytes currently in FIFO
uint8_t   fifoBuffer[128];                // FIFO storage buffer

#define MPU6050_RA_XG_OFFS_TC       0x00  // [7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01  // [7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02  // [7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
//#define MPU6050_RA_X_FINE_GAIN     0x03  // [7:0] X_FINE_GAIN
//#define MPU6050_RA_Y_FINE_GAIN     0x04  // [7:0] Y_FINE_GAIN
//#define MPU6050_RA_Z_FINE_GAIN     0x05  // [7:0] Z_FINE_GAIN
//#define MPU6050_RA_XA_OFFS_H       0x06  // [15:0] XA_OFFS
//#define MPU6050_RA_XA_OFFS_L_TC    0x07
//#define MPU6050_RA_YA_OFFS_H       0x08  // [15:0] YA_OFFS
//#define MPU6050_RA_YA_OFFS_L_TC    0x09
//#define MPU6050_RA_ZA_OFFS_H       0x0A  // [15:0] ZA_OFFS
//#define MPU6050_RA_ZA_OFFS_L_TC    0x0B
//#define MPU6050_RA_XG_OFFS_USRH    0x13  // [15:0] XG_OFFS_USR
//#define MPU6050_RA_XG_OFFS_USRL    0x14
//#define MPU6050_RA_YG_OFFS_USRH    0x15  // [15:0] YG_OFFS_USR
//#define MPU6050_RA_YG_OFFS_USRL    0x16
//#define MPU6050_RA_ZG_OFFS_USRH    0x17  // [15:0] ZG_OFFS_USR
//#define MPU6050_RA_ZG_OFFS_USRL    0x18
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
//#define MPU6050_RA_FF_THR          0x1D
//#define MPU6050_RA_FF_DUR          0x1E
#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22
//#define MPU6050_RA_FIFO_EN         0x23
//#define MPU6050_RA_I2C_MST_CTRL    0x24
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
//#define MPU6050_RA_I2C_SLV0_REG    0x26
//#define MPU6050_RA_I2C_SLV0_CTRL   0x27
//#define MPU6050_RA_I2C_SLV1_ADDR   0x28
//#define MPU6050_RA_I2C_SLV1_REG    0x29
//#define MPU6050_RA_I2C_SLV1_CTRL   0x2A
//#define MPU6050_RA_I2C_SLV2_ADDR   0x2B
//#define MPU6050_RA_I2C_SLV2_REG    0x2C
//#define MPU6050_RA_I2C_SLV2_CTRL   0x2D
//#define MPU6050_RA_I2C_SLV3_ADDR   0x2E
//#define MPU6050_RA_I2C_SLV3_REG    0x2F
//#define MPU6050_RA_I2C_SLV3_CTRL   0x30
//#define MPU6050_RA_I2C_SLV4_ADDR   0x31
//#define MPU6050_RA_I2C_SLV4_REG    0x32
//#define MPU6050_RA_I2C_SLV4_DO     0x33
//#define MPU6050_RA_I2C_SLV4_CTRL   0x34
//#define MPU6050_RA_I2C_SLV4_DI     0x35
//#define MPU6050_RA_I2C_MST_STATUS  0x36
//#define MPU6050_RA_INT_PIN_CFG     0x37
#define MPU6050_RA_INT_ENABLE       0x38
//#define MPU6050_RA_DMP_INT_STATUS  0x39
#define MPU6050_RA_INT_STATUS       0x3A
//#define MPU6050_RA_ACCEL_XOUT_H    0x3B
//#define MPU6050_RA_ACCEL_XOUT_L    0x3C
//#define MPU6050_RA_ACCEL_YOUT_H    0x3D
//#define MPU6050_RA_ACCEL_YOUT_L    0x3E
//#define MPU6050_RA_ACCEL_ZOUT_H    0x3F
//#define MPU6050_RA_ACCEL_ZOUT_L    0x40
//#define MPU6050_RA_TEMP_OUT_H      0x41
//#define MPU6050_RA_TEMP_OUT_L      0x42
//#define MPU6050_RA_GYRO_XOUT_H     0x43
//#define MPU6050_RA_GYRO_XOUT_L     0x44
//#define MPU6050_RA_GYRO_YOUT_H     0x45
//#define MPU6050_RA_GYRO_YOUT_L     0x46
//#define MPU6050_RA_GYRO_ZOUT_H     0x47
//#define MPU6050_RA_GYRO_ZOUT_L     0x48
//#define MPU6050_RA_EXT_SENS_DATA_00   0x49
//#define MPU6050_RA_EXT_SENS_DATA_01   0x4A
//#define MPU6050_RA_EXT_SENS_DATA_02   0x4B
//#define MPU6050_RA_EXT_SENS_DATA_03   0x4C
//#define MPU6050_RA_EXT_SENS_DATA_04   0x4D
//#define MPU6050_RA_EXT_SENS_DATA_05   0x4E
//#define MPU6050_RA_EXT_SENS_DATA_06   0x4F
//#define MPU6050_RA_EXT_SENS_DATA_07   0x50
//#define MPU6050_RA_EXT_SENS_DATA_08   0x51
//#define MPU6050_RA_EXT_SENS_DATA_09   0x52
//#define MPU6050_RA_EXT_SENS_DATA_10   0x53
//#define MPU6050_RA_EXT_SENS_DATA_11   0x54
//#define MPU6050_RA_EXT_SENS_DATA_12   0x55
//#define MPU6050_RA_EXT_SENS_DATA_13   0x56
//#define MPU6050_RA_EXT_SENS_DATA_14   0x57
//#define MPU6050_RA_EXT_SENS_DATA_15   0x58
//#define MPU6050_RA_EXT_SENS_DATA_16   0x59
//#define MPU6050_RA_EXT_SENS_DATA_17   0x5A
//#define MPU6050_RA_EXT_SENS_DATA_18   0x5B
//#define MPU6050_RA_EXT_SENS_DATA_19   0x5C
//#define MPU6050_RA_EXT_SENS_DATA_20   0x5D
//#define MPU6050_RA_EXT_SENS_DATA_21   0x5E
//#define MPU6050_RA_EXT_SENS_DATA_22   0x5F
//#define MPU6050_RA_EXT_SENS_DATA_23   0x60
//#define MPU6050_RA_MOT_DETECT_STATUS  0x61
//#define MPU6050_RA_I2C_SLV0_DO     0x63
//#define MPU6050_RA_I2C_SLV1_DO     0x64
//#define MPU6050_RA_I2C_SLV2_DO     0x65
//#define MPU6050_RA_I2C_SLV3_DO     0x66
//#define MPU6050_RA_I2C_MST_DELAY_CTRL 0x67
//#define MPU6050_RA_SIGNAL_PATH_RESET  0x68
//#define MPU6050_RA_MOT_DETECT_CTRL    0x69
#define MPU6050_RA_USER_CTRL        0x6A

#define MPU6050_RA_PWR_MGMT_1       0x6B
//#define MPU6050_RA_PWR_MGMT_2      0x6C
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
#define MPU6050_RA_FIFO_COUNTH      0x72
//#define MPU6050_RA_FIFO_COUNTL     0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75

//#define MPU6050_TC_PWR_MODE_BIT    7
#define MPU6050_TC_OFFSET_BIT       6
#define MPU6050_TC_OFFSET_LENGTH    6
#define MPU6050_TC_OTP_BNK_VLD_BIT  0

//#define MPU6050_VDDIO_LEVEL_VLOGIC    0
//#define MPU6050_VDDIO_LEVEL_VDD       1

#define MPU6050_CFG_EXT_SYNC_SET_BIT     5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH  3
#define MPU6050_CFG_DLPF_CFG_BIT         2
#define MPU6050_CFG_DLPF_CFG_LENGTH      3

//#define MPU6050_EXT_SYNC_DISABLED       0x0
#define MPU6050_EXT_SYNC_TEMP_OUT_L   0x1
//#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
//#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
//#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
//#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
//#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
//#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7

//#define MPU6050_DLPF_BW_256          0x00
//#define MPU6050_DLPF_BW_188          0x01
//#define MPU6050_DLPF_BW_98           0x02
#define MPU6050_DLPF_BW_42            0x03
//#define MPU6050_DLPF_BW_20           0x04
#define MPU6050_DLPF_BW_10            0x05
//#define MPU6050_DLPF_BW_5            0x06

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

//#define MPU6050_GYRO_FS_250          0x00
//#define MPU6050_GYRO_FS_500          0x01
//#define MPU6050_GYRO_FS_1000         0x02
#define MPU6050_GYRO_FS_2000          0x03

//#define MPU6050_ACONFIG_XA_ST_BIT    7
//#define MPU6050_ACONFIG_YA_ST_BIT    6
//#define MPU6050_ACONFIG_ZA_ST_BIT    5
#define MPU6050_ACONFIG_AFS_SEL_BIT      4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH   2
//#define MPU6050_ACONFIG_ACCEL_HPF_BIT      2
//#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH   3

#define MPU6050_ACCEL_FS_2            0x00
//#define MPU6050_ACCEL_FS_4           0x01
//#define MPU6050_ACCEL_FS_8           0x02
//#define MPU6050_ACCEL_FS_16          0x03

//#define MPU6050_DHPF_RESET           0x00
//#define MPU6050_DHPF_5               0x01
//#define MPU6050_DHPF_2P5             0x02
//#define MPU6050_DHPF_1P25            0x03
//#define MPU6050_DHPF_0P63            0x04
//#define MPU6050_DHPF_HOLD            0x07

//#define MPU6050_TEMP_FIFO_EN_BIT     7
//#define MPU6050_XG_FIFO_EN_BIT       6
//#define MPU6050_YG_FIFO_EN_BIT       5
//#define MPU6050_ZG_FIFO_EN_BIT       4
//#define MPU6050_ACCEL_FIFO_EN_BIT    3
//#define MPU6050_SLV2_FIFO_EN_BIT     2
//#define MPU6050_SLV1_FIFO_EN_BIT     1
//#define MPU6050_SLV0_FIFO_EN_BIT     0

//#define MPU6050_MULT_MST_EN_BIT      7
//#define MPU6050_WAIT_FOR_ES_BIT      6
//#define MPU6050_SLV_3_FIFO_EN_BIT    5
//#define MPU6050_I2C_MST_P_NSR_BIT    4
//#define MPU6050_I2C_MST_CLK_BIT      3
//#define MPU6050_I2C_MST_CLK_LENGTH   4

//#define MPU6050_CLOCK_DIV_348        0x0
//#define MPU6050_CLOCK_DIV_333        0x1
//#define MPU6050_CLOCK_DIV_320        0x2
//#define MPU6050_CLOCK_DIV_308        0x3
//#define MPU6050_CLOCK_DIV_296        0x4
//#define MPU6050_CLOCK_DIV_286        0x5
//#define MPU6050_CLOCK_DIV_276        0x6
//#define MPU6050_CLOCK_DIV_267        0x7
//#define MPU6050_CLOCK_DIV_258        0x8
//#define MPU6050_CLOCK_DIV_500        0x9
//#define MPU6050_CLOCK_DIV_471        0xA
//#define MPU6050_CLOCK_DIV_444        0xB
//#define MPU6050_CLOCK_DIV_421        0xC
//#define MPU6050_CLOCK_DIV_400        0xD
//#define MPU6050_CLOCK_DIV_381        0xE
//#define MPU6050_CLOCK_DIV_364        0xF

//#define MPU6050_I2C_SLV_RW_BIT           7
//#define MPU6050_I2C_SLV_ADDR_BIT         6
//#define MPU6050_I2C_SLV_ADDR_LENGTH      7
//#define MPU6050_I2C_SLV_EN_BIT           7
//#define MPU6050_I2C_SLV_BYTE_SW_BIT      6
//#define MPU6050_I2C_SLV_REG_DIS_BIT      5
//#define MPU6050_I2C_SLV_GRP_BIT          4
//#define MPU6050_I2C_SLV_LEN_BIT          3
//#define MPU6050_I2C_SLV_LEN_LENGTH       4

//#define MPU6050_I2C_SLV4_RW_BIT          7
//#define MPU6050_I2C_SLV4_ADDR_BIT        6
//#define MPU6050_I2C_SLV4_ADDR_LENGTH     7
//#define MPU6050_I2C_SLV4_EN_BIT          7
//#define MPU6050_I2C_SLV4_INT_EN_BIT      6
//#define MPU6050_I2C_SLV4_REG_DIS_BIT     5
//#define MPU6050_I2C_SLV4_MST_DLY_BIT     4
//#define MPU6050_I2C_SLV4_MST_DLY_LENGTH  5

//#define MPU6050_MST_PASS_THROUGH_BIT     7
//#define MPU6050_MST_I2C_SLV4_DONE_BIT    6
//#define MPU6050_MST_I2C_LOST_ARB_BIT     5
//#define MPU6050_MST_I2C_SLV4_NACK_BIT    4
//#define MPU6050_MST_I2C_SLV3_NACK_BIT    3
//#define MPU6050_MST_I2C_SLV2_NACK_BIT    2
//#define MPU6050_MST_I2C_SLV1_NACK_BIT    1
//#define MPU6050_MST_I2C_SLV0_NACK_BIT    0

//#define MPU6050_INTCFG_INT_LEVEL_BIT        7
//#define MPU6050_INTCFG_INT_OPEN_BIT         6
//#define MPU6050_INTCFG_LATCH_INT_EN_BIT     5
//#define MPU6050_INTCFG_INT_RD_CLEAR_BIT     4
//#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT  3
//#define MPU6050_INTCFG_FSYNC_INT_EN_BIT     2
//#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1
//#define MPU6050_INTCFG_CLKOUT_EN_BIT        0

//#define MPU6050_INTMODE_ACTIVEHIGH       0x00
//#define MPU6050_INTMODE_ACTIVELOW        0x01

//#define MPU6050_INTDRV_PUSHPULL          0x00
//#define MPU6050_INTDRV_OPENDRAIN         0x01

//#define MPU6050_INTLATCH_50USPULSE       0x00
//#define MPU6050_INTLATCH_WAITCLEAR       0x01

//#define MPU6050_INTCLEAR_STATUSREAD      0x00
//#define MPU6050_INTCLEAR_ANYREAD         0x01

//#define MPU6050_INTERRUPT_FF_BIT            7
//#define MPU6050_INTERRUPT_MOT_BIT           6
//#define MPU6050_INTERRUPT_ZMOT_BIT          5
//#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
//#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
//#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
//#define MPU6050_INTERRUPT_DMP_INT_BIT       1
//#define MPU6050_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
//#define MPU6050_DMPINT_5_BIT            5
//#define MPU6050_DMPINT_4_BIT            4
//#define MPU6050_DMPINT_3_BIT            3
//#define MPU6050_DMPINT_2_BIT            2
//#define MPU6050_DMPINT_1_BIT            1
//#define MPU6050_DMPINT_0_BIT            0

//#define MPU6050_MOTION_MOT_XNEG_BIT     7
//#define MPU6050_MOTION_MOT_XPOS_BIT     6
//#define MPU6050_MOTION_MOT_YNEG_BIT     5
//#define MPU6050_MOTION_MOT_YPOS_BIT     4
//#define MPU6050_MOTION_MOT_ZNEG_BIT     3
//#define MPU6050_MOTION_MOT_ZPOS_BIT     2
//#define MPU6050_MOTION_MOT_ZRMOT_BIT    0

//#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
//#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
//#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
//#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
//#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
//#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

//#define MPU6050_PATHRESET_GYRO_RESET_BIT     2
//#define MPU6050_PATHRESET_ACCEL_RESET_BIT    1
//#define MPU6050_PATHRESET_TEMP_RESET_BIT     0

//#define MPU6050_DETECT_ACCEL_ON_DELAY_BIT    5
//#define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH 2
//#define MPU6050_DETECT_FF_COUNT_BIT          3
//#define MPU6050_DETECT_FF_COUNT_LENGTH       2
//#define MPU6050_DETECT_MOT_COUNT_BIT         1
//#define MPU6050_DETECT_MOT_COUNT_LENGTH      2

//#define MPU6050_DETECT_DECREMENT_RESET       0x0
//#define MPU6050_DETECT_DECREMENT_1           0x1
//#define MPU6050_DETECT_DECREMENT_2           0x2
//#define MPU6050_DETECT_DECREMENT_4           0x3

#define MPU6050_USERCTRL_DMP_EN_BIT             7
#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
//#define MPU6050_USERCTRL_I2C_IF_DIS_BIT        4
#define MPU6050_USERCTRL_DMP_RESET_BIT          3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
//#define MPU6050_USERCTRL_SIG_COND_RESET_BIT    0

#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
//#define MPU6050_PWR1_CYCLE_BIT         5
//#define MPU6050_PWR1_TEMP_DIS_BIT      3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3 //?1

//#define MPU6050_CLOCK_INTERNAL         0x00
//#define MPU6050_CLOCK_PLL_XGYRO        0x01
//#define MPU6050_CLOCK_PLL_YGYRO        0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
//#define MPU6050_CLOCK_PLL_EXT32K       0x04
//#define MPU6050_CLOCK_PLL_EXT19M       0x05
//#define MPU6050_CLOCK_KEEP_RESET       0x07

//#define MPU6050_PWR2_LP_WAKE_CTRL_BIT       7
//#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    2
//#define MPU6050_PWR2_STBY_XA_BIT            5
//#define MPU6050_PWR2_STBY_YA_BIT            4
//#define MPU6050_PWR2_STBY_ZA_BIT            3
//#define MPU6050_PWR2_STBY_XG_BIT            2
//#define MPU6050_PWR2_STBY_YG_BIT            1
//#define MPU6050_PWR2_STBY_ZG_BIT            0

//#define MPU6050_WAKE_FREQ_1P25      0x0
//#define MPU6050_WAKE_FREQ_2P5       0x1
//#define MPU6050_WAKE_FREQ_5         0x2
//#define MPU6050_WAKE_FREQ_10        0x3

//#define MPU6050_BANKSEL_PRFTCH_EN_BIT       6
//#define MPU6050_BANKSEL_CFG_USER_BANK_BIT   5
//#define MPU6050_BANKSEL_MEM_SEL_BIT         4
//#define MPU6050_BANKSEL_MEM_SEL_LENGTH      5

#define MPU6050_WHO_AM_I_BIT            6
#define MPU6050_WHO_AM_I_LENGTH         6

//#define MPU6050_DMP_MEMORY_BANKS        8
//#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16

#define MPU6050_DMP_CODE_SIZE           1929                  // dmpMemory[]
#define MPU6050_DMP_CONFIG_SIZE         111 // 174 // 192     // dmpConfig[]
#define MPU6050_DMP_UPDATES_SIZE        47                    // dmpUpdates[]

/* ================================================================================================ *
 | 18-byte FIFO packet structure:                                                                   |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][FOOTER]                         |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17                          |
 |                                                                                                  |
 * ================================================================================================ */

// this block of memory gets written to the MPU on start-up, and it seems
// to be volatile memory, so it has to be done each time (it only takes ~1
// second though)
const unsigned char dmpMemory[MPU6050_DMP_CODE_SIZE] PROGMEM = 
{
    // bank 0, 256 bytes
    0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
    0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
    0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
    0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
    0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,

    // bank 1, 256 bytes
    0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
    0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
    0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
    0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,

    // bank 2, 256 bytes
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // bank 3, 256 bytes
    0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F,
    0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2,
    0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF,
    0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C,
    0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1,
    0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01,
    0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80,
    0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0xC5, 0xCD, 0xC7, 0xA9, 0x0C,
    0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80,
    0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E,
    0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9,
    0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24,
    0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0,
    0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86,
    0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,
    0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,

    // bank 4, 256 bytes
    0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
    0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,
    0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,
    0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
    0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,
    0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,
    0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
    0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,
    0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,
    0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,
    0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,
    0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,

    // bank 5, 256 bytes
    0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
    0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,
    0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,
    0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
    0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,
    0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,
    0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
    0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,
    0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A,
    0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60,
    0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97,
    0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04,
    0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78,
    0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79,
    0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68,
    0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68,

    // bank 6, 256 bytes
    0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04,
    0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66,
    0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31,
    0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60,
    0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76,
    0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56,
    0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD,
    0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91,
    0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8,
    0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE,
    0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9,
    0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD,
    0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E,
    0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8,
    0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89,
    0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79,

    // bank 7, 138 bytes (remainder)
    0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8,
    0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA,
    0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB,
    0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3,
    0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3,
    0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,
    0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3, 0xA3,
    0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC,
    0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8, 0xD8, 0xFF 
};

// thanks to Noah Zerkin for piecing this stuff together!
const unsigned char dmpConfig[MPU6050_DMP_CONFIG_SIZE] PROGMEM = 
{
//  BANK    OFFSET  LENGTH  [              DATA                 ]
    0x03,   0x7B,   0x03,   0x4C, 0xCD, 0x6C,                      // FCFG_1 inv_set_gyro_calibration
    0x03,   0xAB,   0x03,   0x36, 0x56, 0x76,                      // FCFG_3 inv_set_gyro_calibration
    0x00,   0x68,   0x04,   0x02, 0xCB, 0x47, 0xA2,                // D_0_104 inv_set_gyro_calibration
    0x02,   0x18,   0x04,   0x00, 0x05, 0x8B, 0xC1,                // D_0_24 inv_set_gyro_calibration
    0x01,   0x0C,   0x04,   0x00, 0x00, 0x00, 0x00,                // D_1_152 inv_set_accel_calibration
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97,    // FCFG_2 inv_set_accel_calibration
    0x03,   0x89,   0x03,   0x26, 0x46, 0x66,                      // FCFG_7 inv_set_accel_calibration
    0x00,   0x6C,   0x02,   0x20, 0x00,                            // D_0_108 inv_set_accel_calibration
    /*
    0x02,   0x40,   0x04,   0x00, 0x00, 0x00, 0x00,                // CPASS_MTX_00 inv_set_compass_calibration
    0x02,   0x44,   0x04,   0x00, 0x00, 0x00, 0x00,                // CPASS_MTX_01
    0x02,   0x48,   0x04,   0x00, 0x00, 0x00, 0x00,                // CPASS_MTX_02
    0x02,   0x4C,   0x04,   0x00, 0x00, 0x00, 0x00,                // CPASS_MTX_10
    0x02,   0x50,   0x04,   0x00, 0x00, 0x00, 0x00,                // CPASS_MTX_11
    0x02,   0x54,   0x04,   0x00, 0x00, 0x00, 0x00,                // CPASS_MTX_12
    0x02,   0x58,   0x04,   0x00, 0x00, 0x00, 0x00,                // CPASS_MTX_20
    0x02,   0x5C,   0x04,   0x00, 0x00, 0x00, 0x00,                // CPASS_MTX_21
    0x02,   0xBC,   0x04,   0x00, 0x00, 0x00, 0x00,                // CPASS_MTX_22
    */
    0x01,   0xEC,   0x04,   0x00, 0x00, 0x40, 0x00,                // D_1_236 inv_apply_endian_accel
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97,    // FCFG_2 inv_set_mpu_sensors
    0x04,   0x02,   0x03,   0x0D, 0x35, 0x5D,                      // CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
  //0x04,   0x02,   0x03,   0x98, 0x98, 0x98,                      // CFG_MOTION_BIAS inv_turn_off bias correction
    0x04,   0x09,   0x04,   0x87, 0x2D, 0x35, 0x3D,                // FCFG_5 inv_set_bias_update
    0x00,   0xA3,   0x01,   0x00,                                  // D_0_163 inv_set_dead_zone
                 // SPECIAL 0x01 = enable interrupts
    0x00,   0x00,   0x00,   0x01,                                  // SET INT_ENABLE at i=22, SPECIAL INSTRUCTION
    0x07,   0x86,   0x01,   0xFE,                                  // CFG_6 inv_set_fifo_interupt
    0x07,   0x41,   0x05,   0xF1, 0x20, 0x28, 0x30, 0x38,          // CFG_8 inv_send_quaternion
    0x07,   0x7E,   0x01,   0x30,                                  // CFG_16 inv_set_footer
    /*
    0x07,   0x46,   0x01,   0x9A,                                  // CFG_GYRO_SOURCE inv_send_gyro
    0x07,   0x47,   0x04,   0xF1, 0x28, 0x30, 0x38,                // CFG_9 inv_send_gyro -> inv_construct3_fifo
    0x07,   0x6C,   0x04,   0xF1, 0x28, 0x30, 0x38,                // CFG_12 inv_send_accel -> inv_construct3_fifo
    0x02,   0x16,   0x02,   0x00, 0x01                             // D_0_22 inv_set_fifo_rate
    */
    0x02,   0x16,   0x02,   0x00, 0x00                             // D_0_22 inv_set_fifo_rate // Original 0x01

    // This very last 0x01 WAS a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
    // 0x01 is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
    // DMP output frequency is calculated easily using this equation: (200Hz / (1 + value))

    // It is important to make sure the host processor can keep up with reading and processing
    // the FIFO output at the desired rate. Handling FIFO overflow cleanly is also a good idea.
};

const unsigned char dmpUpdates[MPU6050_DMP_UPDATES_SIZE] PROGMEM = 
{
    0x01,   0xB2,   0x02,   0xFF, 0xFF,
    0x01,   0x90,   0x04,   0x09, 0x23, 0xA1, 0x35,
    0x01,   0x6A,   0x02,   0x06, 0x00,
    0x01,   0x60,   0x08,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00,   0x60,   0x04,   0x40, 0x00, 0x00, 0x00,
    0x01,   0x62,   0x02,   0x00, 0x00,
    0x00,   0x60,   0x04,   0x00, 0x40, 0x00, 0x00
};

void mpuReset() 
{
    i2c_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);
}

void mpuSetClockSource(uint8_t source) 
{
    i2c_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void mpuSetFullScaleGyroRange(uint8_t range) 
{
    i2c_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void mpuSetFullScaleAccelRange(uint8_t range) 
{
    i2c_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

void mpuSetDLPFMode(uint8_t mode) 
{
    i2c_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

void mpuSetRate(uint8_t rate) 
{
    i2c_WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, rate);
}

void mpuSetSleepEnabled(bool enabled) 
{
   i2c_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

void mpuSetMemoryBank(uint8_t bank, bool prefetchEnabled=false, bool userBank=false) 
{
    bank &= 0x1F;
    if (userBank) bank |= 0x20;
    if (prefetchEnabled) bank |= 0x40;
    i2c_WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_BANK_SEL, bank);
}

void mpuSetMemoryStartAddress(uint8_t address) 
{
    i2c_WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_MEM_START_ADDR, address);
}

int8_t mpuGetXGyroOffsetTC() 
{
    i2c_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}

void mpuSetXGyroOffsetTC(int8_t offset) 
{
    i2c_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

int8_t mpuGetYGyroOffsetTC() 
{
    i2c_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}

void mpuSetYGyroOffsetTC(int8_t offset) 
{
    i2c_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

int8_t mpuGetZGyroOffsetTC() 
{
    i2c_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}

void mpuSetZGyroOffsetTC(int8_t offset) 
{
    i2c_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

void mpuSetSlaveAddress(uint8_t num, uint8_t address) 
{
    if (num > 3) return;
    i2c_WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_I2C_SLV0_ADDR + num*3, address);
}

void mpuSetI2CMasterModeEnabled(bool enabled) 
{
    i2c_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void mpuResetI2CMaster() 
{
    i2c_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, true);
}

bool mpuWriteMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank=0, uint8_t address=0, bool verify=true, bool useProgMem=false) 
{
  mpuSetMemoryBank(bank);
  mpuSetMemoryStartAddress(address);
  uint8_t chunkSize;
  uint8_t *verifyBuffer;
  uint8_t *progBuffer;
  uint16_t i;
  uint8_t j;
  if (verify) verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
  if (useProgMem) progBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
  for (i = 0; i < dataSize;) 
  {
     
    // determine correct chunk size according to bank position and data size
    chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

    // make sure we don't go past the data size
    if (i + chunkSize > dataSize) chunkSize = dataSize - i;

    // make sure this chunk doesn't go past the bank boundary (256 bytes)
    if (chunkSize > 256 - address) chunkSize = 256 - address;
        
    if (useProgMem) 
    {
      // write the chunk of data as specified
      for (j = 0; j < chunkSize; j++) progBuffer[j] = pgm_read_byte(data + i + j);
    }
    else
    {
      // write the chunk of data as specified
      progBuffer = (uint8_t *)data + i;
    }

    i2c_WriteBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_MEM_R_W, chunkSize, progBuffer);

    // verify data if needed
    if (verify && verifyBuffer)
    {
      mpuSetMemoryBank(bank);
      mpuSetMemoryStartAddress(address);
      i2c_ReadBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer);
      if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) 
      {
        /*Serial.print("Block write verification error, bank ");
        Serial.print(bank, DEC);
        Serial.print(", address ");
        Serial.print(address, DEC);
        Serial.print("!\nExpected:");
        for (j = 0; j < chunkSize; j++) 
        {
           Serial.print(" 0x");
           if (progBuffer[j] < 16) Serial.print("0");
           Serial.print(progBuffer[j], HEX);
        }
        Serial.print("\nReceived:");
        for (uint8_t j = 0; j < chunkSize; j++) 
        {
           Serial.print(" 0x");
           if (verifyBuffer[i + j] < 16) Serial.print("0");
           Serial.print(verifyBuffer[i + j], HEX);
        }
        Serial.print("\n");*/
        free(verifyBuffer);
        if (useProgMem) free(progBuffer);
        return false; // uh oh.
      }
    }

    // increase byte index by [chunkSize]
    i += chunkSize;

    // uint8_t automatically wraps to 0 at 256
    address += chunkSize;

    // if we aren't done, update bank (if necessary) and address
    if (i < dataSize) 
    {
       if (address == 0) bank++;
       mpuSetMemoryBank(bank);
       mpuSetMemoryStartAddress(address);
    }
  }
  if (verify) free(verifyBuffer);
  if (useProgMem) free(progBuffer);
  return true;
}

bool mpuWriteProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank=0, uint8_t address=0, bool verify=true) 
{
    return mpuWriteMemoryBlock(data, dataSize, bank, address, verify, true);
}

uint8_t mpuReadMemoryByte() 
{
    i2c_ReadByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_MEM_R_W, buffer);
    return buffer[0];
}

void mpuReadMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank=0, uint8_t address=0) 
{
    mpuSetMemoryBank(bank);
    mpuSetMemoryStartAddress(address);
    uint8_t chunkSize;
    for (uint16_t i = 0; i < dataSize;) 
    {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        // read the chunk of data as specified
        i2c_ReadBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_MEM_R_W, chunkSize, data + i);
        
        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) 
        {
            if (address == 0) bank++;
            mpuSetMemoryBank(bank);
            mpuSetMemoryStartAddress(address);
        }
    }
}

void mpuWriteMemoryByte(uint8_t data) 
{
    i2c_WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_MEM_R_W, data);
}

bool mpuWriteDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, bool useProgMem=false) 
{
    uint8_t *progBuffer, success, special;
    uint16_t i, j;
    if (useProgMem) 
    {
        progBuffer = (uint8_t *)malloc(8); // assume 8-byte blocks, realloc later if necessary
    }

    // config set data is a long string of blocks with the following structure:
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
    uint8_t bank, offset, length;
    for (i = 0; i < dataSize;) 
    {
        if (useProgMem) 
        {
            bank = pgm_read_byte(data + i++);
            offset = pgm_read_byte(data + i++);
            length = pgm_read_byte(data + i++);
        }
        else
        {
            bank = data[i++];
            offset = data[i++];
            length = data[i++];
        }

        // write data or perform special action
        if (length > 0) 
        {
            // regular block of data to write
            /*Serial.print("Writing config block to bank ");
            Serial.print(bank);
            Serial.print(", offset ");
            Serial.print(offset);
            Serial.print(", length=");
            Serial.println(length);*/
            if (useProgMem) 
            {
                if (sizeof(progBuffer) < length) progBuffer = (uint8_t *)realloc(progBuffer, length);
                for (j = 0; j < length; j++) progBuffer[j] = pgm_read_byte(data + i + j);
            }
            else
            {
                progBuffer = (uint8_t *)data + i;
            }
            success = mpuWriteMemoryBlock(progBuffer, length, bank, offset, true);
            i += length;
        }
        else
        {
            // special instruction
            // NOTE: this kind of behavior (what and when to do certain things)
            // is totally undocumented. This code is in here based on observed
            // behavior only, and exactly why (or even whether) it has to be here
            // is anybody's guess for now.
            if (useProgMem) {
                special = pgm_read_byte(data + i++);
            }
            else
            {
                special = data[i++];
            }
            /*Serial.print("Special command code ");
            Serial.print(special, HEX);
            Serial.println(" found...");*/
            if (special == 0x01) 
            {
                // enable DMP-related interrupts
                
                //setIntZeroMotionEnabled(true);
                //setIntFIFOBufferOverflowEnabled(true);
                //setIntDMPEnabled(true);
                i2c_WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, 0x32);  // single operation

                success = true;
            }
            else
            {
                // unknown special command
                success = false;
            }
        }
        
        if (!success)
        {
            if (useProgMem) free(progBuffer);
            return false; // uh oh
        }
    }
    if (useProgMem) free(progBuffer);
    return true;
}

bool mpuWriteProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize) 
{
    return mpuWriteDMPConfigurationSet(data, dataSize, true);
}

void mpuSetIntEnabled(uint8_t enabled) 
{
    i2c_WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, enabled);
}

void mpuSetExternalFrameSync(uint8_t sync) 
{
    i2c_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}

void mpuSetDMPConfig1(uint8_t config) 
{
    i2c_WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_DMP_CFG_1, config);
}

void mpuSetDMPConfig2(uint8_t config) 
{
    i2c_WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_DMP_CFG_2, config);
}

void mpuSetOTPBankValid(bool enabled) 
{
    i2c_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}

uint8_t mpuGetOTPBankValid() 
{
    i2c_ReadBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, buffer);
    return buffer[0];
}

void mpuResetFIFO() 
{
    i2c_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
}

uint16_t mpuGetFIFOCount() 
{
    i2c_ReadBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_COUNTH, 2, buffer);
    return (((uint16_t)buffer[0]) << 8) | buffer[1];
}

void mpuGetFIFOBytes(uint8_t *data, uint8_t length) 
{
    i2c_ReadBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_R_W, length, data);
}

void mpuSetMotionDetectionThreshold(uint8_t threshold) 
{
    i2c_WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_MOT_THR, threshold);
}

void mpuSetMotionDetectionDuration(uint8_t duration) 
{
    i2c_WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_MOT_DUR, duration);
}

void mpuSetZeroMotionDetectionThreshold(uint8_t threshold) 
{
    i2c_WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZRMOT_THR, threshold);
}

void mpuSetZeroMotionDetectionDuration(uint8_t duration) 
{
    i2c_WriteByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZRMOT_DUR, duration);
}

void mpuSetFIFOEnabled(bool enabled) 
{
    i2c_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}

void mpuSetDMPEnabled(bool enabled) 
{
    i2c_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}
void mpuResetDMP() 
{
    i2c_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, true);
}

uint8_t mpuGetIntStatus() 
{
    i2c_ReadByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_STATUS, buffer);
    return buffer[0];
}

uint8_t mpuGetDeviceID() 
{
    i2c_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, buffer);
    return buffer[0];
}

bool mpuTestConnection() 
{
    return mpuGetDeviceID() == 0x34;
}

////////////////////////////////////////////////////////////
/*                    DMP section                         */
////////////////////////////////////////////////////////////

uint8_t dmpInitialize() {
    // reset device
    mpuReset();
    delay(30); // wait after reset

    // disable sleep mode
    mpuSetSleepEnabled(false);

    // get MPU hardware revision
    mpuSetMemoryBank(0x10, true, true);
    mpuSetMemoryStartAddress(0x06);
 
    uint8_t hwRevision = mpuReadMemoryByte();

    mpuSetMemoryBank(0);

    // check OTP bank valid
    uint8_t otpValid = mpuGetOTPBankValid();
    //Serial.print("OTPBankValid = ");
    //Serial.println(otpValid ? F("valid!") : F("invalid!"));

    // get X/Y/Z gyro offsets
    int8_t xgOffsetTC = mpuGetXGyroOffsetTC();
    int8_t ygOffsetTC = mpuGetYGyroOffsetTC();
    int8_t zgOffsetTC = mpuGetZGyroOffsetTC();
/*    Serial.print("XGyroOffsetTC = ");
    Serial.println(xgOffsetTC);
    Serial.print("YGyroOffsetTC = ");
    Serial.println(ygOffsetTC);
    Serial.print("ZGyroOffsetTC = ");
    Serial.println(zgOffsetTC);*/
    
    // setup weird slave stuff (?)
    mpuSetSlaveAddress(0, 0x7F);
    mpuSetI2CMasterModeEnabled(false);
    mpuSetSlaveAddress(0, 0x68);
    mpuResetI2CMaster();
    delay(20);

    // load DMP code into memory banks
    if (mpuWriteProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE)) 
    {
      // write DMP configuration
      if (mpuWriteProgDMPConfigurationSet(dmpConfig, MPU6050_DMP_CONFIG_SIZE)) 
      {
          mpuSetClockSource(MPU6050_CLOCK_PLL_ZGYRO);
          mpuSetIntEnabled(0x12);
          mpuSetRate(4); // 1khz / (1 + 4) = 200 Hz
          mpuSetExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);
          mpuSetDLPFMode(MPU6050_DLPF_BW_42);
          mpuSetFullScaleGyroRange(MPU6050_GYRO_FS_2000);
          mpuSetDMPConfig1(0x03);
          mpuSetDMPConfig2(0x00);
          mpuSetOTPBankValid(false);
          mpuSetXGyroOffsetTC(xgOffsetTC);
          mpuSetYGyroOffsetTC(ygOffsetTC);
          mpuSetZGyroOffsetTC(zgOffsetTC);

          uint8_t dmpUpdate[16], j;
          uint16_t pos = 0;
          for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
          mpuWriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

          for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
          mpuWriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

          mpuResetFIFO();

          fifoCount = mpuGetFIFOCount();

          mpuGetFIFOBytes(fifoBuffer, fifoCount);
          mpuSetMotionDetectionThreshold(2);
          mpuSetZeroMotionDetectionThreshold(156);
          mpuSetMotionDetectionDuration(80);
          mpuSetZeroMotionDetectionDuration(0);
          mpuResetFIFO();
          mpuSetFIFOEnabled(true);
          mpuSetDMPEnabled(true);
          mpuResetDMP();

          for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
          mpuWriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

          for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
          mpuWriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

          for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
          mpuWriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

          while ((fifoCount = mpuGetFIFOCount()) < 3);

          mpuGetFIFOBytes(fifoBuffer, fifoCount);
          mpuIntStatus = mpuGetIntStatus();

          for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
          mpuReadMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

          while ((fifoCount = mpuGetFIFOCount()) < 3);

          mpuGetFIFOBytes(fifoBuffer, fifoCount);
          mpuIntStatus = mpuGetIntStatus();

          for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
          mpuWriteMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

          mpuSetDMPEnabled(false);
          dmpPacketSize = 18; // original 42 bytes;
          mpuResetFIFO();
          mpuGetIntStatus();
      } 
      else 
      {
          return 2; // configuration block loading failed
      }
  } 
  else 
  {
      return 1; // main binary block loading failed
  }
  return 0; // success
}

uint8_t dmpGetQuaternion(int16_t *data, const uint8_t* packet)
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[0] << 8) + packet[1]);
    data[1] = ((packet[4] << 8) + packet[5]);
    data[2] = ((packet[8] << 8) + packet[9]);
    data[3] = ((packet[12] << 8) + packet[13]);
    return 0;
}

uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t* packet) 
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    int16_t qI[4];
    uint8_t status = dmpGetQuaternion(qI, packet);
    if (status == 0) {
        q -> w = (float)qI[0] / 16384.0f;
        q -> x = (float)qI[1] / 16384.0f;
        q -> y = (float)qI[2] / 16384.0f;
        q -> z = (float)qI[3] / 16384.0f;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}

// Quick calculation to obtein Phi angle from quaternion solution (from DMP internal quaternion solution)
float dmpGetPhi()
{
  mpuGetFIFOBytes(fifoBuffer, 16);  // We only read the quaternion
  dmpGetQuaternion(&q, fifoBuffer); 
  mpuResetFIFO();                   // We always reset FIFO

  //return( asin(-2 * (q.x * q.z - q.w * q.y)) * 180 / M_PI);  // roll
  //return Phi angle (robot orientation) from quaternion DMP output
  return (atan2(2 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z)* RAD2GRAD);
}

uint8_t mpu_Initialization()
{
  uint8_t devConnectStatus = 0;
  uint8_t devStatus = 0;
  
  // Manual MPU initialization... 
  // accel=2G, gyro=2000º/s, filter=20Hz BW, output=200Hz
  mpuSetClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpuSetFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpuSetFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpuSetDLPFMode(MPU6050_DLPF_BW_10);              //10,20,42,98,188  // Default factor for BROBOT:10
  mpuSetRate(4);                                   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
  mpuSetSleepEnabled(false);
  delay(1000);
  devStatus = dmpInitialize();             // return status after each device operation (0 = success, !0 = error)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    mpuSetDMPEnabled(true);
    mpuIntStatus = mpuGetIntStatus();
    //dmpReady = true;
    delay(10000);
    devConnectStatus = mpuTestConnection();
    if (devConnectStatus == 0)
    {
      mpuSetMemoryBank(0);
      mpuSetMemoryStartAddress(0x60);
      mpuWriteMemoryByte(0);
      mpuWriteMemoryByte(0x20);
      mpuWriteMemoryByte(0);
      mpuWriteMemoryByte(0);
      mpuResetFIFO();
    }
//    else
      // ERROR!
      // 1 = MPU6050 connections failed
  } 
//  else 
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  return devStatus + 4*devConnectStatus;
}

#endif  // EX_ESP_MPU6050_H
