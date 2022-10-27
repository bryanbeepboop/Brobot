/*
 * MPU9250.h
 *
 *           Author: steveb
 *       Created on: March 12, 2021
 *  Last updated on: March 07, 2022
 *       Updated by: K.Choi
 */

#ifndef MPU9250_H_
#define MPU9250_H_

typedef uint8_t MPU9250_REGISTER;
typedef uint8_t AK8963_REGISTER;

#define MPU9250_REGISTER_SELF_TEST_X_GYRO     ((MPU9250_REGISTER)0x00)
#define MPU9250_REGISTER_SELF_TEST_Y_GYRO     ((MPU9250_REGISTER)0x01)
#define MPU9250_REGISTER_SELF_TEST_Z_GYRO     ((MPU9250_REGISTER)0x02)
#define MPU9250_REGISTER_SELF_TEST_X_ACCEL    ((MPU9250_REGISTER)0x0D)
#define MPU9250_REGISTER_SELF_TEST_Y_ACCEL    ((MPU9250_REGISTER)0x0E)
#define MPU9250_REGISTER_SELF_TEST_Z_ACCEL    ((MPU9250_REGISTER)0x0F)
#define MPU9250_REGISTER_XG_OFFSET_H          ((MPU9250_REGISTER)0x13)
#define MPU9250_REGISTER_XG_OFFSET_L          ((MPU9250_REGISTER)0x14)
#define MPU9250_REGISTER_YG_OFFSET_H          ((MPU9250_REGISTER)0x15)
#define MPU9250_REGISTER_YG_OFFSET_L          ((MPU9250_REGISTER)0x16)
#define MPU9250_REGISTER_ZG_OFFSET_H          ((MPU9250_REGISTER)0x17)
#define MPU9250_REGISTER_ZG_OFFSET_L          ((MPU9250_REGISTER)0x18)
#define MPU9250_REGISTER_SMPLRT_DIV           ((MPU9250_REGISTER)0x19)
#define MPU9250_REGISTER_CONFIG               ((MPU9250_REGISTER)0x1A)
#define MPU9250_REGISTER_GYRO_CONFIG          ((MPU9250_REGISTER)0x1B)
#define MPU9250_REGISTER_ACCEL_CONFIG         ((MPU9250_REGISTER)0x1C)
#define MPU9250_REGISTER_ACCEL_CONFIG_2       ((MPU9250_REGISTER)0x1D)
#define MPU9250_REGISTER_LP_ACCEL_ODR         ((MPU9250_REGISTER)0x1E)
#define MPU9250_REGISTER_WOM_THR              ((MPU9250_REGISTER)0x1F)
#define MPU9250_REGISTER_FIFO_EN              ((MPU9250_REGISTER)0x23)
#define MPU9250_REGISTER_I2C_MST_CTRL         ((MPU9250_REGISTER)0x24)
#define MPU9250_REGISTER_I2C_SLV0_ADDR        ((MPU9250_REGISTER)0x25)
#define MPU9250_REGISTER_I2C_SLV0_REG         ((MPU9250_REGISTER)0x26)
#define MPU9250_REGISTER_I2C_SLV0_CTRL        ((MPU9250_REGISTER)0x27)
#define MPU9250_REGISTER_I2C_SLV1_ADDR        ((MPU9250_REGISTER)0x28)
#define MPU9250_REGISTER_I2C_SLV1_REG         ((MPU9250_REGISTER)0x29)
#define MPU9250_REGISTER_I2C_SLV1_CTRL        ((MPU9250_REGISTER)0x2A)
#define MPU9250_REGISTER_I2C_SLV2_ADDR        ((MPU9250_REGISTER)0x2B)
#define MPU9250_REGISTER_I2C_SLV2_REG         ((MPU9250_REGISTER)0x2C)
#define MPU9250_REGISTER_I2C_SLV2_CTRL        ((MPU9250_REGISTER)0x2D)
#define MPU9250_REGISTER_I2C_SLV3_ADDR        ((MPU9250_REGISTER)0x2E)
#define MPU9250_REGISTER_I2C_SLV3_REG         ((MPU9250_REGISTER)0x2F)
#define MPU9250_REGISTER_I2C_SLV3_CTRL        ((MPU9250_REGISTER)0x30)
#define MPU9250_REGISTER_I2C_SLV4_ADDR        ((MPU9250_REGISTER)0x31)
#define MPU9250_REGISTER_I2C_SLV4_REG         ((MPU9250_REGISTER)0x32)
#define MPU9250_REGISTER_I2C_SLV4_DO          ((MPU9250_REGISTER)0x33)
#define MPU9250_REGISTER_I2C_SLV4_CTRL        ((MPU9250_REGISTER)0x34)
#define MPU9250_REGISTER_I2C_SLV4_DI          ((MPU9250_REGISTER)0x35)
#define MPU9250_REGISTER_I2C_MST_STATUS       ((MPU9250_REGISTER)0x36)
#define MPU9250_REGISTER_INT_PIN_CFG          ((MPU9250_REGISTER)0x37)
#define MPU9250_REGISTER_INT_ENABLE           ((MPU9250_REGISTER)0x38)
#define MPU9250_REGISTER_INT_STATUS           ((MPU9250_REGISTER)0x3A)
#define MPU9250_REGISTER_ACCEL_XOUT_H         ((MPU9250_REGISTER)0x3B)
#define MPU9250_REGISTER_ACCEL_XOUT_L         ((MPU9250_REGISTER)0x3C)
#define MPU9250_REGISTER_ACCEL_YOUT_H         ((MPU9250_REGISTER)0x3D)
#define MPU9250_REGISTER_ACCEL_YOUT_L         ((MPU9250_REGISTER)0x3E)
#define MPU9250_REGISTER_ACCEL_ZOUT_H         ((MPU9250_REGISTER)0x3F)
#define MPU9250_REGISTER_ACCEL_ZOUT_L         ((MPU9250_REGISTER)0x40)
#define MPU9250_REGISTER_TEMP_OUT_H           ((MPU9250_REGISTER)0x41)
#define MPU9250_REGISTER_TEMP_OUT_L           ((MPU9250_REGISTER)0x42)
#define MPU9250_REGISTER_GYRO_XOUT_H          ((MPU9250_REGISTER)0x43)
#define MPU9250_REGISTER_GYRO_XOUT_L          ((MPU9250_REGISTER)0x44)
#define MPU9250_REGISTER_GYRO_YOUT_H          ((MPU9250_REGISTER)0x45)
#define MPU9250_REGISTER_GYRO_YOUT_L          ((MPU9250_REGISTER)0x46)
#define MPU9250_REGISTER_GYRO_ZOUT_H          ((MPU9250_REGISTER)0x47)
#define MPU9250_REGISTER_GYRO_ZOUT_L          ((MPU9250_REGISTER)0x48)
#define MPU9250_REGISTER_EXT_SENS_DATA_00     ((MPU9250_REGISTER)0x49)
#define MPU9250_REGISTER_EXT_SENS_DATA_01     ((MPU9250_REGISTER)0x4A)
#define MPU9250_REGISTER_EXT_SENS_DATA_02     ((MPU9250_REGISTER)0x4B)
#define MPU9250_REGISTER_EXT_SENS_DATA_03     ((MPU9250_REGISTER)0x4C)
#define MPU9250_REGISTER_EXT_SENS_DATA_04     ((MPU9250_REGISTER)0x4D)
#define MPU9250_REGISTER_EXT_SENS_DATA_05     ((MPU9250_REGISTER)0x4E)
#define MPU9250_REGISTER_EXT_SENS_DATA_06     ((MPU9250_REGISTER)0x4F)
#define MPU9250_REGISTER_EXT_SENS_DATA_07     ((MPU9250_REGISTER)0x50)
#define MPU9250_REGISTER_EXT_SENS_DATA_08     ((MPU9250_REGISTER)0x51)
#define MPU9250_REGISTER_EXT_SENS_DATA_09     ((MPU9250_REGISTER)0x52)
#define MPU9250_REGISTER_EXT_SENS_DATA_10     ((MPU9250_REGISTER)0x53)
#define MPU9250_REGISTER_EXT_SENS_DATA_11     ((MPU9250_REGISTER)0x54)
#define MPU9250_REGISTER_EXT_SENS_DATA_12     ((MPU9250_REGISTER)0x55)
#define MPU9250_REGISTER_EXT_SENS_DATA_13     ((MPU9250_REGISTER)0x56)
#define MPU9250_REGISTER_EXT_SENS_DATA_14     ((MPU9250_REGISTER)0x57)
#define MPU9250_REGISTER_EXT_SENS_DATA_15     ((MPU9250_REGISTER)0x58)
#define MPU9250_REGISTER_EXT_SENS_DATA_16     ((MPU9250_REGISTER)0x59)
#define MPU9250_REGISTER_EXT_SENS_DATA_17     ((MPU9250_REGISTER)0x5A)
#define MPU9250_REGISTER_EXT_SENS_DATA_18     ((MPU9250_REGISTER)0x5B)
#define MPU9250_REGISTER_EXT_SENS_DATA_19     ((MPU9250_REGISTER)0x5C)
#define MPU9250_REGISTER_EXT_SENS_DATA_20     ((MPU9250_REGISTER)0x5D)
#define MPU9250_REGISTER_EXT_SENS_DATA_21     ((MPU9250_REGISTER)0x5E)
#define MPU9250_REGISTER_EXT_SENS_DATA_22     ((MPU9250_REGISTER)0x5F)
#define MPU9250_REGISTER_EXT_SENS_DATA_23     ((MPU9250_REGISTER)0x60)
#define MPU9250_REGISTER_I2C_SLV0_DO          ((MPU9250_REGISTER)0x63)
#define MPU9250_REGISTER_I2C_SLV1_DO          ((MPU9250_REGISTER)0x64)
#define MPU9250_REGISTER_I2C_SLV2_DO          ((MPU9250_REGISTER)0x65)
#define MPU9250_REGISTER_I2C_SLV3_DO          ((MPU9250_REGISTER)0x66)
#define MPU9250_REGISTER_I2C_MST_DELAY_CTRL   ((MPU9250_REGISTER)0x67)
#define MPU9250_REGISTER_SIGNAL_PATH_RESET    ((MPU9250_REGISTER)0x68)
#define MPU9250_REGISTER_MOT_DETECT_CTRL      ((MPU9250_REGISTER)0x69)
#define MPU9250_REGISTER_USER_CTRL            ((MPU9250_REGISTER)0x6A)
#define MPU9250_REGISTER_PWR_MGMT_1           ((MPU9250_REGISTER)0x6B)
#define MPU9250_REGISTER_PWR_MGMT_2           ((MPU9250_REGISTER)0x6C)
#define MPU9250_REGISTER_FIFO_COUNTH          ((MPU9250_REGISTER)0x72)
#define MPU9250_REGISTER_FIFO_COUNTL          ((MPU9250_REGISTER)0x73)
#define MPU9250_REGISTER_FIFO_R_W             ((MPU9250_REGISTER)0x74)
#define MPU9250_REGISTER_WHO_AM_I             ((MPU9250_REGISTER)0x75)
#define MPU9250_REGISTER_XA_OFFSET_H          ((MPU9250_REGISTER)0x77)
#define MPU9250_REGISTER_XA_OFFSET_L          ((MPU9250_REGISTER)0x78)
#define MPU9250_REGISTER_YA_OFFSET_H          ((MPU9250_REGISTER)0x7A)
#define MPU9250_REGISTER_YA_OFFSET_L          ((MPU9250_REGISTER)0x7B)
#define MPU9250_REGISTER_ZA_OFFSET_H          ((MPU9250_REGISTER)0x7D)
#define MPU9250_REGISTER_ZA_OFFSET_L          ((MPU9250_REGISTER)0x7E)

#define AK8963_ADDRESS                        0x0C

#define AK8963_REGISTER_WIA                   ((AK8963_REGISTER)0x00)
#define AK8963_REGISTER_INFO                  ((AK8963_REGISTER)0x01)
#define AK8963_REGISTER_ST1                   ((AK8963_REGISTER)0x02)
#define AK8963_REGISTER_HXL                   ((AK8963_REGISTER)0x03)
#define AK8963_REGISTER_HXH                   ((AK8963_REGISTER)0x04)
#define AK8963_REGISTER_HYL                   ((AK8963_REGISTER)0x05)
#define AK8963_REGISTER_HYH                   ((AK8963_REGISTER)0x06)
#define AK8963_REGISTER_HZL                   ((AK8963_REGISTER)0x07)
#define AK8963_REGISTER_HZH                   ((AK8963_REGISTER)0x08)
#define AK8963_REGISTER_ST2                   ((AK8963_REGISTER)0x09)
#define AK8963_REGISTER_CNTL1                 ((AK8963_REGISTER)0x0A)
#define AK8963_REGISTER_CNTL2                 ((AK8963_REGISTER)0x0B)
#define AK8963_REGISTER_ASTC                  ((AK8963_REGISTER)0x0C)
#define AK8963_REGISTER_TS1                   ((AK8963_REGISTER)0x0D)
#define AK8963_REGISTER_TS2                   ((AK8963_REGISTER)0x0E)
#define AK8963_REGISTER_I2CDIS                ((AK8963_REGISTER)0x0F)
#define AK8963_REGISTER_ASAX                  ((AK8963_REGISTER)0x10)
#define AK8963_REGISTER_ASAY                  ((AK8963_REGISTER)0x11)
#define AK8963_REGISTER_ASAZ                  ((AK8963_REGISTER)0x12)


union MPU9250_transaction
{
  struct MPU9250_transaction_field
  {
    struct MPU9250_transaction_field_command
    {
      MPU9250_REGISTER  AD:7;
      uint8_t           R_W:1; /* 0=write, 1=read */
    }       address;
    union MPU9250_transaction_field_data
    {
      struct MPU9250_SELF_TEST_X_GYRO
      {
        uint8_t         xg_st_data:8;
      }               SELF_TEST_X_GYRO;
      struct MPU9250_SELF_TEST_Y_GYRO
      {
        uint8_t         yg_st_data:8;
      }               SELF_TEST_Y_GYRO;
      struct MPU9250_SELF_TEST_Z_GYRO
      {
        uint8_t         zg_st_data:8;
      }               SELF_TEST_Z_GYRO;
      struct MPU9250_SELF_TEST_X_ACCEL
      {
        uint8_t         XA_ST_DATA:8;
      }               SELF_TEST_X_ACCEL;
      struct MPU9250_SELF_TEST_Y_ACCEL
      {
        uint8_t         YA_ST_DATA:8;
      }               SELF_TEST_Y_ACCEL;
      struct MPU9250_SELF_TEST_Z_ACCEL
      {
        uint8_t         ZA_ST_DATA:8;
      }               SELF_TEST_Z_ACCEL;
      struct MPU9250_XG_OFFSET_H
      {
          uint8_t       X_OFFS_USR:8;
      }               XG_OFFSET_H;
      struct MPU9250_XG_OFFSET_L
      {
        uint8_t         X_OFFS_USR:8;
      }               XG_OFFSET_L;
      struct MPU9250_YG_OFFSET_H
      {
        uint8_t         Y_OFFS_USR:8;
      }               YG_OFFSET_H;
      struct MPU9250_YG_OFFSET_L
      {
        uint8_t         Y_OFFS_USR:8;
      }               YG_OFFSET_L;
      struct MPU9250_ZG_OFFSET_H
      {
        uint8_t         Z_OFFS_USR:8;
      }               ZG_OFFSET_H;
      struct MPU9250_ZG_OFFSET_L
      {
        uint8_t         Z_OFFS_USR:8;
      }               ZG_OFFSET_L;
      struct MPU9250_SMPLRT_DIV
      {
        uint8_t         SMPLRT_DIV:8;
      }               SMPLRT_DIV;
      struct MPU9250_CONFIG
      {
        uint8_t         DLPF_CFG:3;
        uint8_t         EXT_SYNC_SET:3;
        uint8_t         FIFO_MODE:1;
        uint8_t         reserved:1;
      }               CONFIG;
      struct MPU9250_GYRO_CONFIG
      {
        uint8_t         FCHOICE_B:2;
        uint8_t         reserved:1;
        uint8_t         GYRO_FS_SEL:2;
        uint8_t         ZGYRO_Cten:1;
        uint8_t         YGYRO_Cten:1;
        uint8_t         XGYRO_Cten:1;
      }               GYRO_CONFIG;
      struct MPU9250_ACCEL_CONFIG
      {
        uint8_t         reserved:3;
        uint8_t         ACCEL_FS_SEL:2;
        uint8_t         az_st_en:1;
        uint8_t         ay_st_en:1;
        uint8_t         ax_st_en:1;
      }               ACCEL_CONFIG;
      struct MPU9250_ACCEL_CONFIG_2
      {
        uint8_t         A_DLPF_CFG:2;
        uint8_t         ACCEL_FCHOICE_B:2;
        uint8_t         reserved:4;
      }               ACCEL_CONFIG_2;
      struct MPU9250_LP_ACCEL_ODR
      {
        uint8_t         Lposc_clksel:4;
        uint8_t         reserved:4;
      }               LP_ACCEL_ODR;
      struct MPU9250_WOM_THR
      {
        uint8_t         WOM_Threshold:8;
      }               WOM_THR;
      struct MPU9250_FIFO_EN
      {
        uint8_t         SLV0:1;
        uint8_t         SLV1:1;
        uint8_t         SLV2:1;
        uint8_t         ACCEL:1;
        uint8_t         GYRO_ZO_UT:1;
        uint8_t         GYRO_YO_UT:1;
        uint8_t         GYRO_XO_UT:1;
        uint8_t         TEMP_FIFO_EN:1;
      }               FIFO_EN;
      struct MPU9250_I2C_MST_CTRL
      {
        uint8_t         I2C_MST_CLK:4;
        uint8_t         I2C_MST_P_NSR:1;
        uint8_t         SLV_3_FIFO_EN:1;
        uint8_t         WAIT_FOR_ES:1;
        uint8_t         MULT_MST_EN:1;
      }               I2C_MST_CTRL;
      struct MPU9250_I2C_SLV0_ADDR
      {
        uint8_t         I2C_ID_0:7;
        uint8_t         I2C_SLV0_RNW:1;
      }               I2C_SLV0_ADDR;
      struct MPU9250_I2C_SLV0_REG
      {
        uint8_t         I2C_SLV0_REG:8;
      }               I2C_SLV0_REG;
      struct MPU9250_I2C_SLV0_CTRL
      {
        uint8_t         I2C_SLV0_LENG:4;
        uint8_t         I2C_SLV0_GRP:1;
        uint8_t         I2C_SLV0_REG_DIS:1;
        uint8_t         I2C_SLV0_BYTE_SW:1;
        uint8_t         I2C_SLV0_EN:1;
      }               I2C_SLV0_CTRL;
      struct MPU9250_I2C_SLV1_ADDR
      {
        uint8_t         I2C_ID_1:7;
        uint8_t         I2C_SLV1_RNW:1;
      }               I2C_SLV1_ADDR;
      struct MPU9250_I2C_SLV1_REG
      {
        uint8_t         I2C_SLV1_REG:8;
      }               I2C_SLV1_REG;
      struct MPU9250_I2C_SLV1_CTRL
      {
        uint8_t         I2C_SLV1_LENG:4;
        uint8_t         I2C_SLV1_GRP:1;
        uint8_t         I2C_SLV1_REG_DIS:1;
        uint8_t         I2C_SLV1_BYTE_SW:1;
        uint8_t         I2C_SLV1_EN:1;
      }               I2C_SLV1_CTRL;
      struct MPU9250_I2C_SLV2_ADDR
      {
        uint8_t         I2C_ID_2:7;
        uint8_t         I2C_SLV2_RNW:1;
      }               I2C_SLV2_ADDR;
      struct MPU9250_I2C_SLV2_REG
      {
        uint8_t         I2C_SLV2_REG:8;
      }               I2C_SLV2_REG;
      struct MPU9250_I2C_SLV2_CTRL
      {
        uint8_t         I2C_SLV2_LENG:4;
        uint8_t         I2C_SLV2_GRP:1;
        uint8_t         I2C_SLV2_REG_DIS:1;
        uint8_t         I2C_SLV2_BYTE_SW:1;
        uint8_t         I2C_SLV2_EN:1;
      }               I2C_SLV2_CTRL;
      struct MPU9250_I2C_SLV3_ADDR
      {
        uint8_t         I2C_ID_3:7;
        uint8_t         I2C_SLV3_RNW:1;
      }               I2C_SLV3_ADDR;
      struct MPU9250_I2C_SLV3_REG
      {
        uint8_t         I2C_SLV3_REG:8;
      }               I2C_SLV3_REG;
      struct MPU9250_I2C_SLV3_CTRL
      {
        uint8_t         I2C_SLV3_LENG:4;
        uint8_t         I2C_SLV3_GRP:1;
        uint8_t         I2C_SLV3_REG_DIS:1;
        uint8_t         I2C_SLV3_BYTE_SW:1;
        uint8_t         I2C_SLV3_EN:1;
      }               I2C_SLV3_CTRL;
      struct MPU9250_I2C_SLV4_ADDR
      {
        uint8_t         I2C_ID_4:7;
        uint8_t         I2C_SLV4_RNW:1;
      }               I2C_SLV4_ADDR;
      struct MPU9250_I2C_SLV4_REG
      {
        uint8_t         I2C_SLV4_REG:8;
      }               I2C_SLV4_REG;
      struct MPU9250_I2C_SLV4_DO
      {
        uint8_t         I2C_SLV4_DO:8;
      }               I2C_SLV4_DO;
      struct MPU9250_I2C_SLV4_CTRL
      {
        uint8_t         I2C_MST_DLY:5;
        uint8_t         I2C_SLV4_REG_DIS:1;
        uint8_t         SLV4_DONE_INT_EN:1;
        uint8_t         I2C_SLV4_EN:1;
      }               I2C_SLV4_CTRL;
      struct MPU9250_I2C_SLV4_DI
      {
        uint8_t         I2C_SLV4_DI:8;
      }               I2C_SLV4_DI;
      struct MPU9250_I2C_MST_STATUS
      {
        uint8_t         I2C_SLV0_NACK:1;
        uint8_t         I2C_SLV1_NACK:1;
        uint8_t         I2C_SLV2_NACK:1;
        uint8_t         I2C_SLV3_NACK:1;
        uint8_t         I2C_SLV4_NACK:1;
        uint8_t         I2C_LOST_ARB:1;
        uint8_t         I2C_SLV4_DONE:1;
        uint8_t         PASS_THROUGH:1;
      }               I2C_MST_STATUS;
      struct MPU9250_INT_PIN_CFG
      {
        uint8_t         reserved:1;
        uint8_t         BYPASS_EN:1;
        uint8_t         FSYNC_INT_MODE_EN:1;
        uint8_t         ACTL_FSYNC:1;
        uint8_t         INT_ANYRD_2CLEAR:1;
        uint8_t         LATCH_INT_EN:1;
        uint8_t         OPEN:1;
        uint8_t         ACTL:1;
      }               INT_PIN_CFG;
      struct MPU9250_INT_ENABLE
      {
        uint8_t         RAW_RDY_EN:1;
        uint8_t         reserved0:2;
        uint8_t         FSYNC_INT_EN:1;
        uint8_t         FIFO_OFLOW_EN:1;
        uint8_t         reserved1:1;
        uint8_t         WOM_EN:1;
        uint8_t         reserved2:1;
      }               INT_ENABLE;
      struct MPU9250_INT_STATUS
      {
        uint8_t         RAW_DATA_RDY_INT:1;
        uint8_t         reserved0:2;
        uint8_t         FSYNC_INT:1;
        uint8_t         FIFO_OFLOW_INT:1;
        uint8_t         reserved1:1;
        uint8_t         WOM_INT:1;
        uint8_t         reserved2:1;
      }               INT_STATUS;
      struct MPU9250_ACCEL_XOUT_H
      {
        uint8_t         ACCEL_XOUT_H:8;
      }               ACCEL_XOUT_H;
      struct MPU9250_ACCEL_XOUT_L
      {
        uint8_t         ACCEL_XOUT_L:8;
      }               ACCEL_XOUT_L;
      struct MPU9250_ACCEL_YOUT_H
      {
        uint8_t         ACCEL_YOUT_H:8;
      }               ACCEL_YOUT_H;
      struct MPU9250_ACCEL_YOUT_L
      {
        uint8_t         ACCEL_YOUT_L:8;
      }               ACCEL_YOUT_L;
      struct MPU9250_ACCEL_ZOUT_H
      {
        uint8_t         ACCEL_ZOUT_H:8;
      }               ACCEL_ZOUT_H;
      struct MPU9250_ACCEL_ZOUT_L
      {
        uint8_t         ACCEL_ZOUT_L:8;
      }               ACCEL_ZOUT_L;
      struct MPU9250_TEMP_OUT_H
      {
        uint8_t         TEMP_OUT_H:8;
      }               TEMP_OUT_H;
      struct MPU9250_TEMP_OUT_L
      {
        uint8_t         TEMP_OUT_L:8;
      }               TEMP_OUT_L;
      struct MPU9250_GYRO_XOUT_H
      {
        uint8_t         GYRO_XOUT_H:8;
      }               GYRO_XOUT_H;
      struct MPU9250_GYRO_XOUT_L
      {
        uint8_t         GYRO_XOUT_L:8;
      }               GYRO_XOUT_L;
      struct MPU9250_GYRO_YOUT_H
      {
        uint8_t         GYRO_YOUT_H:8;
      }               GYRO_YOUT_H;
      struct MPU9250_GYRO_YOUT_L
      {
        uint8_t         GYRO_YOUT_L:8;
      }               GYRO_YOUT_L;
      struct MPU9250_GYRO_ZOUT_H
      {
        uint8_t         GYRO_ZOUT_H:8;
      }               GYRO_ZOUT_H;
      struct MPU9250_GYRO_ZOUT_L
      {
        uint8_t         GYRO_ZOUT_L:8;
      }               GYRO_ZOUT_L;
      struct MPU9250_EXT_SENS_DATA_00
      {
        uint8_t         EXT_SENS_DATA_00:8;
      }               EXT_SENS_DATA_00;
      struct MPU9250_EXT_SENS_DATA_01
      {
        uint8_t         EXT_SENS_DATA_01:8;
      }               EXT_SENS_DATA_01;
      struct MPU9250_EXT_SENS_DATA_02
      {
        uint8_t         EXT_SENS_DATA_02:8;
      }               EXT_SENS_DATA_02;
      struct MPU9250_EXT_SENS_DATA_03
      {
        uint8_t         EXT_SENS_DATA_03:8;
      }               EXT_SENS_DATA_03;
      struct MPU9250_EXT_SENS_DATA_04
      {
        uint8_t         EXT_SENS_DATA_04:8;
      }               EXT_SENS_DATA_04;
      struct MPU9250_EXT_SENS_DATA_05
      {
        uint8_t         EXT_SENS_DATA_05:8;
      }               EXT_SENS_DATA_05;
      struct MPU9250_EXT_SENS_DATA_06
      {
        uint8_t         EXT_SENS_DATA_06:8;
      }               EXT_SENS_DATA_06;
      struct MPU9250_EXT_SENS_DATA_07
      {
        uint8_t         EXT_SENS_DATA_07:8;
      }               EXT_SENS_DATA_07;
      struct MPU9250_EXT_SENS_DATA_08
      {
        uint8_t         EXT_SENS_DATA_08:8;
      }               EXT_SENS_DATA_08;
      struct MPU9250_EXT_SENS_DATA_09
      {
        uint8_t         EXT_SENS_DATA_09:8;
      }               EXT_SENS_DATA_09;
      struct MPU9250_EXT_SENS_DATA_10
      {
        uint8_t         EXT_SENS_DATA_10:8;
      }               EXT_SENS_DATA_10;
      struct MPU9250_EXT_SENS_DATA_11
      {
        uint8_t         EXT_SENS_DATA_11:8;
      }               EXT_SENS_DATA_11;
      struct MPU9250_EXT_SENS_DATA_12
      {
        uint8_t         EXT_SENS_DATA_12:8;
      }               EXT_SENS_DATA_12;
      struct MPU9250_EXT_SENS_DATA_13
      {
        uint8_t         EXT_SENS_DATA_13:8;
      }               EXT_SENS_DATA_13;
      struct MPU9250_EXT_SENS_DATA_14
      {
        uint8_t         EXT_SENS_DATA_14:8;
      }               EXT_SENS_DATA_14;
      struct MPU9250_EXT_SENS_DATA_15
      {
        uint8_t         EXT_SENS_DATA_15:8;
      }               EXT_SENS_DATA_15;
      struct MPU9250_EXT_SENS_DATA_16
      {
        uint8_t         EXT_SENS_DATA_16:8;
      }               EXT_SENS_DATA_16;
      struct MPU9250_EXT_SENS_DATA_17
      {
        uint8_t         EXT_SENS_DATA_17:8;
      }               EXT_SENS_DATA_17;
      struct MPU9250_EXT_SENS_DATA_18
      {
        uint8_t         EXT_SENS_DATA_18:8;
      }               EXT_SENS_DATA_18;
      struct MPU9250_EXT_SENS_DATA_19
      {
        uint8_t         EXT_SENS_DATA_19:8;
      }               EXT_SENS_DATA_19;
      struct MPU9250_EXT_SENS_DATA_20
      {
        uint8_t         EXT_SENS_DATA_20:8;
      }               EXT_SENS_DATA_20;
      struct MPU9250_EXT_SENS_DATA_21
      {
        uint8_t         EXT_SENS_DATA_21:8;
      }               EXT_SENS_DATA_21;
      struct MPU9250_EXT_SENS_DATA_22
      {
        uint8_t         EXT_SENS_DATA_22:8;
      }               EXT_SENS_DATA_22;
      struct MPU9250_EXT_SENS_DATA_23
      {
        uint8_t         EXT_SENS_DATA_23:8;
      }               EXT_SENS_DATA_23;
      struct MPU9250_I2C_SLV0_DO
      {
        uint8_t         I2C_SLV0_DO:8;
      }               I2C_SLV0_DO;
      struct MPU9250_I2C_SLV1_DO
      {
        uint8_t         I2C_SLV1_DO:8;
      }               I2C_SLV1_DO;
      struct MPU9250_I2C_SLV2_DO
      {
        uint8_t         I2C_SLV2_DO:8;
      }               I2C_SLV2_DO;
      struct MPU9250_I2C_SLV3_DO
      {
        uint8_t       I2C_SLV3_DO:8;
      }               I2C_SLV3_DO;
      struct MPU9250_I2C_MST_DELAY_CTRL
      {
        uint8_t         I2C_SLV0_DLY_EN:1;
        uint8_t         I2C_SLV1_DLY_EN:1;
        uint8_t         I2C_SLV2_DLY_EN:1;
        uint8_t         I2C_SLV3_DLY_EN:1;
        uint8_t         I2C_SLV4_DLY_EN:1;
        uint8_t         reserved:2;
        uint8_t         DELAY_ES_SHADOW:1;
      }               I2C_MST_DELAY_CTRL;
      struct MPU9250_SIGNAL_PATH_RESET
      {
        uint8_t         TEMP_RST:1;
        uint8_t         ACCCEL_RST:1;
        uint8_t         GYRO_RST:1;
        uint8_t         reserved:5;
      }               SIGNAL_PATH_RESET;
      struct MPU9250_MOT_DETECT_CTRL
      {
        uint8_t         reserved:6;
        uint8_t         ACCEL_INTEL_MODE:1;
        uint8_t         ACCEL_INTEL_EN:1;
      }               MOT_DETECT_CTRL;
      struct MPU9250_USER_CTRL
      {
        uint8_t         SIG_COND_RST:1;
        uint8_t         I2C_MST_RST:1;
        uint8_t         FIFO_RST:1;
        uint8_t         reserved0:1;
        uint8_t         I2C_IF_DIS:1;
        uint8_t         I2C_MST_EN:1;
        uint8_t         FIFO_EN:1;
        uint8_t         reserved1:1;
      }               USER_CTRL;
      struct MPU9250_PWR_MGMT_1
      {
        uint8_t         CLKSEL:3;
        uint8_t         PD_PTAT:1;
        uint8_t         GYRO_STANDBY:1;
        uint8_t         CYCLE:1;
        uint8_t         SLEEP:1;
        uint8_t         H_RESET:1;
      }               PWR_MGMT_1;
      struct MPU9250_PWR_MGMT_2
      {
        uint8_t         DIS_ZG:1;
        uint8_t         DIS_YG:1;
        uint8_t         DIS_XG:1;
        uint8_t         DIS_ZA:1;
        uint8_t         DIS_YA:1;
        uint8_t         DIS_XA:1;
        uint8_t         reserved:2;
      }               PWR_MGMT_2;
      struct MPU9250_FIFO_COUNTH
      {
        uint8_t         FIFO_CNT:5;
        uint8_t         reserved:3;
      }               FIFO_COUNTH;
      struct MPU9250_FIFO_COUNTL
      {
        uint8_t         FIFO_CNT:8;
      }               FIFO_COUNTL;
      struct MPU9250_FIFO_R_W
      {
        uint8_t         D:8;
      }               FIFO_R_W;
      struct MPU9250_WHO_AM_I
      {
        uint8_t         WHOAMI:8;
      }               WHO_AM_I;
      struct MPU9250_XA_OFFSET_H
      {
        uint8_t         XA_OFFS:8;
      }               XA_OFFSET_H;
      struct MPU9250_XA_OFFSET_L
      {
        uint8_t         reserved:1;
        uint8_t         XA_OFFS:7;
      }               XA_OFFSET_L;
      struct MPU9250_YA_OFFSET_H
      {
        uint8_t         YA_OFFS:8;
      }               YA_OFFSET_H;
      struct MPU9250_YA_OFFSET_L
      {
        uint8_t         reserved:1;
        uint8_t         YA_OFFS:7;
      }               YA_OFFSET_L;
      struct MPU9250_ZA_OFFSET_H
      {
        uint8_t         ZA_OFFS:8;
      }               ZA_OFFSET_H;
      struct MPU9250_ZA_OFFSET_L
      {
        uint8_t         reserved:1;
        uint8_t         ZA_OFFS:7;
      }               ZA_OFFSET_L;

      struct AK8963_WIA
      {
        uint8_t         WIA:8;
      }               WIA;
      struct AK8963_INFO
      {
        uint8_t         INFO:8;
      }               INFO;
      struct AK8963_ST1
      {
        uint8_t         DRDY:1;
        uint8_t         reserved:7;
      }               ST1;
      struct AK8963_HXL
      {
        uint8_t         HXL:8;
      }               HXL;
      struct AK8963_HXH
      {
        uint8_t         HXH:8;
      }               HXH;
      struct AK8963_HYL
      {
        uint8_t         HYL:8;
      }               HYL;
      struct AK8963_HYH
      {
        uint8_t         HYH:8;
      }               HYH;
      struct AK8963_HZL
      {
        uint8_t         HZL:8;
      }               HZL;
      struct AK8963_HZH
      {
        uint8_t         HZH:8;
      }               HZH;
      struct AK8963_ST2
      {
        uint8_t         reserved0:3;
        uint8_t         HOFL:1;
        uint8_t         BITM:1;
        uint8_t         reserved1:3;
      }               ST2;
      struct AK8963_CNTL1
      {
        uint8_t         MODE:3;
        uint8_t         BIT:1;
        uint8_t         reserved:3;
      }               CNTL1;
      struct AK8963_CNTL2
      {
        uint8_t         SRST:1;
        uint8_t         reserved:7;
      }               CNTL2;
      struct AK8963_ASTC
      {
        uint8_t         reserved0:6;
        uint8_t         SELF:1;
        uint8_t         reserved1:1;
      }               ASTC;
      struct AK8963_TS1
      {
        uint8_t         TS1:8;
      }               TS1;
      struct AK8963_TS2
      {
        uint8_t         TS2:8;
      }               TS2;
      struct AK8963_I2CDIS
      {
        uint8_t         I2CDIS:8;
      }               I2CDIS;
      struct AK8963_ASAX
      {
        uint8_t         COEFX:8;
      }               ASAX;
      struct AK8963_ASAY
      {
        uint8_t         COEFY:8;
      }               ASAY;
      struct AK8963_ASAZ
      {
        uint8_t         COEFZ:8;
      }               ASAZ;
    }       data;
  }       field;
  uint8_t value[1+1];
};


#endif /* MPU9250_H_ */
