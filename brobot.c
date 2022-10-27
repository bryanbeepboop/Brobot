/**************************************************
* CMPEN 473, Spring 2022, Penn State University
* 
* Date 2/08/2022
* By Bryan Hong (bsh5290)
* 
***************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include "import_registers.h"
#include "cm.h"
#include "gpio.h"
#include "uart.h"
#include "spi.h"
#include "bsc.h"
#include "pwm.h"
#include "enable_pwm_clock.h"
#include "io_peripherals.h"
#include "wait_period.h"
#include "FIFO.h"
#include "MPU6050.h"
#include "MPU9250.h"
#include "wait_key.h"
#include "raspicam_wrapper.h"

#define PWM_RANGE 100

#define APB_CLOCK 250000000

#define ROUND_DIVISION(x,y) (((x) + (y)/2)/(y))

int key = 0; //to communicate with threads
int current_speed = 0;
char gear = 'S';
int smooth;
int p = 0;
char gear_mem;
int mode = 1; //initial mode is 1 (manual drive)
int mode2_status = 0;
int mode2w = 0;

char queue[10] = ""; //for 13 scheduling
int numQueue = 0;

float avg_speed;
int avg_count;
int usec_count;

int fw;
FILE *fp;
FILE *raw_f;
FILE *p_f;
int record = 0;

float ac_x_max, ac_x_min;
float ac_y_max, ac_y_min;
float ac_z_max, ac_z_min;

float gy_x_max, gy_x_min;
float gy_y_max, gy_y_min;
float gy_z_max, gy_z_min;

unsigned char* image_p;
int g_image_width;
int g_image_height;
int box_size = 10000;

int x = 0;//x:-1 point on left, x:1 point on right
int y = 0;//y:-1 point on back, y:1 point on forward

pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

//sensor calb
union uint16_to_2uint8
{
  struct uint16_to_2uint8_field
  {
    uint8_t   L;  /* Little Endian byte order means that the least significant byte goes in the lowest address */
    uint8_t   H;
  }         field;
  uint16_t  unsigned_value;
  int16_t   signed_value;
};

struct calibration_data
{
  float scale;
  float offset_x;
  float offset_y;
  float offset_z;
};

void read_MPU6050_registers(                          /* read a register */
    uint8_t                         I2C_address,      /* the address of the I2C device to talk to */
    MPU6050_REGISTER                register_address, /* the address to read from */
    uint8_t *                       read_data,        /* the data read from the SPI device */
    size_t                          data_length,      /* the length of data to send/receive */
    volatile struct bsc_register *  bsc )             /* the BSC address */
{
  bsc->S.field.DONE    = 1;
  bsc->A.field.ADDR    = I2C_address;
  bsc->C.field.READ    = 0;
  bsc->DLEN.field.DLEN = 1;
  bsc->FIFO.value      = register_address;
  bsc->C.field.ST      = 1;
  while (bsc->S.field.DONE == 0)
  {
    usleep( 100 );
  }
  bsc->S.field.DONE    = 1;
  bsc->A.field.ADDR    = I2C_address;
  bsc->C.field.READ    = 1;
  bsc->DLEN.field.DLEN = data_length;
  bsc->C.field.ST      = 1;
  while (bsc->S.field.DONE == 0)
  {
    usleep( 100 );
  }

  while (data_length > 0)
  {
    *read_data = bsc->FIFO.field.DATA;

    read_data++;
    data_length--;
  }

  return;
}

union MPU6050_transaction_field_data read_MPU6050_register( /* read a register, returning the read value */
    uint8_t                         I2C_address,            /* the address of the I2C device to talk to */
    MPU6050_REGISTER                register_address,       /* the address to read from */
    volatile struct bsc_register *  bsc )                   /* the BSC address */
{
  union MPU6050_transaction transaction;

  read_MPU6050_registers( I2C_address, register_address, &(transaction.value[1]), 1, bsc );

  return transaction.field.data;
}

void write_MPU6050_register(                                /* write a register */
    uint8_t                               I2C_address,      /* the address of the I2C device to talk to */
    MPU6050_REGISTER                      register_address, /* the address to read from */
    union MPU6050_transaction_field_data  value,            /* the value to write */
    volatile struct bsc_register *        bsc )             /* the BSC address */
{
  union MPU6050_transaction transaction;

  transaction.field.data = value;
  bsc->S.field.DONE    = 1;
  bsc->A.field.ADDR    = I2C_address;
  bsc->C.field.READ    = 0;
  bsc->DLEN.field.DLEN = 2;
  bsc->FIFO.value      = register_address;
  bsc->FIFO.value      = transaction.value[1];
  bsc->C.field.ST      = 1;
  while (bsc->S.field.DONE == 0)
  {
    usleep( 100 );
  }

  return;
}

void calibrate_accelerometer_and_gyroscope(
    struct calibration_data *     calibration_accelerometer,
    struct calibration_data *     calibration_gyroscope,
    volatile struct bsc_register *bsc )
{
  union MPU6050_transaction_field_data  transaction;
  uint8_t                               data_block_fifo_count[2];
  union uint16_to_2uint8                reconstructor;
  uint16_t                              ii;
  uint16_t                              packet_count;
  int32_t                               gyro_bias_x;
  int32_t                               gyro_bias_y;
  int32_t                               gyro_bias_z;
  int32_t                               accel_bias_x;
  int32_t                               accel_bias_y;
  int32_t                               accel_bias_z;
  uint8_t                               data_block_fifo_packet[12];
  union uint16_to_2uint8                reconstructor_accel_x;
  union uint16_to_2uint8                reconstructor_accel_y;
  union uint16_to_2uint8                reconstructor_accel_z;
  union uint16_to_2uint8                reconstructor_gyro_x;
  union uint16_to_2uint8                reconstructor_gyro_y;
  union uint16_to_2uint8                reconstructor_gyro_z;

  // reset device
  transaction.PWR_MGMT_1.CLKSEL       = 0;
  transaction.PWR_MGMT_1.TEMP_DIS     = 0;
  transaction.PWR_MGMT_1.reserved     = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.DEVICE_RESET = 1;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc );
  usleep( 100000 );

  // get stable time source; auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator
  transaction.PWR_MGMT_1.CLKSEL       = 1;
  transaction.PWR_MGMT_1.TEMP_DIS     = 0;
  transaction.PWR_MGMT_1.reserved     = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.DEVICE_RESET = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc );
  transaction.PWR_MGMT_2.STBY_ZG      = 0;
  transaction.PWR_MGMT_2.STBY_YG      = 0;
  transaction.PWR_MGMT_2.STBY_XG      = 0;
  transaction.PWR_MGMT_2.STBY_ZA      = 0;
  transaction.PWR_MGMT_2.STBY_YA      = 0;
  transaction.PWR_MGMT_2.STBY_XA      = 0;
  transaction.PWR_MGMT_2.LP_WAKE_CTRL = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_2, transaction, bsc );
  usleep( 200000 );

  // configure device for bias calculation
  transaction.INT_ENABLE.DATA_RDY_EN    = 0; // disable all interrupts
  transaction.INT_ENABLE.reserved0      = 0;
  transaction.INT_ENABLE.I2C_MST_INT_EN = 0;
  transaction.INT_ENABLE.FIFO_OFLOW_EN  = 0;
  transaction.INT_ENABLE.reserved1      = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_INT_ENABLE, transaction, bsc );
  transaction.FIFO_EN.SLV0_FIFO_EN  = 0; // disable FIFO
  transaction.FIFO_EN.SLV1_FIFO_EN  = 0;
  transaction.FIFO_EN.SLV2_FIFO_EN  = 0;
  transaction.FIFO_EN.ACCEL_FIFO_EN = 0;
  transaction.FIFO_EN.ZG_FIFO_EN    = 0;
  transaction.FIFO_EN.YG_FIFO_EN    = 0;
  transaction.FIFO_EN.XG_FIFO_EN    = 0;
  transaction.FIFO_EN.TEMP_FIFO_EN  = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_FIFO_EN, transaction, bsc );
  transaction.PWR_MGMT_1.CLKSEL       = 0;  // turn on internal clock source
  transaction.PWR_MGMT_1.TEMP_DIS     = 0;
  transaction.PWR_MGMT_1.reserved     = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.DEVICE_RESET = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc );
  transaction.I2C_MST_CTRL.I2C_MST_CLK   = 0; // disable I2C master
  transaction.I2C_MST_CTRL.I2C_MST_P_NSR = 0;
  transaction.I2C_MST_CTRL.SLV_3_FIFO_EN = 0;
  transaction.I2C_MST_CTRL.WAIT_FOR_ES   = 0;
  transaction.I2C_MST_CTRL.MULT_MST_EN   = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_I2C_MST_CTRL, transaction, bsc );
  transaction.USER_CTRL.SIG_COND_RESET  = 0; // disable FIFO and I2C master modes
  transaction.USER_CTRL.I2C_MST_RESET   = 0;
  transaction.USER_CTRL.FIFO_RESET      = 0;
  transaction.USER_CTRL.reserved0       = 0;
  transaction.USER_CTRL.I2C_IF_DIS      = 0;
  transaction.USER_CTRL.I2C_MST_EN      = 0;
  transaction.USER_CTRL.FIFO_EN         = 0;
  transaction.USER_CTRL.reserved1       = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_USER_CTRL, transaction, bsc );
  transaction.USER_CTRL.SIG_COND_RESET  = 0; // reset FIFO and DMP
  transaction.USER_CTRL.I2C_MST_RESET   = 0;
  transaction.USER_CTRL.FIFO_RESET      = 1;
  transaction.USER_CTRL.reserved0       = 0;
  transaction.USER_CTRL.I2C_IF_DIS      = 0;
  transaction.USER_CTRL.I2C_MST_EN      = 0;
  transaction.USER_CTRL.FIFO_EN         = 0;
  transaction.USER_CTRL.reserved1       = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_USER_CTRL, transaction, bsc );
  usleep( 15000 );

  // configure MPU6050 gyro and accelerometer for bias calculation
  transaction.CONFIG.DLPF_CFG     = 1;  // set low-pass filter to 188Hz
  transaction.CONFIG.EXT_SYNC_SET = 0;
  transaction.CONFIG.reserved     = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_CONFIG, transaction, bsc );
  transaction.SMPLRT_DIV.SMPLRT_DIV = 0;  // set sample rate to 1kHz
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_SMPLRT_DIV, transaction, bsc );
  transaction.GYRO_CONFIG.reserved0 = 0;  // set gyro full-scale to 250dps, maximum sensitivity
  transaction.GYRO_CONFIG.FS_SEL    = 0;
  transaction.GYRO_CONFIG.reserved1 = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_GYRO_CONFIG, transaction, bsc );
  transaction.ACCEL_CONFIG.reserved     = 0; // set accelerometer full-scale to 2g, maximum sensitivity
  transaction.ACCEL_CONFIG.ACCEL_FS_SEL = 0;
  transaction.ACCEL_CONFIG.ZA_ST        = 0;
  transaction.ACCEL_CONFIG.YA_ST        = 0;
  transaction.ACCEL_CONFIG.XA_ST        = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_ACCEL_CONFIG, transaction, bsc );

  calibration_accelerometer->scale = 2.0/32768.0;  // measurement scale/signed numeric range
  calibration_accelerometer->offset_x = 0;
  calibration_accelerometer->offset_y = 0;
  calibration_accelerometer->offset_z = 0;

  calibration_gyroscope->scale = 250.0/32768.0;
  calibration_gyroscope->offset_x = 0;
  calibration_gyroscope->offset_y = 0;
  calibration_gyroscope->offset_z = 0;

  // configure FIFO to capture accelerometer and gyro data for bias calculation
  transaction.USER_CTRL.SIG_COND_RESET = 0; // enable FIFO
  transaction.USER_CTRL.I2C_MST_RESET  = 0;
  transaction.USER_CTRL.FIFO_RESET     = 0;
  transaction.USER_CTRL.reserved0    = 0;
  transaction.USER_CTRL.I2C_IF_DIS   = 0;
  transaction.USER_CTRL.I2C_MST_EN   = 0;
  transaction.USER_CTRL.FIFO_EN      = 1;
  transaction.USER_CTRL.reserved1    = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_USER_CTRL, transaction, bsc );
  transaction.FIFO_EN.SLV0_FIFO_EN  = 0;  // enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU6050)
  transaction.FIFO_EN.SLV1_FIFO_EN  = 0;
  transaction.FIFO_EN.SLV2_FIFO_EN  = 0;
  transaction.FIFO_EN.ACCEL_FIFO_EN = 1;
  transaction.FIFO_EN.ZG_FIFO_EN    = 1;
  transaction.FIFO_EN.YG_FIFO_EN    = 1;
  transaction.FIFO_EN.XG_FIFO_EN    = 1;
  transaction.FIFO_EN.TEMP_FIFO_EN  = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_FIFO_EN, transaction, bsc );
  usleep( 40000 );  // accumulate 40 samples in 40 milliseconds = 480 bytes

  // at end of sample accumulation, turn off FIFO sensor read
  transaction.FIFO_EN.SLV0_FIFO_EN  = 0;  // disable gyro and accelerometer sensors for FIFO
  transaction.FIFO_EN.SLV1_FIFO_EN  = 0;
  transaction.FIFO_EN.SLV2_FIFO_EN  = 0;
  transaction.FIFO_EN.ACCEL_FIFO_EN = 0;
  transaction.FIFO_EN.ZG_FIFO_EN    = 0;
  transaction.FIFO_EN.YG_FIFO_EN    = 0;
  transaction.FIFO_EN.XG_FIFO_EN    = 0;
  transaction.FIFO_EN.TEMP_FIFO_EN  = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_FIFO_EN, transaction, bsc );
  read_MPU6050_registers( MPU6050_ADDRESS, MPU6050_REGISTER_FIFO_COUNTH, data_block_fifo_count, sizeof(data_block_fifo_count), bsc ); // read FIFO sample count
  reconstructor.field.H = data_block_fifo_count[0];
  reconstructor.field.L = data_block_fifo_count[1];
  packet_count = reconstructor.unsigned_value / 12; // how many sets of full gyro and accelerometer data for averaging

  accel_bias_x = 0;
  accel_bias_y = 0;
  accel_bias_z = 0;
  gyro_bias_x = 0;
  gyro_bias_y = 0;
  gyro_bias_z = 0;
  for (ii = 0; ii < packet_count; ii++)
  {
    read_MPU6050_registers( MPU6050_ADDRESS, MPU6050_REGISTER_FIFO_R_W, data_block_fifo_packet, sizeof(data_block_fifo_packet), bsc ); // read data for averaging

    reconstructor_accel_x.field.H = data_block_fifo_packet[0];
    reconstructor_accel_x.field.L = data_block_fifo_packet[1];
    reconstructor_accel_y.field.H = data_block_fifo_packet[2];
    reconstructor_accel_y.field.L = data_block_fifo_packet[3];
    reconstructor_accel_z.field.H = data_block_fifo_packet[4];
    reconstructor_accel_z.field.L = data_block_fifo_packet[5];
    reconstructor_gyro_x.field.H  = data_block_fifo_packet[6];
    reconstructor_gyro_x.field.L  = data_block_fifo_packet[7];
    reconstructor_gyro_y.field.H  = data_block_fifo_packet[8];
    reconstructor_gyro_y.field.L  = data_block_fifo_packet[9];
    reconstructor_gyro_z.field.H  = data_block_fifo_packet[10];
    reconstructor_gyro_z.field.L  = data_block_fifo_packet[11];

    accel_bias_x += reconstructor_accel_x.signed_value; // sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias_y += reconstructor_accel_y.signed_value;
    accel_bias_z += reconstructor_accel_z.signed_value;
    gyro_bias_x  += reconstructor_gyro_x.signed_value;
    gyro_bias_y  += reconstructor_gyro_y.signed_value;
    gyro_bias_z  += reconstructor_gyro_z.signed_value;
  }
  accel_bias_x /= (int32_t)packet_count;
  accel_bias_y /= (int32_t)packet_count;
  accel_bias_z /= (int32_t)packet_count;
  gyro_bias_x /= (int32_t)packet_count;
  gyro_bias_y /= (int32_t)packet_count;
  gyro_bias_z /= (int32_t)packet_count;

  if (accel_bias_z > 0) // remove gravity from the z-axis accelerometer bias calculation
  {
    accel_bias_z -= (int32_t)(1.0/calibration_accelerometer->scale);
  }
  else
  {
    accel_bias_z += (int32_t)(1.0/calibration_accelerometer->scale);
  }

  // the code that this is based off of tried to push the bias calculation values to hardware correction registers
  // these registers do not appear to be functioning, so rely on software offset correction

  // output scaled gyro biases
  calibration_gyroscope->offset_x = ((float)gyro_bias_x)*calibration_gyroscope->scale;
  calibration_gyroscope->offset_y = ((float)gyro_bias_y)*calibration_gyroscope->scale;
  calibration_gyroscope->offset_z = ((float)gyro_bias_z)*calibration_gyroscope->scale;

  // output scaled accelerometer biases
  calibration_accelerometer->offset_x = ((float)accel_bias_x)*calibration_accelerometer->scale;
  calibration_accelerometer->offset_y = ((float)accel_bias_y)*calibration_accelerometer->scale;
  calibration_accelerometer->offset_z = ((float)accel_bias_z)*calibration_accelerometer->scale;

  return;
}

void initialize_accelerometer_and_gyroscope(
    struct calibration_data *     calibration_accelerometer,
    struct calibration_data *     calibration_gyroscope,
    volatile struct bsc_register *bsc )
{
  union MPU6050_transaction_field_data  transaction;

  /* print WHO_AM_I */
  printf( "accel WHOAMI (0x68) = 0x%2.2X\n",
      read_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_WHO_AM_I, bsc ).WHO_AM_I.WHOAMI );

  // based off https://github.com/brianc118/MPU9250/blob/master/MPU9250.cpp

  calibrate_accelerometer_and_gyroscope( calibration_accelerometer, calibration_gyroscope, bsc );

  // reset MPU9205
  transaction.PWR_MGMT_1.CLKSEL       = 0;
  transaction.PWR_MGMT_1.TEMP_DIS     = 0;
  transaction.PWR_MGMT_1.reserved     = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.DEVICE_RESET = 1;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc );
  usleep( 1000 ); // wait for all registers to reset

  // clock source
  transaction.PWR_MGMT_1.CLKSEL       = 1;
  transaction.PWR_MGMT_1.TEMP_DIS     = 0;
  transaction.PWR_MGMT_1.reserved     = 0;
  transaction.PWR_MGMT_1.CYCLE        = 0;
  transaction.PWR_MGMT_1.SLEEP        = 0;
  transaction.PWR_MGMT_1.DEVICE_RESET = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc );

  // enable acc & gyro
  transaction.PWR_MGMT_2.STBY_ZG      = 0;
  transaction.PWR_MGMT_2.STBY_YG      = 0;
  transaction.PWR_MGMT_2.STBY_XG      = 0;
  transaction.PWR_MGMT_2.STBY_ZA      = 0;
  transaction.PWR_MGMT_2.STBY_YA      = 0;
  transaction.PWR_MGMT_2.STBY_XA      = 0;
  transaction.PWR_MGMT_2.LP_WAKE_CTRL = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc );

  // use DLPF set gyro bandwidth 184Hz, temperature bandwidth 188Hz
  transaction.CONFIG.DLPF_CFG     = 1;
  transaction.CONFIG.EXT_SYNC_SET = 0;
  transaction.CONFIG.reserved     = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_CONFIG, transaction, bsc );

  // +-250dps
  transaction.GYRO_CONFIG.reserved0 = 0;
  transaction.GYRO_CONFIG.FS_SEL    = 0;
  transaction.GYRO_CONFIG.reserved1 = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_GYRO_CONFIG, transaction, bsc );

  // +-2G
  transaction.ACCEL_CONFIG.reserved     = 0;
  transaction.ACCEL_CONFIG.ACCEL_FS_SEL = 0;
  transaction.ACCEL_CONFIG.ZA_ST        = 0;
  transaction.ACCEL_CONFIG.YA_ST        = 0;
  transaction.ACCEL_CONFIG.XA_ST        = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_ACCEL_CONFIG, transaction, bsc );

  // force into I2C mode, disabling I2C master
  transaction.USER_CTRL.SIG_COND_RESET  = 0;
  transaction.USER_CTRL.I2C_MST_RESET   = 0;
  transaction.USER_CTRL.FIFO_RESET      = 0;
  transaction.USER_CTRL.reserved0       = 0;
  transaction.USER_CTRL.I2C_IF_DIS      = 0;
  transaction.USER_CTRL.I2C_MST_EN      = 0;
  transaction.USER_CTRL.FIFO_EN         = 0;
  transaction.USER_CTRL.reserved1       = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_USER_CTRL, transaction, bsc );

  // enable bypass mode
  transaction.INT_PIN_CFG.reserved        = 0;
  transaction.INT_PIN_CFG.I2C_BYPASS_EN   = 1;
  transaction.INT_PIN_CFG.FSYNC_INT_EN    = 0;
  transaction.INT_PIN_CFG.FSYNC_INT_LEVEL = 0;
  transaction.INT_PIN_CFG.INT_RD_CLEAR    = 0;
  transaction.INT_PIN_CFG.LATCH_INT_EN    = 0;
  transaction.INT_PIN_CFG.INT_OPEN        = 0;
  transaction.INT_PIN_CFG.INT_LEVEL       = 0;
  write_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_INT_PIN_CFG, transaction, bsc );

  return;
}

void read_accelerometer_gyroscope(
    struct calibration_data *     calibration_accelerometer,
    struct calibration_data *     calibration_gyroscope,
    volatile struct bsc_register *bsc )
{
  uint8_t                   data_block[6+2+6];
  union uint16_to_2uint8    ACCEL_XOUT;
  union uint16_to_2uint8    ACCEL_YOUT;
  union uint16_to_2uint8    ACCEL_ZOUT;
  union uint16_to_2uint8    GYRO_XOUT;
  union uint16_to_2uint8    GYRO_YOUT;
  union uint16_to_2uint8    GYRO_ZOUT;
  
  float ac_x, ac_y, ac_z, gy_x, gy_y, gy_z;

  /*
   * poll the interrupt status register and it tells you when it is done
   * once it is done, read the data registers
   */
  do
  {
    usleep( 1000 );
  } while (read_MPU6050_register( MPU6050_ADDRESS, MPU6050_REGISTER_INT_STATUS, bsc ).INT_STATUS.DATA_RDY_INT == 0);

  // read the accelerometer values
  read_MPU6050_registers( MPU6050_ADDRESS, MPU6050_REGISTER_ACCEL_XOUT_H, data_block, sizeof(data_block), bsc );
  ACCEL_XOUT.field.H  = data_block[0];
  ACCEL_XOUT.field.L  = data_block[1];
  ACCEL_YOUT.field.H  = data_block[2];
  ACCEL_YOUT.field.L  = data_block[3];
  ACCEL_ZOUT.field.H  = data_block[4];
  ACCEL_ZOUT.field.L  = data_block[5];
  // TEMP_OUT.field.H = data_block[6];
  // TEMP_OUT.field.L = data_block[7];
  GYRO_XOUT.field.H   = data_block[8];
  GYRO_XOUT.field.L   = data_block[9];
  GYRO_YOUT.field.H   = data_block[10];
  GYRO_YOUT.field.L   = data_block[11];
  GYRO_ZOUT.field.H   = data_block[12];
  GYRO_ZOUT.field.L   = data_block[13];
  
  gy_x = GYRO_XOUT.signed_value*calibration_gyroscope->scale - calibration_gyroscope->offset_x;
  gy_y = GYRO_YOUT.signed_value*calibration_gyroscope->scale - calibration_gyroscope->offset_y;
  gy_z = GYRO_ZOUT.signed_value*calibration_gyroscope->scale - calibration_gyroscope->offset_z;
  
  ac_x = (ACCEL_XOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_x)*9.81;
  ac_y = (ACCEL_YOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_y)*9.81;
  ac_z = (ACCEL_ZOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_z)*9.81;
  
  fprintf( fp, "Gyro X: %.2f deg\ty=%.2f deg\tz=%.2f deg\t",
      gy_x,
      gy_y,
      gy_z );

  fprintf( fp, "Accel X: %.2f m/s^2\ty=%.2f m/s^2\tz=%.2f m/s^2\n",
      ac_x,
      ac_y,
      ac_z );
  //find min max for each param
  if( gy_x < 0 ){
    if( gy_x <= gy_x_min ) gy_x_min = gy_x;
  }
  else{
    if( gy_x >= gy_x_max ) gy_x_max = gy_x;
  }
  
  if( gy_y < 0 ){
    if( gy_y <= gy_y_min ) gy_y_min = gy_y;
  }
  else{
    if( gy_y >= gy_y_max ) gy_y_max = gy_y;
  }
  
  if( gy_z < 0 ){
    if( gy_z <= gy_z_min ) gy_z_min = gy_z;
  }
  else{
    if( gy_z >= gy_z_max ) gy_z_max = gy_z;
  }
  //
  if( ac_x < 0 ){
    if( ac_x <= ac_x_min ) ac_x_min = ac_x;
  }
  else{
    if( ac_x >= ac_x_max ) ac_x_max = ac_x;
  }
  
  if( ac_y < 0 ){
    if( ac_y <= ac_y_min ) ac_y_min = ac_y;
  }
  else{
    if( ac_y >= ac_y_max ) ac_y_max = ac_y;
  }
  
  if( ac_z < 0 ){
    if( ac_z <= ac_z_min ) ac_z_min = ac_z;
  }
  else{
    if( ac_z >= ac_z_max ) ac_z_max = ac_z;
  }
  
  /*
  fprintf( fp, "Gyro X: %.2f deg\ty=%.2f deg\tz=%.2f deg\t",
      GYRO_XOUT.signed_value*calibration_gyroscope->scale - calibration_gyroscope->offset_x,
      GYRO_YOUT.signed_value*calibration_gyroscope->scale - calibration_gyroscope->offset_y,
      GYRO_ZOUT.signed_value*calibration_gyroscope->scale - calibration_gyroscope->offset_z );

  fprintf( fp, "Accel X: %.2f m/s^2\ty=%.2f m/s^2\tz=%.2f m/s^2\n",
      (ACCEL_XOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_x)*9.81,
      (ACCEL_YOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_y)*9.81,
      (ACCEL_ZOUT.signed_value*calibration_accelerometer->scale - calibration_accelerometer->offset_z)*9.81 );
  */
  fprintf( raw_f, "%.2f\t%.2f\t%.2f\t",
      ac_x,
      ac_y,
      ac_z );
  fprintf( raw_f, "%.2f\t%.2f\t%.2f\n",
      gy_x,
      gy_y,
      gy_z );
  
  return;
}
//

struct thread_param{
  volatile struct io_peripherals *io;
  char side;
  struct calibration_data* tp_calibration_accelerometer;
  struct calibration_data* tp_calibration_gyroscope;

};

struct RGB_pixel{
      unsigned char R;
      unsigned char G;
      unsigned char B;
};

typedef struct xy_t{
  int x;
  int y;
} xy_t;

xy_t coordinate;

int mult( int this, int times ){
  int is=0;
  for( int i = 0; i < times; i++){
    is = is + this;
  }
  return is;
}

void resetKB( struct termios *init_setting ){

  tcsetattr( 0, TCSANOW, init_setting );
  
}

void setKB( struct termios *init_setting ){
  
  struct termios new_setting;
  
  tcgetattr( 0, init_setting );
  
  new_setting = *init_setting;
  new_setting.c_lflag &= ~ICANON; //sets terminal to noncanonical mode (input available immediately)
  new_setting.c_cc[VMIN] = 0; //with VMIN & VTIME both set to 0, read() returns immdediately
  new_setting.c_cc[VTIME] = 0;
  new_setting.c_lflag &= ~ECHO;
  
  tcsetattr( 0, TCSANOW, &new_setting );
  
}

char my_getchar(){
  char buff[1];
  int l = read(STDIN_FILENO, buff, 1);
  if( l>0 ){
    printf( "%c\n", buff[0] );
    p = 0;
    return buff[0];
  }
  
  return ( EOF );
  
}

void enqueue( char input ){
  pthread_mutex_lock(&lock);
  
  if( numQueue == 10 ){
    printf("Queue is full, cannot schedule");
    
  }
  else{
    queue[numQueue] = input;
    numQueue++;
    
  }
  pthread_mutex_unlock(&lock);
}

void dequeue(){
  pthread_mutex_lock(&lock);
  
  if( numQueue == 0 ){
    printf("Queue is empty");
  
  }
  else{
    for(int i = 0; i < 10; i++ ){
      queue[i] = queue[i+1];
    }
    numQueue--;
    for(int i = numQueue; i < 10; i++){
      queue[i] = 0;
    }
    
  }
  
  pthread_mutex_unlock(&lock);
  
}

void B_bit_set( char LorR, char d, struct thread_param* param ){
    //edited for reverse L<->R F<->B
    switch( LorR ){
      case 'R':
        if( d == 'B' ){
          GPIO_SET( param->io->gpio, 05 );
          GPIO_CLR( param->io->gpio, 06 );
        }
        else if( d == 'F' ){
          GPIO_CLR( param->io->gpio, 05 );
          GPIO_SET( param->io->gpio, 06 );
        }
        else if ( d == 'S' ){
          GPIO_CLR( param->io->gpio, 05 );
          GPIO_CLR( param->io->gpio, 06 );
        }
        
        break;
        
      case 'L':
        if( d == 'B' ){
          GPIO_SET( param->io->gpio, 22 );
          GPIO_CLR( param->io->gpio, 23 );
        }
        else if( d == 'F' ){
          GPIO_CLR( param->io->gpio, 22 );
          GPIO_SET( param->io->gpio, 23 );
        }
        else if ( d == 'S' ){
          GPIO_CLR( param->io->gpio, 22 );
          GPIO_CLR( param->io->gpio, 23 );
        }
        
        break;
        
    }
  
}

void F_bit_set( char LorR, char d, struct thread_param* param ){
    //edited for reverse L<->R F<->B
    switch( LorR ){
      case 'L':
        if( d == 'F' ){
          GPIO_SET( param->io->gpio, 05 );
          GPIO_CLR( param->io->gpio, 06 );
        }
        else if( d == 'B' ){
          GPIO_CLR( param->io->gpio, 05 );
          GPIO_SET( param->io->gpio, 06 );
        }
        else if ( d == 'S' ){
          GPIO_CLR( param->io->gpio, 05 );
          GPIO_CLR( param->io->gpio, 06 );
        }
        
        break;
        
      case 'R':
        if( d == 'F' ){
          GPIO_SET( param->io->gpio, 22 );
          GPIO_CLR( param->io->gpio, 23 );
        }
        else if( d == 'B' ){
          GPIO_CLR( param->io->gpio, 22 );
          GPIO_SET( param->io->gpio, 23 );
        }
        else if ( d == 'S' ){
          GPIO_CLR( param->io->gpio, 22 );
          GPIO_CLR( param->io->gpio, 23 );
        }
        
        break;
        
    }
  
} 
/*
void set_motor( char m, int speed, struct thread_param* param ){
  //["<", ">"] & ["R", "L"] are switched for camera
  switch( m ){
    case 'R':
      if( speed > 0 ) bit_set( 'R', 'B', param );
      else if( speed < 0 ) bit_set( 'R', 'F', param );
      else bit_set( 'R', 'S', param );
      param->io->pwm->DAT1 = abs(speed);
      printf("left motor: %d\n", speed);
      break;
    case 'L':
      if( speed > 0 ) bit_set( 'L', 'B', param );
      else if( speed < 0 ) bit_set( 'L', 'F', param );
      else bit_set( 'L', 'S', param );
      param->io->pwm->DAT2 = abs(speed);
      printf("right motor: %d\n", speed);
      break;
  }
  
}
*/


xy_t get_coordinate( int code ){
  xy_t c;
  c.x = code % g_image_width;
  c.y = (code - (code % g_image_width))/g_image_width;
  
  return c;
}

int get_pos( int x, int y ){
  int pos;
  
  pos = (y * g_image_width) + x;
  
  return pos;
  
  
}

void find_point( unsigned char* s_image_data, struct raspicam_wrapper_handle *  Camera ){
  //X:(600,680) Y:(490,570)
  struct RGB_pixel* image_pixel;
  unsigned int      pixel_count;
  unsigned int      pixel_index;
  unsigned char     pixel_value;
  int r_avg, g_avg, b_avg;
  int row_count;
  int col_count;
  int tmp_index;
  
  int pixel_height = raspicam_wrapper_getHeight( Camera );
  int pixel_width = raspicam_wrapper_getWidth( Camera );
  int avg_max = 0;
  int g_b_avg = 0;
  int r_b_avg = 0;
  int r_threshold = 120;
  int g_threshold = 170;
  int b_threshold = 170;
  int r_diff = 0;
  int g_diff = 0;
  int position = 0;
  int target = 0;

  //image variable assign
  image_pixel = (struct RGB_pixel *) s_image_data;
  
  // view data as R-byte, G-byte, and B-byte per pixel
  image_pixel = (struct RGB_pixel *)s_image_data;
  pixel_count = pixel_height * pixel_width;
  
  r_avg = 0;
  g_avg = 0;
  b_avg = 0;

  for (pixel_index = 0; pixel_index < pixel_count; pixel_index = pixel_index+20 ){
    //by average 20x20
    if( (pixel_index%pixel_width == 0) && pixel_index != 0 ) pixel_index = pixel_index+mult(pixel_width, 19);
    col_count = 0;
    tmp_index = pixel_index;
    if( (pixel_index%pixel_width == 0) && pixel_index != 0 ) pixel_index = pixel_index+pixel_width;
    r_avg = 0;
    while( col_count < 20 ){
      tmp_index = tmp_index + pixel_width; 
      row_count = 0;
      while( row_count < 20 ){
        r_avg = r_avg + (unsigned int)image_pixel[tmp_index+row_count].R;
        g_avg = g_avg + (unsigned int)image_pixel[tmp_index+row_count].G;
        b_avg = b_avg + (unsigned int)image_pixel[tmp_index+row_count].B;
        row_count++;
        
      }
      col_count++;
    }
    r_avg = r_avg/(20*20);
    g_avg = g_avg/(20*20);
    b_avg = b_avg/(20*20);
    g_b_avg = (g_avg + b_avg) / 2;
    r_b_avg = (r_avg + b_avg) / 2;
    r_diff = r_avg - g_b_avg;
    g_diff = g_avg - r_b_avg;
    //try rgb difference
    
    if( g_diff > 10 ){
      if( g_avg > avg_max){
        avg_max= g_avg;
        position = pixel_index;
      }
    }
    if( pixel_index == ((pixel_count-1)-mult(pixel_width, 19)-19) ) break;

    
  }
  pthread_mutex_lock(&lock);
  if( avg_max == 0 ){
    coordinate.x = -1;
    coordinate.y = -1;
  }
  else{
    //X:(600,680) Y:(490,570)
    coordinate = get_coordinate(position);
    coordinate.x = coordinate.x + 10;
    coordinate.y = coordinate.y + 10;
    if( coordinate.x < 600 ) x = -1;
    else if( coordinate.x > 680 ) x = 1;
    else x = 0;
    
    if( coordinate.y < 490 ) y = 1;
    else if( coordinate.y > 570 ) y = -1;
    else y = 0;
    //printf( "(%d, %d)\n", coordinate.x, coordinate.y );
  }
  pthread_mutex_unlock(&lock);
}

void* process_image( void* arg ){
  struct raspicam_wrapper_handle *  Camera;       /* Camera handle */
  int return_value;                               /* the return value of this program */
  size_t  image_size;
  int image_height;
  int image_width;
  int pixel_count;
  int pixel_index;
  struct RGB_pixel* image_pixel;
  unsigned char* image_data;
  
  int capture = 1;
  
  //Set Up Camera
  printf( "Opening Camera...\n" );
  Camera = raspicam_wrapper_create();
  if (Camera != NULL){
    if( raspicam_wrapper_open(Camera) ){
      printf( "Stabalizing Camera\n" );
      sleep(1);
    }
    else return 0;
  }
  image_size = raspicam_wrapper_getImageTypeSize( Camera, RASPICAM_WRAPPER_FORMAT_RGB );
  image_width = raspicam_wrapper_getWidth( Camera );
  g_image_width = image_width;
  image_height = raspicam_wrapper_getHeight( Camera );
  g_image_height = image_height;
  pixel_count = image_width * image_height;
  image_data = (unsigned char *)malloc( image_size );
  printf("Stabalizing done\n");
  //Set Up Camera Complete
  
  while( mode2_status == 1 && key == 1){
    raspicam_wrapper_grab( Camera );
    raspicam_wrapper_retrieve( Camera, image_data, RASPICAM_WRAPPER_FORMAT_RGB );
    find_point( image_data, Camera ); //stores (x,y) globaly
    usleep(100000); //was 3000
  }
  
  image_p = image_data;
  //borderline set (utility)
  if( capture == 1 ){
    image_pixel = (struct RGB_pixel *) image_p;
    //find point avg values 
    int r = 0;
    int c = 0;
    int index;
    int r_avg = 0;
    int g_avg = 0;
    int b_avg = 0;
    int pos;
    
    if( coordinate.x != -1 ){
      while(c<20){
        r = 0;
        while(r<20){
          pos = get_pos( ((coordinate.x-10)+r), ((coordinate.y-10)+c));
          r_avg = r_avg + image_pixel[pos].R;
          g_avg = g_avg + image_pixel[pos].G;
          b_avg = b_avg + image_pixel[pos].B;
          r++;
        }
        c++;
      }
      
      r_avg = r_avg / 400;
      g_avg = g_avg / 400;
      b_avg = b_avg / 400;
      
      //printf("\n(%d, %d) - R_avg: %d\tG_avg: %d\tB_avg: %d\n", coordinate.x, coordinate.y, r_avg, g_avg, b_avg );
      
      //draw point
      r = 0;
      c = 0;
      while(r<20){
        c = 0;
        while(c<20){
          pos = get_pos( ((coordinate.x-10)+c), ((coordinate.y-10)+r));
          if( r == 0 || r == 19 || c == 0 || c == 19){
            image_pixel[pos].R = 0;
            image_pixel[pos].G = 255;
            image_pixel[pos].B = 0;
          }
          c++;
        }
        r++;
      }
    }
    //draw lines
    int hor_flag_1 = 0;
    int hor_flag_2 = 0;
    for ( pixel_index = 0; pixel_index < pixel_count; pixel_index++){
      if( (pixel_index-600)%image_width == 0 ){
        image_pixel[pixel_index].R = 255;
        image_pixel[pixel_index].G = 0;
        image_pixel[pixel_index].B = 0;
      }
      if( (pixel_index-680)%image_width == 0 ){
        image_pixel[pixel_index].R = 0;
        image_pixel[pixel_index].G = 255;
        image_pixel[pixel_index].B = 0;
      }

      if( pixel_index == image_width*490 ) hor_flag_1 = 1;
      if( pixel_index == image_width*490+image_width ) hor_flag_1 = 0;
      if( hor_flag_1 == 1 ){
        image_pixel[pixel_index].R = 255;
        image_pixel[pixel_index].G = 0;
        image_pixel[pixel_index].B = 0;
      }
      if( pixel_index == image_width*570 ) hor_flag_2 = 1;
      if( pixel_index == image_width*570+image_width ) hor_flag_2 = 0;
      if( hor_flag_2 == 1 ){
        image_pixel[pixel_index].R = 0;
        image_pixel[pixel_index].G = 255;
        image_pixel[pixel_index].B = 0;
      }
    }
    
    /*
    int x = 0;
    int tmp_index;
    int col_count;
    int row_count;
    for (pixel_index = 0; pixel_index < pixel_count; pixel_index = pixel_index+20 ){
      if( (pixel_index%image_width == 0) && pixel_index != 0 ) pixel_index = pixel_index+mult(image_width, 19);
      col_count = 0;
      tmp_index = pixel_index;
      if(x == 0)x=1;
      else if( x== 1)x=2;
      else x=0;
      
      while( col_count < 20 ){
        tmp_index = tmp_index + image_width; 
        row_count = 0;
        while( row_count < 20 ){
          if( x == 0 ){
            image_pixel[tmp_index+row_count].R = 255;
            image_pixel[tmp_index+row_count].G = 0;
            image_pixel[tmp_index+row_count].B = 0;
  
          }
          if( x == 1 ){
            image_pixel[tmp_index+row_count].G = 255;
            image_pixel[tmp_index+row_count].R = 0;
            image_pixel[tmp_index+row_count].B = 0;
  
          }
          if( x == 2 ){
            image_pixel[tmp_index+row_count].B = 255;
            image_pixel[tmp_index+row_count].R = 0;
            image_pixel[tmp_index+row_count].G = 0;
  
          }
          row_count++;
          
        }
        col_count++;
      }
      if( pixel_index == ((pixel_count-1)-mult(image_width, 19)-19) ) break;

    
    }
    */
    
    FILE * outFile = fopen( "hw9test.ppm", "wb" );
      if (outFile != NULL){
        
        fprintf( outFile, "P6\n" );  // write .ppm file header
        fprintf( outFile, "%d %d 255\n", raspicam_wrapper_getWidth( Camera ), raspicam_wrapper_getHeight( Camera ) );
        // write the image data
        fwrite( image_p, 1, raspicam_wrapper_getImageTypeSize( Camera, RASPICAM_WRAPPER_FORMAT_RGB ), outFile );
        fclose( outFile );
        printf( "Image, picture saved as pic1.ppm\n" );
      }
    }
    //boderline set done
      
      
    
  free( image_data );
  raspicam_wrapper_destroy( Camera );
  
  return 0;
}

void* chase_point( void* arg ){
  //X:(600,680) Y:(490,570)
  //x:left-right y: up-down
  //dat2 is left motor, dat 1 is right motor
  struct thread_param* param = (struct thread_param*)arg;
  int L_SPEED = 0;
  int R_SPEED = 0;
  int turn_speed_limit = 70;
  int speed_limit = 60;
  int back_speed_limit = 50;
  int turnTime = 5000;
  //char side;
  //int y_up = 490;
  //int y_down = 570;
  //int x_left = 600;
  //int x_right = 680;
  int p;
  
  B_bit_set( 'L', 'S', param );
  B_bit_set( 'R', 'S', param );
  
  while( mode2_status == 1 ){
    if( mode2w == 1 ){
      if( coordinate.x != -1 ){
        if( y == 1 ){
          //go forward
          L_SPEED = speed_limit;
          R_SPEED = speed_limit;
          B_bit_set( 'L', 'F', param );
          B_bit_set( 'R', 'F', param );
          param->io->pwm->DAT2 = L_SPEED;
          param->io->pwm->DAT1 = R_SPEED;
          
          //turn
          while( mode2_status == 1 && y == 1 && coordinate.x != -1 ){
            if( y != 1 ) break;
            if( x == -1 ){
              p = 0;
              while( x == -1 && mode2_status == 1 && coordinate.x != -1 ){
                if( p == 0 ){
                  printf("TURNING LEFT\n");
                  p = 1;
                }
                if( y != 1 || x != -1 ) break;
                //side quick switch
                if( x > 1 ){
                  L_SPEED = speed_limit;
                  param->io->pwm->DAT2 = L_SPEED;
                  break;
                }
                //turn left
                if( L_SPEED-1 >= 0 ){
                  L_SPEED--;
                  param->io->pwm->DAT2 = L_SPEED;
                }
                if( L_SPEED == 0 ){
                  if( R_SPEED+1 <= turn_speed_limit ){
                    R_SPEED++;
                    param->io->pwm->DAT1 = R_SPEED;
                  }
                }
                usleep(turnTime);
              }
            }
            else if( x == 1 ){
              p = 0;
              while( x == 1 && mode2_status == 1 && coordinate.x != -1 ){
                if( p == 0 ){
                  printf("TURNING_RIGHT\n");
                  p = 1;
                }
                if( y != 1 || x != 1 ) break;
                //side quick switch
                if( x < -1 ){
                  R_SPEED = speed_limit;
                  param->io->pwm->DAT1 = R_SPEED;
                  break;
                }
                //turn right
                if( R_SPEED-1 >= 0 ){
                  R_SPEED--;
                  param->io->pwm->DAT1 = R_SPEED;
                }
                if( R_SPEED == 0 ){
                  if( L_SPEED+1 <= turn_speed_limit ){
                    L_SPEED++;
                    param->io->pwm->DAT2 = L_SPEED;
                  }
                }
  
                usleep(turnTime);
              }
            }
            
            p = 0;
            while( x == 0 && y == 1 && coordinate.x != -1 ){
              if( p == 0 ){
                printf("SPEED BACK UP\n");
                p = 1;
              }
              if( L_SPEED+1 <= speed_limit){
                L_SPEED++;
                //bit_set( 'L', 'F', param );
                param->io->pwm->DAT2 = L_SPEED;
              }
              if( R_SPEED+1 <= speed_limit ){
                R_SPEED++;
                //bit_set( 'R', 'F', param );
                param->io->pwm->DAT1 = R_SPEED;
              }
              if( R_SPEED == 0 ){
                
              usleep(turnTime);
              }
            }
            
          }
          
        }
        else if( y == -1 ){
          //go backward
          L_SPEED = back_speed_limit;
          R_SPEED = back_speed_limit;
          B_bit_set( 'L', 'B', param );
          B_bit_set( 'R', 'B', param );
          param->io->pwm->DAT2 = L_SPEED;
          param->io->pwm->DAT1 = R_SPEED;
          
          //turn
          while( mode2_status == 1 && y == -1 && coordinate.x != -1 ){
            if( y != -1 ) break;
            if( x == -1 ){
              p = 0;
              while( x == -1 && mode2_status == 1 && coordinate.x != -1 ){
                if( p == 0 ){
                  printf("TURNING LEFT\n");
                  p = 1;
                }
                if( y != -1 || x != -1 ) break;
                //side quick switch
                if( x > 1 ){
                  R_SPEED = back_speed_limit;
                  param->io->pwm->DAT1 = R_SPEED;
                  break;
                }
                //turn left
                if( R_SPEED-1 >= 0 ){
                  R_SPEED--;
                  param->io->pwm->DAT1 = R_SPEED;
                }
                if( R_SPEED == 0 ){
                  if( L_SPEED+1 <= turn_speed_limit ){
                    L_SPEED++;
                    param->io->pwm->DAT2 = L_SPEED;
                  }
                }
                usleep(turnTime);
              }
            }
            else if( x == 1 ){
              p = 0;
              while( x == 1 && mode2_status == 1 && coordinate.x != -1 ){
                if( p == 0 ){
                  printf("TURNING_RIGHT\n");
                  p = 1;
                }
                if( y != -1 || x != 1) break;
                //side quick switch
                if( x < -1 ){
                  L_SPEED = back_speed_limit;
                  param->io->pwm->DAT2 = L_SPEED;
                  break;
                }
                //turn right
                if( L_SPEED-1 >= 0 ){
                  L_SPEED--;
                  param->io->pwm->DAT2 = L_SPEED;
                }
                if( L_SPEED == 0 ){
                  if( R_SPEED+1 <= turn_speed_limit ){
                    R_SPEED++;
                    param->io->pwm->DAT1 = R_SPEED;
                  }
                }
  
                usleep(turnTime);
              }
            }
            
            p = 0;
            while( x == 0 && y == -1 && coordinate.x != -1 ){
              if( p == 0 ){
                printf("SPEED BACK UP\n");
                p = 1;
              }
              if( L_SPEED+1 <= speed_limit){
                L_SPEED++;
                //bit_set( 'L', 'B', param );
                param->io->pwm->DAT2 = L_SPEED;
              }
              if( R_SPEED+1 <= speed_limit ){
                R_SPEED++;
                //bit_set( 'R', 'B', param );
                param->io->pwm->DAT1 = R_SPEED;
              }
              if( R_SPEED == 0 ){
                
              usleep(turnTime);
              }
            }
            
          }
        }
        else{
          L_SPEED = 0;
          R_SPEED = 0;
          B_bit_set( 'L', 'S', param );
          B_bit_set( 'R', 'S', param );
          param->io->pwm->DAT1 = R_SPEED;
          param->io->pwm->DAT2 = L_SPEED;
        
        }
      }
      else{
        L_SPEED = 0;
        R_SPEED = 0;
        B_bit_set( 'L', 'S', param );
        B_bit_set( 'R', 'S', param );
        param->io->pwm->DAT1 = R_SPEED;
        param->io->pwm->DAT2 = L_SPEED;
        
      }
    }
    else{
        L_SPEED = 0;
        R_SPEED = 0;
        B_bit_set( 'L', 'S', param );
        B_bit_set( 'R', 'S', param );
        param->io->pwm->DAT1 = R_SPEED;
        param->io->pwm->DAT2 = L_SPEED;
    }
        
    
  }
  
  return 0;
}

void* record_action( void* arg ){
  //speed of each motor every 1000us
  struct thread_param* param = (struct thread_param*)arg;
  FILE *record_f;
  int DAT1;
  int DAT2;
  int gpio_05, gpio_06, gpio_22, gpio_23;
  
  avg_speed = 0;
  avg_count = 0;
  usec_count = 0;
  record_f = fopen( "action_data.txt", "w" );
  
  while( record == 1 && key != 0 ){
    DAT1 = param->io->pwm->DAT1;
    DAT2 = param->io->pwm->DAT2;
    avg_count = avg_count + 2;
    avg_speed = avg_speed + DAT1 + DAT2;
    usec_count++;
    
    if( GPIO_READ( param->io->gpio, 5 ) > 0 ) gpio_05 = 1;
    else gpio_05 = 0;
    if( GPIO_READ( param->io->gpio, 6 ) > 0 ) gpio_06 = 1;
    else gpio_06 = 0;
    if( GPIO_READ( param->io->gpio, 22 ) > 0 ) gpio_22 = 1;
    else gpio_22 = 0;
    if( GPIO_READ( param->io->gpio, 23 ) > 0 ) gpio_23 = 1;
    else gpio_23 = 0;
    
    fprintf( record_f, "%d\t%d\t%d\t%d\t%d\t%d\n", DAT1, DAT2, gpio_05, gpio_06, gpio_22, gpio_23 );
    usleep(1000);
  }
  avg_speed = avg_speed / avg_count;
  fclose( record_f );
  
}

void* run_MPU9250( void* arg ){
  struct thread_param* param = (struct thread_param*)arg;

  initialize_accelerometer_and_gyroscope( (param->tp_calibration_accelerometer), (param->tp_calibration_gyroscope), param->io->bsc );

  ac_x_max = 0;
  ac_x_min = 0;
  ac_y_max = 0;
  ac_y_min = 0;
  ac_z_max = 0;
  ac_z_min = 0;
  
  gy_x_max = 0;
  gy_x_min = 0;
  gy_y_max = 0;
  gy_y_min = 0;
  gy_z_max = 0;
  gy_z_min = 0;
  
  if( mode == 1 || mode == 2 ){
    if( mode == 1 ){
      fp = fopen( "hw7m1data.txt", "w" );
      if( fp == NULL ) printf("could not open file");
      fprintf( fp, "hw7 mode 1 data:\n" );
    }
    if( mode == 2 ){
      fp = fopen( "hw7m2data.txt", "w" );
      if( fp == NULL ) printf("could not open file");
      fprintf( fp, "hw7 mode 2 data:\n" );
    }
   
    raw_f = fopen( "raw_data.txt", "w" );
    
    while( record == 1 && key!= 0 ){
      read_accelerometer_gyroscope( (param->tp_calibration_accelerometer), (param->tp_calibration_gyroscope), param->io->bsc );
      usleep(10000);
    }
    fclose( fp );
    fclose( raw_f );
    
  }
  
  return 0;
}

void* run_script( void *arg ){
  struct thread_param* param = (struct thread_param*)arg;
  char line[16];
  char buf[16];
  int DAT1;
  int DAT2;
  int gpio_05, gpio_06, gpio_22, gpio_23;
  int dp = 0;
  int linenum = 0;
  char delim[] = "\t";
  char *ptr;
  int match_delay = 1000;
  
  FILE *read_f = fopen( "action_data.txt", "r" );
  
  while( fgets(line, sizeof(line), read_f) ){
    strncpy( buf, line, sizeof(line) );
    ptr = strtok( buf, delim );
    dp = 0;
    while( ptr != NULL ){
      if( dp == 0 ) DAT1 = atoi(ptr);
      else if( dp == 1 ) DAT2 = atoi(ptr);
      else if( dp == 2 ) gpio_05 = atoi(ptr);
      else if( dp == 3 ) gpio_06 = atoi(ptr);
      else if( dp == 4 ) gpio_22 = atoi(ptr);
      else if( dp == 5 ) gpio_23 = atoi(ptr);
      dp++;
      ptr = strtok( NULL, delim );
    }
    if( linenum%2 == 0 ){
      if( gpio_05 == 1) GPIO_SET( param->io->gpio, 05 );
      else GPIO_CLR( param->io->gpio, 05 );
      if( gpio_06 == 1) GPIO_SET( param->io->gpio, 06 );
      else GPIO_CLR( param->io->gpio, 06 );
      if( gpio_22 == 1) GPIO_SET( param->io->gpio, 22 );
      else GPIO_CLR( param->io->gpio, 22 );
      if( gpio_23 == 1) GPIO_SET( param->io->gpio, 23 );
      else GPIO_CLR( param->io->gpio, 23 );
      
      param->io->pwm->DAT1 = DAT1;
      param->io->pwm->DAT2 = DAT2;
    }
    
    linenum++;
    usleep(match_delay);
  }
  fclose( read_f );
  param->io->pwm->DAT1 = 0;
  param->io->pwm->DAT2 = 0;

  return 0;
}

void* schedule( void* arg ){
  struct thread_param* param = (struct thread_param*)arg;
  //for mode 1
  int c;

  
  while( key != 0 ){
    if( numQueue != 0 ){
      if( mode == 1 ){
        switch( queue[0] ){
          case 's':
            //STOP
            
            F_bit_set( 'R', 'S', param);
            F_bit_set( 'L', 'S', param);
            
            param->io->pwm->DAT1 = 0;
            param->io->pwm->DAT2 = 0;
            

            current_speed = 0;
            gear_mem = gear;
            gear = 'S';
            
            dequeue();
            break;
            
          case 'w':
            //FORWARD
            if( gear == 'B' ){
              current_speed--;
              while( current_speed >= 0 && key != 0 ){
                param->io->pwm->DAT1 = current_speed;
                param->io->pwm->DAT2 = current_speed;
                current_speed--;
                usleep(2500);
              }
            }
            
            F_bit_set( 'L', 'F', param );
            F_bit_set( 'R', 'F', param );
  
            param->io->pwm->DAT1 = 100;
            param->io->pwm->DAT2 = 100;
            
            current_speed = 100;
            gear = 'F';
  
            dequeue();
            break;
        
          case 'x':
            //BACKWARD
            if( gear == 'F' ){
              current_speed--;
              while( current_speed >= 0 && key != 0 ){
                param->io->pwm->DAT1 = current_speed;
                param->io->pwm->DAT2 = current_speed;
                current_speed--;
                usleep(2500);
              }
            }
            
            F_bit_set( 'L', 'B', param );
            F_bit_set( 'R', 'B', param );
  
            param->io->pwm->DAT1 = 100;
            param->io->pwm->DAT2 = 100;
            
            current_speed = 100;
            gear = 'B';
            
            dequeue();
            break;
            
          case 'i':
            //increase speed
            if( current_speed == 0 ){
              F_bit_set( 'L', gear_mem, param );
              F_bit_set( 'R', gear_mem, param );
            }
            if( current_speed < 100 ){
              param->io->pwm->DAT1 = current_speed + 5;
              param->io->pwm->DAT2 = current_speed + 5;
              current_speed = current_speed + 5;
            }
            
            dequeue();
            break;
            
          case 'j':
            //decrease speed
            if( current_speed == 0 ){
              F_bit_set( 'L', gear, param );
              F_bit_set( 'R', gear, param );
            }
            if( current_speed > 0 ){
              param->io->pwm->DAT1 = current_speed - 5;
              param->io->pwm->DAT2 = current_speed - 5;
              current_speed = current_speed - 5;
            }
            
            dequeue();
            break;
          
          case 'a':
            if( gear == 'F' || gear == 'B' ){
              c = current_speed;
              while( c > 0 && key != 0 ){
                param->io->pwm->DAT1 = c;
                c--;
                usleep(5000);
                
              }
              if( queue[1] == 'a' ){
                c = 1;
                while( c == 1 && key != 0 ){
                  usleep(150000);
                  dequeue();
                  if( queue[1] != 'a' ){
                    c = 0;
                  }
                }
              }
              param->io->pwm->DAT1 = current_speed;
            }
            else if( gear == 'S' ){
              c = 0;
              F_bit_set( 'R', 'F', param );
              F_bit_set( 'L', 'B', param );
              while( c <= 100 ){
                param->io->pwm->DAT1 = c;
                param->io->pwm->DAT2 = c;
                usleep(1500);
                c++;
              }
              c = 100;
              while( c >= 0 ){
                param->io->pwm->DAT1 = c;
                param->io->pwm->DAT2 = c;
                usleep(1500);
                c--;
              }
            }
            dequeue();
            break;
            
          case 'd':
            if( gear == 'F' || gear == 'B' ){
              c = current_speed;
              while( c > 0 && key != 0 ){
                param->io->pwm->DAT2 = c;
                c--;
                usleep(5000);
                
              }
              if( queue[1] == 'd' ){
                c = 1;
                while( c == 1 && key != 0 ){
                  usleep(150000);
                  dequeue();
                  if( queue[1] != 'd' ){
                    c = 0;
                  }
                }
              }
              param->io->pwm->DAT2 = current_speed;
            }
            else if( gear == 'S' ){
              c = 0;
              F_bit_set( 'R', 'B', param );
              F_bit_set( 'L', 'F', param );
              while( c <= 100 && key != 0 ){
                param->io->pwm->DAT1 = c;
                param->io->pwm->DAT2 = c;
                usleep(1500);
                c++;
              }
              c = 100;
              while( c >= 0 && key != 0 ){
                param->io->pwm->DAT1 = c;
                param->io->pwm->DAT2 = c;
                usleep(1500);
                c--;
              }
            }
            dequeue();
            break;
        }
      }
      else if( mode == 2 ){
        switch( queue[0] ){
          case 's':
            mode2w = 0;
            
            
            dequeue();
            break;
            
          case 'w':
            mode2w = 1;
            
            
            dequeue();
            break;
          
        }
      }


    }
    
  }
  return 0;
}

int main( void )
{
  volatile struct io_peripherals *io;
  struct termios term_current_setting;

  io = import_registers();
  setKB( &term_current_setting );
  smooth = 2500;
  gear = 'S'; //set initial gear for start
  char c;
  char recent_c = 'n';
  char file_data;
  
  FILE *fp;
  char line[40];
  char buf[40];
  int dp = 0;
  char *ptr;
  char delim[] = "\t";

  pthread_t scheduler;
  pthread_t chasePoint;
  pthread_t process_image_p;
  pthread_t run_MPU;
  pthread_t recordAction;
  pthread_t runScript;
  
  struct thread_param scheduler_param;
  struct thread_param image_param;
  struct thread_param chase_point_param;
  struct thread_param MPU_param;
  struct thread_param recordAction_param;
  struct thread_param runScript_param;
  
  int ac_x, ac_y, ac_z, gy_x, gy_y, gy_z;
  
  //struct calibration_data calibration_magnetometer;
  struct calibration_data calibration_accelerometer;
  struct calibration_data calibration_gyroscope;
  
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned int)io );
    //set up io
    io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT;  //GPIO06
    io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT;  //GPIO05
    io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;  //GPIO22
    io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;  //GPIO23
    io->gpio->GPFSEL2.field.FSEL4 = GPFSEL_INPUT;  //GPIO24
    io->gpio->GPFSEL2.field.FSEL5 = GPFSEL_INPUT;  //GPIO25
    
    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;  //GPIO12
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;  //GPIO13
    
    io->pwm->RNG1 = PWM_RANGE;     /* the range value, 100 level steps */
    io->pwm->RNG2 = PWM_RANGE;     /* the range value, 100 level steps */
    io->pwm->DAT1 = 0;             /* initial beginning level=1/100=1% */
    io->pwm->DAT2 = 0;             /* initial beginning level=1/100=1% */
    io->pwm->CTL.field.MODE1 = 0;  /* PWM mode */
    io->pwm->CTL.field.MODE2 = 0;  /* PWM mode */
    io->pwm->CTL.field.RPTL1 = 1;  /* not using FIFO, but repeat the last byte anyway */
    io->pwm->CTL.field.RPTL2 = 1;  /* not using FIFO, but repeat the last byte anyway */
    io->pwm->CTL.field.SBIT1 = 0;  /* idle low */
    io->pwm->CTL.field.SBIT2 = 0;  /* idle low */
    io->pwm->CTL.field.POLA1 = 0;  /* non-inverted polarity */
    io->pwm->CTL.field.POLA2 = 0;  /* non-inverted polarity */
    io->pwm->CTL.field.USEF1 = 0;  /* do not use FIFO */
    io->pwm->CTL.field.USEF2 = 0;  /* do not use FIFO */
    io->pwm->CTL.field.MSEN1 = 1;  /* use M/S algorithm, level=pwm.DAT1/PWM_RANGE */
    io->pwm->CTL.field.MSEN2 = 1;  /* use M/S algorithm, level=pwm.DAT2/PWM_RANGE */
    io->pwm->CTL.field.CLRF1 = 1;  /* clear the FIFO, even though it is not used */
    io->pwm->CTL.field.PWEN1 = 1;  /* enable the PWM channel */
    io->pwm->CTL.field.PWEN2 = 1;  /* enable the PWM channel */
    
    enable_pwm_clock( io->cm, io->pwm );
    
    /* set the pin function to alternate function 0 for GPIO02 (I2C1, SDA) */
    /* set the pin function to alternate function 0 for GPIO03 (I2C1, SCL) */
    io->gpio->GPFSEL0.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio->GPFSEL0.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;

    /* configure the I2C interface */
    io->bsc->DIV.field.CDIV  = (PERIPHERAL_CLOCK*10)/400000;
    io->bsc->DEL.field.REDL  = 0x30;
    io->bsc->DEL.field.FEDL  = 0x30;
    io->bsc->CLKT.field.TOUT = 0x40;
    io->bsc->C.field.INTD    = 0;
    io->bsc->C.field.INTT    = 0;
    io->bsc->C.field.INTR    = 0;
    io->bsc->C.field.I2CEN   = 1;
    io->bsc->C.field.CLEAR   = 1;
    
    printf( "q' to Quit\n");

    scheduler_param.io = io;
    chase_point_param.io = io;
    
    MPU_param.io = io;
    MPU_param.tp_calibration_accelerometer = &(calibration_accelerometer);
    MPU_param.tp_calibration_gyroscope = &(calibration_gyroscope);
    recordAction_param.io = io;
    runScript_param.io = io;

    key = 1;
    pthread_create( &scheduler, NULL, schedule, (void*)&scheduler_param );
    
    while( (c = my_getchar()) != 'q' ){
      if( p == 0 ){
        if(mode == 1) printf("HW6m1> ");
        if(mode == 2) printf("HW6m2> ");
        p = 1;
      }
      
      
      switch( c ){
        case 's': //stop
          recent_c = 's';
          if( record == 1 ){
            record = 0;
            pthread_join( run_MPU, NULL );
            pthread_join( recordAction, NULL );
          }
          enqueue( 's' );
          break;
          
        case 'w': //forward
          recent_c = 'w';
          if( mode == 1 || mode == 2 ){
            if( record == 0 ){
              record = 1;
              pthread_create( &run_MPU, NULL, run_MPU9250, (void*)&MPU_param );
              pthread_create( &recordAction, NULL, record_action, (void*)&recordAction_param );
            }
          }
          enqueue( 'w' );
          break;
          
        case 'r':
          if( recent_c == 's'){
            //create thread and join
            if( access( "action_data.txt", F_OK ) == 0 ){
              if( mode == 2 ){
                if( mode2_status == 1 ){
                  mode2_status = 0;
                  pthread_join( chasePoint, NULL );
                  pthread_join( process_image_p, NULL );
                }
              }
              printf( "running script...\n" );
              pthread_create( &runScript, NULL, run_script, (void*)&runScript_param );
              pthread_join( runScript, NULL );
              printf( "finished running script...\n" );
              if( mode == 2 ){
                mode2_status = 1;
                pthread_create( &process_image_p, NULL, process_image, (void*)&image_param );
                sleep(3);
                pthread_create( &chasePoint, NULL, chase_point, (void*)&chase_point_param );
              }
            }
            else{
              printf( "script file action_data.txt does not exist\n" );
            }
          }
          break;
          
        case 'p':
          if( recent_c == 's'){
            //create thread and join
            if( access( "raw_data.txt", F_OK ) == 0 ){
              fp = fopen( "raw_data.txt", "r" );
              printf( "\nAC GY\n" );
              while( fgets(line, sizeof(line), fp) ){
                strncpy( buf, line, sizeof(line) );
                ptr = strtok( buf, delim );
                dp = 0;
                while( ptr != NULL ){
                  if( dp == 0 ) ac_x = (int)(((float)atoi(ptr)-ac_x_min)*(9)/(ac_x_max-ac_x_min)+0);
                  else if( dp == 1 ) ac_y = (int)(((float)atoi(ptr)-ac_y_min)*(9)/(ac_y_max-ac_y_min)+0);
                  else if( dp == 2 ) ac_z = (int)(((float)atoi(ptr)-ac_z_min)*(9)/(ac_z_max-ac_z_min)+0);
                  else if( dp == 3 ) gy_x = (int)(((float)atoi(ptr)-gy_x_min)*(9)/(gy_x_max-gy_x_min)+0);
                  else if( dp == 4 ) gy_y = (int)(((float)atoi(ptr)-gy_y_min)*(9)/(gy_y_max-gy_y_min)+0);
                  else if( dp == 5 ) gy_z = (int)(((float)atoi(ptr)-gy_z_min)*(9)/(gy_z_max-gy_z_min)+0);
                  dp++;
                  ptr = strtok( NULL, delim );
                }
                printf( "%d%d%d%d%d%d\n", ac_x, ac_y, ac_z, gy_x, gy_y, gy_z );
              }
            }
          }
          break;
        
        case 'n':
          if( recent_c == 's' ){
            printf( "\n%.2fm/s\t%.2f\n", avg_speed * 0.0021, 1000*usec_count*avg_speed * 0.0021 );
          }
          break;
          
        case 'x': //backward
          if( mode == 1 ) enqueue( 'x' );
          break;
          
        case 'i': //increase speed
          if( mode == 1 ) enqueue( 'i' );
          break;
          
        case 'j': //decrease speed
          if( mode == 1 ) enqueue( 'j' );
          break;
          
        case 'a': //turn left
          if( mode == 1 ) enqueue( 'a' );
          break;
          
        case 'd': //turn right
          if( mode == 1 ) enqueue( 'd' );
          break;
          
        case 'm':
          enqueue( 'm' );
          break;
          
        case '1':
          if( queue[numQueue-1] == 'm' ){
            if( mode2_status == 1 ){
              mode2_status = 0;
              mode2w = 0;
              pthread_join( chasePoint, NULL );
              pthread_join( process_image_p, NULL );
              
            }
            mode = 1;
            dequeue();
            
            F_bit_set( 'R', 'S', &scheduler_param );
            F_bit_set( 'L', 'S', &scheduler_param );
            
            io->pwm->DAT1 = 0;
            io->pwm->DAT2 = 0;
            current_speed = 0;
            gear_mem = gear;
            gear = 'S';
          }
          break;
          
        case '2':
          if( queue[numQueue-1] == 'm' ){
            if( mode2_status == 0 ){
              mode2_status = 1;
              pthread_create( &process_image_p, NULL, process_image, (void*)&image_param );
              sleep(3);
              pthread_create( &chasePoint, NULL, chase_point, (void*)&chase_point_param );
            }
            mode = 2;
            mode2w = 0;
            dequeue();
            
            F_bit_set( 'R', 'S', &scheduler_param );
            F_bit_set( 'L', 'S', &scheduler_param );
            
            io->pwm->DAT1 = 0;
            io->pwm->DAT2 = 0;
            current_speed = 0;
            gear_mem = gear;
            gear = 'S';
            
          }
          break;
      }
      
      
    }
    
    key = 0; 
    if( mode2_status == 1 ){
      mode2_status = 0;
      pthread_join( chasePoint, NULL );
      pthread_join( process_image_p, NULL );
    }
    pthread_join( scheduler, NULL );
    
    io->pwm->DAT1 = 0;           
    io->pwm->DAT2 = 0;
    
    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_INPUT;  //GPIO12
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_INPUT;  //GPIO13
    
    io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_INPUT;  //GPIO06
    io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_INPUT;  //GPIO05
    io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_INPUT;  //GPIO22
    io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_INPUT;  //GPIO23
    resetKB( &term_current_setting ); //reset to default terminal setting

  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}
