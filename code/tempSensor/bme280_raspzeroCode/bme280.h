#ifndef BME280_H
#define BME280_H

#include <inttypes.h>


#define BME280_I2C_ADDR           0x77

#define BME280_REG_ID             0xd0
#define BME280_REG_RESET          0xe0
#define BME280_REG_CTRL_HUM       0xf2
#define BME280_REG_STATUS         0xf3
#define BME280_REG_CTRL_MEAS      0xf4
#define BME280_REG_CONFIG         0xf5
#define BME280_REG_PRESS_MSB      0xf7
#define BME280_REG_PRESS_LSB      0xf8
#define BME280_REG_PRESS_XLSB     0xf9
#define BME280_REG_TEMP_MSB       0xfa
#define BME280_REG_TEMP_LSB       0xfb
#define BME280_REG_TEMP_XLSB      0xfc
#define BME280_REG_HUM_MSB        0xfd
#define BME280_REG_HUM_LSB        0xfe

#define BME280_ID                 0x60
#define BME280_RESET_MAGIC        0xB6
#define OVERSAMPLING_HUMID        0x01
#define OVERSAMPLING_TEMP         0x01
#define OVERSAMPLING_PRESS        0x01
#define MODE_SLEEP                0x00
#define MODE_FORCED               0x01
#define MODE_NORMAL               0x11
#define ACQUISITION_DELAY_USEC        100000
#define STATUS_REG_POLL_INTERVAL_USEC 1000


// compensated humidity, temperature, and pressure reading
typedef struct {
  double humid_relh;  // % relative humidity
  double temp_degc;   // degrees Centigrade
  double press_hpa;   // hPa (hecto Pascal)
} reading_t;

// raw humidity, temperature, and pressure reading
typedef struct {
  int32_t raw_humid;
  int32_t raw_temp;
  int32_t raw_press;
} raw_reading_t;


// calibration data
typedef struct
{
  uint16_t  dig_T1;
  int16_t   dig_T2;
  int16_t   dig_T3;
  uint16_t  dig_P1;
  int16_t   dig_P2;
  int16_t   dig_P3;
  int16_t   dig_P4;
  int16_t   dig_P5;
  int16_t   dig_P6;
  int16_t   dig_P7;
  int16_t   dig_P8;
  int16_t   dig_P9;
  uint8_t   dig_H1;
  int32_t   dig_H2;
  uint8_t   dig_H3;
  int16_t   dig_H4;
  int16_t   dig_H5;
  uint8_t   dig_H6;
} comp_table_t;

int open_device();
int close_device();
int reset_device(int fd);
int configure_device(int fd);
int read_compensation_table(int fd, comp_table_t *table);
int sample_humid_temp_press(int fd, raw_reading_t *reading);
void compensate_reading(raw_reading_t *raw, reading_t *comp_out,
                        comp_table_t *table);

void print_status_reg(int fd);

int spinwait_until_ready(int fd);



#endif  // BME280_H
