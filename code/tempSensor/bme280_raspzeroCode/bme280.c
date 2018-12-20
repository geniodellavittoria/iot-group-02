#include <stdio.h>
#include <unistd.h>
#include <wiringPiI2C.h>

#include "bme280.h"



static int read_reg8(int fd, int reg);
static int read_reg16_little_endian(int fd, int reg);
static int read_reg16_big_endian(int fd, int reg);
static int read_reg24_big_endian(int fd, int reg);
static int write_reg8(int fd, int reg, int data);



int open_device()
{
  // get open file descriptor to the device
  int fd = wiringPiI2CSetup(BME280_I2C_ADDR);
  if (fd == -1)
  {
    fprintf(stderr, "unable to open I2C device at address 0x%02x\n",
            BME280_I2C_ADDR);
    return fd;
  }

  // verify device ID to make sure we are talking to a BME280
  int id = wiringPiI2CReadReg8(fd, BME280_REG_ID);
  if (id != BME280_ID)
  {
    fprintf(stderr, "device at 0x%02x has ID 0x%02x but expected 0x%02x\n",
            BME280_I2C_ADDR, id, BME280_ID);
  }
  return fd;
}


int close_device(int fd)
{
  return close(fd);
}

int reset_device(int fd)
{
  return wiringPiI2CWriteReg8(fd, BME280_REG_RESET, BME280_RESET_MAGIC);
}


// Sets device into the following configuration:
// - DEVICE MODE:   Sleep Mode (from Sleep, Forced, Normal)
// - HUMIDITY:      oversampling x 1
// - PRESSURE:      oversampling x 1
// - TEMPAREUTRE:   oversampling x 1
// - t_standby:     0.05 ms
// - IIR Filter:    off
// - SPI Interface: off
int configure_device(int fd)
{
  // set config register
  // - t_standby:     0.05 ms
  // - IIR Filter:    off
  // - SPI Interface: off
  int res = write_reg8(fd, BME280_REG_CONFIG, 0x00);
  if (res == -1) {
    return res;
  }

  // set ctrl_hum register
  // note: changes get only become effective after a write to ctrl_meas!
  // - HUMIDITY:      oversampling x 1
  res = write_reg8(fd, BME280_REG_CTRL_HUM, OVERSAMPLING_HUMID);
  if (res == -1) {
    return res;
  }

  // set ctrl_meas register
  // - PRESSURE:      oversampling x 1 (0x01)
  // - TEMPAREUTRE:   oversampling x 1 (0x01)
  // - DEVICE MODE:   Sleep Mode 0x0 (from Sleep, Forced, Normal)
  //
  res = write_reg8(fd, BME280_REG_CTRL_MEAS,
      (OVERSAMPLING_TEMP<<5)|(OVERSAMPLING_PRESS<<2)|MODE_SLEEP);
  if (res == -1) {
    return res;
  }
  return res;
}


int sample_humid_temp_press(int fd, raw_reading_t *reading)
{
  // wait until device becomes ready
  if (spinwait_until_ready(fd) == -1)
  {
    return -1;
  }

  // force aquisition
  int res = write_reg8(fd, BME280_REG_CTRL_MEAS,
      (OVERSAMPLING_TEMP<<5)|(OVERSAMPLING_PRESS<<2)|MODE_FORCED);
  if (res == -1) {
    return -1;
  }
  // We need to wait a little bit before we start reading the status
  // register, otherwise, the device seems ot hang.
  usleep(ACQUISITION_DELAY_USEC);

  // wait until data acquisition is complete
  if (spinwait_until_ready(fd) == -1)
  {
    return -1;
  }

  int humid_reg = read_reg16_big_endian(fd, BME280_REG_HUM_MSB);
  int temp_reg = read_reg24_big_endian(fd, BME280_REG_TEMP_MSB);
  int press_reg = read_reg24_big_endian(fd, BME280_REG_PRESS_MSB);
  if ((humid_reg != -1) && (temp_reg != -1) && (press_reg != -1)) {
    reading->raw_humid = humid_reg;
    // temp: only upper 20 bits contain valid values
    reading->raw_temp = temp_reg >> 4;
    // humid: only upper 20 bits contain valid values
    reading->raw_press = press_reg >> 4;
    return 0;
  }
  else
  {
    return -1;
  }
}


int read_compensation_table(int fd, comp_table_t *table)
{
  int data = read_reg16_little_endian(fd, 0x88);
  if (data == -1)
    return data;
  table->dig_T1 = (uint16_t)(0xffff & data);

  data = read_reg16_little_endian(fd, 0x8a);
  if (data == -1)
    return data;
  table->dig_T2 = (int16_t)data;

  data = read_reg16_little_endian(fd, 0x8c);
  if (data == -1)
    return data;
  table->dig_T3 = (int16_t)data;

  data = read_reg16_little_endian(fd, 0x8e);
  if (data == -1)
    return data;
  table->dig_P1 = (uint16_t)(0xffff & data);

  data = read_reg16_little_endian(fd, 0x90);
  if (data == -1)
    return data;
  table->dig_P2 = (int16_t)data;

  data = read_reg16_little_endian(fd, 0x90);
  if (data == -1)
    return data;
  table->dig_P2 = (int16_t)data;

  data = read_reg16_little_endian(fd, 0x92);
  if (data == -1)
    return data;
  table->dig_P3 = (int16_t)data;

  data = read_reg16_little_endian(fd, 0x94);
  if (data == -1)
    return data;
  table->dig_P4 = (int16_t)data;

  data = read_reg16_little_endian(fd, 0x96);
  if (data == -1)
    return data;
  table->dig_P5 = (int16_t)data;

  data = read_reg16_little_endian(fd, 0x98);
  if (data == -1)
    return data;
  table->dig_P6 = (int16_t)data;

  data = read_reg16_little_endian(fd, 0x9a);
  if (data == -1)
    return data;
  table->dig_P7 = (int16_t)data;

  data = read_reg16_little_endian(fd, 0x9c);
  if (data == -1)
    return data;
  table->dig_P8 = (int16_t)data;

  data = read_reg16_little_endian(fd, 0x9e);
  if (data == -1)
    return data;
  table->dig_P9 = (int16_t)data;

  data = read_reg8(fd, 0xa1);
  if (data == -1)
    return data;
  table->dig_H1 = (uint8_t)(0xff & data);

  data = read_reg16_little_endian(fd, 0xe1);
  if (data == -1)
    return data;
  table->dig_H2 = (int16_t)data;

  data = read_reg8(fd, 0xe3);
  if (data == -1)
    return data;
  table->dig_H3 = (uint8_t)(0xff & data);

  // dig_H4 and dig_H4 require some special treatment
  int byte_e4 = read_reg8(fd, 0xe4);
  int byte_e5 = read_reg8(fd, 0xe5);
  int byte_e6 = read_reg8(fd, 0xe6);
  if (byte_e4 == -1 || byte_e5 == -1 || byte_e6 == -1)
  {
    return -1;
  }
  table->dig_H4 = (int16_t)(0xffff & (byte_e4 << 4)) | (int16_t)(0xf & byte_e5);
  table->dig_H5 = (int16_t)(0xffff & (byte_e6 << 4))
                | (int16_t)(0xf & (byte_e5 >> 4));

  data = read_reg8(fd, 0xe7);
  if (data == -1)
    return data;
  table->dig_H6 = (uint8_t)(0xff & data);

  return 0;
}


void print_status_reg(int fd)
{
  int status = read_reg8(fd, BME280_REG_STATUS);
  printf("status: 0x%02x (measuring:%d, im_update:%d)\n",
      status, (status & 0x4)>0, (status & 0x1)>0);
}


int spinwait_until_ready(int fd)
{
  int status;
  while (((status = read_reg8(fd, BME280_REG_STATUS)) != -1) && status != 0)
  {
    usleep(STATUS_REG_POLL_INTERVAL_USEC);
  }
  return status;
}


static int read_reg8(int fd, int reg)
{
  int data = wiringPiI2CReadReg8(fd, reg);
  if (data == -1)
  {
    fprintf(stderr, "error while reading 8 bytes from reg 0x%02x\n", reg);
  }
  return data;
}


static int read_reg16_little_endian(int fd, int reg)
{
  int data = read_reg8(fd, reg);
  if (data == -1)
  {
    return data;
  }
  int lo8 = data;
  data = read_reg8(fd, reg+1);
  if (data == -1)
  {
    return data;
  }
  int hi8 = data;
  return (hi8<<8) | lo8;
}


static int read_reg16_big_endian(int fd, int reg)
{
  int data = read_reg8(fd, reg);
  if (data == -1)
  {
    return data;
  }
  int hi8 = data;
  data = read_reg8(fd, reg+1);
  if (data == -1)
  {
    return data;
  }
  int lo8 = data;
  return (hi8<<8) | lo8;
}


static int read_reg24_big_endian(int fd, int reg)
{
  int data = read_reg8(fd, reg);
  if (data == -1)
  {
    return data;
  }
  int hi8 = data;
  data = read_reg8(fd, reg+1);
  if (data == -1)
  {
    return data;
  }
  int mid8 = data;
  data = read_reg8(fd, reg+1);
  if (data == -1)
  {
    return data;
  }
  int lo8 = data;
  return (hi8<<16) | (mid8<<8) | lo8;
}

static int write_reg8(int fd, int reg, int data)
{
  if (data < 0 || data > 0xff)
  {
    fprintf(stderr, "trying to write %d not within 0..255\n", data);
    return -1;
  }
  if (wiringPiI2CWriteReg8(fd, reg, data) == -1)
  {
    fprintf(stderr, "error while writing 8 bytes to reg 0x%02x\n", reg);
    return -1;
  }
  return 0;
}


// Conversion Functions from BME280 Data Sheet (BST-BME280-DS001-10, Rev 1.1)
// Appendix A: Alternative conversion formulas
// 8.1. Compensation formulas in double precision floating point (page 49)
//
// Returns temperature in DegC, double precision.
// Output value of "51.23" equals 51.23 degrees C.
// t_fine carries fine temperature as global value
static int32_t t_fine;
static double bme280_compensate_T_double(int32_t adc_T, const comp_table_t* table)
{
  double var1 = (((double)adc_T)/16384.0 - ((double)table->dig_T1)/1024.0)
              * ((double)table->dig_T2);
  double var2 = ((((double)adc_T)/131072.0 - ((double)table->dig_T1)/8192.0)
                * (((double)adc_T)/131072.0 - ((double)table->dig_T1)/8192.0))
              * ((double)table->dig_T3);
  t_fine = (int32_t)(var1 + var2);
  return (var1 + var2) / 5120.0;
}

// Returns pressure in hPa as double.
// Output value of "963.862" equals = 963.862 hPa
static double bme280_compensate_P_double(int32_t adc_P, const comp_table_t* table)
{
  double var1 = ((double)t_fine/2.0) - 64000.0;
  double var2 = var1 * var1 * ((double)table->dig_P6) / 32768.0;
  var2 = var2 + var1 * ((double)table->dig_P5) * 2.0;
  var2 = (var2/4.0)+(((double)table->dig_P4) * 65536.0);
  var1 = (((double)table->dig_P3) * var1 * var1 / 524288.0 +
      ((double)table->dig_P2) * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0)*((double)table->dig_P1);
  if (var1 == 0.0)
  {
    return 0; // avoid exception caused by division by zero
  }
  double p = 1048576.0 - (double)adc_P;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = ((double)table->dig_P9) * p * p / 2147483648.0;
  var2 = p * ((double)table->dig_P8) / 32768.0;
  p = p + (var1 + var2 + ((double)table->dig_P7)) / 16.0;
  return p * 0.01;
}

// Returns humidity in %rH as as double.
// Output value of "46.332" represents 46.332 %rH
static double bme280_compensate_H_double(int32_t adc_H, const comp_table_t* table)
{
  double var_H = (((double)t_fine) - 76800.0);
  var_H = (adc_H - (((double)table->dig_H4) * 64.0 +
             ((double)table->dig_H5) / 16384.0 * var_H))
        * (((double)table->dig_H2) / 65536.0
            * (1.0 + ((double)table->dig_H6) / 67108864.0 * var_H
              * (1.0 + ((double)table->dig_H3) / 67108864.0 * var_H)));
  var_H = var_H * (1.0 - ((double)table->dig_H1) * var_H / 524288.0);
  if (var_H > 100.0)
  {
    var_H = 100.0;
  }
  else if (var_H < 0.0)
  {
    var_H = 0.0;
  }
  return var_H;
}

void compensate_reading(raw_reading_t *raw, reading_t *comp_out,
                       comp_table_t *table)
{
  comp_out->temp_degc = bme280_compensate_T_double(raw->raw_temp, table);
  comp_out->press_hpa = bme280_compensate_P_double(raw->raw_press, table);
  comp_out->humid_relh = bme280_compensate_H_double(raw->raw_humid, table);
}
