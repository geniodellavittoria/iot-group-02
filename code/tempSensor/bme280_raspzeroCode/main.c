#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPiI2C.h>

#include "bme280.h"

#define PRINT_VERBOSE(...) if (verbose) printf(__VA_ARGS__)

static void parse_command_line(int argc, char **argv);

static int verbose = 0;


int main(int argc, char **argv)
{
  parse_command_line(argc, argv);

  int fd = open_device();
  if (fd == -1)
  {
    exit(EXIT_FAILURE);
  }

  PRINT_VERBOSE("resetting device...\n");
  reset_device(fd);

  PRINT_VERBOSE("waiting for it to become ready...\n");
  spinwait_until_ready(fd);
  PRINT_VERBOSE("ready.\n");

  PRINT_VERBOSE("reading out compensation table from NVS...\n");
  comp_table_t comp_table;
  read_compensation_table(fd, &comp_table);

  PRINT_VERBOSE("configuring device...\n");
  configure_device(fd);

  PRINT_VERBOSE("starting data acquisition...\n");
  raw_reading_t raw;
  if (sample_humid_temp_press(fd, &raw) == -1)
  {
    exit(EXIT_FAILURE);
  }

  if (verbose)
  {
    // print raw readings
    printf("humid: %d (0x%08x)\n", raw.raw_humid, raw.raw_humid);
    printf("temp:  %d (0x%08x)\n", raw.raw_temp, raw.raw_temp);
    printf("press: %d (0x%08x)\n", raw.raw_press, raw.raw_press);
  }

  reading_t reading;
  compensate_reading(&raw, &reading, &comp_table);
  printf("humid: %.1f %%relH\n", reading.humid_relh);
  printf("temp: %.2f C\n", reading.temp_degc);
  printf("press: %.1f hPa\n", reading.press_hpa);

  close_device(fd);

  return 0;
}


void parse_command_line(int argc, char **argv)
{
  int opt;
  while ((opt = getopt(argc, argv, "hv")) != -1)
  {
    switch(opt)
    {
      case 'v':
        verbose = 1;
        break;
      default:
        fprintf(stderr, "usage: %s [-h|-v]\n", argv[0]);
        fprintf(stderr, "\t-h  show this help\n");
        fprintf(stderr, "\t-v  verbose output\n");
        exit(opt=='h'?EXIT_SUCCESS:EXIT_FAILURE);
    }
  }
}
