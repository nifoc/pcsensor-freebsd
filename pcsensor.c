/*
 * pcsensor.c by Juan Carlos Perez (c) 2011 (cray@isp-sl.com)
 * based on Temper.c by Robert Kavaler (c) 2009 (relavak.com)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 * THIS SOFTWARE IS PROVIDED BY Juan Carlos Perez ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Robert kavaler BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <usb.h>
#include <time.h>

#define VERSION "1.0.3"

#define VENDOR_ID 0x0c45
#define PRODUCT_ID 0x7401

#define INTERFACE1 0x00
#define INTERFACE2 0x01

#define MAX_DEV 8

#define reqIntLen 8
#define reqBulkLen 8
#define endpoint_Int_in 0x82   /* endpoint 0x81 address for IN */
#define endpoint_Int_out 0x00  /* endpoint 1 address for OUT */
#define endpoint_Bulk_in 0x82  /* endpoint 0x81 address for IN */
#define endpoint_Bulk_out 0x00 /* endpoint 1 address for OUT */
#define timeout 1800           /* timeout in ms */

enum {
  uTemperatura1,
  uTemperatura2,
  uTemperatura3,
  uIni1,
  uIni2,
};

struct str_ques {
  unsigned char data[reqIntLen];
  const char *disc;
};

static struct str_ques ques[8] = {
    {
        {0x01, 0x80, 0x33, 0x01, 0x00, 0x00, 0x00, 0x00}, "uTemperatura1",
    },
    {
        {0x01, 0x80, 0x33, 0x01, 0x00, 0x00, 0x00, 0x00}, "uTemperatura2",
    },
    {
        {0x01, 0x80, 0x33, 0x01, 0x00, 0x00, 0x00, 0x00}, "uTemperatura3",
    },
    {
        {0x01, 0x82, 0x77, 0x01, 0x00, 0x00, 0x00, 0x00}, "uIni1",
    },
    {
        {0x01, 0x86, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00}, "uIni2",
    },
};

static int bsalir = 1;
static int debug = 0;
static int seconds = 5;
static int formato = 0;
static int mrtg = 0;
static int calibration = 0;
static int devlist = 0;
static int devnum = -1;

static usb_dev_handle *handles[MAX_DEV];
static char *devlist_bus[MAX_DEV];
static char *devlist_device[MAX_DEV];

#define dbg(...)                                                               \
  do {                                                                         \
    if (debug) {                                                               \
      printf(__VA_ARGS__);                                                     \
    }                                                                          \
  } while (0)

static void usb_detach(usb_dev_handle *lvr_winusb, int iInterface) {
  int ret;
  ret = usb_detach_kernel_driver_np(lvr_winusb, iInterface);
  if (ret) {
    printf("Detach failed\n");
  }
}

static int find_lvr_winusb() {
  struct usb_bus *bus;
  struct usb_device *dev;
  int i;

  memset(handles, 0, sizeof(handles));
  i = 0;

  for (bus = usb_busses; bus; bus = bus->next) {
    for (dev = bus->devices; dev; dev = dev->next) {
      if (dev->descriptor.idVendor == VENDOR_ID &&
          dev->descriptor.idProduct == PRODUCT_ID) {
        usb_dev_handle *handle;
        dbg("lvr_winusb with Vendor Id: %x and Product Id: %x found.%d\n",
            VENDOR_ID, PRODUCT_ID, i);

        if (!(handle = usb_open(dev))) {
          printf("Could not open USB device\n");
          continue;
        }
        handles[i] = handle;
        devlist_bus[i] = bus->dirname;
        devlist_device[i] = dev->filename;
        i++;
        if (i == MAX_DEV)
          break;
      }
    }
  }
  return i;
}

static int setup_libusb_access() {
  int i = 0;

  if (debug) {
    usb_set_debug(255);
  } else {
    usb_set_debug(0);
  }

  usb_init();
  usb_find_busses();
  usb_find_devices();

  if (!find_lvr_winusb()) {
    fprintf(stderr, "Couldn't find the USB device, Exiting\n");
    return 0;
  }

  for (i = 0; handles[i] != NULL && i < MAX_DEV; i++) {
    usb_detach(handles[i], INTERFACE1);
    usb_detach(handles[i], INTERFACE2);

    if (usb_set_configuration(handles[i], 0x01) < 0) {
      fprintf(stderr, "Could not set configuration 1 on device %d\n", i);
      return 0;
    }

    // Microdia tiene 2 interfaces
    if (usb_claim_interface(handles[i], INTERFACE1) < 0) {
      fprintf(stderr, "Could not claim interface\n");
      return 0;
    }

    if (usb_claim_interface(handles[i], INTERFACE2) < 0) {
      fprintf(stderr, "Could not claim interface\n");
      return 0;
    }
  }

  return i;
}

static void ini_control_transfer(usb_dev_handle *dev) {
  int r;
  char question[] = {0x01, 0x01};

  r = usb_control_msg(dev, 0x21, 0x09, 0x0201, 0x00, (char *)question,
                      sizeof(question), timeout);
  if (r < 0) {
    perror("Fatal error USB control write");
    exit(17);
  }
}

static void interrupt_read(usb_dev_handle *dev, const int question) {

  int r;
  unsigned char answer[reqIntLen] = {0};

  r = usb_interrupt_read(dev, 0x82, (char *)answer, reqIntLen,
                         /*timeout*/ reqIntLen * 10);
  if (r != reqIntLen) {
    if (ETIMEDOUT)
      fprintf(stderr, "timeout question %s\n", ques[question].disc);
    else
      fprintf(stderr, "Error = %d question %s\n", r, ques[question].disc);
    return;
  }
  return;
}

static void control_transfer(usb_dev_handle *dev, const int question) {
  int r;
  r = usb_control_msg(dev, 0x21, 0x09, 0x0200, 0x01,
                      (char *)ques[question].data, reqIntLen,
                      /*timeout*/ reqIntLen * 30);
  if (r < 0) {
    fprintf(stderr, "Fatal error USB control write %s\n", ques[question].disc);
    exit(17);
  }
  interrupt_read(dev, question);
}

static void interrupt_read_temperatura(usb_dev_handle *dev, float *tempC) {

  int r;
  int temperature;
  unsigned char answer[reqIntLen] = {0};

  r = usb_interrupt_read(dev, 0x82, (char *)answer, reqIntLen, timeout);
  if (r != reqIntLen) {
    fprintf(stderr, "Fatal error usb_interrupt_read %s\n", "temperatura");
    exit(17);
  }

  temperature = (answer[3] & 0xFF) + ((signed char)answer[2] << 8);
  temperature += calibration;
  *tempC = temperature * (125.0 / 32000.0);
}

void ex_program(int sig) {
  bsalir = 1;

  (void)signal(SIGINT, SIG_DFL);
}

int main(int argc, char **argv) {
  float tempc;
  int c, i;
  struct tm *local;
  time_t t;

  memset(handles, 0, sizeof(handles));
  while ((c = getopt(argc, argv, "mfcvhl::a:dD::")) != -1)
    switch (c) {
    case 'v':
      debug = 1;
      break;
    case 'c':
      formato = formato + 1; // Celsius
      break;
    case 'f':
      formato = formato + 10; // Fahrenheit
      break;
    case 'm':
      mrtg = 1;
      break;
    case 'd':
      devlist = 1;
      break;
    case 'D':
      if (optarg != NULL) {
        if (!sscanf(optarg, "%i", &devnum) == 1) {
          fprintf(stderr, "Error: '%s' is not numeric.\n", optarg);
          exit(EXIT_FAILURE);
        }
      } else {
        devnum = -10;
      }
      break;
    case 'l':
      if (optarg != NULL) {
        if (!sscanf(optarg, "%i", &seconds) == 1) {
          fprintf(stderr, "Error: '%s' is not numeric.\n", optarg);
          exit(EXIT_FAILURE);
        } else {
          bsalir = 0;
          break;
        }
      } else {
        bsalir = 0;
        seconds = 5;
        break;
      }
    case 'a':
      if (!sscanf(optarg, "%i", &calibration) == 1) {
        fprintf(stderr, "Error: '%s' is not numeric.\n", optarg);
        exit(EXIT_FAILURE);
      } else {
        break;
      }
    case '?':
    case 'h':
      printf("pcsensor version %s\n", VERSION);
      printf("      Aviable options:\n");
      printf("          -h help\n");
      printf("          -v verbose\n");
      printf("          -l[n] loop every 'n' seconds, default value is 5s\n");
      printf("          -c output only in Celsius\n");
      printf("          -f output only in Fahrenheit\n");
      printf("          -a[n] increase or decrease temperature in 'n' degrees "
             "for device calibration\n");
      printf("          -m output for mrtg integration\n");
      printf("          -d output with Bus and Device number\n");
      printf("          -D display device list\n");
      printf("          -D[n] specific device number\n");

      exit(EXIT_FAILURE);
    default:
      fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
      exit(EXIT_FAILURE);
    }

  if (optind < argc) {
    fprintf(stderr, "Non-option ARGV-elements, try -h for help.\n");
    exit(EXIT_FAILURE);
  }

  if (setup_libusb_access() == 0) {
    exit(EXIT_FAILURE);
  }

  (void)signal(SIGINT, ex_program);

  for (i = 0; handles[i] != NULL && i < MAX_DEV; i++) {
    if (devnum == -10) {
      printf("%i is Bus %s Device %s \n", i, devlist_bus[i], devlist_device[i]);
    } else if (i == devnum || devnum == -1) {

      ini_control_transfer(handles[i]);
      //		control_transfer(handles[i], uTemperatura1 );
      control_transfer(handles[i], uIni1);
      control_transfer(handles[i], uIni2);
      control_transfer(handles[i], uTemperatura2);
      interrupt_read_temperatura(handles[i], &tempc);

      t = time(NULL);
      local = localtime(&t);
      if (mrtg) {
        if (formato >= 10) {
          printf("%.2f\n", (9.0 / 5.0 * tempc + 32.0));
          printf("%.2f\n", (9.0 / 5.0 * tempc + 32.0));
        } else {
          printf("%.2f\n", tempc);
          printf("%.2f\n", tempc);
        }

        printf("%02d:%02d\n", local->tm_hour, local->tm_min);

        printf("pcsensor\n");
      } else {
        printf("%04d/%02d/%02d %02d:%02d:%02d ", local->tm_year + 1900,
               local->tm_mon + 1, local->tm_mday, local->tm_hour, local->tm_min,
               local->tm_sec);

        if (devlist > 0) {
          printf("Bus %s Device %s ", devlist_bus[i], devlist_device[i]);
        }
        printf("Temperature");
        if (formato >= 10 || formato == 0) {
          printf(" %.2fF", (9.0 / 5.0 * tempc + 32.0));
        }
        if ((formato % 10) == 1 || formato == 0) {
          printf(" %.2fC", tempc);
        }
        printf("\n");
      }

      usb_release_interface(handles[i], INTERFACE1);
      usb_release_interface(handles[i], INTERFACE2);

      usb_close(handles[i]);
    }
  }

  return 0;
}
