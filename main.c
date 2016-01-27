/**
 * gcc --std=c11 -Wall main.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

const unsigned char PROTO_INSYNC = 0X12;
const unsigned char PROTO_EOC = 0X20;

const unsigned char PROTO_OK = 0X10;
const unsigned char PROTO_FAILED = 0X11;
const unsigned char PROTO_INVALID = 0X13;
const unsigned char PROTO_BAD_SILICON_REV = 0X14;

const unsigned char PROTO_GET_SYNC = 0X21;
const unsigned char PROTO_GET_DEVICE = 0X22;
const unsigned char PROTO_CHIP_ERASE = 0X23;
const unsigned char PROTO_PROG_MULTI = 0X27;
const unsigned char PROTO_GET_CRC = 0X29;
const unsigned char PROTO_GET_OTP = 0X2a;
const unsigned char PROTO_SN = 0X2b;
const unsigned char PROTO_CHIP = 0X2c;
const unsigned char PROTO_SET_DELAY = 0X2d;
const unsigned char PROTO_GET_CHIP_DES = 0X2e;
const unsigned char PROTO_BOOT = 0X30;
const unsigned char PROTO_DEBUG = 0X31;

const unsigned int DEFAULT_OP_TIMEOUT = 10000; // 10s
const unsigned int DEFAULT_CONN_TIMEOUT = 120000; // 10s
const unsigned int CONNECT_RETRY_INTERVAL = 10; // 10ms

const int RESULT_OK = 0;
const int RESULT_TIMEOUT = -1;
const int RESULT_ERROR = -2;

typedef struct {
    int fd;
    int conn_timeout;
    int op_timeout;
    int baud;
} px4bl_agent;

void init_px4bl_agent(px4bl_agent *agent) {
    agent->fd = -1;
    agent->conn_timeout = DEFAULT_CONN_TIMEOUT;
    agent->op_timeout = DEFAULT_OP_TIMEOUT;
    agent->baud = B115200;
}

void deinit_px4bl_agent(px4bl_agent *agent) {
    if (agent->fd > 0) {
        close(agent->fd);
    }
}

/* Primitives */
int px4bl_get_sync(px4bl_agent *agent) {
  int result = -1;
  unsigned char input;
  if ((result = read(agent->fd, &input, 1)) < 0) {
    int error = errno;
    perror("read error");
    printf("result = %d, error = %d\n", result, error);
    return RESULT_TIMEOUT;
  }
  return RESULT_TIMEOUT;
}

int px4bl_get_device(px4bl_agent *agent) {
  return RESULT_ERROR;
}

int px4bl_get_chip_erase(px4bl_agent *agent) {
  return RESULT_ERROR;
}

int px4bl_prog_multi(px4bl_agent *agent, const unsigned char *data, const unsigned int length) {
  return RESULT_ERROR;
}

int px4bl_get_crc(px4bl_agent *agent) {
  return RESULT_ERROR;
}

int px4bl_get_otp(px4bl_agent *agent) {
  return RESULT_ERROR;
}

int px4bl_get_sn(px4bl_agent *agent) {
  return RESULT_ERROR;
}

int px4bl_get_chip(px4bl_agent *agent) {
  return RESULT_ERROR;
}

int px4bl_set_delay(px4bl_agent *agent) {
  return RESULT_ERROR;
}

int px4bl_get_chip_des(px4bl_agent *agent) {
  return RESULT_ERROR;
}

int px4bl_boot(px4bl_agent *agent) {
  return RESULT_ERROR;
}

/* higher level operations */
int px4bl_agent_connect(px4bl_agent *agent) {
    struct termios conf;

    if (-1 == tcgetattr(agent->fd, &conf)) {
      perror("tcgetattr");
      return RESULT_ERROR;
    }
    /* baud rate */
    cfsetispeed(&conf, agent->baud);
    cfsetospeed(&conf, agent->baud);
    /* no varify */
    conf.c_cflag &= ~PARENB; // no even/odd varification
    conf.c_cflag &= ~CSTOPB; // single stop bit
    conf.c_cflag &= ~CSIZE; // the following two line set CSIZE to CS8
    conf.c_cflag |= CS8;
    /* non-cannonical */
    conf.c_lflag &= ~(ECHO | ICANON);
    /* 1 byte at a time, timeout is 10s */
    conf.c_cc[VMIN] = 0;
    conf.c_cc[VTIME] = 10; // 10/10 = 1s
    if (-1 == tcsetattr(agent->fd, TCSANOW, &conf)) {
      perror("tcsetattr");
      return RESULT_ERROR;
    }

    for (int i = 0; i < 10; ++i) {
      printf("Try #%d ...\n", i);
      if (RESULT_OK == px4bl_get_sync(agent)) {
        return RESULT_OK;
      }
    }
    return RESULT_ERROR;
}

typedef struct {
} px4bl_device_info;

int px4bl_agent_get_info(px4bl_agent *agent, px4bl_device_info* out) {
  return RESULT_ERROR;
}

int px4bl_agent_erase(px4bl_agent *agent) {
  return RESULT_ERROR;
}

int px4bl_agent_flash(px4bl_agent *agent) {
  return RESULT_ERROR;
}

void print_usage(const char *prog_name) {
  printf("Usage:\n\t%s <serial dev file>\n", prog_name);
}

int main(int argc, char** argv) {
    px4bl_agent agent;
    char *serial_filename;
    int ret = -1;

    if (argc != 2) {
        print_usage(argv[0]);
        exit(1);
    }

    serial_filename = argv[1];

    init_px4bl_agent(&agent);

    if (-1 == (agent.fd = open(serial_filename, O_RDWR))) {
        printf("Error: failed to open serial %s\n", serial_filename);
        exit(1);
    }

    if (RESULT_OK != (ret = px4bl_agent_connect(&agent))) {
        printf("Error: failed to connect to px4 bootloader (%d)", ret);
    }

cleanup:
    deinit_px4bl_agent(&agent);

    return 0;
}
