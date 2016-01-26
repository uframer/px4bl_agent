#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>


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
}

int px4bl_get_device(px4bl_agent *agent) {
}

int px4bl_get_chip_erase(px4bl_agent *agent) {
}

int px4bl_prog_multi(px4bl_agent *agent, const unsigned char *data, const unsigned int length) {
}

int px4bl_get_crc(px4bl_agent *agent) {
}

int px4bl_get_otp(px4bl_agent *agent) {
}

int px4bl_get_sn(px4bl_agent *agent) {
}

int px4bl_get_chip(px4bl_agent *agent) {
}

int px4bl_set_delay(px4bl_agent *agent) {
}

int px4bl_get_chip_des(px4bl_agent *agent) {
}

int px4bl_boot(px4bl_agent *agent) {
}

/* higher level operations */
int px4bl_agent_connect(px4bl_agent *agent) {
    struct termios conf;

    tcgetattr(agent->fd, &conf);
    /* baud rate */
    cfsetispeed(&conf, agent->baud);
    cfsetospeed(&conf, agent->baud);
    /* no varify */
    conf.c_cflag &= ~PARENB;
    conf.c_cflag &= ~CSTOPB;
    conf.c_cflag &= ~CSIZE;
    conf.c_cflag |= ~CS8;
    /* non-cannonical */
    conf.c_lflag &= ~(ECHO | ICANON);
    /* 1 byte as a time, timeout is 10s */
    conf.c_cc[VMIN] = 1;
    conf.c_cc[VTIME] = 100;
    tcsetattr(agent->fd, TCANOW, &conf);

    for (int i = 0; i < 20; ++i) {
      int result;
      result = write(agent->fd, &PROTO_GET_SYNC, 1);
      result = write(agent->fd, &PROTO_EOC);
      tcflush(agent->fd);
      unsigned char input_byte;
      result = read(agent->fd, &input_byte, 1);
    }
}

typedef struct {
} px4bl_device_info;

int px4bl_agent_get_info(px4bl_agent *agent, px4bl_device_info* out) {
}

int px4bl_agent_erase(px4bl_agent *agent) {
}

int px4bl_agent_flash(px4bl_agent *agent) {
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

    if (-1 == (ret = px4bl_agent_connect(&agent))) {
        printf("Error: failed to connect to px4 bootloader(%x)", ret);
    }

cleanup:
    deinit_px4bl_agent(&agent);

    return 0;
}
