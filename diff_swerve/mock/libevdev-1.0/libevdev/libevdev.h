#pragma once

#include <cstdint>
#include <sys/time.h>

constexpr unsigned EV_KEY = 0x01;
constexpr unsigned EV_ABS = 0x03;
constexpr unsigned ABS_X = 0x00;
constexpr unsigned ABS_Y = 0x01;
constexpr unsigned ABS_Z = 0x02;
constexpr unsigned ABS_RZ = 0x05;
constexpr unsigned BTN_TL = 0x136;
constexpr unsigned BTN_START = 0x13b;
enum libevdev_read_flag {
  LIBEVDEV_READ_FLAG_SYNC = 1,
  LIBEVDEV_READ_FLAG_NORMAL = 2,
  LIBEVDEV_READ_FLAG_FORCE_SYNC = 4,
  LIBEVDEV_READ_FLAG_BLOCKING = 8
};

enum libevdev_read_status {
  LIBEVDEV_READ_STATUS_SUCCESS = 0,
  LIBEVDEV_READ_STATUS_SYNC = 1
};

struct input_event {
  timeval time;
  uint16_t type;
  uint16_t code;
  int32_t value;
};

struct libevdev;

int libevdev_new_from_fd(int fd, libevdev **dev);
int libevdev_next_event(libevdev *dev, unsigned flags, input_event *ev);
int libevdev_get_fd(const libevdev *dev);
void libevdev_free(libevdev *dev);
