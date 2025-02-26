#include "input_socket.h"
#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#define PROGRAM_NAME "wmemulator"

static bool input_socket_init_from_addrinfo(struct addrinfo *addrinfo);

static int sock;
static char buf[512];
static size_t buf_len;

/* Helper function to convert a 32-bit network order float to host float */
static float ntohf(uint32_t net) {
  uint32_t host = ntohl(net);
  float f;
  memcpy(&f, &host, sizeof(f));
  return f;
}

void input_socket_init_unix_at_path(char const *path) {
  struct sockaddr_un address = {.sun_family = AF_UNIX};
  strncpy(address.sun_path, path, sizeof address.sun_path);

  unlink(path);
  input_socket_init((struct sockaddr *)&address, sizeof address);
}

void input_socket_init_ip_on_port(char const *port) {
  struct addrinfo hints = {.ai_family = AF_UNSPEC,
                           .ai_socktype = SOCK_DGRAM,
                           .ai_flags = AI_PASSIVE};
  struct addrinfo *result_info;
  int ret = getaddrinfo(NULL, port, &hints, &result_info);
  if (ret) {
    printf(PROGRAM_NAME ": getaddrinfo: %s\n", gai_strerror(ret));
    exit(1);
  }

  for (struct addrinfo *info = result_info; info; info = info->ai_next) {
    if (input_socket_init_from_addrinfo(info)) {
      freeaddrinfo(result_info);
      printf(PROGRAM_NAME ": successfully bound to port %s\n", port);
      return;
    }
  }

  printf(PROGRAM_NAME ": fatal: can't bind to port %s\n", port);
  exit(1);
}

void input_socket_init(struct sockaddr *socket_address,
                       socklen_t socket_address_size) {
  sock = socket(socket_address->sa_family, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  if (sock == -1) {
    perror(PROGRAM_NAME);
    exit(1);
  }

  if (bind(sock, socket_address, socket_address_size)) {
    perror(PROGRAM_NAME);
    exit(1);
  }
}

static bool input_socket_init_from_addrinfo(struct addrinfo *addrinfo) {
  sock = socket(addrinfo->ai_family, addrinfo->ai_socktype | SOCK_NONBLOCK,
                addrinfo->ai_protocol);
  if (sock == -1) {
    return false;
  }

  if (bind(sock, addrinfo->ai_addr, addrinfo->ai_addrlen)) {
    close(sock);
    return false;
  }

  return true;
}

static void input_socket_unload(void) {
  if (close(sock)) {
    perror(PROGRAM_NAME);
  }
}

static bool input_socket_poll_event(struct input_event *event) {
  if (!buf_len) {
    buf_len = recv(sock, buf, sizeof(buf), 0);
    if (buf_len == -1) {
      buf_len = 0;
      if (!(errno == EAGAIN || errno == EWOULDBLOCK)) {
        perror(PROGRAM_NAME);
      }
      return false;
    }
  }

  /*
   * Check for a binary packet:
   * Our new binary pointer update packet is defined as:
   *   [1 byte event type] + [4 bytes float x] + [4 bytes float y] = 9 bytes
   * total. If the packet length is >= 9 and the first byte is 0x01, we assume
   * it’s the binary pointer update.
   */
  if (buf_len >= 9 && ((unsigned char)buf[0]) == 0x01) {
    uint32_t net_x, net_y;
    memcpy(&net_x, buf + 1, 4);
    memcpy(&net_y, buf + 5, 4);
    float x = ntohf(net_x);
    float y = ntohf(net_y);

    event->type = INPUT_EVENT_TYPE_ANALOG_MOTION;
    event->analog_motion_event.moving = 1;
    event->analog_motion_event.motion = INPUT_ANALOG_MOTION_POINTER;
    event->analog_motion_event.delta_x = x;
    event->analog_motion_event.delta_y = y;

    buf_len = 0;
    return true;
  } else {
    /* Fallback to text-based protocol parsing */
    buf[buf_len] = '\0';
    event->type = INPUT_EVENT_TYPE_BUTTON;

    char event_type_s[32], event_param_s[32];
    int event_status;
    if (sscanf(buf, "%32s %d %32s", event_type_s, &event_status,
               event_param_s) == EOF) {
      printf(PROGRAM_NAME ": received input in invalid format\n");
      buf_len = 0;
      return false;
    }

    if (strcmp(event_type_s, "emulator_control") == 0) {
      event->type = INPUT_EVENT_TYPE_EMULATOR_CONTROL;

      if (strcmp(event_param_s, "quit") == 0) {
        event->emulator_control_event.control = INPUT_EMULATOR_CONTROL_QUIT;
      } else if (strcmp(event_param_s, "power_off") == 0) {
        event->emulator_control_event.control =
            INPUT_EMULATOR_CONTROL_POWER_OFF;
      }
    } else if (strcmp(event_type_s, "hotplug") == 0) {
      event->type = INPUT_EVENT_TYPE_HOTPLUG;

      if (event_status == 0) {
        event->hotplug_event.extension = NoExtension;
      } else if (strcmp(event_param_s, "nunchuk") == 0) {
        event->hotplug_event.extension = Nunchuk;
      } else if (strcmp(event_param_s, "classic") == 0) {
        event->hotplug_event.extension = Classic;
      } else if (strcmp(event_param_s, "balance_board") == 0) {
        event->hotplug_event.extension = BalanceBoard;
      } else {
        event->hotplug_event.extension = NoExtension;
      }
    } else if (strcmp(event_type_s, "button") == 0) {
      event->type = INPUT_EVENT_TYPE_BUTTON;
      event->button_event.pressed = event_status;

#define CHECK(Button)                                                          \
  if (strcmp(event_param_s, #Button) == 0)                                     \
  event->button_event.button = INPUT_BUTTON_##Button
      CHECK(HOME);
      else CHECK(WIIMOTE_UP);
      else CHECK(WIIMOTE_DOWN);
      else CHECK(WIIMOTE_LEFT);
      else CHECK(WIIMOTE_RIGHT);
      else CHECK(WIIMOTE_A);
      else CHECK(WIIMOTE_B);
      else CHECK(WIIMOTE_1);
      else CHECK(WIIMOTE_2);
      else CHECK(WIIMOTE_PLUS);
      else CHECK(WIIMOTE_MINUS);
      else CHECK(NUNCHUK_C);
      else CHECK(NUNCHUK_Z);
      else CHECK(CLASSIC_UP);
      else CHECK(CLASSIC_DOWN);
      else CHECK(CLASSIC_LEFT);
      else CHECK(CLASSIC_RIGHT);
      else CHECK(CLASSIC_A);
      else CHECK(CLASSIC_B);
      else CHECK(CLASSIC_X);
      else CHECK(CLASSIC_Y);
      else CHECK(CLASSIC_L);
      else CHECK(CLASSIC_R);
      else CHECK(CLASSIC_ZL);
      else CHECK(CLASSIC_ZR);
      else CHECK(CLASSIC_PLUS);
      else CHECK(CLASSIC_MINUS);
#undef CHECK
      else {
        printf(PROGRAM_NAME ": received invalid 'button' parameter: %s\n",
               event_param_s);
        buf_len = 0;
        return false;
      }
    } else if (strcmp(event_type_s, "analog_motion") == 0) {
      event->type = INPUT_EVENT_TYPE_ANALOG_MOTION;
      event->analog_motion_event.moving = event_status;

#define CHECK(Motion)                                                          \
  if (strcmp(event_param_s, #Motion) == 0)                                     \
  event->analog_motion_event.motion = INPUT_ANALOG_MOTION_##Motion
      CHECK(IR_UP);
      else CHECK(IR_DOWN);
      else CHECK(IR_LEFT);
      else CHECK(IR_RIGHT);
      else CHECK(STEER_LEFT);
      else CHECK(STEER_RIGHT);
      else CHECK(NUNCHUK_UP);
      else CHECK(NUNCHUK_DOWN);
      else CHECK(NUNCHUK_LEFT);
      else CHECK(NUNCHUK_RIGHT);
      else CHECK(CLASSIC_LEFT_STICK_UP);
      else CHECK(CLASSIC_LEFT_STICK_DOWN);
      else CHECK(CLASSIC_LEFT_STICK_LEFT);
      else CHECK(CLASSIC_LEFT_STICK_RIGHT);
      else CHECK(MOTIONPLUS_UP);
      else CHECK(MOTIONPLUS_DOWN);
      else CHECK(MOTIONPLUS_LEFT);
      else CHECK(MOTIONPLUS_RIGHT);
      else CHECK(MOTIONPLUS_SLOW);
#undef CHECK
      else {
        printf(PROGRAM_NAME
               ": received invalid 'analog_motion' parameter: %s\n",
               event_param_s);
        buf_len = 0;
        return false;
      }
    } else {
      printf(PROGRAM_NAME ": received invalid event type: %s\n", event_type_s);
      buf_len = 0;
      return false;
    }
    buf_len = 0;
    return true;
  }
}

struct input_source input_source_socket = {
    .unload = input_socket_unload, .poll_event = input_socket_poll_event};
