#include "input.h"

#include "SDL/SDL.h"
#include "input_latency.h"
#include "motion.h"
#include <math.h>

int ir_up, ir_down, ir_left, ir_right, steer_left, steer_right, nunchuk_up,
    nunchuk_down, nunchuk_left, nunchuk_right, classic_left_stick_up,
    classic_left_stick_down, classic_left_stick_left, classic_left_stick_right,
    motionplus_up, motionplus_down, motionplus_left, motionplus_right,
    motionplus_slow;
extern int show_reports;

static const double pointer_margin = 0.5;
float pointer_x = 0.5;
float pointer_y = 0.5;
static const uint16_t accelerometer_zero = 0x85 << 2;
static const uint16_t accelerometer_unit = 0x6C;
struct timeval pending_ir_ts = {0, 0};
struct timeval pending_accel_ts = {0, 0};
struct timeval pending_button_ts = {0, 0};

int input_update(struct wiimote_state *state,
                 struct input_source const *source) {
  struct input_event event;

  float pointer_delta_x = 0, pointer_delta_y = 0;

  /* Loop through waiting messages and process them */

  while (source->poll_event(&event)) {
    switch (event.type) {
    case INPUT_EVENT_TYPE_EMULATOR_CONTROL:
      switch (event.emulator_control_event.control) {
      case INPUT_EMULATOR_CONTROL_QUIT:
        return -1;
      case INPUT_EMULATOR_CONTROL_POWER_OFF:
        return -2;
      case INPUT_EMULATOR_CONTROL_TOGGLE_REPORTS:
        show_reports = (show_reports + 1) % 2;
        break;
      }
      break;
    case INPUT_EVENT_TYPE_HOTPLUG:
      switch (event.hotplug_event.extension) {
      case Nunchuk:
        reset_input_nunchuk(&state->usr.nunchuk);
        reset_input_ir(state->usr.ir_object);
        break;
      case Classic:
        reset_input_classic(&state->usr.classic);
        reset_input_ir(state->usr.ir_object);
        break;
      case BalanceBoard:
        reset_input_ir(state->usr.ir_object);
        break;
      case NoExtension:
        reset_input_ir(state->usr.ir_object);
        pointer_x = 0.5;
        pointer_y = 0.5;
        break;
      default:
        goto invalid;
      }

      state->usr.connected_extension_type = event.hotplug_event.extension;
    invalid:
      break;
    case INPUT_EVENT_TYPE_BUTTON: {
      pending_button_ts = event.ts;
      bool pressed = event.button_event.pressed;
      switch (event.button_event.button) {
      case INPUT_BUTTON_HOME:
        state->usr.home = pressed;
        break;

      case INPUT_BUTTON_WIIMOTE_UP:
        state->usr.up = pressed;
        break;
      case INPUT_BUTTON_WIIMOTE_DOWN:
        state->usr.down = pressed;
        break;
      case INPUT_BUTTON_WIIMOTE_LEFT:
        state->usr.left = pressed;
        break;
      case INPUT_BUTTON_WIIMOTE_RIGHT:
        state->usr.right = pressed;
        break;
      case INPUT_BUTTON_WIIMOTE_A:
        state->usr.a = pressed;
        break;
      case INPUT_BUTTON_WIIMOTE_B:
        state->usr.b = pressed;
        break;
      case INPUT_BUTTON_WIIMOTE_1:
        state->usr.one = pressed;
        break;
      case INPUT_BUTTON_WIIMOTE_2:
        state->usr.two = pressed;
        break;
      case INPUT_BUTTON_WIIMOTE_PLUS:
        state->usr.plus = pressed;
        break;
      case INPUT_BUTTON_WIIMOTE_MINUS:
        state->usr.minus = pressed;
        break;

      case INPUT_BUTTON_NUNCHUK_C:
        state->usr.nunchuk.c = pressed;
        break;
      case INPUT_BUTTON_NUNCHUK_Z:
        state->usr.nunchuk.z = pressed;
        break;

      case INPUT_BUTTON_CLASSIC_UP:
        state->usr.classic.up = pressed;
        break;
      case INPUT_BUTTON_CLASSIC_DOWN:
        state->usr.classic.down = pressed;
        break;
      case INPUT_BUTTON_CLASSIC_LEFT:
        state->usr.classic.left = pressed;
        break;
      case INPUT_BUTTON_CLASSIC_RIGHT:
        state->usr.classic.right = pressed;
        break;
      case INPUT_BUTTON_CLASSIC_A:
        state->usr.classic.a = pressed;
        break;
      case INPUT_BUTTON_CLASSIC_B:
        state->usr.classic.b = pressed;
        break;
      case INPUT_BUTTON_CLASSIC_X:
        state->usr.classic.x = pressed;
        break;
      case INPUT_BUTTON_CLASSIC_Y:
        state->usr.classic.y = pressed;
        break;
      case INPUT_BUTTON_CLASSIC_L:
        state->usr.classic.ltrigger = pressed;
        break;
      case INPUT_BUTTON_CLASSIC_R:
        state->usr.classic.rtrigger = pressed;
        break;
      case INPUT_BUTTON_CLASSIC_ZL:
        state->usr.classic.lz = pressed;
        break;
      case INPUT_BUTTON_CLASSIC_ZR:
        state->usr.classic.rz = pressed;
        break;
      case INPUT_BUTTON_CLASSIC_PLUS:
        state->usr.classic.plus = pressed;
        break;
      case INPUT_BUTTON_CLASSIC_MINUS:
        state->usr.classic.minus = pressed;
        break;
      default:
        printf("warning: button %d not handled by input_update\n",
               event.button_event.button);
        break;
      }
      break;
    }
    case INPUT_EVENT_TYPE_ANALOG_MOTION: {
      bool moving = event.analog_motion_event.moving;
      switch (event.analog_motion_event.motion) {
      case INPUT_ANALOG_MOTION_POINTER:
        pointer_delta_x = event.analog_motion_event.delta_x;
        pointer_delta_y = event.analog_motion_event.delta_y;
        /* printf("pointer: %f %f\n", event.analog_motion_event.x, */
        /*        event.analog_motion_event.y); */
        /* pointer_x = event.analog_motion_event.x; */
        /* pointer_y = event.analog_motion_event.y; */
        break;
      case INPUT_ANALOG_MOTION_IR_UP:
        ir_up = moving;
        break;
      case INPUT_ANALOG_MOTION_IR_DOWN:
        ir_down = moving;
        break;
      case INPUT_ANALOG_MOTION_IR_LEFT:
        ir_left = moving;
        break;
      case INPUT_ANALOG_MOTION_IR_RIGHT:
        ir_right = moving;
        break;
      case INPUT_ANALOG_MOTION_IR_RAW: {
        /*
          Use the received IR values to update the wiimote’s IR object.
          For example, assume the IR x and y are normalized in [0,1] and z
          represents an intensity or size.
        */
        /* printf("IR RAW: %f %f\n", event.analog_motion_event.x, */
        /*        event.analog_motion_event.y); */
        /* state->usr.ir_object[0].x = round(event.analog_motion_event.x *
         * 1023); */
        /* state->usr.ir_object[0].y = round(event.analog_motion_event.y * 767);
         */
        pending_ir_ts = event.ts;
        pointer_x = event.analog_motion_event.x;
        pointer_y = event.analog_motion_event.y;
        /* Map the IR z value to a size between, say, 1 and 15.
          (Adjust this mapping to match your device’s characteristics.) */
        /* state->usr.ir_object[0].size = round(1.0 +
         * event.analog_motion_event.z * 14); */
        break;
      }

      case INPUT_ANALOG_MOTION_ACCEL: {
        /* printf("ACCEL: %f %f %f\n", event.analog_motion_event.x, */
        /*        event.analog_motion_event.y, event.analog_motion_event.z); */

        /* event.analog_motion_event.x = */
        /*     fmax(-3.4, fmin(3.4, event.analog_motion_event.x)); */
        /* event.analog_motion_event.y = */
        /*     fmax(-3.4, fmin(3.4, event.analog_motion_event.y)); */
        /* event.analog_motion_event.z = */
        /*     fmax(-3.4, fmin(3.4, event.analog_motion_event.z)); */

        /* state->usr.accel_x = */
        /*     accelerometer_zero + */
        /*     (int)round(accelerometer_unit * -event.analog_motion_event.x); */
        /* state->usr.accel_y = */
        /*     accelerometer_zero + */
        /*     (int)round(accelerometer_unit * event.analog_motion_event.z); */
        /* state->usr.accel_z = */
        /*     accelerometer_zero + */
        /*     (int)round(accelerometer_unit * -event.analog_motion_event.y); */

        pending_accel_ts = event.ts;
        state->usr.accel_x = event.analog_motion_event.x;
        state->usr.accel_y = event.analog_motion_event.y;
        state->usr.accel_z = event.analog_motion_event.z;

        /* printf("ACCEL: %d %d %d\n", state->usr.accel_x, state->usr.accel_y,
         */
        /*        state->usr.accel_z); */

        break;
      }

      case INPUT_ANALOG_MOTION_STEER_LEFT:
        steer_left = moving;
        break;
      case INPUT_ANALOG_MOTION_STEER_RIGHT:
        steer_right = moving;
        break;

      case INPUT_ANALOG_MOTION_NUNCHUK_UP:
        nunchuk_up = moving;
        break;
      case INPUT_ANALOG_MOTION_NUNCHUK_DOWN:
        nunchuk_down = moving;
        break;
      case INPUT_ANALOG_MOTION_NUNCHUK_LEFT:
        nunchuk_left = moving;
        break;
      case INPUT_ANALOG_MOTION_NUNCHUK_RIGHT:
        nunchuk_right = moving;
        break;

      case INPUT_ANALOG_MOTION_CLASSIC_LEFT_STICK_UP:
        classic_left_stick_up = moving;
        break;
      case INPUT_ANALOG_MOTION_CLASSIC_LEFT_STICK_DOWN:
        classic_left_stick_down = moving;
        break;
      case INPUT_ANALOG_MOTION_CLASSIC_LEFT_STICK_LEFT:
        classic_left_stick_left = moving;
        break;
      case INPUT_ANALOG_MOTION_CLASSIC_LEFT_STICK_RIGHT:
        classic_left_stick_right = moving;
        break;

      case INPUT_ANALOG_MOTION_MOTIONPLUS_UP:
        motionplus_up = moving;
        break;
      case INPUT_ANALOG_MOTION_MOTIONPLUS_DOWN:
        motionplus_down = moving;
        break;
      case INPUT_ANALOG_MOTION_MOTIONPLUS_LEFT:
        motionplus_left = moving;
        break;
      case INPUT_ANALOG_MOTION_MOTIONPLUS_RIGHT:
        motionplus_right = moving;
        break;
      case INPUT_ANALOG_MOTION_MOTIONPLUS_SLOW:
        motionplus_slow = moving;
        break;
      }
      break;
    }
    default:
      break;
    }
  }

  pointer_delta_x += ir_right * 0.004 - ir_left * 0.004;
  pointer_delta_y += ir_up * 0.004 - ir_down * 0.004;

  pointer_x = fmax(-pointer_margin,
                   fmin(1.0 + pointer_margin, pointer_x + pointer_delta_x));
  pointer_y = fmax(-pointer_margin,
                   fmin(1.0 + pointer_margin, pointer_y + pointer_delta_y));

  set_motion_state(state, pointer_x, pointer_y);
  /* set_exact_pointer_state(state, pointer_x, pointer_y); */

  state->usr.nunchuk.x = 128 + nunchuk_right * 100 - nunchuk_left * 100;
  state->usr.nunchuk.y = 128 + nunchuk_up * 100 - nunchuk_down * 100;

  state->usr.classic.ls_x =
      32 + classic_left_stick_right * 30 - classic_left_stick_left * 30;
  state->usr.classic.ls_y =
      32 + classic_left_stick_up * 30 - classic_left_stick_down * 30;

  state->usr.motionplus.pitch_left =
      0x1F7F + motionplus_down * 800 * (1 + !motionplus_slow) -
      motionplus_up * 800 * (1 + !motionplus_slow);
  state->usr.motionplus.yaw_down =
      0x1F7F + motionplus_left * 800 * (1 + !motionplus_slow) -
      motionplus_right * 800 * (1 + !motionplus_slow);
  state->usr.motionplus.pitch_slow = motionplus_slow;
  state->usr.motionplus.yaw_slow = motionplus_slow;

  return 0;
}
