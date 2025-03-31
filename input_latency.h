// input_latency.h
#ifndef INPUT_LATENCY_H
#define INPUT_LATENCY_H

#include <sys/time.h>

extern struct timeval pending_ir_ts;
extern struct timeval pending_accel_ts;
extern struct timeval pending_button_ts;

#endif // INPUT_LATENCY_H
