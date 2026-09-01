#ifndef REMOTE_CONTROL_PROTOCOL_H
#define REMOTE_CONTROL_PROTOCOL_H

#include <stdint.h>

uint8_t RemoteControlValuesValid(const int16_t channels[5],
                                 uint8_t switch_left,
                                 uint8_t switch_right);

#endif
