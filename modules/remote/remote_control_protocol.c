#include "remote_control_protocol.h"

#include <stdlib.h>

#define REMOTE_CHANNEL_LIMIT 660
#define REMOTE_SWITCH_UP 1
#define REMOTE_SWITCH_DOWN 2
#define REMOTE_SWITCH_MID 3

static uint8_t SwitchValid(uint8_t value)
{
    return value == REMOTE_SWITCH_UP ||
           value == REMOTE_SWITCH_DOWN ||
           value == REMOTE_SWITCH_MID;
}

uint8_t RemoteControlValuesValid(const int16_t channels[5],
                                 uint8_t switch_left,
                                 uint8_t switch_right)
{
    if (channels == NULL || !SwitchValid(switch_left) || !SwitchValid(switch_right))
        return 0;
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(channels[i]) > REMOTE_CHANNEL_LIMIT)
            return 0;
    return 1;
}
