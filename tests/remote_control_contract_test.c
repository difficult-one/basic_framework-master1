#include <assert.h>
#include <stdio.h>

#include "remote_control.h"
#include "remote_control_protocol.h"

int main(void)
{
    RC_ctrl_t controls[2] = {0};
    controls[TEMP].key[KEY_PRESS].w = 1;
    controls[TEMP].key[KEY_PRESS].d = 1;

    assert(controls[TEMP].key[KEY_STATE].w == 1);
    assert(controls[TEMP].key[KEY_STATE].d == 1);

    const int16_t centered_channels[5] = {0, 0, 0, 0, 0};
    const int16_t invalid_channel[5] = {0, 0, 661, 0, 0};
    assert(RemoteControlValuesValid(centered_channels, RC_SW_UP, RC_SW_MID) == 1);
    assert(RemoteControlValuesValid(invalid_channel, RC_SW_UP, RC_SW_MID) == 0);
    assert(RemoteControlValuesValid(centered_channels, 0, RC_SW_MID) == 0);
    assert(RemoteControlValuesValid(centered_channels, RC_SW_UP, 0) == 0);
    puts("remote control contract tests passed");
    return 0;
}
