/* Adapted from workspace PS2_Get/All_Button/GetXY (ysl, 2021-12-24).
 * Same 01 42 00 polling, active-low button bytes and LX/LY at offsets 7/8.
 * Validate replies before exposing any button; digital mode has no axes. */
#include "car_ps2.h"
int PS2_Decode(const uint8_t *d, unsigned int n, PS2Data *out)
{
    unsigned int required;
    if (n < 3 || d[2] != 0x5a) return 0;
    if (d[1] != 0x41 && d[1] != 0x73 && d[1] != 0x79) return 0;
    required = 3U + 2U * (d[1] & 15U);
    if (n < required) return 0;
    out->id = d[1];
    out->buttons = (~((uint32_t)d[3] | ((uint32_t)d[4] << 8))) & 0xffffU;
    out->lx = d[1] == 0x41 ? 128U : d[7];
    out->ly = d[1] == 0x41 ? 128U : d[8];
    return 1;
}
