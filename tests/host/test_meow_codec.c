#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "meow_codec.h"
#include "meow_types.h"

static void test_range_default_valid(void)
{
    meow_mit_range_t range;
    meow_range_default(&range);
    assert(meow_range_validate(&range));
}

static void test_range_rejects_nonzero_crossing(void)
{
    meow_mit_range_t range;
    meow_range_default(&range);
    range.position_min_rev = 0.1f;
    assert(!meow_range_validate(&range));
    meow_range_default(&range);
    range.torque_max_nm = -1.0f;
    assert(!meow_range_validate(&range));
}

static void test_pack_zero_matches_wire_observation(void)
{
    meow_mit_range_t range;
    uint8_t out[8];
    const uint8_t expected[8] = {0xFF, 0x07, 0x00, 0x00, 0xF0, 0x7F, 0xFF, 0x7F};

    meow_range_default(&range);
    meow_pack_mit(&range, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, out);
    assert(memcmp(out, expected, sizeof(expected)) == 0);
}

static void test_pack_clamps(void)
{
    meow_mit_range_t range;
    uint8_t out[8];

    meow_range_default(&range);
    meow_pack_mit(&range, 99.0f, 99.0f, 999.0f, 999.0f, 99.0f, out);
    /* torque/kd/kp all clamp to 0x0FFF; position and velocity to 0xFFFF/0x0FFF. */
    assert(out[0] == 0xFF && out[1] == 0xFF && out[2] == 0xFF && out[3] == 0xFF);
    assert(out[4] == 0xFF && out[5] == 0xFF && out[6] == 0xFF && out[7] == 0xFF);
}

static void test_q24_roundtrip(void)
{
    assert(meow_float_to_q24(1.0f) == 16777216);
    assert(meow_float_to_q24(-1.0f) == -16777216);
    assert(fabsf(meow_q24_to_float(16777216) - 1.0f) < 1e-6f);
}

static void test_decode_tpdo1(void)
{
    meow_motor_t motor;
    uint8_t frame[8] = {0};

    memset(&motor, 0, sizeof(motor));
    motor.peak_torque_nm = 8.0f;

    /* position = 0.001 rev, timestamp 1000 us, torque 500 permille */
    int32_t pos = (int32_t)(0.001f * 16777216.0f);
    memcpy(&frame[0], &pos, 4);
    frame[4] = 0xE8;
    frame[5] = 0x03;
    frame[6] = 0xF4;
    frame[7] = 0x01;
    meow_decode_tpdo1(&motor, frame, 8, 1000);
    assert(fabsf(motor.state.position_rev - 0.001f) < 1e-5f);
    assert(motor.state.timestamp_us == 1000);
    assert(fabsf(motor.state.torque_nm - 4.0f) < 1e-4f);
    assert(motor.have_last_position);

    /* 1 ms later, another 0.001 rev -> 1 rev/s */
    pos += (int32_t)(0.001f * 16777216.0f);
    memcpy(&frame[0], &pos, 4);
    frame[4] = 0xE8;
    frame[5] = 0x07;
    meow_decode_tpdo1(&motor, frame, 8, 1001);
    assert(fabsf(motor.state.velocity_rev_s - 1.0f) < 0.05f);
}

static void test_decode_tpdo2(void)
{
    meow_motor_t motor;
    uint8_t frame[7] = {4, 0x10, 0x23, 0x7B, 0x00, 0x7C, 0x00};

    memset(&motor, 0, sizeof(motor));
    meow_decode_tpdo2(&motor, frame, 7, 2000);
    assert(motor.state.mode_display == 4);
    assert(motor.state.error_code == 0x2310);
    assert(motor.state.fault);
    assert(fabsf(motor.state.driver_temp_c - 12.3f) < 1e-4f);
    assert(fabsf(motor.state.motor_temp_c - 12.4f) < 1e-4f);
    assert(motor.state.enabled);
}

int main(void)
{
    test_range_default_valid();
    test_range_rejects_nonzero_crossing();
    test_pack_zero_matches_wire_observation();
    test_pack_clamps();
    test_q24_roundtrip();
    test_decode_tpdo1();
    test_decode_tpdo2();
    puts("meow_codec: all tests passed");
    return 0;
}
