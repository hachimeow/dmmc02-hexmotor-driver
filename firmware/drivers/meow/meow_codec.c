#include "meow_codec.h"

#include <math.h>
#include <string.h>

#define MEOW_POSITION_BITS 16U
#define MEOW_TERM_BITS 12U
#define MEOW_Q24_SCALE 16777216.0f

static uint32_t quantize(float value, float min, float max, uint32_t max_code)
{
    float scaled;

    if (value <= min) {
        return 0U;
    }
    if (value >= max) {
        return max_code;
    }
    scaled = (value - min) * (float)max_code / (max - min);
    if (scaled <= 0.0f) {
        return 0U;
    }
    if (scaled >= (float)max_code) {
        return max_code;
    }
    return (uint32_t)scaled;
}

static bool range_pair_valid(float min, float max)
{
    return isfinite(min) && isfinite(max) && (min < max) && (min <= 0.0f) && (max >= 0.0f);
}

void meow_range_default(meow_mit_range_t *range)
{
    if (range == 0) {
        return;
    }

    /* Defaults from the public HexMeow compressed-MIT recipe. */
    range->position_min_rev = -0.5f;
    range->position_max_rev = 0.5f;
    range->velocity_min_rev_s = -10.0f;
    range->velocity_max_rev_s = 10.0f;
    range->torque_min_nm = -10.0f;
    range->torque_max_nm = 10.0f;
    range->kp_min_nm_rev = 0.0f;
    range->kp_max_nm_rev = 100.0f;
    range->kd_min_nm_s_rev = 0.0f;
    range->kd_max_nm_s_rev = 20.0f;
}

bool meow_range_validate(const meow_mit_range_t *range)
{
    if (range == 0) {
        return false;
    }

    return range_pair_valid(range->position_min_rev, range->position_max_rev) &&
           range_pair_valid(range->velocity_min_rev_s, range->velocity_max_rev_s) &&
           range_pair_valid(range->torque_min_nm, range->torque_max_nm) &&
           range_pair_valid(range->kp_min_nm_rev, range->kp_max_nm_rev) &&
           range_pair_valid(range->kd_min_nm_s_rev, range->kd_max_nm_s_rev);
}

void meow_pack_mit(const meow_mit_range_t *range,
                   float position_rev,
                   float velocity_rev_s,
                   float kp_nm_rev,
                   float kd_nm_s_rev,
                   float torque_nm,
                   uint8_t out[8])
{
    uint32_t pos;
    uint32_t vel;
    uint32_t kp;
    uint32_t kd;
    uint32_t torque;
    uint32_t lower;
    uint32_t upper;

    if ((range == 0) || (out == 0)) {
        return;
    }

    pos = quantize(position_rev,
                   range->position_min_rev,
                   range->position_max_rev,
                   (1UL << MEOW_POSITION_BITS) - 1UL);
    vel = quantize(velocity_rev_s,
                   range->velocity_min_rev_s,
                   range->velocity_max_rev_s,
                   (1UL << MEOW_TERM_BITS) - 1UL);
    torque = quantize(torque_nm,
                      range->torque_min_nm,
                      range->torque_max_nm,
                      (1UL << MEOW_TERM_BITS) - 1UL);
    kp = quantize(kp_nm_rev,
                  range->kp_min_nm_rev,
                  range->kp_max_nm_rev,
                  (1UL << MEOW_TERM_BITS) - 1UL);
    kd = quantize(kd_nm_s_rev,
                  range->kd_min_nm_s_rev,
                  range->kd_max_nm_s_rev,
                  (1UL << MEOW_TERM_BITS) - 1UL);

    lower = torque | (kd << 12) | ((kp & 0xFFUL) << 24);
    upper = (kp >> 8) | (vel << 4) | (pos << 16);

    out[0] = (uint8_t)(lower & 0xFFUL);
    out[1] = (uint8_t)((lower >> 8) & 0xFFUL);
    out[2] = (uint8_t)((lower >> 16) & 0xFFUL);
    out[3] = (uint8_t)((lower >> 24) & 0xFFUL);
    out[4] = (uint8_t)(upper & 0xFFUL);
    out[5] = (uint8_t)((upper >> 8) & 0xFFUL);
    out[6] = (uint8_t)((upper >> 16) & 0xFFUL);
    out[7] = (uint8_t)((upper >> 24) & 0xFFUL);
}

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t rd_i16(const uint8_t *p)
{
    return (int16_t)rd_u16(p);
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

float meow_q24_to_float(int32_t raw)
{
    return (float)raw / MEOW_Q24_SCALE;
}

int32_t meow_float_to_q24(float rev)
{
    float scaled = rev * MEOW_Q24_SCALE;
    if (scaled >= 2147483647.0f) {
        return 2147483647;
    }
    if (scaled <= -2147483648.0f) {
        return (-2147483647 - 1);
    }
    return (scaled >= 0.0f) ? (int32_t)(scaled + 0.5f) : (int32_t)(scaled - 0.5f);
}

void meow_decode_tpdo1(meow_motor_t *motor,
                       const uint8_t *data,
                       uint8_t len,
                       uint32_t now_ms)
{
    int32_t position_raw;
    uint16_t timestamp_us;
    int16_t torque_permille;
    float velocity = 0.0f;
    bool sample_gap;

    if ((motor == 0) || (data == 0) || (len < 8U)) {
        return;
    }

    position_raw = (int32_t)rd_u32(&data[0]);
    timestamp_us = rd_u16(&data[4]);
    torque_permille = rd_i16(&data[6]);

    motor->state.position_rev = meow_q24_to_float(position_raw);
    motor->state.timestamp_us = timestamp_us;
    motor->state.torque_nm = (motor->peak_torque_nm > 0.0f)
                                 ? ((float)torque_permille * motor->peak_torque_nm / 1000.0f)
                                 : 0.0f;

    sample_gap = (motor->last_tpdo1_ms != 0U) &&
                 ((now_ms - motor->last_tpdo1_ms) > 50U);

    if (motor->have_last_position && !sample_gap) {
        uint16_t dt_us = (uint16_t)(timestamp_us - motor->last_timestamp_us);
        int32_t delta_raw = (int32_t)((uint32_t)position_raw - (uint32_t)motor->last_position_raw);

        /* The public HexMeow host rejects dt outside 1..50000 us. It also
         * rejects gaps larger than 50 ms to avoid a velocity spike after a
         * missed frame. */
        if ((dt_us >= 1U) && (dt_us <= 50000U)) {
            float delta_rev = (float)delta_raw / MEOW_Q24_SCALE;
            velocity = delta_rev * 1000000.0f / (float)dt_us;

            /* Sanity bound: these joints do not reach 50 rev/s. A larger value
             * is a wrap/noise artifact, not real motion. */
            if ((velocity > 50.0f) || (velocity < -50.0f)) {
                velocity = 0.0f;
                motor->have_last_position = false;
            } else if (motor->velocity_filtered_rev_s == 0.0f) {
                motor->velocity_filtered_rev_s = velocity;
            } else {
                motor->velocity_filtered_rev_s =
                    (motor->velocity_filtered_rev_s * 0.75f) + (velocity * 0.25f);
            }
            motor->state.velocity_rev_s = motor->velocity_filtered_rev_s;
        } else {
            motor->have_last_position = false;
            motor->velocity_filtered_rev_s = 0.0f;
            motor->state.velocity_rev_s = 0.0f;
        }
    } else if (sample_gap) {
        motor->velocity_filtered_rev_s = 0.0f;
        motor->state.velocity_rev_s = 0.0f;
    }

    motor->last_position_raw = position_raw;
    motor->last_timestamp_us = timestamp_us;
    motor->have_last_position = true;
    motor->last_tpdo1_ms = now_ms;
    motor->state.last_update_ms = now_ms;
    motor->last_rx_ms = now_ms;
    motor->state.online = true;
}

void meow_decode_tpdo2(meow_motor_t *motor,
                       const uint8_t *data,
                       uint8_t len,
                       uint32_t now_ms)
{
    if ((motor == 0) || (data == 0) || (len < 7U)) {
        return;
    }

    motor->state.mode_display = data[0];
    motor->state.error_code = rd_u16(&data[1]);
    motor->state.driver_temp_c = (float)rd_i16(&data[3]) * 0.1f;
    motor->state.motor_temp_c = (float)rd_i16(&data[5]) * 0.1f;
    motor->last_tpdo2_ms = now_ms;
    motor->state.last_update_ms = now_ms;
    motor->last_rx_ms = now_ms;
    motor->state.online = true;

    motor->state.fault = (motor->state.error_code != 0U);
    motor->state.enabled = (motor->state.mode_display >= MEOW_MODE_PROFILE_POSITION) &&
                           (motor->state.mode_display <= MEOW_MODE_MIT);

    if (motor->state.fault) {
        motor->fault = true;
        motor->error_count++;
    }
}
