#ifndef MEOW_CODEC_H
#define MEOW_CODEC_H

#include <stdbool.h>
#include <stdint.h>

#include "meow_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void meow_range_default(meow_mit_range_t *range);
bool meow_range_validate(const meow_mit_range_t *range);

/*
 * Pack the five compressed-MIT terms into the 8-byte wire format used by
 * object 0x4103:02/03 (position 16 bit, velocity/torque/kp/kd 12 bit each).
 */
void meow_pack_mit(const meow_mit_range_t *range,
                   float position_rev,
                   float velocity_rev_s,
                   float kp_nm_rev,
                   float kd_nm_s_rev,
                   float torque_nm,
                   uint8_t out[8]);

void meow_decode_tpdo1(meow_motor_t *motor,
                       const uint8_t *data,
                       uint8_t len,
                       uint32_t now_ms);

void meow_decode_tpdo2(meow_motor_t *motor,
                       const uint8_t *data,
                       uint8_t len,
                       uint32_t now_ms);

float meow_q24_to_float(int32_t raw);
int32_t meow_float_to_q24(float rev);

#ifdef __cplusplus
}
#endif

#endif /* MEOW_CODEC_H */
