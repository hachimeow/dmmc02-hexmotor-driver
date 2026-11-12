#ifndef MEOW_MOTOR_H
#define MEOW_MOTOR_H

#include "meow_types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct meow_bus;

meow_status_t meow_motor_probe(struct meow_bus *bus, meow_motor_t *motor);
meow_status_t meow_motor_init(struct meow_bus *bus,
                              meow_motor_t *motor,
                              const meow_mit_range_t *range);

meow_status_t meow_motor_set_mit_range(struct meow_bus *bus,
                                       meow_motor_t *motor,
                                       const meow_mit_range_t *range);
meow_status_t meow_motor_get_mit_range(struct meow_bus *bus,
                                       meow_motor_t *motor,
                                       meow_mit_range_t *range);

meow_status_t meow_motor_set_torque_limit(struct meow_bus *bus,
                                          meow_motor_t *motor,
                                          float torque_limit_nm);
meow_status_t meow_motor_get_torque_limit(struct meow_bus *bus,
                                          meow_motor_t *motor,
                                          float *torque_limit_nm);

meow_status_t meow_motor_set_profile_limits(struct meow_bus *bus,
                                            meow_motor_t *motor,
                                            const meow_profile_limits_t *limits);
meow_status_t meow_motor_get_profile_limits(struct meow_bus *bus,
                                            meow_motor_t *motor,
                                            meow_profile_limits_t *limits);

meow_status_t meow_motor_enable_mit(struct meow_bus *bus, meow_motor_t *motor);
meow_status_t meow_motor_disable(struct meow_bus *bus, meow_motor_t *motor);
meow_status_t meow_motor_clear_error(struct meow_bus *bus, meow_motor_t *motor);

meow_status_t meow_motor_set_velocity(struct meow_bus *bus,
                                      meow_motor_t *motor,
                                      float rev_per_s);
meow_status_t meow_motor_set_position(struct meow_bus *bus,
                                      meow_motor_t *motor,
                                      float rev);
meow_status_t meow_motor_set_torque(struct meow_bus *bus,
                                    meow_motor_t *motor,
                                    float torque_nm);

/*
 * Compressed-MIT target cache. This is non-blocking: meow_bus_sync() packs and
 * sends it on the shared RPDO frame.
 *
 * Units: rev, rev/s, Nm/rev, Nm*s/rev, Nm.
 */
meow_status_t meow_motor_mit(meow_motor_t *motor,
                             float position_rev,
                             float velocity_rev_s,
                             float kp_nm_rev,
                             float kd_nm_s_rev,
                             float torque_nm);

meow_status_t meow_motor_get_state(const meow_motor_t *motor,
                                   meow_motor_state_t *state);

/* Low-rate diagnostic read of 0x4402/0x453F/0x1001/0x4564/0x4577. */
meow_status_t meow_motor_read_status(struct meow_bus *bus, meow_motor_t *motor);

/* Internal: called by the bus dispatcher. */
void meow_motor_on_frame(meow_motor_t *motor,
                         uint32_t cob_id,
                         const uint8_t *data,
                         uint8_t len,
                         uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* MEOW_MOTOR_H */
