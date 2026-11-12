#ifndef MEOW_TYPES_H
#define MEOW_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MEOW_MAX_MOTORS 8U
#define MEOW_MAX_DISCOVERED 32U

#define MEOW_VENDOR_ID 0x00686578UL
#define MEOW_PRODUCT_4310 0x6C64BC78UL
#define MEOW_PRODUCT_4342 0x6C64BCAAUL
#define MEOW_PRODUCT_3525 0x6C64AE8DUL

typedef enum {
    MEOW_MODEL_UNKNOWN = 0,
    MEOW_MODEL_4310,
    MEOW_MODEL_4342,
    MEOW_MODEL_3525,
    MEOW_MODEL_GENERIC
} meow_model_t;

typedef enum {
    MEOW_MODE_DISABLED = 0,
    MEOW_MODE_PROFILE_POSITION = 1,
    MEOW_MODE_PROFILE_VELOCITY = 2,
    MEOW_MODE_TORQUE = 3,
    MEOW_MODE_MIT = 4
} meow_mode_t;

typedef enum {
    MEOW_OK = 0,
    MEOW_ERR_IO,
    MEOW_ERR_TIMEOUT,
    MEOW_ERR_BUSY,
    MEOW_ERR_SDO_ABORT,
    MEOW_ERR_SDO_PROTOCOL,
    MEOW_ERR_NOT_SUPPORTED,
    MEOW_ERR_INVALID_ARG,
    MEOW_ERR_NOT_READY,
    MEOW_ERR_FAULT,
    MEOW_ERR_NOT_CONFIGURED,
    MEOW_ERR_DEVICE_REJECTED,
    MEOW_ERR_IDENTITY_CONFLICT
} meow_status_t;

/*
 * Compressed-MIT mapping ranges (object 0x4103:04..0D).
 *
 * The motor quantizes the five MIT terms into 16/12/12/12/12 bits over these
 * ranges. Ranges must be finite, min < max, and include zero. Units are the
 * protocol-native output-shaft units:
 *
 *   position : rev
 *   velocity : rev/s
 *   torque   : Nm
 *   kp       : Nm/rev
 *   kd       : Nm*s/rev
 */
typedef struct {
    float position_min_rev;
    float position_max_rev;
    float velocity_min_rev_s;
    float velocity_max_rev_s;
    float torque_min_nm;
    float torque_max_nm;
    float kp_min_nm_rev;
    float kp_max_nm_rev;
    float kd_min_nm_s_rev;
    float kd_max_nm_s_rev;
} meow_mit_range_t;

/* Profile planning limits (objects 0x4581 / 0x4583 / 0x4584). */
typedef struct {
    float velocity_limit_rev_s;
    float acceleration_rev_s2;
    float deceleration_rev_s2;
} meow_profile_limits_t;

typedef struct {
    float position_rev;
    float velocity_rev_s;
    float torque_nm;
    float driver_temp_c;
    float motor_temp_c;
    uint32_t timestamp_us;
    uint32_t last_update_ms;
    uint16_t error_code;
    uint8_t error_register;
    uint8_t mode_display;
    uint8_t nmt_state;
    bool online;
    bool enabled;
    bool fault;
} meow_motor_state_t;

typedef struct {
    uint8_t node_id;
    uint8_t slot;
    uint8_t total_motors;

    meow_model_t model;
    uint32_t vendor_id;
    uint32_t product_code;
    uint32_t revision;
    uint32_t serial;

    bool identified;
    bool supported;
    bool ready;
    bool enabled;
    bool fault;
    bool compressed_mit_enabled;
    bool mode_dirty;

    uint8_t mode;
    uint8_t nmt_state;

    float peak_torque_nm;
    float mit_factor;
    float mit_torque_limit_nm;
    float compressed_torque_limit_nm;
    meow_mit_range_t mit_range;
    meow_profile_limits_t profile_limits;

    /* Latest MIT command, packed by meow_bus_sync(). */
    float cmd_position_rev;
    float cmd_velocity_rev_s;
    float cmd_kp_nm_rev;
    float cmd_kd_nm_s_rev;
    float cmd_torque_nm;
    bool cmd_valid;

    /* Feedback and liveness. */
    meow_motor_state_t state;
    uint16_t heartbeat_period_ms;
    uint32_t last_rx_ms;
    uint32_t last_heartbeat_ms;
    uint32_t last_tpdo1_ms;
    uint32_t last_tpdo2_ms;

    /* Q8.24 / timestamp bookkeeping for velocity estimation. */
    int32_t last_position_raw;
    uint16_t last_timestamp_us;
    bool have_last_position;
    float velocity_filtered_rev_s;

    uint32_t error_count;
} meow_motor_t;

const char *meow_model_name(meow_model_t model);
const char *meow_mode_name(uint8_t mode);
const char *meow_status_str(meow_status_t status);
const char *meow_error_name(uint16_t error_code);

#ifdef __cplusplus
}
#endif

#endif /* MEOW_TYPES_H */
