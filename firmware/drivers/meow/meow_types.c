#include "meow_types.h"

const char *meow_model_name(meow_model_t model)
{
    switch (model) {
    case MEOW_MODEL_4310:
        return "hexmeow 4310";
    case MEOW_MODEL_4342:
        return "hexmeow 4342";
    case MEOW_MODEL_3525:
        return "hexmeow 3525";
    case MEOW_MODEL_GENERIC:
        return "hexmeow (Meow protocol)";
    default:
        return "unknown";
    }
}

const char *meow_mode_name(uint8_t mode)
{
    switch (mode) {
    case MEOW_MODE_PROFILE_POSITION:
        return "Profile Position";
    case MEOW_MODE_PROFILE_VELOCITY:
        return "Profile Velocity";
    case MEOW_MODE_TORQUE:
        return "Torque";
    case MEOW_MODE_MIT:
        return "MIT";
    default:
        return "Disabled";
    }
}

const char *meow_status_str(meow_status_t status)
{
    switch (status) {
    case MEOW_OK:
        return "OK";
    case MEOW_ERR_IO:
        return "CAN I/O error";
    case MEOW_ERR_TIMEOUT:
        return "timeout";
    case MEOW_ERR_BUSY:
        return "busy";
    case MEOW_ERR_SDO_ABORT:
        return "SDO abort";
    case MEOW_ERR_SDO_PROTOCOL:
        return "SDO protocol error";
    case MEOW_ERR_NOT_SUPPORTED:
        return "not supported";
    case MEOW_ERR_INVALID_ARG:
        return "invalid argument";
    case MEOW_ERR_NOT_READY:
        return "motor not ready";
    case MEOW_ERR_FAULT:
        return "motor fault";
    case MEOW_ERR_NOT_CONFIGURED:
        return "not configured";
    case MEOW_ERR_DEVICE_REJECTED:
        return "device rejected";
    case MEOW_ERR_IDENTITY_CONFLICT:
        return "identity conflict (duplicate node ID?)";
    default:
        return "unknown error";
    }
}

const char *meow_error_name(uint16_t error_code)
{
    switch (error_code) {
    case 0x0000:
        return "no error";
    case 0x2310:
        return "over-current";
    case 0x3210:
        return "over-voltage";
    case 0x3220:
        return "under-voltage";
    case 0x4210:
        return "driver over-temperature";
    case 0x4310:
        return "motor over-temperature";
    case 0x7305:
        return "encoder error";
    case 0x8130:
        return "heartbeat lost";
    default:
        return "device error";
    }
}
