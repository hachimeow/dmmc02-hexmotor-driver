#ifndef DEMO_H
#define DEMO_H

#include "meow_bus.h"
#include "meow_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

void demo_init(meow_bus_t *bus, meow_motor_t *motor);
void demo_poll(meow_bus_t *bus, meow_motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* DEMO_H */
