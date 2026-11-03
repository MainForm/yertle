#ifndef YERTLE_MOTOR_BUS_MAP_H
#define YERTLE_MOTOR_BUS_MAP_H

#include <stdint.h>

/* Routing only: motor IDs and their calibrated physical mapping are unchanged. */
typedef enum { MOTOR_BUS_USART1 = 0, MOTOR_BUS_USART3 = 1, MOTOR_BUS_INVALID = 255 } motor_bus_id_t;

static inline int motor_bus_index_from_id(uint8_t id)
{
    return id >= 1U && id <= 12U ? (int)id - 1 : -1;
}

static inline motor_bus_id_t motor_bus_for_id(uint8_t id)
{
    return id >= 1U && id <= 6U ? MOTOR_BUS_USART1 :
           id >= 7U && id <= 12U ? MOTOR_BUS_USART3 : MOTOR_BUS_INVALID;
}

static inline uint16_t motor_bus_mask(motor_bus_id_t bus)
{
    return bus == MOTOR_BUS_USART1 ? 0x003fU :
           bus == MOTOR_BUS_USART3 ? 0x0fc0U : 0U;
}

#endif
