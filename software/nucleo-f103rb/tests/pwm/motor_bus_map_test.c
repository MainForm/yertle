#undef NDEBUG
#include <assert.h>
#include "motor_bus_map.h"
int main(void)
{
    assert(motor_bus_index_from_id(0)==-1 && motor_bus_index_from_id(13)==-1);
    assert(motor_bus_for_id(0)==MOTOR_BUS_INVALID && motor_bus_for_id(13)==MOTOR_BUS_INVALID);
    for (uint8_t id=1;id<=12;++id) {
        motor_bus_id_t bus=motor_bus_for_id(id);
        assert(motor_bus_index_from_id(id)==id-1);
        assert(bus==(id<=6?MOTOR_BUS_USART1:MOTOR_BUS_USART3));
        assert(motor_bus_mask(bus)&(1U<<(id-1U)));
    }
    assert((motor_bus_mask(MOTOR_BUS_USART1)&motor_bus_mask(MOTOR_BUS_USART3))==0);
    assert((motor_bus_mask(MOTOR_BUS_USART1)|motor_bus_mask(MOTOR_BUS_USART3))==0xfffU);
    assert(motor_bus_mask(MOTOR_BUS_INVALID)==0);
    return 0;
}
