#include "platform_time.h"
#include "cmsis_os2.h"

uint32_t platform_time_ms(void)
{
    return osKernelGetTickCount();
}
