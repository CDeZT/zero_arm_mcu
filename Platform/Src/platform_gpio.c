#include "platform_gpio.h"
#include "build_config.h"

void platform_gpio_init(void)
{
#if !CONFIG_LIMIT_SWITCH_ENABLED
    return;
#endif
}

bool platform_limit_is_configured(uint8_t joint_index)
{
#if !CONFIG_LIMIT_SWITCH_ENABLED
    (void)joint_index;
    return false;
#else
    (void)joint_index;
    return false;
#endif
}

bool platform_limit_is_active(uint8_t joint_index)
{
#if !CONFIG_LIMIT_SWITCH_ENABLED
    (void)joint_index;
    return false;
#else
    (void)joint_index;
    return false;
#endif
}

void platform_gpio_on_exti(uint16_t gpio_pin)
{
#if !CONFIG_LIMIT_SWITCH_ENABLED
    (void)gpio_pin;
    return;
#endif
}
