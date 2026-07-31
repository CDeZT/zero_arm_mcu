#pragma once

#include <stdbool.h>
#include <stdint.h>

void platform_gpio_init(void);

bool platform_limit_is_configured(uint8_t joint_index);

bool platform_limit_is_active(uint8_t joint_index);

bool platform_required_limits_are_active(uint8_t joint_mask);

bool platform_estop_is_active(void);

void platform_gpio_on_exti(uint16_t gpio_pin);

#if defined(PLATFORM_GPIO_TEST)
void platform_gpio_test_set_idr(uint32_t idr);
void platform_gpio_test_set_estop_idr(uint32_t idr);
#endif

void platform_estop_notify_from_isr(void);
