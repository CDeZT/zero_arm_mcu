#pragma once

#include <stdbool.h>
#include <stdint.h>

void platform_gpio_init(void);

bool platform_limit_is_configured(uint8_t joint_index);

bool platform_limit_is_active(uint8_t joint_index);

void platform_gpio_on_exti(uint16_t gpio_pin);
