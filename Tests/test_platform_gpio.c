#include "platform_gpio.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

enum {
    GPIO_PIN_7 = 1U << 7,
    GPIO_PIN_9 = 1U << 9,
    GPIO_PIN_10 = 1U << 10,
    GPIO_PIN_11 = 1U << 11,
    GPIO_PIN_15 = 1U << 15
};

static uint32_t s_estop_notifications;

void platform_estop_notify_from_isr(void)
{
    s_estop_notifications++;
}

static void test_installed_limit_switches(void)
{
    platform_gpio_init();

    assert(platform_limit_is_configured(0U));
    assert(!platform_limit_is_configured(1U));
    assert(platform_limit_is_configured(2U));
    assert(platform_limit_is_configured(3U));
    assert(platform_limit_is_configured(4U));
    assert(!platform_limit_is_configured(5U));

    platform_gpio_test_set_idr(GPIO_PIN_7 | GPIO_PIN_9);
    assert(platform_limit_is_active(0U));
    assert(!platform_limit_is_active(1U));
    assert(platform_limit_is_active(2U));
    assert(!platform_limit_is_active(3U));
}

static void test_required_startup_limit_mask(void)
{
    const uint8_t required_joint_mask = 0x1DU;
    const uint32_t all_required_pins =
        GPIO_PIN_7 |
        GPIO_PIN_9 |
        GPIO_PIN_10 |
        GPIO_PIN_11;

    platform_gpio_test_set_idr(all_required_pins);
    assert(platform_required_limits_are_active(
        required_joint_mask));

    static const uint32_t required_pins[] = {
        GPIO_PIN_7,
        GPIO_PIN_9,
        GPIO_PIN_10,
        GPIO_PIN_11
    };
    for (size_t index = 0U;
         index < sizeof(required_pins) /
             sizeof(required_pins[0]);
         index++) {
        platform_gpio_test_set_idr(
            all_required_pins &
            ~required_pins[index]);
        assert(!platform_required_limits_are_active(
            required_joint_mask));
    }

    assert(!platform_required_limits_are_active(0U));
    assert(!platform_required_limits_are_active(0x02U));
    assert(!platform_required_limits_are_active(0x40U));
}

static void test_estop_is_active_low_and_falling_edge(void)
{
    s_estop_notifications = 0U;

    platform_gpio_test_set_estop_idr(GPIO_PIN_15);
    assert(!platform_estop_is_active());
    platform_gpio_on_exti(GPIO_PIN_15);
    assert(s_estop_notifications == 0U);

    platform_gpio_test_set_estop_idr(0U);
    assert(platform_estop_is_active());
    platform_gpio_on_exti(GPIO_PIN_7);
    assert(s_estop_notifications == 0U);
    platform_gpio_on_exti(GPIO_PIN_15);
    assert(s_estop_notifications == 1U);
}

int main(void)
{
    test_installed_limit_switches();
    test_required_startup_limit_mask();
    test_estop_is_active_low_and_falling_edge();

    puts("test_platform_gpio: all checks passed");
    return 0;
}
