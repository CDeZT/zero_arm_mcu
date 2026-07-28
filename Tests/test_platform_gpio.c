#include "platform_gpio.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static void test_disabled_stubs_are_safe(void)
{
    platform_gpio_init();

    assert(!platform_limit_is_configured(0U));
    assert(!platform_limit_is_configured(5U));

    assert(!platform_limit_is_active(0U));
    assert(!platform_limit_is_active(3U));

    platform_gpio_on_exti(0x0080U);
    platform_gpio_on_exti(0x0100U);
}

int main(void)
{
    test_disabled_stubs_are_safe();

    puts("test_platform_gpio: all checks passed");
    return 0;
}
