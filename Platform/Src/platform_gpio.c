#include "platform_gpio.h"
#include "build_config.h"
#include "joint_config.h"

#include <stddef.h>

#define GPIOE_IDR_ADDRESS 0x48001010UL
#define GPIOD_IDR_ADDRESS 0x48000C10UL
#define ESTOP_GPIO_PIN     (1UL << 15)

#if defined(PLATFORM_GPIO_TEST)
static volatile uint32_t s_test_gpioe_idr;
static volatile uint32_t s_test_gpiod_idr = ESTOP_GPIO_PIN;
#define GPIOE_IDR_VALUE s_test_gpioe_idr
#define GPIOD_IDR_VALUE s_test_gpiod_idr
#else
#define GPIOE_IDR_VALUE (*(volatile uint32_t *)GPIOE_IDR_ADDRESS)
#define GPIOD_IDR_VALUE (*(volatile uint32_t *)GPIOD_IDR_ADDRESS)
#endif

void platform_gpio_init(void)
{
    /* MX_GPIO_Init configures PE7..PE12 as pull-up rising-edge inputs. */
}

bool platform_limit_is_configured(uint8_t joint_index)
{
#if !CONFIG_LIMIT_SWITCH_ENABLED
    (void)joint_index;
    return false;
#else
    const joint_config_t *config = joint_config_get(joint_index);
    return config != NULL && config->limit_switch_installed;
#endif
}

bool platform_limit_is_active(uint8_t joint_index)
{
#if !CONFIG_LIMIT_SWITCH_ENABLED
    (void)joint_index;
    return false;
#else
    if (!platform_limit_is_configured(joint_index)) {
        return false;
    }

    return ((GPIOE_IDR_VALUE >> (7U + joint_index)) & 1U) != 0U;
#endif
}

bool platform_required_limits_are_active(uint8_t joint_mask)
{
#if !CONFIG_LIMIT_SWITCH_ENABLED
    (void)joint_mask;
    return false;
#else
    const uint8_t all_joint_bits =
        (uint8_t)((1U << ROBOT_JOINT_COUNT) - 1U);

    if (joint_mask == 0U ||
        (joint_mask & (uint8_t)~all_joint_bits) != 0U) {
        return false;
    }

    for (uint8_t joint = 0U;
         joint < ROBOT_JOINT_COUNT;
         joint++) {
        const uint8_t joint_bit =
            (uint8_t)(1U << joint);

        if ((joint_mask & joint_bit) != 0U &&
            (!platform_limit_is_configured(joint) ||
             !platform_limit_is_active(joint))) {
            return false;
        }
    }
    return true;
#endif
}

void platform_gpio_on_exti(uint16_t gpio_pin)
{
    if (gpio_pin == (uint16_t)ESTOP_GPIO_PIN &&
        platform_estop_is_active()) {
        platform_estop_notify_from_isr();
    }
}

__attribute__((weak)) void platform_estop_notify_from_isr(void)
{
}

bool platform_estop_is_active(void)
{
    return (GPIOD_IDR_VALUE & ESTOP_GPIO_PIN) == 0U;
}

#if defined(PLATFORM_GPIO_TEST)
void platform_gpio_test_set_idr(uint32_t idr)
{
    s_test_gpioe_idr = idr;
}

void platform_gpio_test_set_estop_idr(uint32_t idr)
{
    s_test_gpiod_idr = idr;
}
#endif

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    platform_gpio_on_exti(gpio_pin);
}
