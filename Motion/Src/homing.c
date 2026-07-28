#include "homing.h"
#include "build_config.h"

void homing_init(void)
{
#if !CONFIG_HOMING_ENABLED
    return;
#endif
}

bool homing_start(uint8_t joint_mask)
{
#if !CONFIG_HOMING_ENABLED
    (void)joint_mask;
    return false;
#else
    (void)joint_mask;
    return false;
#endif
}

bool homing_is_active(void)
{
#if !CONFIG_HOMING_ENABLED
    return false;
#else
    return false;
#endif
}

void homing_step(uint32_t now_ms)
{
#if !CONFIG_HOMING_ENABLED
    (void)now_ms;
    return;
#else
    (void)now_ms;
#endif
}
