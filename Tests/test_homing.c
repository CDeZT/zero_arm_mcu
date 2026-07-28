#include "homing.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static void test_disabled_stubs_are_safe(void)
{
    homing_init();

    assert(!homing_is_active());

    assert(!homing_start(0x01U));
    assert(!homing_start(0x3FU));

    assert(!homing_is_active());

    homing_step(0U);
    homing_step(1000U);
    homing_step(UINT32_MAX);
}

int main(void)
{
    test_disabled_stubs_are_safe();

    puts("test_homing: all checks passed");
    return 0;
}
