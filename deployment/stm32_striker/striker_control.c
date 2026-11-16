#include "striker_control.h"
#include "striker_config.h"

static StrikerMotorConfig config[4];
static volatile StrikerState state = STRIKER_IDLE;
static volatile StrikerError error = STRIKER_ERROR_NONE;
static uint8_t selected[4];
static uint8_t reached[4];
static int64_t position[4];
static int64_t high_water[4];
static uint32_t no_progress_ms[4];
static uint32_t running_ms;
static uint32_t settling_ms;
static uint32_t quiet_ms;

void StrikerControl_Init(const StrikerMotorConfig motors[4])
{
    uint8_t i;
    for (i = 0; i < 4U; ++i)
    {
        config[i] = motors[i];
        selected[i] = 0;
        reached[i] = 0;
        position[i] = 0;
        high_water[i] = 0;
        no_progress_ms[i] = 0;
    }
    state = STRIKER_IDLE;
    error = STRIKER_ERROR_NONE;
    running_ms = 0;
    settling_ms = 0;
    quiet_ms = 0;
}

uint8_t StrikerControl_Start(uint8_t arm_mask)
{
    uint8_t i;
    uint8_t covered = 0;

    if (state != STRIKER_IDLE && state != STRIKER_DONE)
        return 0;

    if (arm_mask < 1U || arm_mask > 3U || STRIKER_TARGET_REVOLUTIONS == 0U ||
        STRIKER_RUN_PWM == 0U || STRIKER_RUN_PWM > STRIKER_PWM_LIMIT ||
        STRIKER_MIN_PWM > STRIKER_RUN_PWM || STRIKER_MIN_PWM == 0U)
        goto invalid_config;

    for (i = 0; i < 4U; ++i)
    {
        if (config[i].arm > 2U)
            goto invalid_config;
        selected[i] = (uint8_t)((config[i].arm & arm_mask) != 0U);
        reached[i] = 0;
        position[i] = 0;
        high_water[i] = 0;
        no_progress_ms[i] = 0;
        if (selected[i])
        {
            if (config[i].counts_per_rev == 0U ||
                (config[i].pwm_sign != 1 && config[i].pwm_sign != -1) ||
                (config[i].encoder_sign != 1 && config[i].encoder_sign != -1))
                goto invalid_config;
            covered |= config[i].arm;
        }
    }
    if (covered != arm_mask)
        goto invalid_config;

    running_ms = 0;
    settling_ms = 0;
    quiet_ms = 0;
    error = STRIKER_ERROR_NONE;
    state = STRIKER_RUNNING;
    return 1;

invalid_config:
    error = STRIKER_ERROR_CONFIG;
    state = STRIKER_FAULT;
    return 0;
}

static void stop_with_error(StrikerError reason, int32_t pwm[4])
{
    uint8_t i;
    for (i = 0; i < 4U; ++i)
        pwm[i] = 0;
    error = reason;
    state = STRIKER_FAULT;
}

void StrikerControl_Update(const int32_t delta[4], uint32_t elapsed_ms, int32_t pwm[4])
{
    uint8_t i;
    uint8_t all_reached = 1;
    uint8_t moving = 0;

    for (i = 0; i < 4U; ++i)
        pwm[i] = 0;
    if (state != STRIKER_RUNNING && state != STRIKER_SETTLING)
        return;

    if (elapsed_ms >= STRIKER_MAX_RUN_MS - running_ms)
    {
        stop_with_error(STRIKER_ERROR_TIMEOUT, pwm);
        return;
    }
    running_ms += elapsed_ms;

    for (i = 0; i < 4U; ++i)
    {
        int64_t increment;
        int64_t target;
        int64_t remaining;
        uint32_t slowdown;
        uint32_t duty;

        if (!selected[i])
            continue;
        increment = (int64_t)delta[i] * config[i].encoder_sign;
        position[i] += increment;
        if (delta[i] != 0)
            moving = 1;
        if (position[i] < -(int64_t)STRIKER_REVERSE_TOLERANCE_COUNTS)
        {
            stop_with_error(STRIKER_ERROR_DIRECTION, pwm);
            return;
        }
        if (state == STRIKER_SETTLING || reached[i])
            continue;

        target = (int64_t)config[i].counts_per_rev * STRIKER_TARGET_REVOLUTIONS;
        remaining = target - position[i];
        if (remaining <= 0)
        {
            reached[i] = 1;
            continue;
        }
        all_reached = 0;

        if (position[i] > high_water[i])
        {
            high_water[i] = position[i];
            no_progress_ms[i] = 0;
        }
        else if (elapsed_ms >= STRIKER_STALL_MS - no_progress_ms[i])
        {
            stop_with_error(STRIKER_ERROR_STALL, pwm);
            return;
        }
        else
        {
            no_progress_ms[i] += elapsed_ms;
        }

        /* Slow down over the final half output revolution; no wheel mm/s PI. */
        slowdown = config[i].counts_per_rev / 2U;
        if (slowdown == 0U)
            slowdown = 1U;
        duty = STRIKER_RUN_PWM;
        if (remaining < (int64_t)slowdown)
        {
            duty = (uint32_t)((int64_t)STRIKER_RUN_PWM * remaining / slowdown);
            if (duty < STRIKER_MIN_PWM)
                duty = STRIKER_MIN_PWM;
        }
        pwm[i] = (int32_t)duty * config[i].pwm_sign;
    }

    if (state == STRIKER_RUNNING && all_reached)
    {
        state = STRIKER_SETTLING;
        settling_ms = 0;
        quiet_ms = 0;
        return;
    }
    if (state == STRIKER_SETTLING)
    {
        settling_ms += elapsed_ms;
        quiet_ms = moving ? 0U : quiet_ms + elapsed_ms;
        if (quiet_ms >= STRIKER_SETTLE_MS)
            state = STRIKER_DONE;
        else if (settling_ms >= STRIKER_MAX_SETTLE_MS)
            stop_with_error(STRIKER_ERROR_NOT_SETTLED, pwm);
    }
}

StrikerState StrikerControl_State(void)
{
    return state;
}

StrikerError StrikerControl_Error(void)
{
    return error;
}
