#include "balance.h"
#include "mpu6050.h"
#include <math.h>

#define BALANCE_PERIOD_MS           10U
#define BALANCE_START_ANGLE_DEG     10.0f
#define BALANCE_START_RATE_DPS      5.0f
#define BALANCE_TILT_LIMIT_DEG      30.0f
#define BALANCE_OUTPUT_LIMIT        250.0f
#define BALANCE_KP                  12.0f /* PWM per mille / deg */
#define BALANCE_KD                  2.0f  /* PWM per mille / (deg/s) */

static float target_deg;
static uint32_t period_ms;
static uint32_t sample_ms;

BalanceStatus Balance_StartPreview(BalanceSample *sample)
{
    MPU6050_Data data;
    const uint32_t now = HAL_GetTick();

    /* A zero interval seeds the filter from the current accelerometer sample. */
    if (MPU6050_UpdateAngle(0.0f, &data) != HAL_OK) {
        return BALANCE_IMU_ERROR;
    }

    const float pitch = MPU6050_GetAngleDeg();
    const float rate = MPU6050_GetPitchRateDps(&data);
    if (fabsf(pitch) > BALANCE_START_ANGLE_DEG || fabsf(rate) > BALANCE_START_RATE_DPS) {
        return BALANCE_NOT_UPRIGHT;
    }

    target_deg = pitch;
    period_ms = now;
    sample_ms = now;
    *sample = (BalanceSample){ .target_deg = target_deg, .pitch_deg = pitch, .rate_dps = rate };
    return BALANCE_SAMPLE_READY;
}

BalanceStatus Balance_PollPreview(BalanceSample *sample)
{
    const uint32_t now = HAL_GetTick();
    const uint32_t phase_ms = now - period_ms;
    MPU6050_Data data;

    if (phase_ms < BALANCE_PERIOD_MS) {
        return BALANCE_WAITING;
    }
    if (phase_ms >= 2U * BALANCE_PERIOD_MS) {
        return BALANCE_SAMPLE_LATE;
    }

    /* Keep the 10 ms schedule and integrate using actual time between reads. */
    const float dt_s = (float)(now - sample_ms) * 0.001f;
    period_ms += BALANCE_PERIOD_MS;
    sample_ms = now;
    if (MPU6050_UpdateAngle(dt_s, &data) != HAL_OK) {
        return BALANCE_IMU_ERROR;
    }
    if ((uint32_t)(HAL_GetTick() - period_ms) >= BALANCE_PERIOD_MS) {
        return BALANCE_SAMPLE_LATE;
    }

    const float pitch = MPU6050_GetAngleDeg();
    const float rate = MPU6050_GetPitchRateDps(&data);
    const float error = target_deg - pitch;
    if (fabsf(error) > BALANCE_TILT_LIMIT_DEG) {
        return BALANCE_TILT_LIMIT;
    }

    /* Nose up is positive pitch and the vehicle must catch itself backward. */
    float duty = BALANCE_KP * error - BALANCE_KD * rate;
    if (duty > BALANCE_OUTPUT_LIMIT) {
        duty = BALANCE_OUTPUT_LIMIT;
    } else if (duty < -BALANCE_OUTPUT_LIMIT) {
        duty = -BALANCE_OUTPUT_LIMIT;
    }
    *sample = (BalanceSample){
        .target_deg = target_deg, .pitch_deg = pitch, .rate_dps = rate,
        .forward_permille = (int16_t)duty
    };
    return BALANCE_SAMPLE_READY;
}
