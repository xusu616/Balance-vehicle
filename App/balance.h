#ifndef APP_BALANCE_H
#define APP_BALANCE_H

#include <stdint.h>

typedef enum {
    BALANCE_WAITING,
    BALANCE_SAMPLE_READY,
    BALANCE_NOT_UPRIGHT,
    BALANCE_IMU_ERROR,
    BALANCE_SAMPLE_LATE,
    BALANCE_TILT_LIMIT
} BalanceStatus;

typedef struct {
    float target_deg;
    float pitch_deg;
    float rate_dps;
    int16_t forward_permille; /* Positive commands motion toward the nose. */
} BalanceSample;

/* Seed the filter and capture the hand-held upright pose as the target. */
BalanceStatus Balance_StartPreview(BalanceSample *sample);
/* Poll after a successful start. Any failure requires a new start. */
BalanceStatus Balance_PollPreview(BalanceSample *sample);

#endif
