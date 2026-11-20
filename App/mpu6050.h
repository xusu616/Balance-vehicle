#ifndef APP_MPU6050_H
#define APP_MPU6050_H

#include "stm32f1xx_hal.h"

/* GY-521 module on I2C1 (PB6 = SCL, PB7 = SDA, 100 kHz).
 * AD0 must be tied to GND for the default 7-bit address 0x68.
 */
#define MPU6050_I2C_ADDR_7BIT       0x68U

#define MPU6050_WHO_AM_I_MPU6050    0x68U
#define MPU6050_WHO_AM_I_MPU6500    0x70U
#define MPU6050_WHO_AM_I_MPU9250    0x71U

/* --- Pitch axis selection -------------------------------------------------
 * The balance loop needs the tilt angle about the wheel axle (the axis that
 * points left-right through both wheels). Mount the GY-521 flat on the car
 * and choose the sensor axis that runs along that axle:
 *   MPU6050_PITCH_AXIS_X: the module's X axis is parallel to the axle
 *   MPU6050_PITCH_AXIS_Y: the module's Y axis is parallel to the axle
 *
 * Verify with the 'g' command: the chosen gyro axis must show the largest
 * rate when the car is rocked forward/back. The accelerometer angle is
 * computed from Y/Z for the X axis and -X/Z for the Y axis so that it
 * agrees with the corresponding gyro direction.
 *
 * If the angle moves in the wrong direction, flip the matching sign below
 * (+1.0f / -1.0f). ACCEL_SIGN fixes a mirrored static angle; RATE_SIGN fixes
 * a gyro that integrates the wrong way (the two must agree while tilting).
 * ------------------------------------------------------------------------- */
#define MPU6050_PITCH_AXIS_X        0
#define MPU6050_PITCH_AXIS_Y        1

/* Current installation: Y along the axle, negative gyro Y when the nose rises. */
#ifndef MPU6050_PITCH_AXIS
#define MPU6050_PITCH_AXIS          MPU6050_PITCH_AXIS_Y
#endif
#ifndef MPU6050_PITCH_RATE_SIGN
#define MPU6050_PITCH_RATE_SIGN     (-1.0f)
#endif
#ifndef MPU6050_PITCH_ACCEL_SIGN
#define MPU6050_PITCH_ACCEL_SIGN    (-1.0f)
#endif
/* Complementary filter time constant in seconds; gyro weight uses actual dt. */
#ifndef MPU6050_FILTER_TAU_S
#define MPU6050_FILTER_TAU_S        (0.5f)
#endif

typedef struct {
    float accel_g[3];   /* X, Y, Z in g */
    float gyro_dps[3];  /* X, Y, Z in deg/s, gyro bias already subtracted */
    float temp_c;
} MPU6050_Data;

/* Probe WHO_AM_I, reset the device and configure:
 * - wake up, PLL clocked from the X gyro
 * - 200 Hz sample rate, DLPF ~42 Hz
 * - gyro +-500 dps (65.5 LSB/dps), accel +-4 g (8192 LSB/g)
 * Does not calibrate; call MPU6050_CalibrateGyro() with the car still.
 */
HAL_StatusTypeDef MPU6050_Init(void);

HAL_StatusTypeDef MPU6050_ReadWhoAmI(uint8_t *who_am_i);

/* Read accel + gyro + temperature in one 14-byte burst. */
HAL_StatusTypeDef MPU6050_ReadData(MPU6050_Data *data);

/* Average `samples` readings at least 5 ms apart; the car must stay still.
 * Returns HAL_ERROR on any I2C failure (bias is then left unchanged). */
HAL_StatusTypeDef MPU6050_CalibrateGyro(uint16_t samples);
void MPU6050_GetGyroBias(float bias_dps[3]);

/* Read one sample, update the complementary pitch filter over dt_s seconds,
 * and optionally copy the sample into out_data (may be NULL).
 * The first call, dt_s <= 0 or gaps > 250 ms seed the angle from gravity. */
HAL_StatusTypeDef MPU6050_UpdateAngle(float dt_s, MPU6050_Data *out_data);

/* Pitch in degrees. Check nose-up direction after mounting the module. */
float MPU6050_GetAngleDeg(void);
void MPU6050_ResetAngle(void);

/* Re-init the STM32 I2C1 peripheral and probe; no SCL bus-clear pulses. */
HAL_StatusTypeDef MPU6050_BusRecover(void);

#endif /* APP_MPU6050_H */
