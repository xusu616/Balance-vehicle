#include "mpu6050.h"
#include "i2c.h"
#include <math.h>
#include <string.h>

/* Register map (MPU6050 / MPU6500 compatible subset) */
#define MPU6050_REG_SMPLRT_DIV      0x19U
#define MPU6050_REG_CONFIG          0x1AU
#define MPU6050_REG_GYRO_CONFIG     0x1BU
#define MPU6050_REG_ACCEL_CONFIG    0x1CU
#define MPU6050_REG_ACCEL_XOUT_H    0x3BU
#define MPU6050_REG_GYRO_XOUT_H     0x43U
#define MPU6050_REG_PWR_MGMT_1      0x6BU
#define MPU6050_REG_WHO_AM_I        0x75U

#define MPU6050_I2C_ADDR_8BIT       (MPU6050_I2C_ADDR_7BIT << 1)
#define MPU6050_I2C_TIMEOUT_MS      20U
#define MPU6050_ACCEL_LSB_PER_G     8192.0f  /* +-4 g range */
#define MPU6050_GYRO_LSB_PER_DPS    65.5f    /* +-500 dps range */
#define MPU6050_RAD_TO_DEG          (180.0f / 3.14159265f)
#define MPU6050_SAMPLE_PERIOD_MS   5U       /* 200 Hz sensor register update */
#define MPU6050_MAX_FILTER_DT_S    0.25f

static float gyro_bias_dps[3];
static float pitch_deg;
static uint8_t pitch_initialized;

static HAL_StatusTypeDef WriteReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR_8BIT, reg,
                             I2C_MEMADD_SIZE_8BIT, &value, 1U,
                             MPU6050_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR_8BIT, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len,
                            MPU6050_I2C_TIMEOUT_MS);
}

static int16_t Be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

HAL_StatusTypeDef MPU6050_ReadWhoAmI(uint8_t *who_am_i)
{
    if (who_am_i == NULL) {
        return HAL_ERROR;
    }
    return ReadRegs(MPU6050_REG_WHO_AM_I, who_am_i, 1U);
}

HAL_StatusTypeDef MPU6050_Init(void)
{
    uint8_t who = 0U;
    uint8_t attempt;

    /* The module can need a few tens of ms after power-up before it ACKs. */
    for (attempt = 0U; attempt < 3U; attempt++) {
        if (MPU6050_ReadWhoAmI(&who) == HAL_OK) {
            break;
        }
        HAL_Delay(50U);
    }
    if (attempt == 3U) {
        return HAL_ERROR;
    }
    if (who != MPU6050_WHO_AM_I_MPU6050 &&
        who != MPU6050_WHO_AM_I_MPU6500 &&
        who != MPU6050_WHO_AM_I_MPU9250) {
        return HAL_ERROR;
    }

    if (WriteReg(MPU6050_REG_PWR_MGMT_1, 0x80U) != HAL_OK) { /* device reset */
        return HAL_ERROR;
    }
    HAL_Delay(100U);
    if (WriteReg(MPU6050_REG_PWR_MGMT_1, 0x01U) != HAL_OK) { /* wake, PLL from X gyro */
        return HAL_ERROR;
    }
    HAL_Delay(10U);

    if (WriteReg(MPU6050_REG_SMPLRT_DIV, 0x04U) != HAL_OK) {   /* 1 kHz / 5 = 200 Hz */
        return HAL_ERROR;
    }
    if (WriteReg(MPU6050_REG_CONFIG, 0x03U) != HAL_OK) {       /* DLPF ~42 Hz */
        return HAL_ERROR;
    }
    if (WriteReg(MPU6050_REG_GYRO_CONFIG, 0x08U) != HAL_OK) {  /* +-500 dps */
        return HAL_ERROR;
    }
    if (WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x08U) != HAL_OK) { /* AFS_SEL=1, +-4 g */
        return HAL_ERROR;
    }

    memset(gyro_bias_dps, 0, sizeof(gyro_bias_dps));
    MPU6050_ResetAngle();
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadData(MPU6050_Data *data)
{
    uint8_t buf[14];
    int16_t ax, ay, az, temp, gx, gy, gz;

    if (data == NULL) {
        return HAL_ERROR;
    }
    if (ReadRegs(MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf)) != HAL_OK) {
        return HAL_ERROR;
    }

    ax = Be16(&buf[0]);
    ay = Be16(&buf[2]);
    az = Be16(&buf[4]);
    temp = Be16(&buf[6]);
    gx = Be16(&buf[8]);
    gy = Be16(&buf[10]);
    gz = Be16(&buf[12]);

    data->accel_g[0] = (float)ax / MPU6050_ACCEL_LSB_PER_G;
    data->accel_g[1] = (float)ay / MPU6050_ACCEL_LSB_PER_G;
    data->accel_g[2] = (float)az / MPU6050_ACCEL_LSB_PER_G;

    data->gyro_dps[0] = (float)gx / MPU6050_GYRO_LSB_PER_DPS - gyro_bias_dps[0];
    data->gyro_dps[1] = (float)gy / MPU6050_GYRO_LSB_PER_DPS - gyro_bias_dps[1];
    data->gyro_dps[2] = (float)gz / MPU6050_GYRO_LSB_PER_DPS - gyro_bias_dps[2];

    data->temp_c = (float)temp / 340.0f + 36.53f;
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_CalibrateGyro(uint16_t samples)
{
    uint8_t buf[6];
    int64_t sum[3] = {0, 0, 0};
    uint16_t i;

    if (samples == 0U) {
        return HAL_ERROR;
    }
    for (i = 0U; i < samples; i++) {
        if (ReadRegs(MPU6050_REG_GYRO_XOUT_H, buf, sizeof(buf)) != HAL_OK) {
            return HAL_ERROR;
        }
        sum[0] += Be16(&buf[0]);
        sum[1] += Be16(&buf[2]);
        sum[2] += Be16(&buf[4]);
        HAL_Delay(MPU6050_SAMPLE_PERIOD_MS);
    }

    gyro_bias_dps[0] = (float)sum[0] / ((float)samples * MPU6050_GYRO_LSB_PER_DPS);
    gyro_bias_dps[1] = (float)sum[1] / ((float)samples * MPU6050_GYRO_LSB_PER_DPS);
    gyro_bias_dps[2] = (float)sum[2] / ((float)samples * MPU6050_GYRO_LSB_PER_DPS);
    MPU6050_ResetAngle();
    return HAL_OK;
}

void MPU6050_GetGyroBias(float bias_dps[3])
{
    if (bias_dps != NULL) {
        bias_dps[0] = gyro_bias_dps[0];
        bias_dps[1] = gyro_bias_dps[1];
        bias_dps[2] = gyro_bias_dps[2];
    }
}

HAL_StatusTypeDef MPU6050_UpdateAngle(float dt_s, MPU6050_Data *out_data)
{
    MPU6050_Data data;
    float rate_dps;
    float forward_g;
    float accel_angle;

    if (MPU6050_ReadData(&data) != HAL_OK) {
        pitch_initialized = 0U;
        return HAL_ERROR;
    }

#if MPU6050_PITCH_AXIS == MPU6050_PITCH_AXIS_Y
    rate_dps = data.gyro_dps[1];
    forward_g = -data.accel_g[0]; /* Positive Y rotation tilts +Z toward -X in sensor coordinates. */
#else
    rate_dps = data.gyro_dps[0];
    forward_g = data.accel_g[1];
#endif
    accel_angle = MPU6050_PITCH_ACCEL_SIGN *
                  atan2f(forward_g, data.accel_g[2]) * MPU6050_RAD_TO_DEG;

    if (pitch_initialized == 0U || !(dt_s > 0.0f && dt_s <= MPU6050_MAX_FILTER_DT_S)) {
        pitch_deg = accel_angle;
        pitch_initialized = 1U;
    } else {
        const float alpha = MPU6050_FILTER_TAU_S / (MPU6050_FILTER_TAU_S + dt_s);
        pitch_deg = alpha *
                    (pitch_deg + MPU6050_PITCH_RATE_SIGN * rate_dps * dt_s) +
                    (1.0f - alpha) * accel_angle;
    }

    if (out_data != NULL) {
        *out_data = data;
    }
    return HAL_OK;
}

float MPU6050_GetAngleDeg(void)
{
    return pitch_deg;
}

void MPU6050_ResetAngle(void)
{
    pitch_deg = 0.0f;
    pitch_initialized = 0U;
}

HAL_StatusTypeDef MPU6050_BusRecover(void)
{
    uint8_t who = 0U;

    (void)HAL_I2C_DeInit(&hi2c1);
    MX_I2C1_Init();
    HAL_Delay(10U);
    return MPU6050_ReadWhoAmI(&who);
}
