#include "motor_test.h"
#include "motor.h"
#include "encoder.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define TEST_PULSE_MS        400U
#define LONG_PULSE_MS       5000U /* a/A (Left) & t/T (Right): 5s run */
#define BOTH_PULSE_MS       3000U /* b/B (Both wheels): 3s run */
#define TEST_REST_MS        1000U
#define HEARTBEAT_MS        1000U

typedef enum {
    TARGET_LEFT = 0,
    TARGET_RIGHT = 1,
    TARGET_BOTH = 2
} TestTarget;

static uint16_t test_duty = 250U; /* Default 25% duty cycle */
static uint8_t running;
static uint32_t started_at;
static uint32_t active_pulse_ms = TEST_PULSE_MS;
static uint32_t stopped_at;
static uint32_t last_heartbeat_at;
static int32_t pulse_start_enc_r;
static int32_t pulse_start_enc_l;

static void UartPrint(const char *msg)
{
    (void)HAL_UART_Transmit(&huart2, (const uint8_t *)msg,
                            (uint16_t)strlen(msg), 100U);
}

static void PrintHelp(void)
{
    static const char text[] =
        "\r\n=== Balance-Vehicle Dual Motor & Encoder Test (USART2 115200) ===\r\n"
        "Left  (AO1/AO2, PA8, PB12/13, EncA PA0/1): l/L = 0.4s +/- | a/A = 5s +/-\r\n"
        "Right (BO1/BO2, PA9, PB14/15, EncB PA6/7): r/R = 0.4s +/- | t/T = 5s +/-\r\n"
        "Both  (Left + Right together)            : b/B = 3.0s +/-\r\n"
        "Duty / Control: 1..5 = 10%..50% | x or space = Emergency Stop\r\n"
        "Utils         : c = Clear encoders | d = GPIO/TIM1 state | ? = Help\r\n";
    UartPrint(text);
}

static void PrintDriverState(void)
{
    static char buf[192];
    (void)snprintf(buf, sizeof(buf),
                   ">> LEFT(A) : AIN1=%u AIN2=%u CCR1=%lu CH1=%u | RIGHT(B): BIN1=%u BIN2=%u CCR2=%lu CH2=%u\r\n",
                   (unsigned int)HAL_GPIO_ReadPin(LEFT_AIN1_GPIO_Port, LEFT_AIN1_Pin),
                   (unsigned int)HAL_GPIO_ReadPin(LEFT_AIN2_GPIO_Port, LEFT_AIN2_Pin),
                   (unsigned long)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1),
                   (unsigned int)((htim1.Instance->CCER & TIM_CCER_CC1E) != 0U),
                   (unsigned int)HAL_GPIO_ReadPin(RIGHT_BIN1_GPIO_Port, RIGHT_BIN1_Pin),
                   (unsigned int)HAL_GPIO_ReadPin(RIGHT_BIN2_GPIO_Port, RIGHT_BIN2_Pin),
                   (unsigned long)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2),
                   (unsigned int)((htim1.Instance->CCER & TIM_CCER_CC2E) != 0U));
    UartPrint(buf);
    (void)snprintf(buf, sizeof(buf),
                   ">> COMMON  : STBY=%u ARR=%lu CEN=%u MOE=%u | GPIOB ODR=%04lX IDR=%04lX\r\n",
                   (unsigned int)HAL_GPIO_ReadPin(TB6612_STBY_GPIO_Port, TB6612_STBY_Pin),
                   (unsigned long)__HAL_TIM_GET_AUTORELOAD(&htim1),
                   (unsigned int)((htim1.Instance->CR1 & TIM_CR1_CEN) != 0U),
                   (unsigned int)((htim1.Instance->BDTR & TIM_BDTR_MOE) != 0U),
                   (unsigned long)GPIOB->ODR,
                   (unsigned long)GPIOB->IDR);
    UartPrint(buf);
}

static uint8_t SingleMotorPinsMatch(MotorId motor, int16_t duty)
{
    GPIO_TypeDef *port = (motor == MOTOR_LEFT) ? LEFT_AIN1_GPIO_Port : RIGHT_BIN1_GPIO_Port;
    uint16_t pin1 = (motor == MOTOR_LEFT) ? LEFT_AIN1_Pin : RIGHT_BIN1_Pin;
    uint16_t pin2 = (motor == MOTOR_LEFT) ? LEFT_AIN2_Pin : RIGHT_BIN2_Pin;
    return HAL_GPIO_ReadPin(port, pin1) == (duty > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET) &&
           HAL_GPIO_ReadPin(port, pin2) == (duty > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static uint8_t CheckControlPins(TestTarget target, int16_t duty)
{
    if (HAL_GPIO_ReadPin(TB6612_STBY_GPIO_Port, TB6612_STBY_Pin) != GPIO_PIN_SET) {
        UartPrint(">> ERROR: STBY(PB0) readback is 0; stopped! Check PB0/VCC wiring.\r\n");
        return 0U;
    }
    if ((target == TARGET_LEFT || target == TARGET_BOTH) &&
        SingleMotorPinsMatch(MOTOR_LEFT, duty) == 0U) {
        UartPrint(">> ERROR: LEFT AIN1/AIN2(PB12/PB13) readback mismatch; stopped!\r\n");
        return 0U;
    }
    if ((target == TARGET_RIGHT || target == TARGET_BOTH) &&
        SingleMotorPinsMatch(MOTOR_RIGHT, duty) == 0U) {
        UartPrint(">> ERROR: RIGHT BIN1/BIN2(PB14/PB15) readback mismatch; stopped!\r\n");
        return 0U;
    }
    return 1U;
}

static void StartPulse(TestTarget target, int16_t duty, uint32_t now, uint32_t duration_ms)
{
    static char buf[96];
    const char *name;
    HAL_StatusTypeDef status = Motor_Enable();

    (void)Encoder_GetDelta(MOTOR_LEFT);
    (void)Encoder_GetDelta(MOTOR_RIGHT);
    pulse_start_enc_l = Encoder_GetTotal(MOTOR_LEFT);
    pulse_start_enc_r = Encoder_GetTotal(MOTOR_RIGHT);

    if (status == HAL_OK) {
        if (target == TARGET_LEFT) {
            status = Motor_SetSpeed(MOTOR_LEFT, duty);
        } else if (target == TARGET_RIGHT) {
            status = Motor_SetSpeed(MOTOR_RIGHT, duty);
        } else {
            status = Motor_SetSpeed(MOTOR_LEFT, duty);
            if (status == HAL_OK) {
                status = Motor_SetSpeed(MOTOR_RIGHT, duty);
            }
        }
    }

    if (status != HAL_OK) {
        Motor_EmergencyStop();
        running = 0U;
        stopped_at = now;
        UartPrint(">> ERROR: Motor enable/set failed!\r\n");
        return;
    }

    started_at = HAL_GetTick();
    active_pulse_ms = duration_ms;
    running = 1U;

    name = (target == TARGET_LEFT)  ? "LEFT(AO1/AO2)" :
           (target == TARGET_RIGHT) ? "RIGHT(BO1/BO2)" : "BOTH(A+B)";
    (void)snprintf(buf, sizeof(buf),
                   ">> RUN %s: duty=%+d%% (%lums)\r\n",
                   name, (int)(duty / 10), (unsigned long)duration_ms);
    UartPrint(buf);
    PrintDriverState();

    if (CheckControlPins(target, duty) == 0U) {
        Motor_EmergencyStop();
        running = 0U;
        stopped_at = HAL_GetTick();
    }
}

void MotorTest_Init(void)
{
    running = 0U;
    test_duty = 250U;
    stopped_at = HAL_GetTick();
    last_heartbeat_at = stopped_at;
    pulse_start_enc_l = 0;
    pulse_start_enc_r = 0;

    Motor_EmergencyStop();
    (void)Encoder_Init();

    PrintHelp();

    {
        static char rst[96];
        uint32_t csr = RCC->CSR;
        (void)snprintf(rst, sizeof(rst),
                       ">> BOOT reset cause:%s%s%s%s%s\r\n",
                       (csr & RCC_CSR_PORRSTF)  ? " POWER(POR/BOR)" : "",
                       (csr & RCC_CSR_PINRSTF)  ? " NRST-pin" : "",
                       (csr & RCC_CSR_SFTRSTF)  ? " software" : "",
                       (csr & RCC_CSR_IWDGRSTF) ? " IWDG" : "",
                       (csr & RCC_CSR_LPWRRSTF) ? " low-power" : "");
        UartPrint(rst);
        RCC->CSR |= RCC_CSR_RMVF;
    }
}

void MotorTest_Poll(void)
{
    uint8_t command;
    TestTarget target;
    int16_t duty;
    uint32_t now = HAL_GetTick();
    uint32_t duration_ms = TEST_PULSE_MS;

    /* 1. Stop pulse when deadline expires and report both encoder deltas */
    if (running != 0U && (uint32_t)(now - started_at) >= active_pulse_ms) {
        static char buf[112];
        Motor_EmergencyStop();
        running = 0U;
        stopped_at = now;
        (void)Encoder_GetDelta(MOTOR_LEFT);
        (void)Encoder_GetDelta(MOTOR_RIGHT);
        (void)snprintf(buf, sizeof(buf),
                       ">> STOP | Pulse delta -> EncA(Left PA0/1)=%+ld, EncB(Right PA6/7)=%+ld\r\n",
                       (long)(Encoder_GetTotal(MOTOR_LEFT) - pulse_start_enc_l),
                       (long)(Encoder_GetTotal(MOTOR_RIGHT) - pulse_start_enc_r));
        UartPrint(buf);
    }

    /* 2. Periodic 1s status: print BOTH Left (EncA) and Right (EncB) counts & RPM */
    if ((uint32_t)(now - last_heartbeat_at) >= HEARTBEAT_MS) {
        uint32_t dt_ms = (uint32_t)(now - last_heartbeat_at);
        int16_t d_l = Encoder_GetDelta(MOTOR_LEFT);
        int16_t d_r = Encoder_GetDelta(MOTOR_RIGHT);
        int32_t tot_l = Encoder_GetTotal(MOTOR_LEFT);
        int32_t tot_r = Encoder_GetTotal(MOTOR_RIGHT);
        static char buf[140];

        last_heartbeat_at = now;

        (void)snprintf(buf, sizeof(buf),
                       "[%lus] Duty=%u%% | EncA(L PA0/1): tot=%ld spd=%+d %+ldrpm | EncB(R PA6/7): tot=%ld spd=%+d %+ldrpm\r\n",
                       (unsigned long)(now / 1000U),
                       (unsigned int)(test_duty / 10U),
                       (long)tot_l, (int)d_l, (long)Encoder_DeltaToRPM(d_l, dt_ms),
                       (long)tot_r, (int)d_r, (long)Encoder_DeltaToRPM(d_r, dt_ms));
        UartPrint(buf);
    }

    /* 3. Clear UART Overrun Error if multiple bytes arrived while busy */
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
    }

    /* 4. Non-blocking 1-byte UART receive */
    if (HAL_UART_Receive(&huart2, &command, 1U, 0U) != HAL_OK) {
        return;
    }
    if (command == '\r' || command == '\n') {
        return;
    }

    if (command == 'x' || command == ' ') {
        Motor_EmergencyStop();
        running = 0U;
        stopped_at = now;
        UartPrint(">> EMERGENCY STOP!\r\n");
        return;
    }
    if (command == 'd') {
        PrintDriverState();
        return;
    }
    if (running != 0U) {
        UartPrint(">> Busy running, ignored (press x to stop).\r\n");
        return;
    }
    if (command == '?') {
        PrintHelp();
        return;
    }
    if (command == 'c' || command == 'C') {
        Encoder_ResetAll();
        UartPrint(">> Both encoders cleared to 0.\r\n");
        return;
    }
    if (command >= '1' && command <= '5') {
        static char buf[64];
        test_duty = (uint16_t)(command - '0') * 100U;
        (void)snprintf(buf, sizeof(buf), ">> Duty set to %u%%\r\n",
                       (unsigned int)(test_duty / 10U));
        UartPrint(buf);
        return;
    }
    if ((uint32_t)(now - stopped_at) < TEST_REST_MS) {
        UartPrint(">> Wait 1s cool-down before next pulse.\r\n");
        return;
    }

    switch (command) {
    case 'l': target = TARGET_LEFT;  duty = (int16_t)test_duty;  duration_ms = TEST_PULSE_MS; break;
    case 'L': target = TARGET_LEFT;  duty = -(int16_t)test_duty; duration_ms = TEST_PULSE_MS; break;
    case 'a': target = TARGET_LEFT;  duty = (int16_t)test_duty;  duration_ms = LONG_PULSE_MS; break;
    case 'A': target = TARGET_LEFT;  duty = -(int16_t)test_duty; duration_ms = LONG_PULSE_MS; break;
    case 'r': target = TARGET_RIGHT; duty = (int16_t)test_duty;  duration_ms = TEST_PULSE_MS; break;
    case 'R': target = TARGET_RIGHT; duty = -(int16_t)test_duty; duration_ms = TEST_PULSE_MS; break;
    case 't': target = TARGET_RIGHT; duty = (int16_t)test_duty;  duration_ms = LONG_PULSE_MS; break;
    case 'T': target = TARGET_RIGHT; duty = -(int16_t)test_duty; duration_ms = LONG_PULSE_MS; break;
    case 'b': target = TARGET_BOTH;  duty = (int16_t)test_duty;  duration_ms = BOTH_PULSE_MS; break;
    case 'B': target = TARGET_BOTH;  duty = -(int16_t)test_duty; duration_ms = BOTH_PULSE_MS; break;
    default:
        UartPrint(">> Unknown cmd (send ? for help)\r\n");
        return;
    }
    StartPulse(target, duty, now, duration_ms);
}
