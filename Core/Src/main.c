/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Private typedef -----------------------------------------------------------*/
#define MPU6050_ADDR 0x68
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define ACCEL_XOUT_H_REG 0x3B
#define GYRO_XOUT_H_REG 0x43

#define FILTER_SIZE 20
#define GYRO_THRESHOLD 0.5f
#define MAX_PWM 2000    // Giá trị PWM tối đa
#define MIN_PWM 0       // Giá trị PWM tối thiểu

typedef struct {
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];
} Kalman_t;

typedef struct {
    float Kp;           // Hệ số tỉ lệ
    float Ki;           // Hệ số tích phân
    float Kd;           // Hệ số vi phân
    float setpoint;     // Góc mục tiêu (thường là 0 để cân bằng)
    float error;        // Sai số hiện tại
    float last_error;   // Sai số trước đó
    float integral;     // Tổng tích phân
    float output;       // Giá trị điều khiển đầu ra
} PID_t;

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 1.0f,
    .angle = 0.0f,
    .bias = 0.0f,
    .P[0][0] = 0.0f,
    .P[0][1] = 0.0f,
    .P[1][0] = 0.0f,
    .P[1][1] = 0.0f
};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 1.0f,
    .angle = 0.0f,
    .bias = 0.0f,
    .P[0][0] = 0.0f,
    .P[0][1] = 0.0f,
    .P[1][0] = 0.0f,
    .P[1][1] = 0.0f
};

PID_t pid = {
    .Kp = 10.0f,        // Tăng để phản ứng nhanh hơn
    .Ki = 50.0f,       // Giảm để giảm tích lũy sai số
    .Kd = 1.0f,        // Tăng để cải thiện đáp ứng động
    .setpoint = 0.0f,  // Góc cân bằng mục tiêu
    .error = 0.0f,
    .last_error = 0.0f,
    .integral = 0.0f,
    .output = 0.0f
};

float gyroX_offset = 0.0f, gyroY_offset = 0.0f;
float pitch_offset = 0.0f, roll_offset = 0.0f;
float gx_buffer[FILTER_SIZE] = {0}, gy_buffer[FILTER_SIZE] = {0};
int filter_index = 0;
float gx_filtered = 0.0f, gy_filtered = 0.0f;

volatile int16_t Accel[3], Gyro[3];
volatile uint8_t data_ready = 0;
volatile int motor_state = 0;
volatile int16_t pwm_value = 0;
volatile uint8_t bluetooth_data[1]; // Buffer nhận dữ liệu Bluetooth
volatile uint8_t bt_command = 0;    // Lệnh từ Bluetooth

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void); // Thêm khởi tạo UART1

uint8_t MPU6050_Init(void);
void MPU6050_Read_Accel_Gyro(int16_t *AccelData, int16_t *GyroData);
void CalibrateGyro(int16_t *GyroData, int samples);
void CalibrateAngles(float *pitch_offset, float *roll_offset, int samples);
void FilterGyro(float *gx, float *gy);
float LowPassFilter(float new_value, float old_value, float alpha);
void ControlMotor(int16_t left_pwm, int16_t right_pwm);
float Kalman_GetAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt);
float PID_Compute(PID_t *pid, float input, float dt);
void ProcessBluetoothCommand(uint8_t command);

/* Private user code ---------------------------------------------------------*/
uint8_t MPU6050_Init(void)
{
    uint8_t check, Data;

    if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR << 1, 3, 1000) != HAL_OK) {
        return 1;
    }

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR << 1, WHO_AM_I_REG, 1, &check, 1, 1000);
    if (check != 0x68 && check != 0x70) {
        return 1;
    }

    Data = 0;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR << 1, PWR_MGMT_1_REG, 1, &Data, 1, 1000);
    return 0;
}

void MPU6050_Read_Accel_Gyro(int16_t *AccelData, int16_t *GyroData)
{
    uint8_t Rec_Data[14];
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR << 1, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, 1000) == HAL_OK) {
        AccelData[0] = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        AccelData[1] = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        AccelData[2] = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
        GyroData[0]  = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
        GyroData[1]  = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
        GyroData[2]  = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);
    }
}

void CalibrateGyro(int16_t *GyroData, int samples)
{
    gyroX_offset = 0.0f;
    gyroY_offset = 0.0f;
}

void CalibrateAngles(float *pitch_offset, float *roll_offset, int samples)
{
    *pitch_offset = 0.0f;
    *roll_offset = -1.5f;
}

void FilterGyro(float *gx, float *gy)
{
    gx_buffer[filter_index] = *gx;
    gy_buffer[filter_index] = *gy;

    float gx_sum = 0, gy_sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        gx_sum += gx_buffer[i];
        gy_sum += gy_buffer[i];
    }
    *gx = gx_sum / FILTER_SIZE;
    *gy = gy_sum / FILTER_SIZE;

    if (fabs(*gx) < GYRO_THRESHOLD) *gx = 0.0f;
    if (fabs(*gy) < GYRO_THRESHOLD) *gy = 0.0f;

    filter_index = (filter_index + 1) % FILTER_SIZE;
}

float LowPassFilter(float new_value, float old_value, float alpha)
{
    return alpha * new_value + (1.0f - alpha) * old_value;
}

void ControlMotor(int16_t left_pwm, int16_t right_pwm)
{
    // Giới hạn giá trị PWM
    if (left_pwm > MAX_PWM) left_pwm = MAX_PWM;
    if (left_pwm < -MAX_PWM) left_pwm = -MAX_PWM;
    if (right_pwm > MAX_PWM) right_pwm = MAX_PWM;
    if (right_pwm < -MAX_PWM) right_pwm = -MAX_PWM;

    // Điều khiển động cơ trái (Motor A: IN1, IN2, PWM CH1)
    if (left_pwm > 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // IN1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // IN2
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_pwm);
    } else if (left_pwm < 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // IN1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // IN2
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -left_pwm);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // IN1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // IN2
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    }

    // Điều khiển động cơ phải (Motor B: IN3, IN4, PWM CH2)
    if (right_pwm > 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);   // IN3
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // IN4
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, right_pwm);
    } else if (right_pwm < 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // IN3
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);   // IN4
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -right_pwm);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // IN3
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // IN4
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }
}

float Kalman_GetAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt)
{
    if (isnan(newAngle) || isnan(newRate) || isinf(newAngle) || isinf(newRate)) {
        return 0.0f;
    }

    float rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    float S = Kalman->P[0][0] + Kalman->R_measure;
    float K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    float y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}

float PID_Compute(PID_t *pid, float input, float dt)
{
    pid->error = pid->setpoint - input;
    float p_term = pid->Kp * pid->error;

    pid->integral += pid->error * dt;
    if (pid->integral > 100.0f) pid->integral = 100.0f;  // Giới hạn trên
    if (pid->integral < -100.0f) pid->integral = -100.0f; // Giới hạn dưới
    float i_term = pid->Ki * pid->integral;

    float derivative = (pid->error - pid->last_error) / dt;
    float d_term = pid->Kd * derivative;

    pid->output = p_term + i_term + d_term;

    if (pid->output > MAX_PWM) pid->output = MAX_PWM;
    if (pid->output < -MAX_PWM) pid->output = -MAX_PWM;

    pid->last_error = pid->error;

    return pid->output;
}

void ProcessBluetoothCommand(uint8_t command)
{
    switch (command) {
        case 'F': // Tiến
            bt_command = 1;
            break;
        case 'B': // Lùi
            bt_command = 2;
            break;
        case 'L': // Quay trái
            bt_command = 3;
            break;
        case 'R': // Quay phải
            bt_command = 4;
            break;
        case 'S': // Dừng
            bt_command = 0;
            break;
        default:
            bt_command = 0;
            break;
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_I2C1_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init(); // Khởi tạo UART1

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Khởi động PWM kênh 1
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Khởi động PWM kênh 2
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_UART_Receive_IT(&huart1, (uint8_t *)bluetooth_data, 1);

    float roll, pitch;
    float dt = 0.002f; // 500 Hz
    const float DEAD_ZONE = -1.5f; // Vùng chết ±5 độ
    int16_t base_pwm = 250; // Giá trị PWM cơ bản cho di chuyển

    if (MPU6050_Init() == 0) {
        CalibrateGyro((int16_t*)Gyro, 500);
        CalibrateAngles(&pitch_offset, &roll_offset, 500);

        while (1)
        {
            if (data_ready) {
                float ax = Accel[0] / 16384.0f;
                float ay = Accel[1] / 16384.0f;
                float az = Accel[2] / 16384.0f;
                float gx = (Gyro[0] / 131.0f) - gyroX_offset;
                float gy = (Gyro[1] / 131.0f) - gyroY_offset;

                FilterGyro(&gx, &gy);
                gx_filtered = LowPassFilter(gx, gx_filtered, 0.1f);
                gy_filtered = LowPassFilter(gy, gy_filtered, 0.1f);

                float accelRoll = (fabs(az) > 0.01f) ? atan2(ay, az) * 180 / M_PI : 0.0f;
                float accelPitch = (fabs(sqrt(ay*ay + az*az)) > 0.01f) ? atan2(-ax, sqrt(ay*ay + az*az)) * 180 / M_PI : 0.0f;

                roll = Kalman_GetAngle(&KalmanX, accelRoll - roll_offset, gx_filtered, dt);
                pitch = Kalman_GetAngle(&KalmanY, accelPitch - pitch_offset, gy_filtered, dt);

                float pid_output = PID_Compute(&pid, roll, dt);
                int16_t left_pwm = 0, right_pwm = 0;


                switch (bt_command) {
                    case 0: // Dừng
                        if (roll >= -DEAD_ZONE && roll <= DEAD_ZONE) {
                            left_pwm = 0;
                            right_pwm = 0;
                        } else {
                            left_pwm = (int16_t)pid_output;
                            right_pwm = (int16_t)pid_output;
                        }
                        break;
                    case 1: // Tiến
                        left_pwm = base_pwm + (int16_t)pid_output;
                        right_pwm = base_pwm + (int16_t)pid_output;
                        break;
                    case 2: // Lùi
                        left_pwm = -base_pwm + (int16_t)pid_output;
                        right_pwm = -base_pwm + (int16_t)pid_output;
                        break;
                    case 3: // Quay trái
                        left_pwm = -base_pwm + (int16_t)pid_output;
                        right_pwm = base_pwm + (int16_t)pid_output;
                        break;
                    case 4: // Quay phải
                        left_pwm = base_pwm + (int16_t)pid_output;
                        right_pwm = -base_pwm + (int16_t)pid_output;
                        break;
                }

                ControlMotor(left_pwm, right_pwm);
                data_ready = 0;
            }
        }
    } else {
        while (1) {
            HAL_Delay(1000);
        }
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 350;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);
    HAL_TIM_PWM_Init(&htim2);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 50;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 4;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 15999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim3);
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Cấu hình chân UART1 (PA9: TX, PA10: RX)
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        MPU6050_Read_Accel_Gyro((int16_t*)Accel, (int16_t*)Gyro);
        data_ready = 1;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        ProcessBluetoothCommand(bluetooth_data[0]);
        HAL_UART_Receive_IT(&huart1, (uint8_t *)bluetooth_data, 1);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
