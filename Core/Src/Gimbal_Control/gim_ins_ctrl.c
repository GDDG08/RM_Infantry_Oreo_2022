/*
 * @Project      : RM_Infantry_Neptune
 * @FilePath     : \infantry_-neptune\Core\Src\Gimbal_Control\gim_ins_ctrl.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 20:00:25
 */

#include "gim_ins_ctrl.h"
#include "cmsis_os.h"
#include "kalman_alg.h"

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)

#define INS_TASK_PERIOD 1
#define dt 0.0025f

volatile uint8_t ins_flag = 0;

TaskHandle_t INS_task_local_handler;

float Ins_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float ins_temp_pid_param[3] = {60, 40, 20};
uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

PID_PIDTypeDef Ins_ImuTempPid;
PID_PIDParamTypeDef Ins_ImuTempPidParam;
PWM_PWMHandleTypeDef Ins_ImuTempPwm;

Kalman_KalmanTypeDef Ins_ImuKF;
float ins_yaw_bias = 0;

INS_IMUDataTypeDef INS_IMUData;

/**
 * @brief          INS task
 * @param          NULL
 * @retval         NULL
 */
void Ins_Task(void const* argument) {
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));
    while (!GLOBAL_INIT_FLAG) {
        osDelay(1);
    }

    ins_flag |= (1 << INS_INIT_SHIFT);

    for (;;) {
#if __FN_IF_ENABLE(__IMU_HI22X)
        vTaskDelete(INS_task_local_handler);

#else
        // wait spi DMA tansmit done
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {
        }
        Ins_DecodeIMUData();

        INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();
        MAG_MAGDataTypeDef* mag = MAG_GetMAGDataPtr();

        Ins_TempControl();
        AHRS_MahonyUpdate(Ins_quat, -imu->speed.pitch, imu->speed.row, -imu->speed.yaw,
                          -imu->accel.pitch, imu->accel.row, -imu->accel.yaw,
                          mag->mag_x, mag->mag_y, mag->mag_z);

        imu->last_angle.pitch = imu->now_angle.pitch;
        imu->last_angle.yaw = imu->now_angle.yaw;
        imu->last_angle.row = imu->now_angle.row;

        AHRS_GetAngle(Ins_quat, &imu->now_angle.yaw, &imu->now_angle.row, &imu->now_angle.pitch);

        Ins_ImuKF.MeasuredVector[0] = imu->now_angle.yaw;
        Ins_ImuKF.ControlVector[0] = imu->speed.yaw;

        Kalman_FilterUpdate(&Ins_ImuKF);

        imu->now_angle.yaw = Ins_ImuKF.FilteredValue[0];
        ins_yaw_bias = Ins_ImuKF.FilteredValue[1];

        ins_flag &= INS_CLEAR_FINISH_FLAG;

        if (imu->now_angle.pitch - imu->last_angle.pitch < -181)
            imu->count.pitch++;
        if (imu->now_angle.pitch - imu->last_angle.pitch > 181)
            imu->count.pitch--;
        imu->angle.pitch = (float)imu->count.pitch * 360.0f + imu->now_angle.pitch;

        if (imu->now_angle.yaw - imu->last_angle.yaw < -181)
            imu->count.yaw++;
        if (imu->now_angle.yaw - imu->last_angle.yaw > 181)
            imu->count.yaw--;
        imu->angle.yaw = (float)imu->count.yaw * 360.0f + imu->now_angle.yaw;

        if (imu->now_angle.row - imu->last_angle.row < -181)
            imu->count.row++;
        if (imu->now_angle.row - imu->last_angle.row > 181)
            imu->count.row--;
        imu->angle.row = (float)imu->count.row * 360.0f + imu->now_angle.row;

        imu->angle.pitch = imu->angle.pitch + imu->pitch_angle_offset;
        imu->angle.yaw = imu->angle.yaw - imu->yaw_angle_offset;

#endif

        osDelay(INS_TASK_PERIOD);
    }
}

/**
 * @brief          Ins Spi Init
 * @param          pvParameters: NULL
 * @retval         NULL
 */
void Ins_InsInit() {
    // wait a time
    HAL_Delay(INS_TASK_INIT_TIME);

    Ins_InitIMU();

#if __FN_IF_ENABLE(__IMU_HI22X)
#else
    MAG_Init();
    PWM_InitPWM(&Ins_ImuTempPwm, &htim12, TIM_CHANNEL_2);
    PWM_StartPWM(&Ins_ImuTempPwm);

    INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();
    MAG_MAGDataTypeDef* mag = MAG_GetMAGDataPtr();

    if ((imu->state != INS_STATE_ERROR) && (imu->state != INS_STATE_LOST)) {
        Ins_DecodeIMUData();
    }
    if ((mag->state != MAG_STATE_ERROR) && (mag->state != MAG_STATE_LOST))
        MAG_MAGUpdate();

    PID_InitPIDParam(&Ins_ImuTempPidParam, ins_temp_pid_param[0], ins_temp_pid_param[1], ins_temp_pid_param[2],
                     1000, 1000, 0, 0, 0, 0, 0, 0, PID_POSITION);
    AHRS_Init(Ins_quat);

    Spi_Init(Const_BMI055_SPI_HANDLER);

    Spi_DMAInit(Const_BMI055_SPI_HANDLER, (uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    static float P_Init[4] = {
        0.005, 0.005,
        0.005, 0.005};
    static float F_Init[4] = {
        1, -dt,
        0, 1};
    static float Q_Init[4] = {
        0.001,
        0,
        0,
        0.005,
    };
    static float H_Init[2] = {
        1,
        0,
    };
    static float R_Init[1] = {
        0.056,
    };
    static float B_Init[2] = {
        dt,
        0,
    };
    static float state_min_variance[2] = {0.05, 0.05};

    Ins_ImuKF.UseAutoAdjustment = 0;
    Kalman_FilterInit(&Ins_ImuKF, 2, 1, 1);

    memcpy(Ins_ImuKF.B_data, B_Init, sizeof(B_Init));
    memcpy(Ins_ImuKF.P_data, P_Init, sizeof(P_Init));
    memcpy(Ins_ImuKF.F_data, F_Init, sizeof(F_Init));
    memcpy(Ins_ImuKF.Q_data, Q_Init, sizeof(Q_Init));
    memcpy(Ins_ImuKF.R_data, R_Init, sizeof(R_Init));
    memcpy(Ins_ImuKF.H_data, H_Init, sizeof(H_Init));
    memcpy(Ins_ImuKF.StateMinVariance, state_min_variance, sizeof(state_min_variance));
#endif
}

/**
 * @brief      Initialization IMU
 * @param      NULL
 * @retval     NULL
 */
void Ins_InitIMU() {
    INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();
    uint8_t err = 0;

#if __FN_IF_ENABLE(__IMU_BMI055)
    err = BMI055_Init();
#endif
#if __FN_IF_ENABLE(__IMU_BMI88)
    err = BMI088_Init();
#endif
#if __FN_IF_ENABLE(__IMU_HI22X)
    err = HI22X_Init();
#endif

    if (err) {
        imu->state = INS_STATE_CONNECTED;
        imu->last_update_time = HAL_GetTick();
    } else {
        imu->state = INS_STATE_LOST;
    }
}

/**
 * @brief      Get pinter to the IMU data object
 * @param      NULL
 * @retval     Pointer to IMU data object
 */
INS_IMUDataTypeDef* Ins_GetIMUDataPtr() {
    return &INS_IMUData;
}

/**
 * @brief      Judge IMU offline
 * @param      NULL
 * @retval     Offline or not��1 is offline��0 is not��
 */
uint8_t Ins_IsIMUOffline() {
    INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();

    uint8_t ret;
#if __FN_IF_ENABLE(__IMU_BMI055)
    ret = BMI055_IsBMI055Offline();
#endif
#if __FN_IF_ENABLE(__IMU_BMI88)
    ret = BMI088_IsBMI088Offline();
#endif
#if __FN_IF_ENABLE(__IMU_HI22X)
    ret = HI22X_IsHI22XOffline();
#endif
    if (ret == INS_STATE_LOST) {
        imu->state = INS_STATE_LOST;
    }

    return imu->state == INS_STATE_LOST;
}

/**
 * @brief      IMU decode data function    ��For BMI0xx)
 * @param      NULL
 * @retval     NULL
 */
void Ins_DecodeIMUData() {
    INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();
    MAG_MAGDataTypeDef* mag = MAG_GetMAGDataPtr();
#if __FN_IF_ENABLE(__IMU_BMI055)
    BMI055_BMI055DataTypeDef* bmi055 = BMI055_GetBMI055DataPtr();
    imu->speed.pitch = bmi055->gyro.x;
    imu->speed.row = bmi055->gyro.y;
    imu->speed.yaw = bmi055->gyro.z;
    imu->accel.pitch = bmi055->accel.x;
    imu->accel.row = bmi055->accel.y;
    imu->accel.yaw = bmi055->accel.z;
    imu->temperature = bmi055->temperature;
#if __FN_IF_ENABLE(__IMU_NINE_AXIS)
    imu->mag.pitch = mag->mag_x;
    imu->mag.row = mag->mag_y;
    imu->mag.yaw = mag->mag_z;
#endif
#endif
#if __FN_IF_ENABLE(__IMU_BMI088)
    BMI088_BMI088DataTypeDef* bmi088 = BMI088_GetBMI088DataPtr();
    imu->speed.pitch = bmi088->gyro.x;
    imu->speed.row = bmi088->gyro.y;
    imu->speed.yaw = bmi088->gyro.z;
    imu->accel.pitch = bmi088->accel.x;
    imu->accel.row = bmi088->accel.y;
    imu->accel.yaw = bmi088->accel.z;
    imu->temperature = bmi088->temperature;
#endif
#if __FN_IF_ENABLE(__IMU_HI22X)
    HI22X_HI22XDataTypeDef* hi22x = HI22X_GetHI22XDataPtr();
    imu->angle.pitch = hi22x->angle.pitch;
    imu->angle.row = hi22x->angle.row;
    imu->angle.yaw = hi22x->angle.yaw;
    imu->speed.pitch = hi22x->speed.pitch;
    imu->speed.row = hi22x->speed.row;
    imu->speed.yaw = hi22x->speed.yaw;
#endif
}

/**
 * @brief          Control the temperature of IMU
 * @param          NULL
 * @retval         NULL
 */
void Ins_TempControl() {
    INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();

    uint16_t tempPWM;

    PID_SetPIDRef(&Ins_ImuTempPid, IMU_REF_TEMP);
    PID_SetPIDFdb(&Ins_ImuTempPid, imu->temperature);
    PID_CalcPID(&Ins_ImuTempPid, &Ins_ImuTempPidParam);
    if (Ins_ImuTempPid.output < 0.0f) {
        Ins_ImuTempPid.output = 0.0f;
    }
    tempPWM = Ins_ImuTempPid.output / 1000.0f;
    PWM_SetPWMDuty(&Ins_ImuTempPwm, tempPWM);
}

/**
 * @brief          Ins Gpio Exit Callbcak
 * @param          GPIO_Pin :Specifies the pins connected EXTI line
 * @retval         NULL
 */
void Ins_GPIOExitCallback(GPIO_GPIOTypeDef* gpio) {
    if (gpio == BMI_INT1) {
        if (ins_flag & (1 << INS_INIT_SHIFT)) {
            if ((!(ins_flag & (1 << INS_GYRO_UPDATE_SHIFT))) && (!(ins_flag & (1 << INS_ACCEL_FINISH_SHIFT)))) {
                ins_flag |= (1 << INS_ACCEL_UPDATE_SHIFT);
                Ins_ImuDMACmd();
            }
        }
    }

    else if (gpio == BMI_INT3) {
        if (ins_flag & (1 << INS_INIT_SHIFT)) {
            if ((!(ins_flag & (1 << INS_ACCEL_UPDATE_SHIFT))) && (!(ins_flag & (1 << INS_GYRO_FINISH_SHIFT)))) {
                ins_flag |= (1 << INS_GYRO_UPDATE_SHIFT);
                Ins_ImuDMACmd();
            }
        }
    }

    else if (gpio == IST8310_DRDY) {
        if (ins_flag & (1 << INS_INIT_SHIFT)) {
            if (!(ins_flag & (1 << INS_MAG_FINISH_SHIFT))) {
                ins_flag |= (1 << INS_MAG_UPDATE_SHIFT);
                MAG_MAGUpdate();
                ins_flag &= ~(1 << INS_MAG_UPDATE_SHIFT);
                ins_flag |= (1 << INS_MAG_FINISH_SHIFT);
            }
        }
    }
}

/**
 * @brief          Ins Spi DMA Recive Control
 * @param          pvParameters: NULL
 * @retval         NULL
 */
static void Ins_ImuDMACmd() {
#if __FN_IF_ENABLE(__IMU_HI22X)
#else
    if ((ins_flag & (1 << INS_GYRO_UPDATE_SHIFT)) && !(Const_BMI055_SPI_HANDLER->hdmatx->Instance->CR & DMA_SxCR_EN) && !(Const_BMI055_SPI_HANDLER->hdmarx->Instance->CR & DMA_SxCR_EN)) {
        GPIO_Reset(CS_GYRO);
        Spi_DMAEnable(Const_BMI055_SPI_HANDLER, (uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        return;
    }
    if ((ins_flag & (1 << INS_ACCEL_UPDATE_SHIFT)) && !(Const_BMI055_SPI_HANDLER->hdmatx->Instance->CR & DMA_SxCR_EN) && !(Const_BMI055_SPI_HANDLER->hdmarx->Instance->CR & DMA_SxCR_EN)) {
        GPIO_Reset(CS_ACCEL);
        Spi_DMAEnable(Const_BMI055_SPI_HANDLER, (uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        return;
    }
#endif
}

/**
 * @brief          Ins Spi DMA Recive interrupt Control
 * @param          pvParameters: NULL
 * @retval         NULL
 */
void Ins_DMAIRQHandler() {
#if __FN_IF_ENABLE(__IMU_HI22X)
#else
    if (__HAL_DMA_GET_FLAG(Const_BMI055_SPI_HANDLER->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(Const_BMI055_SPI_HANDLER->hdmarx)) != RESET) {
        __HAL_DMA_CLEAR_FLAG(Const_BMI055_SPI_HANDLER->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(Const_BMI055_SPI_HANDLER->hdmarx));

        // gyro read over
        if (ins_flag & (1 << INS_GYRO_UPDATE_SHIFT)) {
            GPIO_Set(CS_GYRO);
            BMI055_GyroReadOver(gyro_dma_rx_buf);
            ins_flag &= ~(1 << INS_GYRO_UPDATE_SHIFT);
            ins_flag |= (1 << INS_GYRO_FINISH_SHIFT);
        }

        // accel read over
        if (ins_flag & (1 << INS_ACCEL_UPDATE_SHIFT)) {
            GPIO_Set(CS_ACCEL);
            BMI055_AccelReadOver(accel_dma_rx_buf);
            ins_flag &= ~(1 << INS_ACCEL_UPDATE_SHIFT);
            ins_flag |= (1 << INS_ACCEL_FINISH_SHIFT);
        }

        Ins_ImuDMACmd();

        if ((ins_flag & (1 << INS_GYRO_FINISH_SHIFT)) && (ins_flag & (1 << INS_ACCEL_FINISH_SHIFT))) {
            if (ins_flag & (1 << INS_INIT_SHIFT)) {
                // wake up the task
                if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
                    static BaseType_t xHigherPriorityTaskWoken;
                    vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                }
            }
        }
    }
#endif
}

#endif
