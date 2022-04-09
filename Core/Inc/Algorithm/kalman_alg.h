/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Algorithm\kalman_alg.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-07-24 10:27:08
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-23 21:38:32
 */

#ifndef KALMAN_ALG_H
#define KALMAN_ALG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "math_alg.h"

typedef struct kf_t {
    float* FilteredValue;
    float* MeasuredVector;
    float* ControlVector;

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t* MeasurementMap;       // how measurement relates to the state
    float* MeasurementDegree;      // elements of each measurement in H
    float* MatR_DiagonalElements;  // variance for each measurement
    float* StateMinVariance;       // suppress filter excessive convergence
    uint8_t* temp;
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5, NonMeasurement;

    mat xhat;       // x(k|k)
    mat xhatminus;  // x(k|k-1)
    mat u;          // control vector u
    mat z;          // measurement vector z
    mat P;          // covariance matrix P(k|k)
    mat Pminus;     // covariance matrix P(k|k-1)
    mat F, FT;      // state transition matrix F FT
    mat B;          // control matrix B
    mat H, HT;      // measurement matrix H
    mat Q;          // process noise covariance matrix Q
    mat R;          // measurement noise covariance matrix R
    mat K;          // kalman gain  K
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;

    void (*User_Func0_f)(struct kf_t* kf);
    void (*User_Func1_f)(struct kf_t* kf);
    void (*User_Func2_f)(struct kf_t* kf);
    void (*User_Func3_f)(struct kf_t* kf);
    void (*User_Func4_f)(struct kf_t* kf);
    void (*User_Func5_f)(struct kf_t* kf);
    void (*User_Func6_f)(struct kf_t* kf);

    float *xhat_data, *xhatminus_data;
    float* u_data;
    float* z_data;
    float *P_data, *Pminus_data;
    float *F_data, *FT_data;
    float* B_data;
    float *H_data, *HT_data;
    float* Q_data;
    float* R_data;
    float* K_data;
    float *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1;
} Kalman_KalmanTypeDef;

extern uint16_t sizeof_float, sizeof_double;

void Kalman_FilterInit(Kalman_KalmanTypeDef* kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);
float* Kalman_FilterUpdate(Kalman_KalmanTypeDef* kf);
static void Kalman_Adjustment_H_K_R(Kalman_KalmanTypeDef* kf);

typedef struct {
    // KF Filter Parameters:
    mat KF_A, KF_C;
    mat XLast, Xpre, Xopt;
    mat PLast, Ppre, Popt;
    mat Q, R;
    mat Kf;

    // Cycle Time
    // float cvkf_t;
    float angle, angle_p_err, max_speed, min_speed;
    int switch_mode,
        measure_mode,
        targer_change;
} Kalman_CVKalmanTypeDef;

typedef struct {
    float KF_A[4], KF_C[2];
    float XLast[2], Xpre[2], Xopt[2];
    float PLast[4], Ppre[4], Popt[4];
    float Q[4], R[1];
    float Kf[2];
    // float cvkf_t;
} Kalman_CVKalmanInitDataTypeDef;

typedef struct {
    int total;         //???:????????
    int basicprocess;  // CVKF??????(????)
    int predict;       //??????????????
    int limit;         //?????????????
    int jumpjudge;     //????????
    int offset;
    int output;                 //??????????
    int lowfilter;              //?????????????
    int dead_domain_delta_ref;  //?????????????

} Kalman_CVKalmanControlTypeDef;

// CVKF Inner Function:
void Kalman_CVKalmanInitYawParam(Kalman_CVKalmanInitDataTypeDef* cvkf_data, float KF_T, float init_angle_yaw, float init_angle_speed);
// void Kalman_CVKalmanInitYawParam(Kalman_CVKalmanInitDataTypeDef *cvkf_data, float KF_T, float init_angle_yaw);
void Kalman_CVKalmanInitPitchParam(Kalman_CVKalmanInitDataTypeDef* cvkf_data, float KF_T, float init_angle_pitch, float init_angle_speed);
// void Kalman_CVKalmanInitPitchParam(Kalman_CVKalmanInitDataTypeDef *cvkf_data, float KF_T, float init_angle_pitch);
void Kalman_CVInitSetYaw(Kalman_CVKalmanInitDataTypeDef* cvkf_data, float init_angle_yaw, float init_angle_speed);
void Kalman_CVInitSetPitch(Kalman_CVKalmanInitDataTypeDef* cvkf_data, float init_angle_pitch, float init_angle_speed);
void Kalman_CVKalmanInit(Kalman_CVKalmanTypeDef* cvkf, Kalman_CVKalmanInitDataTypeDef* cvkf_data);
void Kalman_CV_Limit_Speed(Kalman_CVKalmanTypeDef* cvkf);
float Kalman_CV_CalInitSpeed(float delta_err_angle);

void Kalman_TurnOffCVKF(Kalman_CVKalmanTypeDef* cvkf);
void Kalman_TurnOnCVKF(Kalman_CVKalmanTypeDef* cvkf);
void Kalman_TurnOnMeasureUpdate(Kalman_CVKalmanTypeDef* cvkf);
void Kalman_CalcPredict(Kalman_CVKalmanTypeDef* cvkf);
void Kalman_CalcKFGain(Kalman_CVKalmanTypeDef* cvkf);
void Kalman_CalcCorrect(Kalman_CVKalmanTypeDef* cvkf, float angle);
void Kalman_Update(Kalman_CVKalmanTypeDef* cvkf);
void Kalman_MeasurementCalc(Kalman_CVKalmanTypeDef* cvkf, float angle);
void Kalman_NonMeasurementCalc(Kalman_CVKalmanTypeDef* cvkf);
float Kalman_Predict_nT(Kalman_CVKalmanTypeDef* cvkf, int nT);

float Kalman_JudgeChange(Kalman_CVKalmanTypeDef* cvkf, float m_angle);

#ifdef __cplusplus
}
#endif

#endif
