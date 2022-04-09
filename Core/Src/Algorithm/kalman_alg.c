/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Algorithm\kalman_alg.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:59:30
 */

#include "kalman_alg.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

uint16_t sizeof_float, sizeof_double;

/**
 * @brief      Initialization of Kalman filter
 * @param      kf :Structure pointer of Kalman filter
 * @param      xhatSize :State variable matrix size
 * @param      uSize :Control matrix size
 * @param      zSize :Observation matrix size
 * @retval     NULL
 */
void Kalman_FilterInit(Kalman_KalmanTypeDef* kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize) {
    sizeof_float = sizeof(float);
    sizeof_double = sizeof(double);

    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;

    kf->MeasurementValidNum = 0;
    kf->NonMeasurement = 0;

    // measurement flags
    kf->MeasurementMap = (uint8_t*)malloc(sizeof(uint8_t) * zSize);
    memset(kf->MeasurementMap, 0, sizeof(uint8_t) * zSize);
    kf->MeasurementDegree = (float*)malloc(sizeof_float * zSize);
    memset(kf->MeasurementDegree, 0, sizeof_float * zSize);
    kf->MatR_DiagonalElements = (float*)malloc(sizeof_float * zSize);
    memset(kf->MatR_DiagonalElements, 0, sizeof_float * zSize);
    kf->StateMinVariance = (float*)malloc(sizeof_float * xhatSize);
    memset(kf->StateMinVariance, 0, sizeof_float * xhatSize);
    kf->temp = (uint8_t*)malloc(sizeof(uint8_t) * zSize);
    memset(kf->temp, 0, sizeof(uint8_t) * zSize);

    // filter data
    kf->FilteredValue = (float*)malloc(sizeof_float * xhatSize);
    memset(kf->FilteredValue, 0, sizeof_float * xhatSize);
    kf->MeasuredVector = (float*)malloc(sizeof_float * zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * zSize);
    kf->ControlVector = (float*)malloc(sizeof_float * uSize);
    memset(kf->ControlVector, 0, sizeof_float * uSize);

    // xhat x(k|k)
    kf->xhat_data = (float*)malloc(sizeof_float * xhatSize);
    memset(kf->xhat_data, 0, sizeof_float * xhatSize);
    mat_init(&kf->xhat, kf->xhatSize, 1, (float*)kf->xhat_data);

    // xhatminus x(k|k-1)
    kf->xhatminus_data = (float*)malloc(sizeof_float * xhatSize);
    memset(kf->xhatminus_data, 0, sizeof_float * xhatSize);
    mat_init(&kf->xhatminus, kf->xhatSize, 1, (float*)kf->xhatminus_data);

    if (uSize != 0) {
        // control vector u
        kf->u_data = (float*)malloc(sizeof_float * uSize);
        memset(kf->u_data, 0, sizeof_float * uSize);
        mat_init(&kf->u, kf->uSize, 1, (float*)kf->u_data);
    }

    // measurement vector z
    kf->z_data = (float*)malloc(sizeof_float * zSize);
    memset(kf->z_data, 0, sizeof_float * zSize);
    mat_init(&kf->z, kf->zSize, 1, (float*)kf->z_data);

    // covariance matrix P(k|k)
    kf->P_data = (float*)malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->P_data, 0, sizeof_float * xhatSize * xhatSize);
    mat_init(&kf->P, kf->xhatSize, kf->xhatSize, (float*)kf->P_data);

    // create covariance matrix P(k|k-1)
    kf->Pminus_data = (float*)malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Pminus_data, 0, sizeof_float * xhatSize * xhatSize);
    mat_init(&kf->Pminus, kf->xhatSize, kf->xhatSize, (float*)kf->Pminus_data);

    // state transition matrix F FT
    kf->F_data = (float*)malloc(sizeof_float * xhatSize * xhatSize);
    kf->FT_data = (float*)malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->F_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->FT_data, 0, sizeof_float * xhatSize * xhatSize);
    mat_init(&kf->F, kf->xhatSize, kf->xhatSize, (float*)kf->F_data);
    mat_init(&kf->FT, kf->xhatSize, kf->xhatSize, (float*)kf->FT_data);

    if (uSize != 0) {
        // control matrix B
        kf->B_data = (float*)malloc(sizeof_float * xhatSize * uSize);
        memset(kf->B_data, 0, sizeof_float * xhatSize * uSize);
        mat_init(&kf->B, kf->xhatSize, kf->uSize, (float*)kf->B_data);
    }

    // measurement matrix H
    kf->H_data = (float*)malloc(sizeof_float * zSize * xhatSize);
    kf->HT_data = (float*)malloc(sizeof_float * xhatSize * zSize);
    memset(kf->H_data, 0, sizeof_float * zSize * xhatSize);
    memset(kf->HT_data, 0, sizeof_float * xhatSize * zSize);
    mat_init(&kf->H, kf->zSize, kf->xhatSize, (float*)kf->H_data);
    mat_init(&kf->HT, kf->xhatSize, kf->zSize, (float*)kf->HT_data);

    // process noise covariance matrix Q
    kf->Q_data = (float*)malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Q_data, 0, sizeof_float * xhatSize * xhatSize);
    mat_init(&kf->Q, kf->xhatSize, kf->xhatSize, (float*)kf->Q_data);

    // measurement noise covariance matrix R
    kf->R_data = (float*)malloc(sizeof_float * zSize * zSize);
    memset(kf->R_data, 0, sizeof_float * zSize * zSize);
    mat_init(&kf->R, kf->zSize, kf->zSize, (float*)kf->R_data);

    // kalman gain K
    kf->K_data = (float*)malloc(sizeof_float * xhatSize * zSize);
    memset(kf->K_data, 0, sizeof_float * xhatSize * zSize);
    mat_init(&kf->K, kf->xhatSize, kf->zSize, (float*)kf->K_data);

    kf->S_data = (float*)malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data = (float*)malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data1 = (float*)malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_vector_data = (float*)malloc(sizeof_float * kf->xhatSize);
    kf->temp_vector_data1 = (float*)malloc(sizeof_float * kf->xhatSize);
    mat_init(&kf->S, kf->xhatSize, kf->xhatSize, (float*)kf->S_data);
    mat_init(&kf->temp_matrix, kf->xhatSize, kf->xhatSize, (float*)kf->temp_matrix_data);
    mat_init(&kf->temp_matrix1, kf->xhatSize, kf->xhatSize, (float*)kf->temp_matrix_data1);
    mat_init(&kf->temp_vector, kf->xhatSize, 1, (float*)kf->temp_vector_data);
    mat_init(&kf->temp_vector1, kf->xhatSize, 1, (float*)kf->temp_vector_data1);

    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;
}

/**
 * @brief      Update data with Kalman filter
 * @param      kf :Structure pointer of Kalman filter
 * @retval     Filtered data pointer
 */
float* Kalman_FilterUpdate(Kalman_KalmanTypeDef* kf) {
    // matrix H K R auto adjustment
    if (kf->UseAutoAdjustment != 0)
        Kalman_Adjustment_H_K_R(kf);
    else {
        memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
        memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);
    }

    if (kf->z_data[0] == 0) {
        kf->NonMeasurement = 1;
    }

    memcpy(kf->u_data, kf->ControlVector, sizeof_float * kf->uSize);
    memset(kf->ControlVector, 0, sizeof_float * kf->uSize);

    if (kf->User_Func0_f != NULL)
        kf->User_Func0_f(kf);

    // 1. xhat'(k)= A��xhat(k-1) + B��u
    if (!kf->SkipEq1) {
        if (kf->uSize > 0) {
            kf->MatStatus = mat_mult(&kf->F, &kf->xhat, &kf->temp_vector);
            kf->temp_vector1.numRows = kf->xhatSize;
            kf->temp_vector1.numCols = 1;
            kf->MatStatus = mat_mult(&kf->B, &kf->u, &kf->temp_vector1);
            kf->MatStatus = mat_add(&kf->temp_vector, &kf->temp_vector1, &kf->xhatminus);
        } else {
            kf->MatStatus = mat_mult(&kf->F, &kf->xhat, &kf->xhatminus);
        }
    }

    if (kf->User_Func1_f != NULL)
        kf->User_Func1_f(kf);

    // 2. P'(k) = A��P(k-1)��AT + Q
    if (!kf->SkipEq2) {
        kf->MatStatus = mat_trans(&kf->F, &kf->FT);
        kf->MatStatus = mat_mult(&kf->F, &kf->P, &kf->Pminus);
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->FT.numCols;
        kf->MatStatus = mat_mult(&kf->Pminus, &kf->FT, &kf->temp_matrix);  // temp_matrix = F P(k-1) FT
        kf->MatStatus = mat_add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
    }

    if (kf->User_Func2_f != NULL)
        kf->User_Func2_f(kf);

    if ((kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0) && (kf->NonMeasurement == 0)) {
        // 3. K(k) = P'(k)��HT / (H��P'(k)��HT + R)
        if (!kf->SkipEq3) {
            kf->MatStatus = mat_trans(&kf->H, &kf->HT);  // z|x => x|z
            kf->temp_matrix.numRows = kf->H.numRows;
            kf->temp_matrix.numCols = kf->Pminus.numCols;
            kf->MatStatus = mat_mult(&kf->H, &kf->Pminus, &kf->temp_matrix);  // temp_matrix = H��P'(k)
            kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
            kf->temp_matrix1.numCols = kf->HT.numCols;
            kf->MatStatus = mat_mult(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1);  // temp_matrix1 = H��P'(k)��HT
            kf->S.numRows = kf->R.numRows;
            kf->S.numCols = kf->R.numCols;
            kf->MatStatus = mat_add(&kf->temp_matrix1, &kf->R, &kf->S);  // S = H P'(k) HT + R
            kf->MatStatus = mat_inv(&kf->S, &kf->temp_matrix1);          // temp_matrix1 = inv(H��P'(k)��HT + R)
            kf->temp_matrix.numRows = kf->Pminus.numRows;
            kf->temp_matrix.numCols = kf->HT.numCols;
            kf->MatStatus = mat_mult(&kf->Pminus, &kf->HT, &kf->temp_matrix);  // temp_matrix = P'(k)��HT
            kf->MatStatus = mat_mult(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
        }

        if (kf->User_Func3_f != NULL)
            kf->User_Func3_f(kf);

        // 4. xhat(k) = xhat'(k) + K(k)��(z(k) - H��xhat'(k))
        if (!kf->SkipEq4) {
            kf->temp_vector.numRows = kf->H.numRows;
            kf->temp_vector.numCols = 1;
            kf->MatStatus = mat_mult(&kf->H, &kf->xhatminus, &kf->temp_vector);  // temp_vector = H xhat'(k)
            kf->temp_vector1.numRows = kf->z.numRows;
            kf->temp_vector1.numCols = 1;
            kf->MatStatus = mat_sub(&kf->z, &kf->temp_vector, &kf->temp_vector1);  // temp_vector1 = z(k) - H��xhat'(k)
            kf->temp_vector.numRows = kf->K.numRows;
            kf->temp_vector.numCols = 1;
            kf->MatStatus = mat_mult(&kf->K, &kf->temp_vector1, &kf->temp_vector);  // temp_vector = K(k)��(z(k) - H��xhat'(k))
            kf->MatStatus = mat_add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
        }

        if (kf->User_Func4_f != NULL)
            kf->User_Func4_f(kf);

        // 5. P(k) = (1-K(k)��H)��P'(k) ==> P(k) = P'(k)-K(k)��H��P'(k)
        if (!kf->SkipEq5) {
            kf->temp_matrix.numRows = kf->K.numRows;
            kf->temp_matrix.numCols = kf->H.numCols;
            kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
            kf->temp_matrix1.numCols = kf->Pminus.numCols;
            kf->MatStatus = mat_mult(&kf->K, &kf->H, &kf->temp_matrix);                  // temp_matrix = K(k)��H
            kf->MatStatus = mat_mult(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1);  // temp_matrix1 = K(k)��H��P'(k)
            kf->MatStatus = mat_sub(&kf->Pminus, &kf->temp_matrix1, &kf->P);
        }
    } else {
        // xhat(k) = xhat'(k)
        // P(k) = P'(k)
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
        kf->NonMeasurement = 0;
    }

    if (kf->User_Func5_f != NULL)
        kf->User_Func5_f(kf);

    // suppress filter excessive convergence
    for (uint8_t i = 0; i < kf->xhatSize; i++) {
        if (kf->P_data[i * kf->xhatSize + i] < kf->StateMinVariance[i])
            kf->P_data[i * kf->xhatSize + i] = kf->StateMinVariance[i];
    }

    if (kf->UseAutoAdjustment != 0) {
        memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);
        memset(kf->H_data, 0, sizeof_float * kf->xhatSize * kf->zSize);
    }

    memcpy(kf->FilteredValue, kf->xhat_data, sizeof_float * kf->xhatSize);

    if (kf->User_Func6_f != NULL)
        kf->User_Func6_f(kf);

    return kf->FilteredValue;
}

/**
 * @brief      Auto adjust H K R matrix
 * @param      kf :Structure pointer of Kalman filter
 * @retval     NULL
 */
static void Kalman_Adjustment_H_K_R(Kalman_KalmanTypeDef* kf) {
    kf->MeasurementValidNum = 0;

    memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);

    // recognize measurement validity and adjust matrices H R K
    for (uint8_t i = 0; i < kf->zSize; i++) {
        if (kf->z_data[i] != 0) {
            // rebuild vector z
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            kf->temp[kf->MeasurementValidNum] = i;
            // rebuild matrix H
            kf->H_data[kf->xhatSize * kf->MeasurementValidNum + kf->MeasurementMap[i] - 1] = kf->MeasurementDegree[i];
            kf->MeasurementValidNum++;
        }
    }
    for (uint8_t i = 0; i < kf->MeasurementValidNum; i++) {
        // rebuild matrix R
        kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
    }

    // adjust the dimensions of system matrices
    kf->H.numRows = kf->MeasurementValidNum;
    kf->H.numCols = kf->xhatSize;
    kf->HT.numRows = kf->xhatSize;
    kf->HT.numCols = kf->MeasurementValidNum;
    kf->R.numRows = kf->MeasurementValidNum;
    kf->R.numCols = kf->MeasurementValidNum;
    kf->K.numRows = kf->xhatSize;
    kf->K.numCols = kf->MeasurementValidNum;
    kf->z.numRows = kf->MeasurementValidNum;
}

float Pinit_0 = 8.0f;
float Pinit_1 = 3000.0f;
float MAX_SPEED_KF = 60.0f;

/**
 * @brief      Initialization of CV Kalman filter yaw parameters
 * @param      cvkf_data: Initialized Kalman filter structure
 * @param      KF_T: Kalman
 * @param      init_angle_yaw: Initialized yaw axis angle
 * @retval     NULL
 */
void Kalman_CVKalmanInitYawParam(Kalman_CVKalmanInitDataTypeDef* cvkf_data, float KF_T, float init_angle_yaw, float init_angle_speed) {
    float KF_A[4] = {1.0f, 0.0f,
                     0.0f, 1.0f};
    KF_A[1] = KF_T;
    float KF_C[2] = {1.0f, 0.0f};
    float XLast[2] = {0.0f};
    XLast[0] = init_angle_yaw;
    XLast[1] = init_angle_speed;
    float PLast[4] = {1.0f, 0.0f,
                      0.0f, 1.0f};
    float Ppre[4] = {0.0f};
    float Popt[4] = {0.0f};
    float Q[4] = {0.02f, 0.0f,
                  0.0f, 40.0f};
    float R[1] = {0.5f};
    float Kf[2] = {0.0f};

    for (int i = 0; i < 4; i++) {
        cvkf_data->KF_A[i] = KF_A[i];
        if (i < 2) {
            cvkf_data->KF_C[i] = KF_C[i];
            cvkf_data->XLast[i] = XLast[i];
            cvkf_data->Xpre[i] = XLast[i];
            cvkf_data->Xopt[i] = XLast[i];

            cvkf_data->Kf[i] = Kf[i];
        }
        cvkf_data->PLast[i] = PLast[i];
        cvkf_data->Ppre[i] = Ppre[i];
        cvkf_data->Popt[i] = Popt[i];
        cvkf_data->Q[i] = Q[i];
    }
    cvkf_data->R[0] = R[0];
}

/**
 * @brief      Initialization of CV Kalman filter pitch parameters
 * @param      cvkf_data: Initialized Kalman filter structure
 * @param      KF_T: Kalman
 * @param      init_angle_pitch: Initialized pitch axis angle
 * @retval     NULL
 */
void Kalman_CVKalmanInitPitchParam(Kalman_CVKalmanInitDataTypeDef* cvkf_data, float KF_T, float init_angle_pitch, float init_angle_speed) {
    float KF_A[4] = {1.0f, 0.0f,
                     0.0f, 1.0f};
    KF_A[1] = KF_T;
    float KF_C[2] = {1.0f, 0.0f};
    float XLast[2] = {0.0f};
    XLast[0] = init_angle_pitch;
    XLast[1] = init_angle_speed;
    float PLast[4] = {1.0f, 0.0f,
                      0.0f, 1.0f};
    float Ppre[4] = {0.0f};
    float Popt[4] = {0.0f};
    float Q[4] = {0.01f, 0.0f,
                  0.0f, 40.0f};
    float R[1] = {0.5f};
    float Kf[2] = {0.0f};

    for (int i = 0; i < 4; i++) {
        cvkf_data->KF_A[i] = KF_A[i];
        if (i < 2) {
            cvkf_data->KF_C[i] = KF_C[i];
            cvkf_data->XLast[i] = XLast[i];
            cvkf_data->Xpre[i] = XLast[i];
            cvkf_data->Xopt[i] = XLast[i];

            cvkf_data->Kf[i] = Kf[i];
        }
        cvkf_data->PLast[i] = PLast[i];
        cvkf_data->Ppre[i] = Ppre[i];
        cvkf_data->Popt[i] = Popt[i];
        cvkf_data->Q[i] = Q[i];
    }
    cvkf_data->R[0] = R[0];
}

/**
 * @brief      Initialization of CV Kalman filter yaw
 * @param      cvkf_data: Initialized Kalman filter structure
 * @param      init_angle_yaw: Initialized yaw angle
 * @retval     NULL
 */
void Kalman_CVInitSetYaw(Kalman_CVKalmanInitDataTypeDef* cvkf_data, float init_angle_yaw, float init_angle_speed) {
    float XLast[2] = {0.0f};
    XLast[0] = init_angle_yaw;
    XLast[1] = init_angle_speed;
    float PLast[4] = {Pinit_0, 0.0f,
                      0.0f, Pinit_1};
    for (int i = 0; i < 4; i++) {
        if (i < 2) {
            cvkf_data->XLast[i] = XLast[i];
            cvkf_data->Xpre[i] = XLast[i];
            cvkf_data->Xopt[i] = XLast[i];
        }
        cvkf_data->PLast[i] = PLast[i];
    }
}

/**
 * @brief      Initialization of CV Kalman filter pitch
 * @param      cvkf_data: Initialized Kalman filter structure
 * @param      init_angle_pitch: Initialized pitch angle
 * @retval     NULL
 */
void Kalman_CVInitSetPitch(Kalman_CVKalmanInitDataTypeDef* cvkf_data, float init_angle_pitch, float init_angle_speed) {
    float XLast[2] = {0.0f};
    XLast[0] = init_angle_pitch;
    XLast[1] = init_angle_speed;
    float PLast[4] = {Pinit_0, 0.0f,
                      0.0f, Pinit_1};
    for (int i = 0; i < 4; i++) {
        if (i < 2) {
            cvkf_data->XLast[i] = XLast[i];
            cvkf_data->Xpre[i] = XLast[i];
            cvkf_data->Xopt[i] = XLast[i];
        }
        cvkf_data->PLast[i] = PLast[i];
    }
}

/**
 * @brief      Initialize cvkf structure and allocate memory space
 * @param      cvkf: Kalman filter structure
 * @param      cvkf_data: Initialized Kalman filter structure
 * @retval     NULL
 */
void Kalman_CVKalmanInit(Kalman_CVKalmanTypeDef* cvkf, Kalman_CVKalmanInitDataTypeDef* cvkf_data) {
    mat_init(&cvkf->KF_A, 2, 2, (float*)cvkf_data->KF_A);
    mat_init(&cvkf->KF_C, 1, 2, (float*)cvkf_data->KF_C);
    mat_init(&cvkf->XLast, 2, 1, (float*)cvkf_data->XLast);
    mat_init(&cvkf->Xpre, 2, 1, (float*)cvkf_data->Xpre);
    mat_init(&cvkf->Xopt, 2, 1, (float*)cvkf_data->Xopt);
    mat_init(&cvkf->PLast, 2, 2, (float*)cvkf_data->PLast);
    mat_init(&cvkf->Ppre, 2, 2, (float*)cvkf_data->Ppre);
    mat_init(&cvkf->Popt, 2, 2, (float*)cvkf_data->Popt);
    mat_init(&cvkf->Q, 2, 2, (float*)cvkf_data->Q);
    mat_init(&cvkf->R, 1, 1, (float*)cvkf_data->R);
    mat_init(&cvkf->Kf, 2, 1, (float*)cvkf_data->Kf);
    // cvkf->cvkf_t = cvkf_data->cvkf_t;
    cvkf->angle = cvkf_data->Xopt[0];
    cvkf->angle_p_err = 0.0f;
    cvkf->switch_mode = 1;    // Default: OPEN
    cvkf->measure_mode = 0;   // Default: None Update Measurement
    cvkf->targer_change = 0;  // Default: Target Follow no change
    cvkf->max_speed = MAX_SPEED_KF;
    cvkf->min_speed = 3.5f;  // For Prediction AngleSpeed Dead Region
}

/**
 * @brief      Turn off Kalman filter
 * @param      cvkf: Kalman filter structure
 * @retval     NULL
 */
void Kalman_TurnOffCVKF(Kalman_CVKalmanTypeDef* cvkf) {
    cvkf->switch_mode = 0;
}

/**
 * @brief      Turn on Kalman filter
 * @param      cvkf: Kalman filter structure
 * @retval     NULL
 */
void Kalman_TurnOnCVKF(Kalman_CVKalmanTypeDef* cvkf) {
    cvkf->switch_mode = 1;
}

/**
 * @brief      Turn on Kalman filter update
 * @param      cvkf: Kalman filter structure
 * @retval     NULL
 */
void Kalman_TurnOnMeasureUpdate(Kalman_CVKalmanTypeDef* cvkf) {
    cvkf->measure_mode = 1;
}

/**
 * @brief      The model is used for prediction and variance iteration
 * @param      cvkf: Kalman filter structure
 * @retval     NULL
 */
void Kalman_CalcPredict(Kalman_CVKalmanTypeDef* cvkf) {
    float _temp1[4] = {0.0f};
    float _temp2[4] = {0.0f};
    mat _t1, _t2;
    mat_init(&_t1, 2, 2, (float*)_temp1);
    mat_init(&_t2, 2, 2, (float*)_temp2);

    // X_pre = A*X_last
    mat_mult(&cvkf->KF_A, &cvkf->XLast, &cvkf->Xpre);

    // PPre = A*PLast*A'+ Q;
    mat_mult(&cvkf->KF_A, &cvkf->PLast, &cvkf->Ppre);  // Ppre = A*Plast
    mat_trans(&cvkf->KF_A, &_t1);                      // _temp = A'
    mat_mult(&cvkf->Ppre, &_t1, &_t2);                 // Ppre = Ppre*_temp
    mat_add(&_t2, &cvkf->Q, &cvkf->Ppre);              // PPre = PPre+ Q

    // Set Best Angle:
    cvkf->angle = cvkf->Xpre.pData[0];
}

/**
 * @brief      Calculation of Kalman filter gain
 * @param      cvkf: Kalman filter structure
 * @retval     NULL
 */
void Kalman_CalcKFGain(Kalman_CVKalmanTypeDef* cvkf) {
    float _temp1[2] = {0.0f};
    float _temp2[2] = {0.0f};
    float _temp3[1] = {0.0f};
    float _temp4[1] = {0.0f};
    float _temp5[2] = {0.0f};
    mat _t1, _t2, _t3, _t4, _t5;
    mat_init(&_t1, 1, 2, (float*)_temp1);
    mat_init(&_t2, 2, 1, (float*)_temp2);
    mat_init(&_t3, 1, 1, (float*)_temp3);
    mat_init(&_t4, 1, 1, (float*)_temp4);
    mat_init(&_t5, 2, 1, (float*)_temp5);

    // Kf = PPre*C'/(C*PPre*C'+ R);
    mat_mult(&cvkf->KF_C, &cvkf->Ppre, &_t1);  // PLast = C*Ppre
    mat_trans(&cvkf->KF_C, &_t2);              // _temp = C'
    mat_mult(&_t1, &_t2, &_t3);                // Kf = C*Ppre*C'
    mat_add(&_t3, &cvkf->R, &_t4);             // Kf = (C*PPre*C'+ R);
    mat_inv(&_t4, &_t3);                       // Kf = inv(Kf)
    mat_mult(&cvkf->Ppre, &_t2, &_t5);         // _t1 = PPre*C'
    mat_mult(&_t5, &_t3, &cvkf->Kf);           // Kf = PPre*C'*inv(C*PPre*C'+ R)
}

/**
 * @brief      Using measurement data to update and predict optimal state estimation
 * @param      cvkf: Kalman filter structure
 * @param      angle: Measured angle
 * @retval     NULL
 */
void Kalman_CalcCorrect(Kalman_CVKalmanTypeDef* cvkf, float angle) {
    // Init Measurement Mat:
    static float _ym[1] = {0.0f};
    _ym[0] = angle;
    static float _temp1[1] = {0.0f};
    static float _temp2[1] = {0.0f};
    static float _temp3[2] = {0.0f};
    mat Ym, _t1, _t2, _t3;
    mat_init(&Ym, 1, 1, (float*)_ym);
    mat_init(&_t1, 1, 1, (float*)_temp1);
    mat_init(&_t2, 1, 1, (float*)_temp2);
    mat_init(&_t3, 2, 1, (float*)_temp3);

    // XOpt = XPre + Kf*(Ym - C*XPre):
    mat_mult(&cvkf->KF_C, &cvkf->Xpre, &_t1);
    mat_sub(&Ym, &_t1, &_t2);
    mat_mult(&cvkf->Kf, &_t2, &_t3);
    mat_add(&cvkf->Xpre, &_t3, &cvkf->Xopt);

    // Init Eye:
    static float _eye[] = {1.0f, 0.0f, 0.0f, 1.0f};
    static float _temp4[4] = {0.0f};
    mat Eye, _t4;
    mat_init(&Eye, 2, 2, (float*)_eye);
    mat_init(&_t4, 2, 2, (float*)_temp4);

    // POpt = (eye(length(XOpt))-Kf*C)*PPre:
    mat_mult(&cvkf->Kf, &cvkf->KF_C, &cvkf->Popt);
    mat_sub(&Eye, &cvkf->Popt, &_t4);
    mat_mult(&_t4, &cvkf->Ppre, &cvkf->Popt);

    // Set Best Angle:
    cvkf->angle = cvkf->Xopt.pData[0];
}

/**
 * @brief      The last state is set to the optimal value to prepare for the next iteration
 * @param      cvkf: Kalman filter structure
 * @retval     NULL
 */
void Kalman_Update(Kalman_CVKalmanTypeDef* cvkf) {
    // Calculate the variance corresponding to the angle state
    cvkf->angle_p_err = fabs(cvkf->PLast.pData[0] - cvkf->Popt.pData[0]);

    int count = cvkf->PLast.numCols * cvkf->PLast.numRows;
    for (int i = 0; i < count; i++) {
        cvkf->PLast.pData[i] = cvkf->Popt.pData[i];
    }
    count = cvkf->XLast.numCols * cvkf->XLast.numRows;
    for (int i = 0; i < count; i++) {
        cvkf->XLast.pData[i] = cvkf->Xopt.pData[i];
    }
}

void Kalman_Update_none_data(Kalman_CVKalmanTypeDef* cvkf) {
    int count = cvkf->PLast.numCols * cvkf->PLast.numRows;
    for (int i = 0; i < count; i++) {
        cvkf->PLast.pData[i] = cvkf->Ppre.pData[i];
    }
    count = cvkf->XLast.numCols * cvkf->XLast.numRows;
    for (int i = 0; i < count; i++) {
        cvkf->XLast.pData[i] = cvkf->Xpre.pData[i];
    }
}

/**
 * @brief      KF filtering using measured data normally
 * @param      cvkf: Kalman filter structure
 * @param      angle: Measured angle
 * @retval     NULL
 */
void Kalman_MeasurementCalc(Kalman_CVKalmanTypeDef* cvkf, float angle) {
    Kalman_CalcPredict(cvkf);
    Kalman_CalcKFGain(cvkf);
    Kalman_CalcCorrect(cvkf, angle);
    Kalman_CV_Limit_Speed(cvkf);
    Kalman_Update(cvkf);
    cvkf->measure_mode = 0;  // After Correction: Measurement used
}

/**
 * @brief      KF filtering without measured data normally
 * @param      cvkf: Kalman filter structure
 * @param      angle: Measured angle
 * @retval     NULL
 */
void Kalman_NonMeasurementCalc(Kalman_CVKalmanTypeDef* cvkf) {
    Kalman_CalcPredict(cvkf);
    Kalman_Update_none_data(cvkf);
}

/**
 * @brief      Forecast N cycles without variance iteration
 * @param      cvkf: Kalman filter structure
 * @param      nT: Forecast period
 * @retval     Mat: [angle_yaw omega_yaw angle_pitch omega_pitch]
 */
float Kalman_Predict_nT(Kalman_CVKalmanTypeDef* cvkf, int nT) {
    //	if (nT == 0)	// None Prediction
    //		return cvkf->angle;
    //	float _temp1[4] = {1.0f,0.0f,0.0f,1.0f};
    //	_temp1[0] = cvkf->KF_A.pData[0];
    //	_temp1[1] = cvkf->KF_A.pData[1]*nT;
    //	_temp1[2] = cvkf->KF_A.pData[2];
    //	_temp1[3] = cvkf->KF_A.pData[3];
    //	float _temp2[2] = {0.0f};
    //	mat _t1, _t2;
    //	mat_init(&_t1, 2, 2, (float *)_temp1);
    //	mat_init(&_t2, 2, 1, (float *)_temp2);
    //
    //	mat_mult(&_t1, &cvkf->XLast, &_t2);
    //	return (float)_t2.pData[0];

    // Prediction Without DSP:
    float pre_time = cvkf->KF_A.pData[1] * nT;
    float pre_angle = cvkf->XLast.pData[0];
    float pre_speed = cvkf->XLast.pData[1];
    // Set Min Piction Speed:
    if (fabs(pre_speed) >= cvkf->min_speed) {
        pre_angle += pre_speed * pre_time;
    }
    return pre_angle;
}

/**
 * @brief      Forecast N cycles without variance iteration
 * @param      cvkf: Kalman filter structure
 * @param      m_angle: Forecast period
 * @retval     last_target: last terget angle
 */
float Kalman_JudgeChange(Kalman_CVKalmanTypeDef* cvkf, float m_angle) {
    int nT = 10;
    static uint32_t target_change_time = 0;
    static float last_target = 0.0f;
    float angle_predict = Kalman_Predict_nT(cvkf, nT);
    // Whether Large Gab Change Appear?
    if (fabs(m_angle - cvkf->angle) >= fabs(angle_predict - cvkf->angle) + 1.5f) {
        uint32_t change_now = HAL_GetTick();
        if ((int)(change_now - target_change_time) >= 500 || (int)(change_now - target_change_time) <= -500) {
            cvkf->targer_change = 1;
            last_target = m_angle;
        }
    } else {
        target_change_time = HAL_GetTick();
        last_target = m_angle;
        cvkf->targer_change = 0;
    }

    return last_target;
}
/*

*/
void Kalman_CV_Limit_Speed(Kalman_CVKalmanTypeDef* cvkf) {
    float max_angle_speed = cvkf->max_speed;
    if (cvkf->Xopt.pData[1] > max_angle_speed) {
        cvkf->Xopt.pData[1] = max_angle_speed;
    } else if (cvkf->Xopt.pData[1] < -max_angle_speed) {
        cvkf->Xopt.pData[1] = -max_angle_speed;
    }
    // Set Dead domain : 5 degree
    //	if(fabs(cvkf->Xopt.pData[1])<= cvkf->min_speed){
    //		cvkf->Xopt.pData[1] = 0;
    //	}
}
/*

*/
float Kalman_CV_CalInitSpeed(float delta_err_angle) {
    static float delta_angle = 0.0f;
    if (fabs(delta_err_angle) <= delta_angle) {
        return 0.0f;
    } else if (delta_err_angle > delta_angle) {
        return 10.0f;
    } else if (delta_err_angle < -delta_angle) {
        return -10.0f;
    }
    return 0.0f;
}
/*

*/
float poly_pitch_offset(float object_angle, int Speed) {
    float offset_angle = object_angle;

    // Dead Domain:
    if (object_angle < -10.0f) {
        offset_angle = 0.0f;
    }
    return object_angle;
}
