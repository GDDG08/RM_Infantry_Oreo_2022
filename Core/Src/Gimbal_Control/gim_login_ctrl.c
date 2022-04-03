/*
 * @Project      : RM_Infantry_Neptune
 * @FilePath     : \infantry_-neptune\Core\Src\Gimbal_Control\gim_login_ctrl.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 20:00:28
 */

#include "gim_login_ctrl.h"
#include "gim_remote_ctrl.h"

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)

uint8_t LOGIN_ON_FLAG = LOGIN_OFF;
uint32_t LOGIN_CODE = 0;
uint16_t LOGIN_PASSWORD = 0;

#define LOGIN_ERROR_NUMBER 10
#define LOGIN_STATIC_KEY_HEAD 0x6c
#define LOGIN_STATIC_KEY_END 0xc6

/**
 * @brief      Login initialization
 * @param      NULL
 * @retval     NULL
 */
void Login_Init() {
    LOGIN_ON_FLAG = 0;
    LOGIN_CODE = Login_CreateCode();
    LOGIN_PASSWORD = Login_GetPassword(LOGIN_CODE);
}

/**
 * @brief      Login code generation
 * @param      NULL
 * @retval     Login code
 */
uint32_t Login_CreateCode() {
    uint16_t rand_code = rand() & 0xffff;
    uint32_t code = LOGIN_STATIC_KEY_END | (rand_code << 16);
    code |= (((uint32_t)LOGIN_STATIC_KEY_END) << 24);
    return code;
}

/**
 * @brief      Login password generation
 * @param      code :Login code
 * @retval     Login password
 */
uint32_t Login_GetCode() {
    return LOGIN_CODE;
}

/**
 * @brief      Login password generation
 * @param      code :Login code
 * @retval     Login password
 */
uint16_t Login_GetPassword(uint32_t code) {
    uint8_t buff[4];
    uint16_t decode;

    buff[0] = code & 0xff;
    buff[1] = (code >> 8) & 0xff;
    buff[2] = (code >> 16) & 0xff;
    buff[3] = (code >> 24) & 0xff;

    decode = CRC_GetCRC16CheckSum(buff, 4, CRC16_INIT);

    // Because the key position is not enough, the hexadecimal containing E and F is not generated
    uint32_t err_e = 0x0e;
    uint32_t err_f = 0x0f;
    for (int i = 0; i < 4; i++) {
        if ((((decode & (err_e << (i * 4))) == err_e) || ((decode & (err_f << (i * 4))) == err_f)))
            decode &= (0x0c << (i * 4));
    }
    return decode;
}

/**
 * @brief      Login password verification
 * @param      password :Verified password
 * @retval      1 : code match 2 : not match
 */
uint32_t Login_CheckCode(uint16_t password) {
    if (password == Login_GetPassword(LOGIN_CODE)) {
        return CODE_MATCH;
    } else
        return CODE_NO_MATCH;
}

/**
 * @brief      Login On
 * @param      NULL
 * @retval     NULL
 */
void Login_LoginOn() {
    LOGIN_ON_FLAG = LOGIN_ON;
}

/**
 * @brief      Login control command
 * @param      NULL
 * @retval     NULL
 */
void Login_LoginCmd() {
    if (LOGIN_ON_FLAG != 0)
        return;

    Remote_RemoteDataTypeDef* key = Remote_GetRemoteDataPtr();
    static uint8_t type_num = 0;
    static uint8_t wrong_num = 0;
    static uint16_t type_code = 0;

    // Input error exceeds a certain number to reset the password
    if (wrong_num >= LOGIN_ERROR_NUMBER) {
        LOGIN_CODE = Login_CreateCode();
        LOGIN_PASSWORD = Login_GetPassword(LOGIN_CODE);
        wrong_num = 0;
    }

    //  Confirm password
    static int flag_code_shift = 0;
    if (key->key.shift == 1) {
        if (flag_code_shift == 1) {
            type_num = 0;
            if (CODE_MATCH == Login_CheckCode(type_code)) {
                Login_LoginOn();
            } else {
                type_code = 0;
                wrong_num++;
            }
            flag_code_shift = 0;
        }
    } else
        flag_code_shift = 1;

    // Input password
    static int flag_code_w = 0;
    if (key->key.w == 1) {
        if (flag_code_w == 1) {
            type_num++;
            if (type_num <= 8)
                type_code |= (CODE_KEY_W << (type_num * 4));
            flag_code_w = 0;
        }
    } else
        flag_code_w = 1;

    static int flag_code_a = 0;
    if (key->key.a == 1) {
        if (flag_code_a == 1) {
            type_num++;
            if (type_num <= 8)
                type_code |= (CODE_KEY_A << (type_num * 4));
            flag_code_a = 0;
        }
    } else
        flag_code_a = 1;

    static int flag_code_s = 0;
    if (key->key.s == 1) {
        if (flag_code_s == 1) {
            type_num++;
            if (type_num <= 8)
                type_code |= (CODE_KEY_S << (type_num * 4));
            flag_code_s = 0;
        }
    } else
        flag_code_s = 1;

    static int flag_code_d = 0;
    if (key->key.d == 1) {
        if (flag_code_d == 1) {
            type_num++;
            if (type_num <= 8)
                type_code |= (CODE_KEY_D << (type_num * 4));
            flag_code_d = 0;
        }
    } else
        flag_code_d = 1;

    static int flag_code_q = 0;
    if (key->key.q == 1) {
        if (flag_code_q == 1) {
            type_num++;
            if (type_num <= 8)
                type_code |= (CODE_KEY_Q << (type_num * 4));
            flag_code_q = 0;
        }
    } else
        flag_code_q = 1;

    static int flag_code_e = 0;
    if (key->key.e == 1) {
        if (flag_code_e == 1) {
            type_num++;
            if (type_num <= 8)
                type_code |= (CODE_KEY_E << (type_num * 4));
            flag_code_e = 0;
        }
    } else
        flag_code_e = 1;

    static int flag_code_r = 0;
    if (key->key.r == 1) {
        if (flag_code_r == 1) {
            type_num++;
            if (type_num <= 8)
                type_code |= (CODE_KEY_R << (type_num * 4));
            flag_code_r = 0;
        }
    } else
        flag_code_r = 1;

    static int flag_code_f = 0;
    if (key->key.f == 1) {
        if (flag_code_f == 1) {
            type_num++;
            if (type_num <= 8)
                type_code |= (CODE_KEY_F << (type_num * 4));
            flag_code_f = 0;
        }
    } else
        flag_code_f = 1;

    static int flag_code_g = 0;
    if (key->key.g == 1) {
        if (flag_code_g == 1) {
            type_num++;
            if (type_num <= 8)
                type_code |= (CODE_KEY_G << (type_num * 4));
            flag_code_g = 0;
        }
    } else
        flag_code_g = 1;

    static int flag_code_z = 0;
    if (key->key.z == 1) {
        if (flag_code_z == 1) {
            type_num++;
            if (type_num <= 8)
                type_code |= (CODE_KEY_Z << (type_num * 4));
            flag_code_z = 0;
        }
    } else
        flag_code_z = 1;

    static int flag_code_x = 0;
    if (key->key.x == 1) {
        if (flag_code_x == 1) {
            type_num++;
            if (type_num <= 8)
                type_code |= (CODE_KEY_Z << (type_num * 4));
            flag_code_x = 0;
        }
    } else
        flag_code_x = 1;

    static int flag_code_c = 0;
    if (key->key.c == 1) {
        if (flag_code_c == 1) {
            type_num++;
            if (type_num <= 8)
                type_code |= (CODE_KEY_C << (type_num * 4));
            flag_code_c = 0;
        }
    } else
        flag_code_c = 1;

    static int flag_code_v = 0;
    if (key->key.v == 1) {
        if (flag_code_v == 1) {
            type_num++;
            if (type_num <= 8)
                type_code |= (CODE_KEY_V << (type_num * 4));
            flag_code_v = 0;
        }
    } else
        flag_code_v = 1;

    static int flag_code_b = 0;
    if (key->key.b == 1) {
        if (flag_code_b == 1) {
            type_num++;
            if (type_num <= 8)
                type_code |= (CODE_KEY_B << (type_num * 4));
            flag_code_b = 0;
        }
    } else
        flag_code_b = 1;
}

#endif
