/*
 * @Project      : RM_Infantry_Neptune_frame
 * @FilePath     : \infantry_-neptune\Core\Inc\Common_Contrrol\watchdog_ctrl.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:50:05
 */

#ifndef WATCHDOG_CTRL_H
#define WATCHDOG_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

void WatchDog_Task(void const* argument);
void WatchDog_FeedDog(void);

#ifdef __cplusplus
}
#endif

#endif
