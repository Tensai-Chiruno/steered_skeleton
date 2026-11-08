/**
 *******************************************************************************
 * @file      :main_task.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_TASK_MAIN_TASK_HPP_
#define CHASSIS_TASK_MAIN_TASK_HPP_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>  /* 本头文件被 main.c 以 C 方式包含，不能用 <cstdint> */
#ifdef __cplusplus
extern "C" {
#endif
/* Exported function prototypes ----------------------------------------------*/
void MainTaskInit(void);
void MainTask(void);

/** @brief 主循环每转一圈调一次，用于「主上下文活性」检查（P1-8）。 */
void MainLoopHeartbeat(void);

/** @brief 距离上次主循环心跳的毫秒数；从未心跳过时返回 UINT32_MAX。 */
uint32_t MainLoopIdleMs(void);
#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_TASK_MAIN_TASK_HPP_ */
