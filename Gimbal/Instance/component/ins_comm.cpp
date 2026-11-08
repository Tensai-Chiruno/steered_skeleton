/**
 *******************************************************************************
 * @file      :ins_comm.cpp
 * @brief     :遥控器解码对象单例（Gimbal）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "ins_comm.hpp"

/* Private variables ---------------------------------------------------------*/
namespace {
hello_world::remote_control::DT7 g_remote_control;
}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::remote_control::DT7 *GetRemoteControl() { return &g_remote_control; }

/* Private function definitions ----------------------------------------------*/
