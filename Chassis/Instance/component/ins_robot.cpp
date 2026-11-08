/**
 *******************************************************************************
 * @file      :ins_robot.cpp
 * @brief     :底盘板 Robot / Chassis 单例
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "ins_robot.hpp"

#include "chassis.hpp"
#include "ins_chassis_iksolver.hpp"
#include "ins_comm.hpp"
#include "robot.hpp"

/* Private variables ---------------------------------------------------------*/
namespace {
robot::Chassis g_chassis;
robot::Robot g_robot;
}  // namespace

/* Exported function definitions ---------------------------------------------*/
robot::Robot *GetRobot() { return &g_robot; }

robot::Chassis *GetChassis() { return &g_chassis; }
