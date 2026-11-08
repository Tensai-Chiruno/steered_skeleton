/**
 *******************************************************************************
 * @file      :ins_chassis_iksolver.cpp
 * @brief     :底盘逆解器单例
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "ins_chassis_iksolver.hpp"

#include "chassis_ik_solver.hpp"

/* Private constants ---------------------------------------------------------*/
namespace {

/* 几何参数来自考核说明 6.2（已按实机测量补齐） */
constexpr float kWheelRadius = 0.066f;
constexpr float kHalfWheelbase = 0.167f;
constexpr float kHalfTrack = 0.167f;

/* 舵角变化率上限：先给 8 rad/s（约 1ms 单拍 0.008 rad ≈ 0.46°）。
 * 台架上看舵机是否还猛打再调。 */
constexpr float kMaxSteerRate = 8.0f;

}  // namespace

/* Private variables ---------------------------------------------------------*/
namespace {
hello_world::chassis_ik_solver::ChassisIkSolver g_ik;
bool g_inited = false;
}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::chassis_ik_solver::ChassisIkSolver *GetChassisIkSolver() {
  if (!g_inited) {
    hello_world::chassis_ik_solver::ChassisIkSolver::Params p;
    p.wheel_radius = kWheelRadius;
    p.half_wheelbase = kHalfWheelbase;
    p.half_track = kHalfTrack;
    p.max_steer_rate = kMaxSteerRate;
    g_ik.init(p);
    g_inited = true;
  }
  return &g_ik;
}
