/**
 *******************************************************************************
 * @file      :chassis_ik_solver.cpp
 * @brief     :舵向轮底盘逆运动学实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 * @attention :
 *  【推导（本文件的核心，三行）】
 *    每个轮心速度 = 底盘平动 + 绕 z 转动带来的牵连速度：
 *        v_ix = vx - w * r_iy
 *        v_iy = vy + w * r_ix
 *    舵角就是该速度矢量的方向：
 *        theta_i = atan2(v_iy, v_ix)
 *    轮速是它的大小除以轮半径：
 *        omega_i = |v_i| / R
 *
 *    对四轮独立转向底盘，这是全部公式。真正难的是下面两个边界。
 *
 *  【边界一：180° 二义性（最容易把车开翻的一条）】
 *    theta 与 theta+π 产生同一个运动（轮子反向转即可）。
 *    如果每次都用 atan2 的原始结果，当期望方向绕过 ±π 时会算出一个与上次
 *    相差接近 180° 的目标角，舵电机就会带着轮子原地甩半圈——实车上表现为
 *    「车突然抽一下」，而且此时四个轮子的朝向可能互相打架，把车顶起来。
 *    正确做法：若新角与上次相差超过 90°，就取「反向角 + 轮速取反」这个
 *    等价解，保证每次舵角只需要转不超过 90°。
 *
 *  【边界二：atan2(0, 0)】
 *    底盘速度为 0 时四个轮心的速度矢量都是零，atan2 的返回值未定义（实现上传
 *    0）。若不加判断，静止悬停时舵角可能随机跳。做法是低于阈值时保持上次舵角、
 *    轮速给 0。
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "chassis_ik_solver.hpp"

namespace hello_world {
namespace chassis_ik_solver {

namespace {

/** 轮位符号（x 向前、y 向左）：左前 +,+ ／ 左后 -,+ ／ 右后 -,- ／ 右前 +,- */
constexpr float kSignX[ChassisIkSolver::kWheelNum] = {+1.0f, -1.0f, -1.0f,
                                                      +1.0f};
constexpr float kSignY[ChassisIkSolver::kWheelNum] = {+1.0f, +1.0f, -1.0f,
                                                      -1.0f};

}  // namespace

float ChassisIkSolver::wrapPi(float angle) {
  float a = std::fmod(angle + kPi, 2.0f * kPi);
  if (a < 0.0f) {
    a += 2.0f * kPi;
  }
  return a - kPi;
}

void ChassisIkSolver::init(const Params &params) {
  params_ = params;
  reset();
}

void ChassisIkSolver::reset(void) {
  for (int i = 0; i < kWheelNum; ++i) {
    wheel_[i].angle = 0.0f;
    wheel_[i].speed = 0.0f;
  }
}

void ChassisIkSolver::solve(float vx, float vy, float w,
                            WheelState out[kWheelNum]) {
  float raw_speed[kWheelNum];
  float abs_max = 0.0f;

  for (int i = 0; i < kWheelNum; ++i) {
    const float rx = kSignX[i] * params_.half_wheelbase;
    const float ry = kSignY[i] * params_.half_track;

    /* 轮心速度：底盘平动 + ω × r（2D 叉乘） */
    const float vxi = vx - w * ry;
    const float vyi = vy + w * rx;
    const float mag = std::sqrt(vxi * vxi + vyi * vyi);

    if (mag < params_.min_linear_speed) {
      /* 边缘二：速度为零，保持上次舵角，不做 atan2 */
      wheel_[i].speed = 0.0f;
      raw_speed[i] = 0.0f;
      continue;
    }

    float angle = std::atan2(vyi, vxi);
    float speed = mag / params_.wheel_radius;

    /* 边缘一：与上次相比超过 90° 就取反向等价解 */
    const float diff = wrapPi(angle - wheel_[i].angle);
    if (diff > kHalfPi) {
      angle = wrapPi(angle - kPi);
      speed = -speed;
    } else if (diff < -kHalfPi) {
      angle = wrapPi(angle + kPi);
      speed = -speed;
    }

    wheel_[i].angle = wrapPi(angle);
    wheel_[i].speed = speed;
    raw_speed[i] = speed;

    const float a = std::fabs(speed);
    if (a > abs_max) {
      abs_max = a;
    }
  }

  /* 超限时按同一比例缩放全部轮速，保持合成方向不变。
   * 逐轮各自削顶会改变合成速度方向，车会走斜线而不是走直线。 */
  float scale = 1.0f;
  if (params_.max_wheel_speed > 0.0f && abs_max > params_.max_wheel_speed) {
    scale = params_.max_wheel_speed / abs_max;
  }

  for (int i = 0; i < kWheelNum; ++i) {
    wheel_[i].speed = raw_speed[i] * scale;
    out[i] = wheel_[i];
  }
}

}  // namespace chassis_ik_solver
}  // namespace hello_world
