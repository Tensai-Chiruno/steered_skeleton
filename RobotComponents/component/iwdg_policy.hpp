/**
 *******************************************************************************
 * @file      :iwdg_policy.hpp
 * @brief     :看门狗刷新策略（编译期开关）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

#ifndef ROBOT_COMPONENTS_IWDG_POLICY_HPP_
#define ROBOT_COMPONENTS_IWDG_POLICY_HPP_

/* 1 = 无条件保活：CommTask 每 1 ms 刷一次。
 *     台架/调试阶段必须用这个值，否则 IWDG 204 ms 超时会让板子反复复位，
 *     连 Ozone 都连不上。
 *
 * 0 = 按数据源喂狗（【第 10 项验收形态】）：
 *     云台板 → 只在 DBUS 解包成功处刷新
 *     底盘板 → 只在收到云台帧（0x102）处刷新
 *     效果就是考核要求的「关遥控器即杀程序」。
 *
 * 验收前把这里改成 0，重新编译烧写；两种形态都已经写在代码里，
 * 不需要再改逻辑，只翻一个宏。
 */
#define IWDG_UNCONDITIONAL 1

#endif /* ROBOT_COMPONENTS_IWDG_POLICY_HPP_ */
