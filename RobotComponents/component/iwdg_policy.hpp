/**
 *******************************************************************************
 * @file      :iwdg_policy.hpp
 * @brief     :看门狗喂狗开关（考核说明 4.10 的验收形态）
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

/* 1 = 正常：CommTask 每 1 ms 刷新一次（考核说明 4.10 要求的位置）
 * 0 = 完全不喂（**仅供验收演示**：烧进去后板子应在约 200 ms 内复位）
 *
 * 注意 0 不是「按数据源喂狗」，新文档的验收方式是「临时屏蔽喂狗调用」，
 * 所以这里必须是真的完全不刷新，否则演示不出复位。
 *
 * 骨架文档 2.2 的提醒：IWDG 已初始化，不喂狗 200 ms 就会复位并无限循环——
 * 这是预期行为，所以平时调试请保持 1。
 */
#define IWDG_FEED_IN_COMM_TASK 1

#endif /* ROBOT_COMPONENTS_IWDG_POLICY_HPP_ */
