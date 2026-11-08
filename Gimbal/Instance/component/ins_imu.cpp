/**
 *******************************************************************************
 * @file      :ins_imu.cpp
 * @brief     :IMU 单例（Gimbal）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-08      Tensai-Chiruno  1. 首版
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "ins_imu.hpp"

#include "imu.hpp"
#include "spi.h"
#include "stm32_hal.hpp"

/* Private constants ---------------------------------------------------------*/
namespace {
/* BMI088 两片 CS 的引脚归属按 CubeMX 里 spi.c 的引脚分布推定：
 *   实测 spi.c 的 SPI1 是 PB3(SCK)/PB4(MISO)/PA7(MOSI)（不是 PA5/PA6/PA7）；
 *   PA4 给加速度计 CS、PB0 给陀螺仪 CS，与官方 C 型板参考实现一致。
 *   仍建议到基地用万用表通断档确认这两条线，
 *   若反了只交换下面两组参数，不要动 imu.cpp 的逻辑。 */
GPIO_TypeDef *const kAccCsPort = GPIOA;
const uint16_t kAccCsPin = GPIO_PIN_4;
GPIO_TypeDef *const kGyroCsPort = GPIOB;
const uint16_t kGyroCsPin = GPIO_PIN_0;
}  // namespace

/* Private variables ---------------------------------------------------------*/
namespace {
hello_world::imu::Imu g_imu;
bool g_inited = false;
}  // namespace

/* Exported function definitions ---------------------------------------------*/
hello_world::imu::Imu *GetImu() {
  if (!g_inited) {
    hello_world::imu::Imu::Params p;
    g_imu.init(&hspi1, kAccCsPort, kAccCsPin, kGyroCsPort, kGyroCsPin, p);
    g_inited = true;
  }
  return &g_imu;
}
