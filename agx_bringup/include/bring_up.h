//-----------------------------------------------------------------------------
// Copyright(c) 2016-2025 松灵机器人
// All rights reserved.
//
// File: bring_up.h
// Version: 1.0.00
//
// Create:  Feb.20 2025
// Author:  youguo.chu@agilex.ai
// Summary: 底盘通信包入口
//-----------------------------------------------------------------------------

#ifndef BRING_UP_H_
#define BRING_UP_H_

#include <ros/ros.h>

#include "can/socket_can.h"
#include "can/chassis_service.h"

using json = nlohmann::json;

namespace agx::nav {

/**
 * 底盘通信包入口
 */
class BringUp
{
public:
  BringUp();
  ~BringUp();

private:
  std::unique_ptr<ChassisService> m_chassis_service;

};

}

#endif
