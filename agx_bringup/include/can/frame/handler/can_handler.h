//-----------------------------------------------------------------------------
// Copyright(c) 2016-2025 松灵机器人
// All rights reserved.
//
// File: base_frame.h
// Version: 1.0.00
//
// Create:  Feb.21 2025
// Author:  youguo.chu@agilex.ai
// Summary: can报文操作
//-----------------------------------------------------------------------------

#ifndef CAN_HANDLER_H_
#define CAN_HANDLER_H_

#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <map>
#include <cstring>
#include <unistd.h>
#include <nlohmann/json.hpp>

#include <can/chassis_status.h>
#include <can/frame/handler/base_handler.h>
#include <can/frame/handler/handler_111.h>
#include <can/frame/handler/handler_121.h>
#include <can/frame/handler/handler_141.h>
#include <can/frame/handler/handler_423.h>
#include <can/frame/handler/handler_441.h>
#include <can/frame/handler/handler_461.h>

namespace agx::nav {

/**
 * can指令操作
 */
class CanHandler
{
public:
  static CanHandler &get_instance() {
    static CanHandler instance;
    return instance;
  }
  void handle(uint16_t instruction, const json& data);

private:
  CanHandler();
  ~CanHandler();
  CanHandler(const CanHandler &) = delete;
  CanHandler(const CanHandler &&) = delete;
  CanHandler &operator=(const CanHandler &) = delete;

private:
  std::map<int, std::unique_ptr<BaseHandler>> m_handler_map;

};

}

#endif
