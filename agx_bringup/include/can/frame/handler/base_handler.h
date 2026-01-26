//-----------------------------------------------------------------------------
// Copyright(c) 2016-2025 松灵机器人
// All rights reserved.
//
// File: base_frame.h
// Version: 1.0.00
//
// Create:  Feb.21 2025
// Author:  youguo.chu@agilex.ai
// Summary: 基础报文解析
//-----------------------------------------------------------------------------

#ifndef BASE_HANDLER_H_
#define BASE_HANDLER_H_

#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <map>
#include <cstring>
#include <unistd.h>
#include <nlohmann/json.hpp>


#include <can/socket_can.h>

using json = nlohmann::json;

namespace agx::nav {

/**
 * can指令操作基类
 */
class BaseHandler
{
public:
  BaseHandler();
  ~BaseHandler();
  virtual void handle(const json& data) = 0;

protected:
  void get_can_frame(struct can_frame& frame, uint16_t& instruction_id,
      int8_t& bit0, int8_t& bit1, int8_t& bit2, int8_t& bit3, int8_t& bit4,
      int8_t& bit5, int8_t& bit6, int8_t& bit7);

protected:
  int8_t zero = 0;
  int8_t one = 1;

};

}

#endif
