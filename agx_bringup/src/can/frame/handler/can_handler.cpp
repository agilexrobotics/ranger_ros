#include "can/frame/handler/can_handler.h"

namespace agx::nav {

CanHandler::CanHandler()
{
  std::unique_ptr<Handler111> handler_111(new Handler111());
  m_handler_map.insert(std::make_pair(agx::nav::Instruction::CTRL_MOTION, std::move(handler_111)));
  std::unique_ptr<Handler121> handler_121(new Handler121());
  m_handler_map.insert(std::make_pair(agx::nav::Instruction::LIGHT_CTRL, std::move(handler_121)));
  std::unique_ptr<Handler141> handler_141(new Handler141());
  m_handler_map.insert(std::make_pair(agx::nav::Instruction::MOTION_MODEL, std::move(handler_141)));
  std::unique_ptr<Handler423> handler_423(new Handler423());
  m_handler_map.insert(std::make_pair(agx::nav::Instruction::CTRL_SWITCH, std::move(handler_423)));
  std::unique_ptr<Handler441> handler_441(new Handler441());
  m_handler_map.insert(std::make_pair(agx::nav::Instruction::CLEAR_FAULT, std::move(handler_441)));
  std::unique_ptr<Handler461> handler_461(new Handler461());
  m_handler_map.insert(std::make_pair(agx::nav::Instruction::HIBERNATE, std::move(handler_461)));
}

CanHandler::~CanHandler()
{}

void CanHandler::handle(uint16_t instruction, const json& data)
{
  auto it = m_handler_map.find(instruction);  
  if (it != m_handler_map.end())
  {
    it->second->handle(data);
  }
}

}
