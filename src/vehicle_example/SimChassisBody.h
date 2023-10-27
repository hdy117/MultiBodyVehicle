#pragma once

#include "VehicleCore.h"

namespace tx_car {
/**
 * @brief SimChassisBody, chassis body
 */
class SimChassisBody : public VehicleTemplateBase {
public:
  SimChassisBody();
  virtual ~SimChassisBody(){};

public:
  virtual void construct(VehicleConstructHelper &helper) override;
  virtual void initialize(VehicleInitHelper &helper) override;
  virtual void stepSimulation(VehicleStepHelper &helper) override{};
  virtual void stop(VehicleStopHelper &helper) override {}

public:
  // local data
  tx_car::MBD_ChassisBody m_chassisBodyData;
};
} // namespace tx_car