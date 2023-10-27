#pragma once

#include "VehicleCore.h"

#include "SimAxle_DW.h"
#include "SimChassisBody.h"
#include "SimDoubleWishbone.h"

namespace tx_car {
class SimVehicle_DWDW : public VehicleTemplateBase {
public:
  SimVehicle_DWDW();
  virtual ~SimVehicle_DWDW() {}

  virtual void construct(VehicleConstructHelper &helper) override;
  virtual void initialize(VehicleInitHelper &helper) override;
  virtual void stepSimulation(VehicleStepHelper &helper) override;
  virtual void stop(VehicleStopHelper &helper) override;

public:
  const SimAxle &getFrontAxle() const { return m_frontAxle; }
  const SimAxle &getRearAxle() const { return m_rearAxle; }
  const SimChassisBody &getChassisBody() const { return m_chassis; }

private:
  SimChassisBody m_chassis;
  SimAxle m_frontAxle;
  SimAxle m_rearAxle;
};
} // namespace tx_car