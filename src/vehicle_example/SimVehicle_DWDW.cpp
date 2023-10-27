#include "SimVehicle_DWDW.h"

namespace tx_car {
/*=======================*/
SimVehicle_DWDW::SimVehicle_DWDW() {}

void SimVehicle_DWDW::construct(VehicleConstructHelper &helper) {
  // set axle property
  {
    m_frontAxle.setParent(&m_chassis);
    m_frontAxle.setAxleNumer(AxleNum::AXLE_0);
    m_frontAxle.setSteerable(true);

    m_rearAxle.setParent(&m_chassis);
    m_rearAxle.setAxleNumer(AxleNum::AXLE_1);
    m_rearAxle.setSteerable(false);
  }

  // do construct
  m_chassis.construct(helper);
  m_frontAxle.construct(helper);
  m_rearAxle.construct(helper);
}

void SimVehicle_DWDW::initialize(VehicleInitHelper &helper) {
  m_chassis.initialize(helper);
  m_frontAxle.initialize(helper);
  m_rearAxle.initialize(helper);
}

void SimVehicle_DWDW::stepSimulation(VehicleStepHelper &helper) {
  m_chassis.stepSimulation(helper);
  m_frontAxle.stepSimulation(helper);
  m_rearAxle.stepSimulation(helper);
}

void SimVehicle_DWDW::stop(VehicleStopHelper &helper) {
  m_chassis.stop(helper);
  m_frontAxle.stop(helper);
  m_rearAxle.stop(helper);
}
} // namespace tx_car