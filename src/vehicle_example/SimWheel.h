#pragma once

#include "VehicleCore.h"

namespace tx_car {
/**
 * @brief SimWheel, wheel of vehicle
 */
class SimWheel : public VehicleTemplateBase {
public:
  SimWheel() {}
  virtual ~SimWheel() {}

  virtual void construct(VehicleConstructHelper &helper) override;
  virtual void initialize(VehicleInitHelper &helper) override;
  virtual void stepSimulation(VehicleStepHelper &helper) override;
  virtual void stop(VehicleStopHelper &helper) override {}

public:
  void setParent(SimDoubleWishBone *parent) { m_parent = parent; }
  void setSide(const Side &side = Side::SIDE_LEFT) { m_side = side; }
  void setAxleNumer(const AxleNum &axleNum = AxleNum::AXLE_0) {
    m_axleNum = axleNum;
  }
  void setDoubleWheel(bool doubleWheel = false) { m_doubleWheel = doubleWheel; }
  void fetchWheelData(const tx_car::MBD_Vehicle_DW_DW& vehParam);
  btScalar calWheelRadius();

public:
  Side getSide() const { return m_side; }
  bool getDoubleWheel() const { return m_doubleWheel; }
  AxleNum getAxleNum() const { return m_axleNum; }
  SimDoubleWishBone *getParent() { return m_parent; }

public:
  tx_car::MBD_Wheel m_wheelData;

  SimDoubleWishBone *m_parent;

  AxleNum m_axleNum;
  Side m_side;
  bool m_doubleWheel;
  btScalar m_wheelRadius;
  int m_wheelSign; // 1:left, -1:right
};
} // namespace tx_car