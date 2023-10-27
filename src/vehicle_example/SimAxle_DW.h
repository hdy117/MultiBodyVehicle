#pragma once

#include "VehicleCore.h"

#include "SimDoubleWishbone.h"

namespace tx_car {

/**
 * @brief axle
 */
class SimAxle : public VehicleTemplateBase {
public:
  SimAxle();
  virtual ~SimAxle() {}

  virtual void construct(VehicleConstructHelper &helper) override;
  virtual void initialize(VehicleInitHelper &helper) override;
  virtual void stepSimulation(VehicleStepHelper &helper) override;
  virtual void stop(VehicleStopHelper &helper) override {}

public:
  void setAxleNumer(const AxleNum &axleNum = AxleNum::AXLE_0) {
    m_axleNumber = axleNum;
  }
  void setSteerable(bool steerable = false) { m_steerable = steerable; }
  void setParent(SimChassisBody *parent) { m_parent = parent; }

public:
  AxleNum getAxleNumer() const {
    return m_axleNumber;
  }

  bool getSteerable() const { return m_steerable; }

  const tx_car::MBD_Axle_DoubleWishBone &getAxleData() const {
    return m_axleData;
  }

  SimChassisBody *getParent() const { return m_parent; }

  const SimDoubleWishBone &getLeftSuspension() const { return m_leftSusp; }
  const SimDoubleWishBone &getRightSuspension() const { return m_rightSusp; }

private:
  // data
  tx_car::MBD_Axle_DoubleWishBone m_axleData;

  // property
  bool m_steerable;
  AxleNum m_axleNumber;

  // suspension
  SimDoubleWishBone m_leftSusp;
  SimDoubleWishBone m_rightSusp;

  // drive axle inertia
  btVector3 m_axleInertia;

  // parent
  SimChassisBody *m_parent;
};
} // namespace tx_car