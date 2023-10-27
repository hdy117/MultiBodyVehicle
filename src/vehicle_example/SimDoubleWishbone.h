#pragma once

#include "VehicleCore.h"
#include "SimWheel.h"

namespace tx_car {

/**
 * @brief double wishbone suspension
 */
class SimDoubleWishBone : public VehicleTemplateBase {
public:
  SimDoubleWishBone();
  virtual ~SimDoubleWishBone() {}

  virtual void construct(VehicleConstructHelper &helper) override;
  virtual void initialize(VehicleInitHelper &helper) override;
  virtual void stepSimulation(VehicleStepHelper &helper) override;
  virtual void stop(VehicleStopHelper &helper) override {}

public:
  void setSide(const Side &side = Side::SIDE_LEFT) { m_side = side; }
  void setAxleNumer(const AxleNum &axleNum = AxleNum::AXLE_0) {
    m_axleNum = axleNum;
  }
  void setDoubleWheel(bool doubleWheel = false) { m_doubleWheel = doubleWheel; }
  void setParent(SimAxle *parent = nullptr) { m_parent = parent; }

public:
  Side getSide() const { return m_side; }
  bool getDoubleWheel() const { return m_doubleWheel; }
  AxleNum getAxleNum() const { return m_axleNum; }
  const SimRigidBody &getUCA() const { return m_LCA_Body; }
  SimAxle *getParent() { return m_parent; }
  SimRigidBody *getSpindle() { return &m_spindleBody; }

  // get hinge
  btHingeConstraint *getUCA_ChassisBody_Front_Hinge() const {
    return m_UCA_ChassisBody_Front_Hinge;
  }
  btHingeConstraint *getUCA_ChassisBody_Rear_Hinge() const {
    return m_UCA_ChassisBody_Rear_Hinge;
  }

  btHingeConstraint *getLCA_ChassisBody_Front_Hinge() const {
    return m_LCA_ChassisBody_Front_Hinge;
  }
  btHingeConstraint *getLCA_ChassisBody_Rear_Hinge() const {
    return m_LCA_ChassisBody_Rear_Hinge;
  }

  btHingeConstraint *getUpright_UCA_Hinge() const {
    return m_Upright_UCA_Hinge;
  }
  btHingeConstraint *getUpright_LCA_Hinge() const {
    return m_Upright_LCA_Hinge;
  }

protected:
  /**
   * @brief get suspension data by axle and side, also calculate toe and camber
   * fliper
   * @param vehParam
   */
  void fetchSuspData(const tx_car::MBD_Vehicle_DW_DW &vehParam);

  void constructSpindle(btMultiBodyDynamicsWorld *world);
  void constructUpright(btMultiBodyDynamicsWorld *world);
  void constructLCA(btMultiBodyDynamicsWorld *world);
  void constructUCA(btMultiBodyDynamicsWorld *world);
  void constructSpringDamperConnector(btMultiBodyDynamicsWorld *world);

  /**
   * @brief add constraints
   * @param world
   */
  void addConstraint(btMultiBodyDynamicsWorld *world);
  /**
   * @brief add rotation hinge around y axis only constraint between spindle and
   * upright
   * @param world
   */
  void addSpindleAndUpright_HingeConstraint(btMultiBodyDynamicsWorld *world);

  /**
   * @brief add spherical constraint between upright and LCA
   * @param world
   */
  void addUprightAndLCA_Constraint(btMultiBodyDynamicsWorld *world);

  /**
   * @brief add spherical constraint between upright and UCA
   * @param world
   */
  void addUprightAndUCA_Constraint(btMultiBodyDynamicsWorld *world);

  /**
   * @brief add hinge constraint between UCA and chassis body
   * @param world
   */
  void addUCA_AndChassisBody_HingeConstraint(btMultiBodyDynamicsWorld *world);

  /**
   * @brief add hinge constraint between LCA and chassis body
   * @param world
   */
  void addLCA_AndChassisBody_HingeConstraint(btMultiBodyDynamicsWorld *world);

  /**
   * @brief add_SpringDamperConnector_HingeConstraint
   * @param world 
  */
  void add_SpringDamperConnector_HingeConstraint(btMultiBodyDynamicsWorld *world);

  /**
   * @brief add spring-damper
   * @param world
   */
  void addSpringDamper(btMultiBodyDynamicsWorld *world);
  void addSpringDamper2(btMultiBodyDynamicsWorld *world);

public:
  // data
  tx_car::MBD_DoubleWishBoneSusp m_suspData;

  // spring-damper
  //btTypedConstraintPtr m_springDamper;
  std::shared_ptr<SimDistanceSpringConstraint> m_springDamper;

  // spindle, upright, LCA, UCA
  SimRigidBody m_spindleBody;
  SimRigidBody m_uprightBody;
  SimRigidBody m_LCA_Body; // lower control arm body
  SimRigidBody m_UCA_Body; // upper control arm body
  SimRigidBody m_SpringDamper_LCA_Body; // SpringDamper connector in LCA
  SimRigidBody m_SpringDamper_Chassis_Body; // SpringDamper connector in Chassis

  // front and back position of LCA and UCA connected point with chassis body
  btVector3 m_localPos_LCA_Front,
      m_localPos_LCA_Rear; // LCA front and rear local position in Axle frame
  btVector3 m_localPos_UCA_Front,
      m_localPos_UCA_Rear; // LCA front and rear local position in Axle frame

  // hinge constraint
  btHingeConstraint *m_UCA_ChassisBody_Front_Hinge = nullptr;
  btHingeConstraint *m_UCA_ChassisBody_Rear_Hinge = nullptr;

  btHingeConstraint *m_LCA_ChassisBody_Front_Hinge = nullptr;
  btHingeConstraint *m_LCA_ChassisBody_Rear_Hinge = nullptr;

  btHingeConstraint *m_Upright_UCA_Hinge = nullptr;
  btHingeConstraint *m_Upright_LCA_Hinge = nullptr;

  // camber, toe fliper
  int m_camberFliper, m_toeFliper;

  // property
  Side m_side;
  bool m_doubleWheel;
  AxleNum m_axleNum;

  // wheel
  SimWheel m_wheel;

  // parent
  SimAxle *m_parent;
};
} // namespace tx_car