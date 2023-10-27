#include "SimWheel.h"
#include "SimDoubleWishbone.h"
#include "SimAxle_DW.h"

namespace tx_car {
void SimWheel::fetchWheelData(const tx_car::MBD_Vehicle_DW_DW &vehParam) {
  // todo, support double wheel, @dongyuanhu
  
  m_wheelRadius = 0.34;
  m_wheelSign = 1;

  if (m_axleNum == AxleNum::AXLE_0 && m_side == Side::SIDE_LEFT) {
    m_wheelData.CopyFrom(vehParam.front().wheelleft().at(0));
    m_wheelSign = 1;
  } else if (m_axleNum == AxleNum::AXLE_0 && m_side == Side::SIDE_RIGHT) {
    m_wheelData.CopyFrom(vehParam.front().wheelright().at(0));
    m_wheelSign = -1;
  }else if (m_axleNum == AxleNum::AXLE_1 && m_side == Side::SIDE_LEFT) {
    m_wheelData.CopyFrom(vehParam.rear().wheelleft().at(0));
    m_wheelSign = 1;
  } else if (m_axleNum == AxleNum::AXLE_1 && m_side == Side::SIDE_RIGHT) {
    m_wheelData.CopyFrom(vehParam.rear().wheelright().at(0));
    m_wheelSign = -1;
  } else {
    LOG_ERROR << "unnknow axle:" << (int)m_axleNum << ", side:" << (int)m_side
              << "\n";
    throw std::runtime_error("unknown wheel side and axle number");
  }
}
btScalar SimWheel::calWheelRadius() {
  m_wheelRadius =
      m_wheelData.radius().val() +
      m_wheelData.width().val() * m_wheelData.aspectratio().val() * 0.01;
  
  LOG_INFO << "wheel radius is " << m_wheelRadius << ".\n";

  return m_wheelRadius;
}
void SimWheel::construct(VehicleConstructHelper &helper) {
  // get vehicle parameter
  const auto &vehParam = helper.getVehicle_DWDW_Param();
  auto world = helper.getMultiBodyDynamicsWorld();

  // get wheel data
  fetchWheelData(vehParam);

  // calculate wheel radius
  m_wheelRadius = calWheelRadius();

  // get Spindle
  const auto &spindleSimBody = m_parent->getSpindle();

  // set frame in parent
  auto wheelPosInAxle =
      VehicleUtils::paramVec3TobtVector3(m_wheelData.offsetinaxle());
  const auto& spindleRot =
      spindleSimBody->m_thisFrameInParent.getThisRotationInParent();
  const auto &spindlePosInParent =
      spindleSimBody->m_thisFrameInParent.getThisPositionInParent();
  m_simBody.m_thisFrameInParent.setThisTransformInParent(wheelPosInAxle, spindleRot);

  VehicleUtils::printbtVector3(wheelPosInAxle, "wheelPosInAxle");

  // create wheel rigid body
  m_simBody.m_mass = m_wheelData.mass().val();
  m_simBody.m_inertia =
      VehicleUtils::paramVec3TobtVector3(m_wheelData.inertia());
  m_simBody.m_cylinderHalf =
      btVector3(m_wheelRadius, m_wheelData.width().val() * 0.5,
                m_wheelData.width().val() * 0.5);
  m_simBody.m_chassisBodyCollider =
      btCollisionShapePtr(new btCylinderShape(m_simBody.m_cylinderHalf));

  m_simBody.m_body = btRigidBodyPtr(VehicleUtils::createRigidBodyWithInertia(
      m_simBody.m_mass, m_simBody.m_inertia, m_simBody.m_thisFrameInParent.getThisTransformInParent(),
      m_simBody.m_chassisBodyCollider.get()));

  m_simBody.m_body->setFriction(0.9);

  world->addRigidBody(m_simBody.m_body.get());

  // add fix constraint with spindle
  btTransform frameInSpindle;
  btTransform frameInWheel;

  frameInSpindle.setIdentity();
  frameInWheel.setIdentity();

  btVector3 midPos = (wheelPosInAxle - spindlePosInParent) * 0.5;
  frameInWheel.setOrigin(
      btVector3(0, (-m_wheelData.width().val() * 0.5 - 0.015)*m_wheelSign, 0));
  frameInSpindle.setOrigin(btVector3(
      0, (spindleSimBody->m_cylinderHalf.y() + 0.015) * m_wheelSign, 0));

  /*auto fixConstraint = new btGeneric6DofConstraint(
      *spindleSimBody->m_body.get(), *m_simBody.m_body.get(), frameInSpindle,
      frameInWheel, true);*/
  auto fixConstraint = new btFixedConstraint(*spindleSimBody->m_body.get(),
                                             *m_simBody.m_body.get(),
                                             frameInSpindle, frameInWheel);

  for (auto i = 0; i <= 5; ++i) {
    fixConstraint->setLinearLowerLimit(btVector3(0, 0, 0));
    fixConstraint->setLinearUpperLimit(btVector3(0, 0, 0));
    fixConstraint->setAngularLowerLimit(btVector3(0, 0, 0));
    fixConstraint->setAngularUpperLimit(btVector3(0, 0, 0));

    fixConstraint->setParam(BT_CONSTRAINT_STOP_ERP, const_ERP, i);
    fixConstraint->setParam(BT_CONSTRAINT_ERP, const_ERP, i);
    fixConstraint->setParam(BT_CONSTRAINT_STOP_CFM, const_CFM, i);
    fixConstraint->setParam(BT_CONSTRAINT_CFM, const_CFM, i);
  }

  m_simBody.m_constraints["Wheel_Spindle"] = btTypedConstraintPtr(fixConstraint);

  world->addConstraint(fixConstraint);
}
void SimWheel::initialize(VehicleInitHelper &helper) {
  // chassis init transform in global
  const auto &axleInGlobal =
      m_parent->getParent()->getSimBody().m_thisFrameInGlobal;

  // set chassis body init location
  btTransform worldTransform = SimFrame::transfromLocalIntoSuper(
      axleInGlobal.getThisTransformInParent(),
      m_simBody.m_thisFrameInParent.getThisTransformInParent());

  VehicleUtils::printbtVector3(worldTransform.getOrigin(), "wheelPosInGlobal");
  
  m_simBody.m_body->setWorldTransform(worldTransform);

  // update chassis body frame in global
  m_simBody.m_thisFrameInGlobal.setThisTransformInParent(
      worldTransform.getOrigin(), worldTransform.getRotation());
}
void SimWheel::stepSimulation(VehicleStepHelper &helper) {}
} // namespace tx_car