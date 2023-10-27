#include "SimChassisBody.h"

namespace tx_car {
/*=======================*/
SimChassisBody::SimChassisBody() {}

void SimChassisBody::construct(VehicleConstructHelper &helper) {
  const auto &vehParam = helper.getVehicle_DWDW_Param();
  auto m_world = helper.getMultiBodyDynamicsWorld();

  // copy data
  m_chassisBodyData.CopyFrom(vehParam.chassisbody());

  // half size of bounding box
  m_simBody.m_halfBoundingBox =
      VehicleUtils::paramVec3TobtVector3(vehParam.chassisbody().boundingbox());

  // add collision shape
  m_simBody.m_chassisBodyCollider =
      btCollisionShapePtr(new btBoxShape(m_simBody.m_halfBoundingBox));

  // set local transform
  /*auto cgOffset =
      VehicleUtils::paramVec3TobtVector3(vehParam.chassisbody().offsetofcg());*/
  m_simBody.m_thisFrameInParent.setThisTransformInParent(
      btVector3(0, 0, 0), btQuaternion(0, 0, 0, 1));

  // mass and inertia
  m_simBody.m_mass = vehParam.chassisbody().mass().val();

  m_simBody.m_inertia =
      VehicleUtils::paramVec3TobtVector3(vehParam.chassisbody().inertia()) *
      m_simBody.m_mass / 2086;

  LOG_INFO << "chasssis body mass[kg]:" << m_simBody.m_mass << "\n";
  VehicleUtils::printbtVector3(m_simBody.m_inertia, "chassisbody.m_inertia");

  // create chassis body with specific inertia
  m_simBody.m_body = btRigidBodyPtr(VehicleUtils::createRigidBodyWithInertia(
      m_simBody.m_mass, m_simBody.m_inertia,
      m_simBody.m_thisFrameInParent.getThisTransformInParent(),
      m_simBody.m_chassisBodyCollider.get()));

  // set CG offset
  /*m_simBody.m_body->setCenterOfMassTransform(
      btTransform(btQuaternion(0, 0, 0, 1), cgOffset));*/

  m_world->addRigidBody(m_simBody.m_body.get(),0,0);
}

void SimChassisBody::initialize(VehicleInitHelper &helper) {
  // set chassis body init location
  btTransform worldTransform =
      btTransform(helper.getInitRotation(), helper.getInitPosition()) *
      m_simBody.m_thisFrameInParent.getThisTransformInParent();

  // update rigid body initial transform
  m_simBody.m_body->setWorldTransform(worldTransform);

  // update chassis body frame in global
  m_simBody.m_thisFrameInGlobal.setThisTransformInParent(
      worldTransform.getOrigin(), worldTransform.getRotation());
}

} // namespace tx_car