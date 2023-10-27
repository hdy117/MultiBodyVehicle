#include "SimAxle_DW.h"
#include "SimChassisBody.h"

namespace tx_car {

SimAxle::SimAxle()
    : m_axleNumber(AxleNum::AXLE_0), m_steerable(false), m_parent(nullptr) {
}

void SimAxle::construct(VehicleConstructHelper &helper) {
  // get necessary info
  const auto &vehParam = helper.getVehicle_DWDW_Param();
  auto world = helper.getMultiBodyDynamicsWorld();

  // get axle data
  if (m_axleNumber == AxleNum::AXLE_0) {
    // copy data
    m_axleData.CopyFrom(vehParam.front());
  } else if (m_axleNumber == AxleNum::AXLE_1) {
    // copy data
    m_axleData.CopyFrom(vehParam.rear());
  } else {
    LOG(ERROR) << "unknown axle numer " << static_cast<int>(m_axleNumber)
               << "\n";
    throw std::runtime_error("unknow axle numer");
  }

  // offset of CG
  auto cgOffset = -1 * VehicleUtils::paramVec3TobtVector3(vehParam.chassisbody().offsetofcg());

  VehicleUtils::printbtVector3(cgOffset, "cgOffset");

  // get axle offset in chassis body
  auto axleOffsetInBody =
      VehicleUtils::paramVec3TobtVector3(m_axleData.offsetinchassisbody()) +
      cgOffset;
  m_simBody.m_thisFrameInParent.setThisTransformInParent(
      axleOffsetInBody, btQuaternion(0, 0, 0, 1));

  // inertia
  btVector3 halfAxleInertia = {0, m_axleData.axleinertia().val(), 0};

  // left susp construct
  {
    m_leftSusp.setParent(this);
    m_leftSusp.setDoubleWheel(false);
    m_leftSusp.setSide(Side::SIDE_LEFT);
    m_leftSusp.setAxleNumer(m_axleNumber);

    m_leftSusp.construct(helper);
  }

  // right susp construct
  {
    m_rightSusp.setParent(this);
    m_rightSusp.setDoubleWheel(false);
    m_rightSusp.setSide(Side::SIDE_RIGHT);
    m_rightSusp.setAxleNumer(m_axleNumber);

    m_rightSusp.construct(helper);
  }

  // construct steer if necessary
  if (m_steerable) {
    /*to do, add steer construct, @dongyuanhu*/
  }
}

void SimAxle::initialize(VehicleInitHelper &helper) {
  // chassis init transform in global
  const auto &parentInGlobal = m_parent->getSimBody().m_thisFrameInGlobal;

  // set chassis body init location
  btTransform worldTransform = SimFrame::transfromLocalIntoSuper(
      parentInGlobal.getThisTransformInParent(),
      m_simBody.m_thisFrameInParent.getThisTransformInParent());

  // update axle frame in global
  m_simBody.m_thisFrameInGlobal.setThisTransformInParent(
      worldTransform.getOrigin(), worldTransform.getRotation());

  // initialize suspension
  m_leftSusp.initialize(helper);
  m_rightSusp.initialize(helper);
}

void SimAxle::stepSimulation(VehicleStepHelper &helper) {
  m_leftSusp.stepSimulation(helper);
  m_rightSusp.stepSimulation(helper);
}

} // namespace tx_car