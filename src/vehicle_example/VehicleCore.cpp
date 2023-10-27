#include "VehicleCore.h"

namespace tx_car {
VehicleHelperBase::VehicleHelperBase() {
  m_vehicleParam_DWDW.Clear();
  m_stepTime = 0.001f;
  m_world = nullptr;
}
VehicleHelperBase::~VehicleHelperBase() {}

void VehicleHelperBase::setVehicleDoubleWishboneParam(
    const tx_car::MBD_Vehicle_DW_DW &vehicleParam) {
  m_vehicleParam_DWDW = vehicleParam;
}

void VehicleHelperBase::setMultiBodyDynamicsWOrld(
    btMultiBodyDynamicsWorld *p_world) {
  m_world = p_world;
}

void VehicleHelperBase::setStepTime(btScalar stepTime) {
  m_stepTime = stepTime;
}

SimDistanceSpringConstraint::SimDistanceSpringConstraint(
    btRigidBody *bodyA, btRigidBody *bodyB, const btVector3 &pivotInA,
    const btVector3 &pivotInB, btScalar stepTime) {
  m_bodyA = bodyA;
  m_bodyB = bodyB;
  m_pivotInA = pivotInA;
  m_pivotInB = pivotInB;

  m_currentLength = calculateDistane();
  m_stepTime = stepTime;
}
SimDistanceSpringConstraint::~SimDistanceSpringConstraint() {

}
void SimDistanceSpringConstraint::setStiffness(btScalar stiffness) {
  m_stiffness = stiffness;
}
void SimDistanceSpringConstraint::setDamping(btScalar damping) {
  m_damping = damping;
}
void SimDistanceSpringConstraint::setFreeLength(btScalar freeLength ) {
  m_freeLength = freeLength;
  m_preLength = m_freeLength;
}
btScalar SimDistanceSpringConstraint::getCurrentLength() {
  return m_currentLength;
}
btScalar SimDistanceSpringConstraint::getCurrentVelocity() {
  return m_currentVelocity;
}
void SimDistanceSpringConstraint::calSpringVelocity() {
  m_currentVelocity = (m_currentLength - m_preLength) / m_stepTime;
}
void SimDistanceSpringConstraint::getForceGlobal_A(btVector3 &force,
                                                   btVector3 &pivotInA) {
  force = m_springForceGlobal_OnA;
  pivotInA = m_pivotInA;
}
void SimDistanceSpringConstraint::getForceGlobal_B(btVector3 &force,
                                                   btVector3 &pivotInB) {
  force = m_springForceGlobal_OnB;
  pivotInB = m_pivotInB;
}
btScalar SimDistanceSpringConstraint::getSpringForce() { return m_springForce; }

void SimDistanceSpringConstraint::calSpringForce() {
  m_springForce = -m_stiffness * (m_currentLength - m_freeLength) -
         m_damping * getCurrentVelocity();
}
void SimDistanceSpringConstraint::calSpringForceInGlobal() {
  m_springForceGlobal_OnA = m_springForce * m_dirFromB2A;
  m_springForceGlobal_OnB = m_springForce * m_dirFromA2B;
}
btScalar SimDistanceSpringConstraint::calculateDistane() {
  const auto &worldTransA = m_bodyA->getWorldTransform();
  const auto &worldTransB = m_bodyB->getWorldTransform();

  btVector3 pivotInA_Global =
      quatRotate(worldTransA.getRotation(), m_pivotInA) +
      worldTransA.getOrigin();
  btVector3 pivotInB_Global =
      quatRotate(worldTransB.getRotation(), m_pivotInB) +
      worldTransB.getOrigin();
  m_dirFromA2B = (pivotInA_Global - pivotInB_Global).normalize();
  m_dirFromB2A = (pivotInB_Global - pivotInA_Global).normalize();

  return (pivotInA_Global - pivotInB_Global).norm();
}
void SimDistanceSpringConstraint::update() { 
    // update distance
    m_currentLength = calculateDistane();
    calSpringVelocity();
    calSpringForce();
    calSpringForceInGlobal();

    // force
    btVector3 forceOnLCA, pivotInLCA;
    btVector3 forceOnChassisBody, pivotInChassisBody;

    getForceGlobal_A(forceOnLCA, pivotInLCA);
    getForceGlobal_B(forceOnChassisBody, pivotInChassisBody);

    m_bodyA->applyForce(forceOnLCA, pivotInLCA);
    m_bodyB->applyForce(forceOnChassisBody, pivotInChassisBody);
    LOG_INFO << "m_preLength:" << m_preLength
             << ", spring length:" << getCurrentLength()
             << ", velocity:" << getCurrentVelocity()
             << ", springForce:" << getSpringForce() << "\n";

    m_preLength = m_currentLength;
}

tx_car::MBD_Vehicle_DW_DW VehicleHelperBase::getVehicle_DWDW_Param() {
  return m_vehicleParam_DWDW;
}

btMultiBodyDynamicsWorld *VehicleHelperBase::getMultiBodyDynamicsWorld() {
  return m_world;
}

btScalar VehicleHelperBase::getStepTime() { return m_stepTime; }
} // namespace tx_car
