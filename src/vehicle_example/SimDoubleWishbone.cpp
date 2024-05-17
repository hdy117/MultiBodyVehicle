#include "SimDoubleWishbone.h"

#include "SimAxle_DW.h"
#include "SimChassisBody.h"
#include "SimWheel.h"

#define SIM_DEBUG_DRAW_CONSTRAINT
// #define SIM_DEBUG_DRAW_LCA_HINGE_CONSTRAINT
// #define SIM_DEBUG_DRAW_UCA_HINGE_CONSTRAINT
#define SIM_DEBUG_DRAW_SPRINGDAMPER_CONSTRAINT

namespace tx_car {
/*=======================*/
SimDoubleWishBone::SimDoubleWishBone()
    : m_side(Side::SIDE_LEFT),
      m_doubleWheel(false),
      m_axleNum(AxleNum::AXLE_0),
      m_parent(nullptr) {
  m_camberFliper = 1;
  m_toeFliper = 1;
}

void SimDoubleWishBone::fetchSuspData(
    const tx_car::MBD_Vehicle_DW_DW &vehParam) {
  // copy data
  if (getSide() == Side::SIDE_LEFT && getAxleNum() == AxleNum::AXLE_0) {
    m_suspData.CopyFrom(vehParam.front().suspleft());
    m_toeFliper = -1;
    m_camberFliper = 1;
  } else if (getSide() == Side::SIDE_RIGHT && getAxleNum() == AxleNum::AXLE_0) {
    m_suspData.CopyFrom(vehParam.front().suspright());
    m_toeFliper = 1;
    m_camberFliper = -1;
  } else if (getSide() == Side::SIDE_LEFT && getAxleNum() == AxleNum::AXLE_1) {
    m_suspData.CopyFrom(vehParam.rear().suspleft());
    m_toeFliper = -1;
    m_camberFliper = 1;
  } else if (getSide() == Side::SIDE_RIGHT && getAxleNum() == AxleNum::AXLE_1) {
    m_suspData.CopyFrom(vehParam.rear().suspright());
    m_toeFliper = 1;
    m_camberFliper = -1;
  } else {
    LOG_ERROR << "unknown suspension side:" << static_cast<int>(getSide())
              << ", unknow axle number:" << static_cast<int>(getAxleNum())
              << ".\n";
    throw std::runtime_error("unknow suspension side and axle number.");
  }
}

void SimDoubleWishBone::constructSpindle(btMultiBodyDynamicsWorld *world) {
  // get axle data
  const auto &axleData = m_parent->getAxleData();

  // get spindle data
  const auto &spindleData = m_suspData.spindle();

  // calculate rot of spindle in parent frame using toe and camber angle
  double yaw = axleData.toe().val() * m_toeFliper * DEG_2_RAD;
  double roll = axleData.camber().val() * m_camberFliper * DEG_2_RAD;
  btQuaternion spindleRotInParent;
  spindleRotInParent.setEulerZYX(yaw, 0.0, roll);

  m_spindleBody.m_thisFrameInParent.setThisTransformInParent(
      VehicleUtils::paramVec3TobtVector3(m_suspData.spindle().offset()),
      spindleRotInParent);

  // create body
  m_spindleBody.m_mass = spindleData.mass().val();
  m_spindleBody.m_inertia =
      VehicleUtils::paramVec3TobtVector3(spindleData.inertia());
  m_spindleBody.m_cylinderHalf =
      btVector3(spindleData.radius().val(), spindleData.width().val() / 2,
                spindleData.width().val() / 2);
  m_spindleBody.m_chassisBodyCollider =
      btCollisionShapePtr(new btCylinderShape(m_spindleBody.m_cylinderHalf));

  m_spindleBody.m_body =
      btRigidBodyPtr(VehicleUtils::createRigidBodyWithInertia(
          m_spindleBody.m_mass, m_spindleBody.m_inertia,
          m_spindleBody.m_thisFrameInParent.getThisTransformInParent(),
          m_spindleBody.m_chassisBodyCollider.get()));

  world->addRigidBody(m_spindleBody.m_body.get());
}

void SimDoubleWishBone::constructUpright(btMultiBodyDynamicsWorld *world) {
  // get upright data
  const auto &upright = m_suspData.upright();

  m_uprightBody.m_thisFrameInParent.setThisTransformInParent(
      VehicleUtils::paramVec3TobtVector3(m_suspData.upright().offset()),
      btQuaternion(0, 0, 0, 1));

  // create body
  m_uprightBody.m_mass = upright.mass().val();
  m_uprightBody.m_inertia =
      VehicleUtils::paramVec3TobtVector3(upright.inertia());
  m_uprightBody.m_radius = upright.radius().val();
  /*m_uprightBody.m_chassisBodyCollider =
      btCollisionShapePtr(new btSphereShape(m_uprightBody.m_radius));*/
  m_uprightBody.m_chassisBodyCollider = btCollisionShapePtr(
      new btCylinderShape(btVector3(m_uprightBody.m_radius, 0.05, 0.05)));

  m_uprightBody.m_body =
      btRigidBodyPtr(VehicleUtils::createRigidBodyWithInertia(
          m_uprightBody.m_mass, m_uprightBody.m_inertia,
          m_uprightBody.m_thisFrameInParent.getThisTransformInParent(),
          m_uprightBody.m_chassisBodyCollider.get()));

  world->addRigidBody(m_uprightBody.m_body.get());
}

void SimDoubleWishBone::constructLCA(btMultiBodyDynamicsWorld *world) {
  // get LCA data
  const auto &LCA = m_suspData.lowercontrolarm();

  // calculate rot of lower control arm
  const auto &LCA_Front_ChassisBody = VehicleUtils::paramVec3TobtVector3(
      m_suspData.lowercontrolarm().offsetinchassisfront());
  const auto &LCA_Back_ChassisBody = VehicleUtils::paramVec3TobtVector3(
      m_suspData.lowercontrolarm().offsetinchassisback());
  btVector3 LCA_Middle_ChassisBody =
      (LCA_Front_ChassisBody + LCA_Back_ChassisBody) * 0.5;
  btVector3 LCA_InAxle = VehicleUtils::paramVec3TobtVector3(LCA.offset());
  btVector3 dirAxle = LCA_Middle_ChassisBody -
                      VehicleUtils::paramVec3TobtVector3(LCA.offsetinupright());
  btScalar rollOfLowerControlArm = 0.0;
  if (m_side == Side::SIDE_LEFT) {
    rollOfLowerControlArm = std::atan2(dirAxle[2], dirAxle[1]) - M_PI;
  } else {
    rollOfLowerControlArm = std::atan2(dirAxle[2], dirAxle[1]);
  }

  // rollOfLowerControlArm = 0.0;
  LOG_INFO << "rollOfLowerControlArm[deg]:" << rollOfLowerControlArm * RAD_2_DEG
           << "\n";

  m_LCA_Body.m_thisFrameInParent.setThisTransformInParent(
      VehicleUtils::paramVec3TobtVector3(LCA.offset()),
      btQuaternion(btVector3(1, 0, 0), rollOfLowerControlArm));

  // create body
  m_LCA_Body.m_mass = LCA.mass().val();
  m_LCA_Body.m_inertia = VehicleUtils::paramVec3TobtVector3(LCA.inertia());
  m_LCA_Body.m_radius = LCA.radius().val();
  /*m_LCA_Body.m_chassisBodyCollider =
      btCollisionShapePtr(new btSphereShape(m_LCA_Body.m_radius));*/
  m_LCA_Body.m_chassisBodyCollider = btCollisionShapePtr(
      new btCylinderShape(btVector3(m_LCA_Body.m_radius, 0.05, 0.05)));

  m_LCA_Body.m_body = btRigidBodyPtr(VehicleUtils::createRigidBodyWithInertia(
      m_LCA_Body.m_mass, m_LCA_Body.m_inertia,
      m_LCA_Body.m_thisFrameInParent.getThisTransformInParent(),
      m_LCA_Body.m_chassisBodyCollider.get()));

  world->addRigidBody(m_LCA_Body.m_body.get());
}

void SimDoubleWishBone::constructUCA(btMultiBodyDynamicsWorld *world) {
  // get UCA data
  const auto &UCA = m_suspData.uppercontrolarm();

  // calculate rot of upper control arm
  const auto &UCA_Front_ChassisBody = VehicleUtils::paramVec3TobtVector3(
      m_suspData.uppercontrolarm().offsetinchassisfront());
  const auto &UCA_Back_ChassisBody = VehicleUtils::paramVec3TobtVector3(
      m_suspData.uppercontrolarm().offsetinchassisback());
  btVector3 UCA_Middle_ChassisBody =
      (UCA_Front_ChassisBody + UCA_Back_ChassisBody) * 0.5;
  btVector3 UCA_InAxle = VehicleUtils::paramVec3TobtVector3(UCA.offset());
  btVector3 dirAxle = UCA_Middle_ChassisBody -
                      VehicleUtils::paramVec3TobtVector3(UCA.offsetinupright());
  btScalar rollOfUpperControlArm = 0.0;
  if (m_side == Side::SIDE_LEFT) {
    rollOfUpperControlArm = std::atan2(dirAxle[2], dirAxle[1]) - M_PI;
  } else {
    rollOfUpperControlArm = std::atan2(dirAxle[2], dirAxle[1]);
  }
  // rollOfUpperControlArm = 0.0;

  LOG_INFO << "rollOfUpperControlArm[deg]:" << rollOfUpperControlArm * RAD_2_DEG
           << "\n";

  m_UCA_Body.m_thisFrameInParent.setThisTransformInParent(
      UCA_InAxle, btQuaternion(btVector3(1, 0, 0), rollOfUpperControlArm));

  // create body
  m_UCA_Body.m_mass = UCA.mass().val();
  m_UCA_Body.m_inertia = VehicleUtils::paramVec3TobtVector3(UCA.inertia());
  m_UCA_Body.m_radius = UCA.radius().val();
  /*m_UCA_Body.m_chassisBodyCollider =
      btCollisionShapePtr(new btSphereShape(m_UCA_Body.m_radius));*/
  m_UCA_Body.m_chassisBodyCollider = btCollisionShapePtr(
      new btCylinderShape(btVector3(m_UCA_Body.m_radius, 0.05, 0.05)));

  m_UCA_Body.m_body = btRigidBodyPtr(VehicleUtils::createRigidBodyWithInertia(
      m_UCA_Body.m_mass, m_UCA_Body.m_inertia,
      m_UCA_Body.m_thisFrameInParent.getThisTransformInParent(),
      m_UCA_Body.m_chassisBodyCollider.get()));

  world->addRigidBody(m_UCA_Body.m_body.get());
}

void SimDoubleWishBone::constructSpringDamperConnector(
    btMultiBodyDynamicsWorld *world) {
  // get spring-damper local position
  btVector3 sp_localPosInAxle =
      VehicleUtils::paramVec3TobtVector3(m_suspData.spring().offsetinaxle());
  btVector3 sp_localPosInChassisBody =
      VehicleUtils::paramVec3TobtVector3(m_suspData.spring().offsetinchassis());

  // set property of rigidbody
  btScalar mass = 0.1;
  btScalar radius = 0.05;
  m_SpringDamper_LCA_Body.m_mass = mass;
  m_SpringDamper_LCA_Body.m_radius = radius;
  m_SpringDamper_LCA_Body.m_chassisBodyCollider =
      btCollisionShapePtr(new btSphereShape(m_SpringDamper_LCA_Body.m_radius));
  m_SpringDamper_LCA_Body.m_chassisBodyCollider->calculateLocalInertia(
      m_SpringDamper_LCA_Body.m_mass, m_SpringDamper_LCA_Body.m_inertia);
  m_SpringDamper_LCA_Body.m_thisFrameInParent.setThisTransformInParent(
      sp_localPosInAxle, btQuaternion(0, 0, 0, 1));

  m_SpringDamper_Chassis_Body.m_mass = mass;
  m_SpringDamper_Chassis_Body.m_radius = radius;
  m_SpringDamper_Chassis_Body.m_chassisBodyCollider = btCollisionShapePtr(
      new btSphereShape(m_SpringDamper_Chassis_Body.m_radius));
  m_SpringDamper_Chassis_Body.m_chassisBodyCollider->calculateLocalInertia(
      m_SpringDamper_Chassis_Body.m_mass,
      m_SpringDamper_Chassis_Body.m_inertia);
  m_SpringDamper_Chassis_Body.m_thisFrameInParent.setThisTransformInParent(
      sp_localPosInChassisBody, btQuaternion(0, 0, 0, 1));

  // create rigid body
  m_SpringDamper_LCA_Body
      .m_body = btRigidBodyPtr(VehicleUtils::createRigidBodyWithInertia(
      m_SpringDamper_LCA_Body.m_mass, m_SpringDamper_LCA_Body.m_inertia,
      m_SpringDamper_LCA_Body.m_thisFrameInParent.getThisTransformInParent(),
      m_SpringDamper_LCA_Body.m_chassisBodyCollider.get()));
  m_SpringDamper_Chassis_Body.m_body =
      btRigidBodyPtr(VehicleUtils::createRigidBodyWithInertia(
          m_SpringDamper_Chassis_Body.m_mass,
          m_SpringDamper_Chassis_Body.m_inertia,
          m_SpringDamper_Chassis_Body.m_thisFrameInParent
              .getThisTransformInParent(),
          m_SpringDamper_Chassis_Body.m_chassisBodyCollider.get()));

  // disable collision
  m_SpringDamper_LCA_Body.m_body->setCollisionFlags(
      m_SpringDamper_LCA_Body.m_body->getCollisionFlags() |
      btCollisionObject::CF_NO_CONTACT_RESPONSE);
  m_SpringDamper_Chassis_Body.m_body->setCollisionFlags(
      m_SpringDamper_Chassis_Body.m_body->getCollisionFlags() |
      btCollisionObject::CF_NO_CONTACT_RESPONSE);

  // add rigid body
  world->addRigidBody(m_SpringDamper_LCA_Body.m_body.get(), 0, 0);
  world->addRigidBody(m_SpringDamper_Chassis_Body.m_body.get(), 0, 0);
}

void SimDoubleWishBone::construct(VehicleConstructHelper &helper) {
  LOG_INFO << SIM_SEPERATOR << "\n";
  LOG_INFO << "Axle:" << (int)m_axleNum << ", Side:" << (int)m_side << "\n";
  // get world
  auto world = helper.getMultiBodyDynamicsWorld();
  const auto &vehParam = helper.getVehicle_DWDW_Param();

  fetchSuspData(vehParam);

  // spindle
  constructSpindle(world);

  // up-right
  constructUpright(world);

  // upper control arm
  constructUCA(world);

  // lower control arm
  constructLCA(world);

  // spring-damper connector construct
  constructSpringDamperConnector(world);

  // add constraint
  addConstraint(world);

  /*todo, add double wheel, @dongyaunhu*/
  {
    m_wheel.setParent(this);
    m_wheel.setAxleNumer(m_axleNum);
    m_wheel.setSide(m_side);
    m_wheel.setDoubleWheel(m_doubleWheel);

    m_wheel.construct(helper);
  }
}

void SimDoubleWishBone::addConstraint(btMultiBodyDynamicsWorld *world) {
  addSpindleAndUpright_HingeConstraint(world);
  addUprightAndLCA_Constraint(world);
  addUprightAndUCA_Constraint(world);
  addUCA_AndChassisBody_HingeConstraint(world);
  addLCA_AndChassisBody_HingeConstraint(world);
  add_SpringDamperConnector_HingeConstraint(world);
  // addSpringDamper(world);
  addSpringDamper2(world);
}

void SimDoubleWishBone::add_SpringDamperConnector_HingeConstraint(
    btMultiBodyDynamicsWorld *world) {
  // LCA and SpringDamper LCA
  auto LCA_Body = m_LCA_Body.m_body;
  auto sp_LCA_Body = m_SpringDamper_LCA_Body.m_body;

  btVector3 hingleAxle{1, 0, 0};  // spindle and upright rotate around y-axis

  btVector3 LCA_LocalInAxle =
      m_LCA_Body.m_thisFrameInParent.getThisPositionInParent();
  btVector3 sp_localInAxle =
      m_SpringDamper_LCA_Body.m_thisFrameInParent.getThisPositionInParent();

  auto constraint = new btHingeConstraint(
      *LCA_Body.get(), *sp_LCA_Body.get(), sp_localInAxle - LCA_LocalInAxle,
      btVector3(0, 0, 0), hingleAxle, hingleAxle);

  const btScalar springDamperHingeAngleLimit = 30.0;

  constraint->setLimit(-springDamperHingeAngleLimit * DEG_2_RAD,
                       springDamperHingeAngleLimit * DEG_2_RAD);

  m_LCA_Body.m_constraints["LCA_SpringDamper"] =
      btTypedConstraintPtr(constraint);

  for (auto i = 0; i <= 5; ++i) {
    constraint->setParam(BT_CONSTRAINT_STOP_ERP, const_ERP, i);
    constraint->setParam(BT_CONSTRAINT_ERP, const_ERP, i);
    constraint->setParam(BT_CONSTRAINT_STOP_CFM, const_CFM, i);
    constraint->setParam(BT_CONSTRAINT_CFM, const_CFM, i);
  }

  world->addConstraint(constraint);

  // chassis and SpringDamper chassis
  const auto &simChassisBody = m_parent->getParent()->getSimBody();
  const auto &simAxleBody = m_parent->getSimBody();

  auto chassis_body = simChassisBody.m_body;
  auto sp_chassis_Body = m_SpringDamper_Chassis_Body.m_body;

  const auto &sp_LocalPosInChassis =
      m_SpringDamper_Chassis_Body.m_thisFrameInParent.getThisPositionInParent();
  const auto &axle_LocalPosInChassis =
      simAxleBody.m_thisFrameInParent.getThisPositionInParent();

  btVector3 pivotInChassis = sp_LocalPosInChassis + axle_LocalPosInChassis;

  VehicleUtils::printbtVector3(pivotInChassis,
                               "spring-damper connector in chassis");

  constraint = new btHingeConstraint(*sp_chassis_Body.get(),
                                     *chassis_body.get(), btVector3(0, 0, 0),
                                     pivotInChassis, hingleAxle, hingleAxle);

  constraint->setLimit(-springDamperHingeAngleLimit * DEG_2_RAD,
                       springDamperHingeAngleLimit * DEG_2_RAD);

  for (auto i = 0; i <= 5; ++i) {
    constraint->setParam(BT_CONSTRAINT_STOP_ERP, const_ERP, i);
    constraint->setParam(BT_CONSTRAINT_ERP, const_ERP, i);
    constraint->setParam(BT_CONSTRAINT_STOP_CFM, const_CFM, i);
    constraint->setParam(BT_CONSTRAINT_CFM, const_CFM, i);
  }

  std::string constraintName = "Chassis_SpringDamper" +
                               std::to_string((int)m_axleNum) + "_" +
                               std::to_string((int)m_side);
  m_LCA_Body.m_constraints[constraintName] = btTypedConstraintPtr(constraint);

  world->addConstraint(constraint);
}

void SimDoubleWishBone::addSpringDamper(btMultiBodyDynamicsWorld *world) {
  // get axle and chassis simbody
  auto axleSimBody =
      m_parent->getSimBody();  // SimAxle has no rigid body for now !!!
  auto chassisSimBody = m_parent->getParent()->getSimBody();

  // get chassis and UCA rigid body
  auto sp_chassisBody = m_SpringDamper_Chassis_Body.m_body;
  auto sp_LCA_Body = m_SpringDamper_LCA_Body.m_body;

  // get axle local position in chassis
  const auto &axle_LocalPosInChassisBody =
      axleSimBody.m_thisFrameInParent.getThisPositionInParent();

  // get spring-damper local position
  btVector3 sp_localPosInAxle =
      m_SpringDamper_LCA_Body.m_thisFrameInParent.getThisPositionInParent();
  btVector3 sp_localPosInChassisBody =
      m_SpringDamper_Chassis_Body.m_thisFrameInParent.getThisPositionInParent();

  // get spring and damper
  btScalar stiffness = m_suspData.spring().stiffness().val();
  btScalar damping = m_suspData.damper().damping().val();
  btScalar freeLength = m_suspData.spring().freelength().val();

  // local pivot in LCA and ChassisBody
  btTransform transInLCA;
  btTransform transInChassis;

  // in axle frame, from LCA connect point to chassis connect point
  btVector3 dirAxle = sp_localPosInChassisBody - sp_localPosInAxle;
  // btVector3 dirAxle = sp_localPosInChassisBody - (sp_localPosInAxle);
  btScalar rollOfSpringDamper = -std::atan2(dirAxle[1], dirAxle[2]);
  VehicleUtils::printbtVector3(axle_LocalPosInChassisBody,
                               "axle_LocalPosInChassisBody");
  VehicleUtils::printbtVector3(sp_localPosInAxle, "sp_localPosInAxle");
  VehicleUtils::printbtVector3(sp_localPosInChassisBody,
                               "sp_localPosInChassisBody");
  LOG_INFO << "rollOfSpringDamper[deg]:" << (rollOfSpringDamper)*RAD_2_DEG
           << "\n";
  LOG_INFO << "stiffness[]:" << stiffness << ", damping[]:" << damping << "\n";
  btQuaternion springDamperRot(btVector3(1, 0, 0), rollOfSpringDamper);

  transInLCA.setOrigin(btVector3(0, 0, 0));
  transInChassis.setOrigin(btVector3(0, 0, 0));

  transInLCA.setRotation(springDamperRot);
  transInChassis.setRotation(springDamperRot);

  VehicleUtils::printbtVector3(transInLCA.getOrigin(),
                               "spring transInLCA.getOrigin()");
  VehicleUtils::printbtVector3(transInChassis.getOrigin(),
                               "spring transInChassis.getOrigin()");

  auto springDamper = new btGeneric6DofSpring2Constraint(
      *sp_LCA_Body.get(), *sp_chassisBody.get(), transInLCA, transInChassis);
  m_SpringDamper_LCA_Body.m_constraints["Spring_Damper"] =
      btTypedConstraintPtr(springDamper);

  /*for (auto i = 0; i <= 2; ++i) {
    springDamper->enableSpring(i, true);
    springDamper->setStiffness(i, stiffness);
    springDamper->setDamping(i, damping);
  }*/
  for (auto i = 0; i <= 5; ++i) {
    springDamper->enableSpring(i, false);
  }

  springDamper->enableSpring(2, true);
  springDamper->setStiffness(2, stiffness);
  springDamper->setDamping(2, damping);
  springDamper->enableSpring(3, true);
  springDamper->setStiffness(3, 5e4);
  springDamper->setDamping(3, 1e2);

  springDamper->setLinearLowerLimit(btVector3(0, 0, -1));
  springDamper->setLinearUpperLimit(btVector3(0, 0, 1));
  springDamper->setAngularLowerLimit(btVector3(-10 * RAD_2_DEG, 0, 0));
  springDamper->setAngularUpperLimit(btVector3(10 * RAD_2_DEG, 0, 0));
  /*springDamper->setAngularLowerLimit(btVector3(0, 0, 0));
  springDamper->setAngularUpperLimit(btVector3(0, 0, 0));*/

  /*springDamper->setEquilibriumPoint(2, freeLength);*/
  springDamper->setEquilibriumPoint();

  for (auto i = 0; i <= 5; ++i) {
    springDamper->setParam(BT_CONSTRAINT_STOP_ERP, const_ERP, i);
    springDamper->setParam(BT_CONSTRAINT_ERP, const_ERP, i);
    springDamper->setParam(BT_CONSTRAINT_STOP_CFM, const_CFM, i);
    springDamper->setParam(BT_CONSTRAINT_CFM, const_CFM, i);
  }

  world->addConstraint(springDamper);

#if defined(SIM_DEBUG_DRAW_CONSTRAINT) && \
    defined(SIM_DEBUG_DRAW_SPRINGDAMPER_CONSTRAINT)
  springDamper->setDbgDrawSize(0.5);
  world->debugDrawConstraint(springDamper);
#endif
}

void SimDoubleWishBone::addSpringDamper2(btMultiBodyDynamicsWorld *world) {
  // get chassis and UCA rigid body
  auto chassisBody = m_SpringDamper_Chassis_Body.m_body;
  auto LCA_Body = m_SpringDamper_LCA_Body.m_body;

  // get spring and damper
  btScalar stiffness = m_suspData.spring().stiffness().val();
  btScalar damping = m_suspData.damper().damping().val();
  btScalar freeLength = m_suspData.spring().freelength().val();

  m_springDamper = std::make_shared<SimDistanceSpringConstraint>(
      LCA_Body.get(), chassisBody.get(), btVector3(0, 0, 0), btVector3(0, 0, 0),
      0.001);

  m_springDamper->setFreeLength(freeLength);
  m_springDamper->setStiffness(stiffness);
  m_springDamper->setDamping(damping);
}

void SimDoubleWishBone::addUCA_AndChassisBody_HingeConstraint(
    btMultiBodyDynamicsWorld *world) {
  // get axle and chassis simbody
  auto axleSimBody =
      m_parent->getSimBody();  // SimAxle has no rigid body for now !!!
  auto chassisSimBody = m_parent->getParent()->getSimBody();

  // get chassis and UCA rigid body
  auto chassisBody = chassisSimBody.m_body;
  auto UCA_Body = m_UCA_Body.m_body;

  // get local position
  const auto &UCA_LocalPos =
      m_UCA_Body.m_thisFrameInParent.getThisPositionInParent();
  const auto &axleLocalPos =
      axleSimBody.m_thisFrameInParent.getThisPositionInParent();
  const auto &chassisBodyLocalPos =
      chassisSimBody.m_thisFrameInParent.getThisPositionInParent();

  // hinge local position in Axle
  m_localPos_UCA_Front = VehicleUtils::paramVec3TobtVector3(
      m_suspData.uppercontrolarm().offsetinchassisfront());
  m_localPos_UCA_Rear = VehicleUtils::paramVec3TobtVector3(
      m_suspData.uppercontrolarm().offsetinchassisback());

  // hinge info in UCA
  btVector3 hingeAxis{1, 0, 0};
  btVector3 pivotInUCA_front = m_localPos_UCA_Front - UCA_LocalPos;
  btVector3 pivotInUCA_rear = m_localPos_UCA_Rear - UCA_LocalPos;
  pivotInUCA_front = quatRotate(
      m_UCA_Body.m_thisFrameInParent.getThisRotationInParent().inverse(),
      pivotInUCA_front);
  pivotInUCA_rear = quatRotate(
      m_UCA_Body.m_thisFrameInParent.getThisRotationInParent().inverse(),
      pivotInUCA_rear);

  VehicleUtils::printbtVector3(pivotInUCA_front,
                               "UCA and Body, pivotInUCA_front");
  VehicleUtils::printbtVector3(pivotInUCA_rear,
                               "UCA and Body, pivotInUCA_rear");
  VehicleUtils::printbtVector3(pivotInUCA_front - pivotInUCA_rear,
                               "UCA and Body, pivotInUCA_rear_to_front");

  // hinge local position in chassis body
  m_localPos_UCA_Front =
      m_localPos_UCA_Front +
      axleSimBody.m_thisFrameInParent.getThisPositionInParent();
  m_localPos_UCA_Rear =
      m_localPos_UCA_Rear +
      axleSimBody.m_thisFrameInParent.getThisPositionInParent();

  // hinge info
  btVector3 pivotInChassisBody_front = m_localPos_UCA_Front;
  btVector3 pivotInChassisBody_rear = m_localPos_UCA_Rear;

  VehicleUtils::printbtVector3(pivotInChassisBody_front,
                               "UCA and Body, pivotInChassisBody_front");
  VehicleUtils::printbtVector3(pivotInChassisBody_rear,
                               "UCA and Body, pivotInChassisBody_rear");
  VehicleUtils::printbtVector3(
      pivotInChassisBody_front - pivotInChassisBody_rear,
      "UCA and Body, pivotInChassisBody_rear_to_front");

  auto hingeFront = new btHingeConstraint(
      *chassisBody.get(), *UCA_Body.get(), pivotInChassisBody_front,
      pivotInUCA_front, hingeAxis, hingeAxis, true);
  auto hingeRear = new btHingeConstraint(
      *chassisBody.get(), *UCA_Body.get(), pivotInChassisBody_rear,
      pivotInUCA_rear, hingeAxis, hingeAxis, true);

  m_UCA_Body.m_constraints["UCA_ChassisBody_Front"] =
      btTypedConstraintPtr(hingeFront);
  m_UCA_Body.m_constraints["UCA_ChassisBody_Rear"] =
      btTypedConstraintPtr(hingeRear);

  // set hinge angle limit
  hingeFront->setLimit(-const_ControlArm_ChassisBody_Angle_Limit,
                       const_ControlArm_ChassisBody_Angle_Limit);
  hingeRear->setLimit(-const_ControlArm_ChassisBody_Angle_Limit,
                      const_ControlArm_ChassisBody_Angle_Limit);
  /*if (m_side == Side::SIDE_LEFT) {
    hingeFront->setLimit(-30 * DEG_2_RAD, 0 * DEG_2_RAD);
    hingeRear->setLimit(-30 * DEG_2_RAD, 0 * DEG_2_RAD);
  } else {
    hingeFront->setLimit(0 * DEG_2_RAD, 30 * DEG_2_RAD);
    hingeRear->setLimit(0 * DEG_2_RAD, 30 * DEG_2_RAD);
  }*/
  for (auto i = 0; i <= 5; ++i) {
    hingeFront->setParam(BT_CONSTRAINT_STOP_ERP, const_ERP, i);
    hingeFront->setParam(BT_CONSTRAINT_ERP, const_ERP, i);
    hingeFront->setParam(BT_CONSTRAINT_STOP_CFM, const_CFM, i);
    hingeFront->setParam(BT_CONSTRAINT_CFM, const_CFM, i);

    hingeRear->setParam(BT_CONSTRAINT_STOP_ERP, const_ERP, i);
    hingeRear->setParam(BT_CONSTRAINT_ERP, const_ERP, i);
    hingeRear->setParam(BT_CONSTRAINT_STOP_CFM, const_CFM, i);
    hingeRear->setParam(BT_CONSTRAINT_CFM, const_CFM, i);
  }
  /*hingeFront->setAngularOnly(true);
  hingeRear->setAngularOnly(true);*/

  m_UCA_ChassisBody_Front_Hinge = hingeFront;
  m_UCA_ChassisBody_Rear_Hinge = hingeRear;

  world->addConstraint(hingeFront);
  world->addConstraint(hingeRear);

#if defined(SIM_DEBUG_DRAW_CONSTRAINT) && \
    defined(SIM_DEBUG_DRAW_UCA_HINGE_CONSTRAINT)
  hingeFront->setDbgDrawSize(0.5);
  hingeRear->setDbgDrawSize(0.5);
  world->debugDrawConstraint(hingeFront);
  world->debugDrawConstraint(hingeRear);
#endif  // SIM_DEBUG_DRAW_CONSTRAINT
}

void SimDoubleWishBone::addLCA_AndChassisBody_HingeConstraint(
    btMultiBodyDynamicsWorld *world) {
  // get axle and chassis simbody
  auto axleSimBody =
      m_parent->getSimBody();  // SimAxle has no rigid body for now !!!
  auto chassisSimBody = m_parent->getParent()->getSimBody();

  // get chassis and UCA rigid body
  auto chassisBody = chassisSimBody.m_body;
  auto LCA_Body = m_LCA_Body.m_body;

  // get local position
  const auto &LCA_LocalPos =
      m_LCA_Body.m_thisFrameInParent.getThisPositionInParent();
  const auto &axleLocalPos =
      axleSimBody.m_thisFrameInParent.getThisPositionInParent();
  const auto &chassisBodyLocalPos =
      chassisSimBody.m_thisFrameInParent.getThisPositionInParent();

  // hinge local position in Axle
  m_localPos_LCA_Front = VehicleUtils::paramVec3TobtVector3(
      m_suspData.lowercontrolarm().offsetinchassisfront());
  m_localPos_LCA_Rear = VehicleUtils::paramVec3TobtVector3(
      m_suspData.lowercontrolarm().offsetinchassisback());

  // hinge info
  btVector3 hingeAxis{1, 0, 0};
  btVector3 pivotInLCA_front = m_localPos_LCA_Front - LCA_LocalPos;
  btVector3 pivotInLCA_rear = m_localPos_LCA_Rear - LCA_LocalPos;

  pivotInLCA_front = quatRotate(
      m_LCA_Body.m_thisFrameInParent.getThisRotationInParent().inverse(),
      pivotInLCA_front);
  pivotInLCA_rear = quatRotate(
      m_LCA_Body.m_thisFrameInParent.getThisRotationInParent().inverse(),
      pivotInLCA_rear);

  VehicleUtils::printbtVector3(pivotInLCA_front,
                               "LCA and Body, pivotInLCA_front");
  VehicleUtils::printbtVector3(pivotInLCA_rear,
                               "LCA and Body, pivotInLCA_rear");
  VehicleUtils::printbtVector3(pivotInLCA_front - pivotInLCA_rear,
                               "LCA and Body, pivotInLCA_rear_to_front");

  // hinge local position in chassis body
  m_localPos_LCA_Front =
      m_localPos_LCA_Front +
      axleSimBody.m_thisFrameInParent.getThisPositionInParent();
  m_localPos_LCA_Rear =
      m_localPos_LCA_Rear +
      axleSimBody.m_thisFrameInParent.getThisPositionInParent();

  // hinge info
  btVector3 pivotInChassisBody_front = m_localPos_LCA_Front;
  btVector3 pivotInChassisBody_rear = m_localPos_LCA_Rear;

  VehicleUtils::printbtVector3(pivotInChassisBody_front,
                               "LCA and Body, pivotInChassisBody_front");
  VehicleUtils::printbtVector3(pivotInChassisBody_rear,
                               "LCA and Body, pivotInChassisBody_rear");
  VehicleUtils::printbtVector3(
      pivotInChassisBody_front - pivotInChassisBody_rear,
      "LCA and Body, pivotInChassisBody_rear_to_front");

  auto hingeFront = new btHingeConstraint(
      *chassisBody.get(), *LCA_Body.get(), pivotInChassisBody_front,
      pivotInLCA_front, hingeAxis, hingeAxis);
  auto hingeRear = new btHingeConstraint(*chassisBody.get(), *LCA_Body.get(),
                                         pivotInChassisBody_rear,
                                         pivotInLCA_rear, hingeAxis, hingeAxis);

  /*hingeFront->setAngularOnly(true);
  hingeRear->setAngularOnly(true);*/

  m_LCA_Body.m_constraints["LCA_ChassisBody_Front"] =
      btTypedConstraintPtr(hingeFront);
  m_LCA_Body.m_constraints["LCA_ChassisBody_Rear"] =
      btTypedConstraintPtr(hingeRear);

  // set hinge angle limit
  hingeFront->setLimit(-const_ControlArm_ChassisBody_Angle_Limit,
                       const_ControlArm_ChassisBody_Angle_Limit);
  hingeRear->setLimit(-const_ControlArm_ChassisBody_Angle_Limit,
                      const_ControlArm_ChassisBody_Angle_Limit);

  /*if (m_side == Side::SIDE_LEFT) {
    hingeFront->setLimit(-50 * DEG_2_RAD, -25 * DEG_2_RAD);
    hingeRear->setLimit(-50 * DEG_2_RAD, -25 * DEG_2_RAD);
  } else {
    hingeFront->setLimit(25 * DEG_2_RAD, 50 * DEG_2_RAD);
    hingeRear->setLimit(25 * DEG_2_RAD, 50 * DEG_2_RAD);
  }*/

  // second parameter is softness
  for (auto i = 0; i <= 5; ++i) {
    hingeFront->setParam(BT_CONSTRAINT_STOP_ERP, const_ERP, i);
    hingeFront->setParam(BT_CONSTRAINT_ERP, const_ERP, i);
    hingeFront->setParam(BT_CONSTRAINT_STOP_CFM, const_CFM, i);
    hingeFront->setParam(BT_CONSTRAINT_CFM, const_CFM, i);

    hingeRear->setParam(BT_CONSTRAINT_STOP_ERP, const_ERP, i);
    hingeRear->setParam(BT_CONSTRAINT_ERP, const_ERP, i);
    hingeRear->setParam(BT_CONSTRAINT_STOP_CFM, const_CFM, i);
    hingeRear->setParam(BT_CONSTRAINT_CFM, const_CFM, i);
  }

  m_LCA_ChassisBody_Front_Hinge = hingeFront;
  m_LCA_ChassisBody_Rear_Hinge = hingeRear;

  world->addConstraint(hingeFront);
  world->addConstraint(hingeRear);

#if defined(SIM_DEBUG_DRAW_CONSTRAINT) && \
    defined(SIM_DEBUG_DRAW_LCA_HINGE_CONSTRAINT)
  hingeFront->setDbgDrawSize(0.5);
  hingeRear->setDbgDrawSize(0.5);
  world->debugDrawConstraint(hingeFront);
  world->debugDrawConstraint(hingeRear);
#endif
}

void SimDoubleWishBone::addUprightAndUCA_Constraint(
    btMultiBodyDynamicsWorld *world) {
  // get UCA data
  const auto &UCA = m_suspData.uppercontrolarm();

  auto uprightBody = m_uprightBody.m_body;
  auto UCA_Body = m_UCA_Body.m_body;

  const auto &UCA_ConnectUpright_LocalPos =
      VehicleUtils::paramVec3TobtVector3(UCA.offsetinupright());
  const auto &uprightLocalPos =
      m_uprightBody.m_thisFrameInParent.getThisPositionInParent();
  const auto &UCA_LocalPos =
      m_UCA_Body.m_thisFrameInParent.getThisPositionInParent();

  btVector3 pivotInUpright = UCA_ConnectUpright_LocalPos - uprightLocalPos;
  btVector3 pivotInUCA = UCA_ConnectUpright_LocalPos - UCA_LocalPos;
  pivotInUCA = quatRotate(
      m_UCA_Body.m_thisFrameInParent.getThisRotationInParent().inverse(),
      pivotInUCA);
  btVector3 hingeAxle{1, 0, 0};

  VehicleUtils::printbtVector3(pivotInUpright,
                               "Upright and UCA, pivotInUpright");
  VehicleUtils::printbtVector3(pivotInUCA, "Upright and UCA, pivotInUCA");

  // create spherical constraint for front suspension and hinge for rear
  // suspension
  /*if (m_axleNum == AxleNum::AXLE_0) {
    auto spherical = new btPoint2PointConstraint(
        *uprightBody.get(), *UCA_Body.get(), pivotInUpright, pivotInUCA);

    m_UCA_Body.m_constraints["Upright_UCA"] = btTypedConstraintPtr(spherical);

    world->addConstraint(spherical);
    LOG_INFO << "front axle, Spindle and UCA use btPoint2PointConstraint.\n";
  } else if (m_axleNum == AxleNum::AXLE_1)*/
  {
    auto hinge =
        new btHingeConstraint(*uprightBody.get(), *UCA_Body.get(),
                              pivotInUpright, pivotInUCA, hingeAxle, hingeAxle);
    m_Upright_UCA_Hinge = hinge;

    // set hinge angle limit
    hinge->setLimit(-const_Upright_ControlArm_Angle_Limit,
                    const_Upright_ControlArm_Angle_Limit);
    // hinge->setMaxMotorImpulse(maxImpulse);
    /*if (m_side == Side::SIDE_LEFT) {
      hinge->setLimit(10 * DEG_2_RAD, 35 * DEG_2_RAD);
    } else {
      hinge->setLimit(-35 * DEG_2_RAD, -10 * DEG_2_RAD);
    }*/

    for (auto i = 0; i <= 5; ++i) {
      hinge->setParam(BT_CONSTRAINT_STOP_ERP, const_ERP, i);
      hinge->setParam(BT_CONSTRAINT_ERP, const_ERP, i);
      hinge->setParam(BT_CONSTRAINT_STOP_CFM, const_CFM, i);
      hinge->setParam(BT_CONSTRAINT_CFM, const_CFM, i);
    }

    m_UCA_Body.m_constraints["Upright_UCA"] = btTypedConstraintPtr(hinge);

    world->addConstraint(hinge);
    LOG_INFO << "Spindle and UCA use btHingeConstraint.\n";
  }
}

void SimDoubleWishBone::addUprightAndLCA_Constraint(
    btMultiBodyDynamicsWorld *world) {
  // get LCA data
  const auto &LCA = m_suspData.lowercontrolarm();

  auto uprightBody = m_uprightBody.m_body;
  auto LCA_Body = m_LCA_Body.m_body;

  const auto &LCA_ConnectUpright_LocalPos =
      VehicleUtils::paramVec3TobtVector3(LCA.offsetinupright());
  const auto &uprightLocalPos =
      m_uprightBody.m_thisFrameInParent.getThisPositionInParent();
  const auto &LCA_LocalPos =
      m_LCA_Body.m_thisFrameInParent.getThisPositionInParent();

  btVector3 pivotInUpright = LCA_ConnectUpright_LocalPos - uprightLocalPos;
  btVector3 pivotInLCA = LCA_ConnectUpright_LocalPos - LCA_LocalPos;
  pivotInLCA = quatRotate(
      m_LCA_Body.m_thisFrameInParent.getThisRotationInParent().inverse(),
      pivotInLCA);
  btVector3 hingeAxle{1, 0, 0};

  VehicleUtils::printbtVector3(pivotInUpright,
                               "Upright and LCA, pivotInUpright");
  VehicleUtils::printbtVector3(pivotInLCA, "Upright and LCA, pivotInLCA");
  // create spherical constraint for front suspension and hinge for rear
  // suspension
  /*if (m_axleNum == AxleNum::AXLE_0) {
    auto spherical = new btPoint2PointConstraint(
        *uprightBody.get(), *LCA_Body.get(), pivotInUpright, pivotInLCA);

    m_LCA_Body.m_constraints["Upright_LCA"] = btTypedConstraintPtr(spherical);

    world->addConstraint(spherical);
    LOG_INFO << "front axle, Spindle and UCA use btPoint2PointConstraint.\n";
  } else if (m_axleNum == AxleNum::AXLE_1)*/
  {
    auto hinge =
        new btHingeConstraint(*uprightBody.get(), *LCA_Body.get(),
                              pivotInUpright, pivotInLCA, hingeAxle, hingeAxle);
    // set hinge angle limit
    hinge->setLimit(-const_Upright_ControlArm_Angle_Limit,
                    const_Upright_ControlArm_Angle_Limit);
    /*if (m_side == Side::SIDE_LEFT) {
      hinge->setLimit(10 * DEG_2_RAD, 35 * DEG_2_RAD);
    } else {
      hinge->setLimit(-35 * DEG_2_RAD, -10 * DEG_2_RAD);
    }*/

    for (auto i = 0; i <= 5; ++i) {
      hinge->setParam(BT_CONSTRAINT_STOP_ERP, const_ERP, i);
      hinge->setParam(BT_CONSTRAINT_ERP, const_ERP, i);
      hinge->setParam(BT_CONSTRAINT_STOP_CFM, const_CFM, i);
      hinge->setParam(BT_CONSTRAINT_CFM, const_CFM, i);
    }
    /*hinge->setAngularOnly(true);*/

    m_Upright_LCA_Hinge = hinge;

    m_LCA_Body.m_constraints["Upright_LCA"] = btTypedConstraintPtr(hinge);

    world->addConstraint(hinge);
    LOG_INFO << "Spindle and UCA use btHingeConstraint.\n";
  }
}

void SimDoubleWishBone::addSpindleAndUpright_HingeConstraint(
    btMultiBodyDynamicsWorld *world) {
  auto spindleBody = m_spindleBody.m_body;
  auto uprightBody = m_uprightBody.m_body;

  const auto &spindleLocalPos =
      m_spindleBody.m_thisFrameInParent.getThisPositionInParent();
  const auto &uprightLocalPos =
      m_uprightBody.m_thisFrameInParent.getThisPositionInParent();

  // middle poisition of upright and spindle
  btVector3 midPos = (spindleLocalPos + uprightLocalPos) * 0.5;

  btVector3 pivotInSpindle = midPos - spindleLocalPos;
  btVector3 pivotInUpright = midPos - uprightLocalPos;
  btVector3 hingleAxle{0, 1, 0};  // spindle and upright rotate around y-axis

  VehicleUtils::printbtVector3(pivotInSpindle,
                               "Spindle and Upright, pivotInSpindle");
  VehicleUtils::printbtVector3(pivotInUpright,
                               "Spindle and Upright, pivotInUpright");

  auto hinge = new btHingeConstraint(*spindleBody.get(), *uprightBody.get(),
                                     pivotInSpindle, pivotInUpright, hingleAxle,
                                     hingleAxle);
  for (auto i = 0; i <= 5; ++i) {
    /*if (i != 1)*/ {
      hinge->setParam(BT_CONSTRAINT_STOP_ERP, const_ERP, i);
      hinge->setParam(BT_CONSTRAINT_ERP, const_ERP, i);
      hinge->setParam(BT_CONSTRAINT_STOP_CFM, const_CFM, i);
      hinge->setParam(BT_CONSTRAINT_CFM, const_CFM, i);
    }
  }

  m_spindleBody.m_constraints["Spindle_Upright"] = btTypedConstraintPtr(hinge);

  world->addConstraint(hinge);
}

void SimDoubleWishBone::initialize(VehicleInitHelper &helper) {
  // axle init transform in global
  const auto &parentInGlobal = m_parent->getSimBody().m_thisFrameInGlobal;

  auto transfromPartToGlobal = [](const SimFrame &parentInGlobal,
                                  SimRigidBody &simbody) {
    // set chassis body init location
    btTransform worldTransform = SimFrame::transfromLocalIntoSuper(
        parentInGlobal.getThisTransformInParent(),
        simbody.m_thisFrameInParent.getThisTransformInParent());

    // update rigid body initial transform
    simbody.m_body->setWorldTransform(worldTransform);

    // update body frame in global
    simbody.m_thisFrameInGlobal.setThisTransformInParent(
        worldTransform.getOrigin(), worldTransform.getRotation());
  };

  // initialize spindle
  transfromPartToGlobal(parentInGlobal, m_spindleBody);

  // initialize upright
  transfromPartToGlobal(parentInGlobal, m_uprightBody);

  // initialize UCA
  transfromPartToGlobal(parentInGlobal, m_UCA_Body);

  // initialize LCA
  transfromPartToGlobal(parentInGlobal, m_LCA_Body);

  // initialize spring-damper connector
  transfromPartToGlobal(parentInGlobal, m_SpringDamper_LCA_Body);
  transfromPartToGlobal(parentInGlobal, m_SpringDamper_Chassis_Body);

  // initialize wheel
  m_wheel.initialize(helper);
}

void SimDoubleWishBone::stepSimulation(VehicleStepHelper &helper) {
  if (m_springDamper.get() != nullptr) m_springDamper->update();
  LOG_INFO << SIM_SEPERATOR;
  LOG_INFO << "Axle:" << (int)m_axleNum << ", Side:" << (int)m_side << "\n";
  auto hingeLCA_SP = reinterpret_cast<btHingeConstraint *>(
      m_LCA_Body.m_constraints["LCA_SpringDamper"].get());
  LOG_INFO << "hingeLCA_SP angle[deg]:"
           << hingeLCA_SP->getHingeAngle() * RAD_2_DEG;

  auto hingeUCA_Chassis_Front = reinterpret_cast<btHingeConstraint *>(
      m_UCA_Body.m_constraints["UCA_ChassisBody_Front"].get());
  LOG_INFO << "hingeUCA_Chassis_Front angle[deg]:"
           << hingeUCA_Chassis_Front->getHingeAngle() * RAD_2_DEG;
  auto hingeUCA_Upright = reinterpret_cast<btHingeConstraint *>(
      m_UCA_Body.m_constraints["Upright_UCA"].get());
  LOG_INFO << "hingeUCA_Upright angle[deg]:"
           << hingeUCA_Upright->getHingeAngle() * RAD_2_DEG;

  ///*if (m_axleNum == AxleNum::AXLE_1)*/ {
  //  btHingeConstraint *upright_LCA_Hinge =
  //      reinterpret_cast<btHingeConstraint *>(
  //          m_LCA_Body.m_constraints.find("Upright_LCA")->second.get());
  //  LOG_INFO << "Upright and LCA hinge angle[deg]:"
  //           << upright_LCA_Hinge->getHingeAngle() * RAD_2_DEG << ".\n";
  //}
}
}  // namespace tx_car