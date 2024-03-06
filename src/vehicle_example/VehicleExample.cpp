#include "VehicleExample.h"
#include "proto_helper.h"

CommonExampleInterface *
StandaloneExampleCreateFunc(CommonExampleOptions &options) {
  return new tx_car::VehicleExample(options.m_guiHelper);
}

namespace tx_car {
VehicleExample::VehicleExample(GUIHelperInterface *helper)
    : CommonMultiBodyBase(helper) {
  m_GroundBoxHalfLength = 5; // 10 by 10
  m_GroundHalfHeight = 0.05; // 0.1 height

  m_ballRadius = 0.2;
  m_ballMass = 3.0;

  m_roofInitZ = 6.0;
  m_roofBoxHalfLength = 0.5;
  m_roofBoxHalfHeight = 0.05;

  m_BodyInitZ = (m_roofInitZ - m_ballRadius * 2.0 * m_ballGapCoeff);
}

VehicleExample::~VehicleExample() {}

void VehicleExample::setGUI_UpAxis_Z() { m_guiHelper->setUpAxis(2); }
void VehicleExample::setGravity(const btVector3 &gravity) {
  m_dynamicsWorld->setGravity(gravity);
}
void VehicleExample::resetCamera() {
  btScalar scale(0.5);
  m_guiHelper->resetCamera(2, -45, -45, -m_GroundBoxHalfLength * scale,
                           -m_GroundBoxHalfLength * scale, 3.0);
}

btRigidBodyPtr VehicleExample::createGround() {
  return createStaticPlane(btVector3(m_GroundBoxHalfLength,
                                     m_GroundBoxHalfLength, m_GroundHalfHeight),
                           btQuaternion(0, 0, 0, 1),
                           btVector3(0, 0, -m_GroundHalfHeight));
}

btRigidBodyPtr
VehicleExample::createStaticPlane(const btVector3 &halfExtendSize,
                                  const btQuaternion &rot,
                                  const btVector3 &offset) {
  btCollisionShapePtr planeCollisionShape(new btBoxShape(halfExtendSize));
  auto staticPlane =
      createRigidBody(0, btTransform(rot, offset), planeCollisionShape.get());

  m_collisionShapes.push_back(planeCollisionShape);
  auto plane = btRigidBodyPtr(staticPlane);
  plane->setFriction(0.9);
  m_bodies.push_back(plane);

  return plane;
}

// init vehicle data from file
void VehicleExample::getVehicleParam(const std::string& jsonFilePath) {
  std::string jsonContent;

  if (!tx_car::loadFromFile(jsonContent, jsonFilePath)) {
    LOG_ERROR << "fail to load json from file, " << jsonFilePath << "\n";
    throw std::runtime_error("fail to load json from file");
    return;
  }

  try{
    tx_car::jsonToProto(jsonContent, m_vehParam);
  }
  catch (const std::exception &e) {
    LOG_ERROR << "tx_car::jsonToProto(jsonContent, m_vehParam) error, " << e.what() << "\n";
  }
  LOG_0 << "m_vehParam:" << m_vehParam.DebugString() << "\n";
}

void VehicleExample::createSimpleVehicle(
    const btVector3 &chassisBodyInitPos,
    const btQuaternion &chassisBodyInitRot) {
  // get roof position
  auto roofPosition = m_roof->getWorldTransform().getOrigin();
  auto ballPosition = roofPosition;
  ballPosition.setZ(3);
  ballPosition.setY(3);
  ballPosition.setX(0);
  LOG(INFO) << "roofPosition:" << roofPosition.x() << ", " << roofPosition.y()
            << ", " << roofPosition.z() << "\n";
  LOG(INFO) << "ballPosition:" << ballPosition.x() << ", " << ballPosition.y()
            << ", " << ballPosition.z() << "\n";

  // create rigid body
  auto cylinderShape = btCollisionShapePtr(
      new btCylinderShape(btVector3(m_ballRadius, 0.1, 0.1)));
  m_collisionShapes.push_back(cylinderShape);

  auto cylinder = createRigidBody(
      m_ballMass,
      btTransform(btQuaternion(btVector3(0, 1, 0), 0), ballPosition),
      cylinderShape.get());
  auto cylinderBody = btRigidBodyPtr(cylinder);
  m_bodies.push_back(cylinderBody);

  // create spring-damper
  btTransform transInA, transInB;

  transInA.setOrigin(btVector3(0, 0, 0));
  transInB.setOrigin(btVector3(0, -0.15, 0));

  btVector3 jointLocalInB(0, transInB.getOrigin().y(), 0);
  btVector3 jointBInGlobal = ballPosition + jointLocalInB;
  btVector3 dirAxle = roofPosition - jointBInGlobal;
  btScalar rollOfSpringDamper = 0.0;
  rollOfSpringDamper = -std::atan2(dirAxle[1], dirAxle[2]);
  LOG_INFO << "rollOfSpringDamper[deg]:" << (rollOfSpringDamper)*RAD_2_DEG
           << "\n";
  transInA.setRotation(btQuaternion(btVector3(1, 0, 0), rollOfSpringDamper));
  transInB.setRotation(btQuaternion(btVector3(1, 0, 0), rollOfSpringDamper));

  // auto* spring = new btGeneric6DofSpringConstraint(*m_roof.get(),
  // *cylinderBody.get(), transInA, transInB, true);
  auto *spring = new btGeneric6DofSpring2Constraint(
      *m_roof.get(), *cylinderBody.get(), transInA, transInB);
  auto springPtr = btTypedConstraintPtr(spring);
  m_constraintMap["SP_1"] = springPtr;

  m_dynamicsWorld->addConstraint(spring);
  spring->setDbgDrawSize(btScalar(1));

  int springAxis = 2;

  // disable other DoF
  for (auto i = 0; i < 6; ++i) {
    // if (i != springAxis) {
    /*if (i <= 2) {
      spring->enableSpring(i, true);
    } else*/
    {
      spring->enableSpring(i, false);
      spring->setStiffness(i, 1e9);
      spring->setDamping(i, 1e2);
    }
  }

  spring->enableSpring(springAxis, true);
  spring->enableSpring(3, false);

  spring->setLinearLowerLimit(btVector3(-0, -0, -5));
  spring->setLinearUpperLimit(btVector3(0, 0, 5));
  spring->setAngularLowerLimit(btVector3(-2 * DEG_2_RAD, -0, -0));
  spring->setAngularUpperLimit(btVector3(2 * DEG_2_RAD, 0, 0));

  spring->setStiffness(springAxis, 100.0);
  spring->setDamping(springAxis, 1);
  spring->setStiffness(3, 100);
  spring->setDamping(3, 1e1);

  /*spring->setStiffness(springAxis, 100.0);
  spring->setDamping(springAxis, 10);*/
  spring->setEquilibriumPoint(); // use this, do not set value of
                                 // EquilibriumPoint manually.
}

// create vehicle model
void VehicleExample::createVehicleDWDW(tx_car::MBD_Vehicle_DW_DW &dwdwParam) {
  // construct helper
  {
    m_cnstHelper.setMultiBodyDynamicsWOrld(m_dynamicsWorld);
    m_initHelper.setMultiBodyDynamicsWOrld(m_dynamicsWorld);
    m_stepHelper.setMultiBodyDynamicsWOrld(m_dynamicsWorld);
    m_stopHelper.setMultiBodyDynamicsWOrld(m_dynamicsWorld);

    m_cnstHelper.setVehicleDoubleWishboneParam(dwdwParam);
    m_initHelper.setVehicleDoubleWishboneParam(dwdwParam);
    m_stepHelper.setVehicleDoubleWishboneParam(dwdwParam);
    m_stopHelper.setVehicleDoubleWishboneParam(dwdwParam);
  }

  // init helper
  {
    m_initHelper.setInitPosition(btVector3(0, 0, 1.0));
    m_initHelper.setInitRotation(btQuaternion(btVector3(0, 0, 1), 0.25 * M_PI));
  }

  //  construct
  { m_vehicleDWDW.construct(m_cnstHelper); }

  // init
  { m_vehicleDWDW.initialize(m_initHelper); }
}

// hingleconstraint test
void VehicleExample::hingeTest() {
  // get roof info
  auto rootTransformInWorld = m_roof->getWorldTransform();
  btVector3 rootHalfBBX(m_roofBoxHalfLength, m_roofBoxHalfLength,
                        m_roofBoxHalfHeight);

  // Create rigid bodies
  btVector3 boxHalfBBX(0.2, 0.2, 0.2);
  btCollisionShape *shapeA = new btBoxShape(boxHalfBBX);
  m_collisionShapes.push_back(btCollisionShapePtr(shapeA));

  btTransform boxTransform = rootTransformInWorld;
  boxTransform.setIdentity();
  btVector3 boxPosition = rootTransformInWorld.getOrigin();
  boxPosition[0] = boxPosition[0] - rootHalfBBX[0] - boxHalfBBX[0] - 1.0;
  boxTransform.setOrigin(boxPosition);

  auto boxBody = createRigidBody(3.0, boxTransform, shapeA);
  m_bodies.push_back(btRigidBodyPtr(boxBody));

  m_dynamicsWorld->addRigidBody(boxBody);

  // add constraint
  btVector3 hingleAxis(1, 0, 0);
  auto hinge = new btHingeConstraint(
      *m_roof.get(), *boxBody, btVector3(-rootHalfBBX[0] - 0.5, 0, 0),
      btVector3(boxHalfBBX[0] + 0.5, 0, 0), hingleAxis, hingleAxis);

  m_dynamicsWorld->addConstraint(hinge);
}

void VehicleExample::initPhysics() {
  LOG_INFO << "VehicleExample::initPhysics begin.\n";

  // create world
  createEmptyDynamicsWorld();

  LOG_INFO << "VehicleExample::initPhysics createEmptyDynamicsWorld.\n";

  // set gravity and up axis
  setGUI_UpAxis_Z();
  LOG_INFO << "VehicleExample::initPhysics setGUI_UpAxis_Z.\n";
  setGravity();
  LOG_INFO << "VehicleExample::initPhysics setGravity.\n";

  // prepare drawer
  m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
  LOG_INFO << "VehicleExample::initPhysics createPhysicsDebugDrawer.\n";

  if (m_dynamicsWorld->getDebugDrawer())
    m_dynamicsWorld->getDebugDrawer()->setDebugMode(
        btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawConstraints);

  // create ground
  m_ground = createGround();
  m_bodyMap["ground"] = m_ground;
  LOG_INFO << "VehicleExample::initPhysics createGround.\n";

  // create rood
  m_roof = createStaticPlane(
      btVector3(m_roofBoxHalfLength, m_roofBoxHalfLength, m_roofBoxHalfHeight),
      btQuaternion(0, 0, 0, 1), btVector3(0, 0, m_roofInitZ));
  m_bodyMap["roof"] = m_roof;
  LOG_INFO << "VehicleExample::initPhysics m_roof.\n";

  // load param
  LOG_INFO << "m_const_vehParamPath:" << m_const_vehParamPath << "\n";
  getVehicleParam(m_const_vehParamPath);
  LOG_INFO
      << "VehicleExample::initPhysics getVehicleParam(m_const_vehParamPath).\n";

  // create double wishbone, double wishbone vehicle
  createVehicleDWDW(m_vehParam);
  LOG_INFO << "VehicleExample::initPhysics createVehicleDWDW(m_vehParam).\n";

  // create simple vehicle example
  createSimpleVehicle(btVector3(), btQuaternion(0, 0, 0, 1));
  LOG_INFO << "VehicleExample::initPhysics createSimpleVehicle.\n";

  // hinge test
  hingeTest();
  LOG_INFO << "VehicleExample::initPhysics hingeTest.\n";

  // auto add graphics
  m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
  LOG_INFO << "VehicleExample::initPhysics autogenerateGraphicsObjects.\n";

  m_dynamicsWorld->getSolverInfo().m_numIterations = 150;
  m_dynamicsWorld->getSolverInfo().m_splitImpulse = true;

  LOG_INFO << "VehicleExample::initPhysics end.\n";
}

void VehicleExample::renderScene() {
  m_dynamicsWorld->debugDrawWorld();
  CommonMultiBodyBase::renderScene();
}

void VehicleExample::stepSimulation(float deltaTime) {
  LOG_INFO << "VehicleExample::stepSimulation(float deltaTime) begin.\n";

  /*if (m_stepCounter++ < 50)*/ {
    m_dynamicsWorld->stepSimulation(deltaTime);
    m_vehicleDWDW.stepSimulation(m_stepHelper);
    const auto &frontLeftSusp =
        m_vehicleDWDW.getFrontAxle().getLeftSuspension();
    /*LOG_INFO << "getUpright_LCA_Hinge[deg]:"
             << frontLeftSusp.getUpright_LCA_Hinge()->getHingeAngle() *
                    RAD_2_DEG
             << "\n";
    LOG_INFO << "getUpright_UCA_Hinge[deg]:"
             << frontLeftSusp.getUpright_UCA_Hinge()->getHingeAngle() *
                    RAD_2_DEG
             << "\n";*/
  }
  // LOG(INFO) << "ball Vz:" <<
  // m_constraintMap["SP_1"]->getRigidBodyB().getLinearVelocity().z() << "\n";
  // btScalar roll, pitch, yaw;
  // m_constraintMap["SP_1"]->getRigidBodyB().getWorldTransform().getRotation().getEulerZYX(yaw,
  // pitch, roll); LOG(INFO) << "ball roll:" << roll / M_PI * 180.0 << ",
  // pitch:" << pitch / M_PI * 180.0 << ", yaw:" << yaw / M_PI * 180.0 << "\n";
  // std::this_thread::sleep_for(std::chrono::milliseconds(20));
  /*const auto &chassisSimbody = m_vehicleDWDW.getChassisBody();
  LOG(INFO) << "chassis body mass:"
            << chassisSimbody.getSimBody().m_body->getMass() << ".\n";
  VehicleUtils::printbtVector3(
      chassisSimbody.getSimBody().m_body->getLocalInertia(),
      "chassis body inertia");
  VehicleUtils::printbtVector3(
      chassisSimbody.getSimBody().m_body->getWorldTransform().getOrigin(),
      "chassis body origin");*/

  LOG_INFO << "VehicleExample::stepSimulation(float deltaTime) end.\n";
}

void VehicleExample::exitPhysics() {
  LOG_INFO << "VehicleExample::exitPhysics() begin.\n";
  // CommonMultiBodyBase::exitPhysics();
  LOG_INFO << "VehicleExample::exitPhysics() end.\n";
}
} // namespace tx_car