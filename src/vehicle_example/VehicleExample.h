#pragma once

#include "SimVehicle_DWDW.h"

#include <map>

CommonExampleInterface *
StandaloneExampleCreateFunc(CommonExampleOptions &options);

namespace tx_car {

class VehicleExample;
using VehicleExamplePtr = std::shared_ptr<VehicleExample>;

class VehicleExample : public CommonMultiBodyBase {
public:
  VehicleExample(GUIHelperInterface *helper);
  virtual ~VehicleExample();

public:
  virtual void initPhysics() override;
  virtual void stepSimulation(float deltaTime) override;
  virtual void exitPhysics() override;
  virtual void resetCamera() override;
  virtual void renderScene() override;

public:
  btRigidBodyPtr createGround();
  btRigidBodyPtr createStaticPlane(const btVector3 &halfExtendSize,
                                   const btQuaternion &rot,
                                   const btVector3 &offset);

  // hingle2constraint test
  void hingeTest();

  // init vehicle data from file
  void getVehicleParam(const std::string &jsonFilePath);

  // create simple vehicle
  void createSimpleVehicle(const btVector3 &chassisBodyInitPos,
                           const btQuaternion &chassisBodyInitRot);

  // create vehicle model
  void createVehicleDWDW(tx_car::MBD_Vehicle_DW_DW &dwdwParam);

public:
  void setGUI_UpAxis_Z();
  void setGravity(const btVector3 &gravity = {0, 0, -9.81});

private:
  // ground info
  btScalar m_GroundBoxHalfLength, m_GroundHalfHeight;

  // vehicle parameter
  tx_car::MBD_Vehicle_DW_DW m_vehParam;
  const std::string m_const_vehParamPath =
      "E:\\work\\SimReposity\\MultiBodyVehicle\\param\\vehicle_dwdw.json";

  // vehicle model
  SimVehicle_DWDW m_vehicleDWDW;
  VehicleConstructHelper m_cnstHelper;
  VehicleInitHelper m_initHelper;
  VehicleStepHelper m_stepHelper;
  VehicleStopHelper m_stopHelper;

  // ball info
  btScalar m_BodyInitZ, m_ballRadius, m_ballMass;
  const btScalar m_ballGapCoeff = 1.2;

  // wheel info
  btScalar m_wheelRadius = 0.367, m_wheelHalfWidt = 0.225 / 2;

  // roof info
  btScalar m_roofBoxHalfLength, m_roofBoxHalfHeight;
  btScalar m_roofInitZ;

  // multibody
  btMultiBodyPtr m_multiBody;

  // pointers
  std::map<std::string, btRigidBodyPtr> m_bodyMap;
  std::map<std::string, btCollisionShapePtr> m_collisionShapeMap;
  std::map<std::string, btTypedConstraintPtr> m_constraintMap;
  std::vector<btCollisionShapePtr> m_collisionShapes;
  std::vector<btRigidBodyPtr> m_bodies;
  btRigidBodyPtr m_ground, m_roof;

  bool m_firstFrame = true;
  uint64_t m_stepCounter = 0;
};
} // namespace tx_car