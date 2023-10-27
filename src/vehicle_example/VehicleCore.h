
#pragma once

#include <map>

#include "SimConstants.h"
#include "SimLog.h"
#include "VehicleTypes.h"
#include "VehicleUtils.h"

#define SIM_SEPERATOR "*******************************************************************************"

namespace tx_car {

// axle number
enum class AxleNum { AXLE_0, AXLE_1 };

// side
enum class Side { SIDE_LEFT, SIDE_RIGHT };

class SimChassisBody;
class SimWheel;
class SimDoubleWishBone;
class SimAxle;
class SimVehicle_DWDW;

class SimFrame {
  /**
   * @brief aux frame, include position and orentation.
   * this transform is expressed in parent frame.
   */
public:
  SimFrame() {
    m_localTransInParent.setIdentity();
    m_localTransInParent.setOrigin(btVector3(0, 0, 0));
  }
  virtual ~SimFrame() {}

  /**
   * @brief set transform in parent coordinate
   * @param thisPosInParent
   * @param thisRotInParent
   */
  void setThisTransformInParent(const btVector3 &thisPosInParent,
                                const btQuaternion &thisRotInParent) {
    m_localTransInParent.setOrigin(thisPosInParent);
    m_localTransInParent.setRotation(thisRotInParent);
  }

public:
  /**
   * @brief get position in parent frame
   * @return
   */
  btVector3 getThisPositionInParent() const {
    return m_localTransInParent.getOrigin();
  }
  /**
   * @brief get rotation in parent frame
   * @return
   */
  btQuaternion getThisRotationInParent() const {
    return m_localTransInParent.getRotation();
  }
  /**
   * @brief get transform in parent frame
   * @return
   */
  btTransform getThisTransformInParent() const { return m_localTransInParent; }

public:
  /**
   * @brief calculate local transform expressed in supper
   * @param parentTransInSuper, parent frame expressed in it's super coordinate
   * @param localTransInParent, local frame expressed in parent
   * @return
   */
  static btTransform
  transfromLocalIntoSuper(const btTransform &parentTransInSuper,
                          const btTransform &localTransInParent) {
    // get parent in super coordinate
    const btVector3 &parentPosInGlobal = parentTransInSuper.getOrigin();
    const btQuaternion &parentRotInGlobal = parentTransInSuper.getRotation();

    // set chassis body init location
    btTransform worldTransform;
    worldTransform.setRotation(parentRotInGlobal *
                               localTransInParent.getRotation());
    worldTransform.setOrigin(
        parentPosInGlobal +
        quatRotate(parentRotInGlobal, localTransInParent.getOrigin()));

    return worldTransform;
  };

private:
  btTransform m_localTransInParent;
};

class SimFrameMoving : public SimFrame {
  /**
   * @brief aux moving frame, including linear and angular dynamics information.
   * this transform is expressed in parent frame.
   *
   */
public:
  /**
   * @brief get linear velocity in parent frame
   * @return
   */
  btVector3 getPosDt_InParent() { return m_velocityLocal; }
  /**
   * @brief get linear acceleration in parent frame
   * @return
   */
  btVector3 getPosDtDt_InParent() { return m_accLocal; }
  /**
   * @brief get angular velocity in parent frame
   * @return
   */
  btVector3 getOmega_InParent() { return m_omegaLocal; }
  /**
   * @brief get angular acceleration in parent frame
   * @return
   */
  btVector3 getOmegaDt_InParent() { return m_omegaDtGlobal; }
  /**
   * @brief get linear velocity in global frame
   * @return
   */
  btVector3 getPosDt_InGlobal() { return m_velocityGlobal; }
  /**
   * @brief get linear acceleration in global frame
   * @return
   */
  btVector3 getPosDtDt_InGlobal() { return m_accGlobal; }
  /**
   * @brief get angular velocity in global frame
   * @return
   */
  btVector3 getOmega_InGlobal() { return m_omegaGlobal; }
  /**
   * @brief get angular acceleration in global frame
   * @return
   */
  btVector3 getOmegaDt_InGlobal() { return m_omegaDtGlobal; }

public:
  /*to do, calculate linear and angular dynamics information, @dongyuanhu*/
private:
  btVector3 m_velocityLocal, m_velocityGlobal;
  btVector3 m_accLocal, m_accGlobal;
  btVector3 m_omegaLocal, m_omegaGlobal;
  btVector3 m_omegaDtLocal, m_omegaDtGlobal;
};

/**
 * @brief base helper class for vehicle model
 */
class VehicleHelperBase {
public:
  VehicleHelperBase();
  virtual ~VehicleHelperBase();

  /**
   * @brief set double wishbone vehicle parameter
   * @param vehicleParam
   */
  void
  setVehicleDoubleWishboneParam(const tx_car::MBD_Vehicle_DW_DW &vehicleParam);

  /**
   * @brief Set the Multi Body Dynamics W Orld object
   *
   * @param p_world
   */
  void setMultiBodyDynamicsWOrld(btMultiBodyDynamicsWorld *p_world);

  /**
   * @brief Set the Step Time object
   *
   * @param stepTime, unit s
   */
  void setStepTime(btScalar stepTime);

public:
  /**
   * @brief get double wishbone vehicle parameter
   * @return
   */
  tx_car::MBD_Vehicle_DW_DW getVehicle_DWDW_Param();

  /**
   * @brief get btMultiBodyDynamicsWorld pointer
   * @return
   */
  btMultiBodyDynamicsWorld *getMultiBodyDynamicsWorld();

  /**
   * @brief get step time, unit s
   * @return
   */
  btScalar getStepTime();

private:
  tx_car::MBD_Vehicle_DW_DW m_vehicleParam_DWDW;
  btMultiBodyDynamicsWorld *m_world;
  btScalar m_stepTime;
};

/**
 * @brief construct helper class for vehicle model
 */
class VehicleConstructHelper : public VehicleHelperBase {};

/**
 * @brief init helper class for vehicle model
 */
class VehicleInitHelper : public VehicleHelperBase {
public:
  btVector3 getInitPosition() { return m_initPos; }
  btQuaternion getInitRotation() { return m_initRot; }

public:
  void setInitPosition(const btVector3 &initPosition) {
    m_initPos = initPosition;
  }
  void setInitRotation(const btQuaternion &initQuaternion) {
    m_initRot = initQuaternion;
  }

private:
  btVector3 m_initPos;
  btQuaternion m_initRot;
};

/**
 * @brief step helper class for vehicle model
 */
class VehicleStepHelper : public VehicleHelperBase {};

/**
 * @brief stop helper class for vehicle model
 */
class VehicleStopHelper : public VehicleHelperBase {};

/**
 * @brief sim rigid body
 */
struct SimRigidBody {
  // rigidbody
  btVector3 m_halfBoundingBox;
  btVector3 m_cylinderHalf;
  btScalar m_radius;
  btRigidBodyPtr m_body;
  btCollisionShapePtr m_chassisBodyCollider;
  std::map<std::string, btTypedConstraintPtr> m_constraints;

  // mass and inertia
  btScalar m_mass;
  btVector3 m_inertia;

  // local transform
  SimFrame m_thisFrameInParent;
  SimFrame m_thisFrameInGlobal;

  // local moving transform
  SimFrameMoving m_thisFrameInParent_Moving;
  SimFrameMoving m_thisFrameInGlobal_Moving;

  SimRigidBody() {
    m_halfBoundingBox = btVector3(0, 0, 0);
    m_cylinderHalf = btVector3(0, 0, 0);
    m_radius = 0.0;
    m_mass = 0.0;
    m_inertia = btVector3(0, 0, 0);
  }
  virtual ~SimRigidBody() {}
};

class SimDistanceSpringConstraint{
public:
  SimDistanceSpringConstraint(btRigidBody *bodyA, btRigidBody *bodyB,
                             const btVector3 &pivotInA,
                             const btVector3 &pivotInB, btScalar stepTime=0.001);
  virtual ~SimDistanceSpringConstraint();

  void setStiffness(btScalar stiffness);
  void setDamping(btScalar damping);
  void setFreeLength(btScalar freeLength = 1.0);
  void update();
  btScalar getCurrentLength();
  btScalar getCurrentVelocity();
  void getForceGlobal_A(btVector3 &force, btVector3 &pivotInA);
  void getForceGlobal_B(btVector3 &force, btVector3 &pivotInB);
  btScalar getSpringForce();

protected:
  btScalar calculateDistane();
  void calSpringForce();
  void calSpringForceInGlobal();
  void calSpringVelocity();

private:
  btRigidBody *m_bodyA, *m_bodyB;
  btVector3 m_pivotInA, m_pivotInB;
  btVector3 m_dirFromA2B, m_dirFromB2A;
  btScalar m_stiffness;
  btScalar m_damping;
  btScalar m_freeLength;
  btScalar m_currentLength, m_preLength;
  btScalar m_currentVelocity;
  btVector3 m_springForceGlobal_OnA, m_springForceGlobal_OnB;
  btScalar m_springForce;
  btScalar m_stepTime;
};

class VehicleTemplateBase;
using VehicleTemplateBasePtr = std::shared_ptr<VehicleTemplateBase>;

/**
 * @brief vehicle base class
 */
class VehicleTemplateBase {
public:
  VehicleTemplateBase() {}
  virtual ~VehicleTemplateBase() {}

  VehicleTemplateBase(const VehicleTemplateBase &) = delete;
  VehicleTemplateBase &operator=(const VehicleTemplateBase &) = delete;

public:
  /**
   * @brief construct vehicle model, in vehicle coordinate
   * @param helper
   */
  virtual void construct(VehicleConstructHelper &helper) = 0;

  /**
   * @brief initialize vehicle, including initial speed, initial transform in
   * global coordinate
   * @param helper
   */
  virtual void initialize(VehicleInitHelper &helper) = 0;

  /**
   * @brief update world, this may not be necessary, but let's put it here for
   * now with a default implementation
   * @param helper
   */
  virtual void stepSimulation(VehicleStepHelper &helper) {}

  /**
   * @brief stop simulation
   * @param helper
   */
  virtual void stop(VehicleStopHelper &helper) = 0;

public:
  virtual SimRigidBody *mutableSimBody() { return &m_simBody; }

public:
  const SimRigidBody &getSimBody() const { return m_simBody; }

protected:
  SimRigidBody m_simBody;
};
} // namespace tx_car
