#pragma once

#include "SimCore.h"

#include <map>

#include "CommonInterfaces/CommonMultiBodyBase.h"
#include "CommonInterfaces//CommonGUIHelperInterface.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"


CommonExampleInterface* StandaloneExampleCreateFunc(CommonExampleOptions& options);

class ChainsExample;
using ChainsExamplePtr = std::shared_ptr<ChainsExample>;
using btRigidBodyPtr = std::shared_ptr<btRigidBody>;
using btCollisionShapePtr = std::shared_ptr<btCollisionShape>;
using btMultiBodyPtr = std::shared_ptr<btMultiBody>;
using btTypedConstraintPtr = std::shared_ptr<btTypedConstraint>;
using btMultiBodyLinkColliderPtr = std::shared_ptr<btMultiBodyLinkCollider>;

class ChainsExample : public CommonMultiBodyBase {
public:
	ChainsExample(GUIHelperInterface* helper);
	virtual ~ChainsExample();
public:
	virtual void initPhysics() override;
	virtual void stepSimulation(float deltaTime) override;
	virtual void exitPhysics() override;
	virtual void resetCamera() override;
	virtual void renderScene() override;

public:
	btRigidBodyPtr createGround();
	btRigidBodyPtr createStaticPlane(const btVector3& halfExtendSize, const btQuaternion& rot, const btVector3& offset);

	// create spring-damper
	void createSpringDamper_Example(btScalar ballRadius, const btVector3& offset);
	void createPoint2Point_example(btScalar ballRadius, const btVector3& offset);

public:
	void setGUI_UpAxis_Z();
	void setGravity(const btVector3& gravity = { 0,0,-9.81 });
    void createRevoluteChains(const btVector3& chainsBasePos);
    void createConeChains(const btVector3& chainsBasePos);
    void createDoF6SpringChains(const btVector3& chainsBasePos);
    void createPoint2PointChains(const btVector3& chainsBasePos);

private:
	// ground info
	btScalar m_GroundBoxHalfLength, m_GroundHalfHeight;

	// ball info
	btScalar m_BodyInitZ, m_ballRadius, m_ballMass;
	const btScalar m_ballGapCoeff = 1.2;

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
	std::vector<btMultiBodyLinkColliderPtr> m_colliders;
	std::vector<btMultiBodyPtr> m_multibodies;
	const size_t m_const_size_100 = 100;
	btRigidBodyPtr m_ground, m_roof;
};