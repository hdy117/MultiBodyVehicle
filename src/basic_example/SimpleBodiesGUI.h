#pragma once

#include <memory>

#include "CommonInterfaces/CommonRigidBodyBase.h"
#include "CommonInterfaces/CommonExampleInterface.h"

using btRigidBodyPtr = std::shared_ptr<btRigidBody>;

class SimpleBodiesGUI : public CommonRigidBodyBase {
public:
	SimpleBodiesGUI(struct GUIHelperInterface* helper);
	virtual ~SimpleBodiesGUI();
public:
	virtual void initPhysics() override;
	virtual void exitPhysics() override;
	void setGravity(const btVector3& gravity = { 0,0,-9.81 });
public:
	virtual void resetCamera() override;
	void setGUI_UpAxis_Z();
private:
	btScalar m_GroundBoxHalfLength, m_GroundHalfHeight;
	btScalar m_BodyInitZ, m_ballRadius;
	std::vector<std::shared_ptr<btCollisionShape>> m_collisionShapes;
	std::vector<btRigidBodyPtr> m_bodies;
	const size_t m_constBody_size = 100;
};

CommonExampleInterface* StandaloneExampleCreateFunc(CommonExampleOptions& options);




