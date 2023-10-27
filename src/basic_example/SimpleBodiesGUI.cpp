#include "SimCore.h"

#include "SimpleBodiesGUI.h"
#include "btBulletDynamicsCommon.h"

/// This is a Hello World program for running a basic Bullet physics simulation

#ifdef BUILD_SIM_DLL
#undef SIM_BUILD_DLL
#endif

CommonExampleInterface* StandaloneExampleCreateFunc(CommonExampleOptions& options) {
	return new SimpleBodiesGUI(options.m_guiHelper);
}

SimpleBodiesGUI::SimpleBodiesGUI(struct GUIHelperInterface* helper): CommonRigidBodyBase(helper) {
	m_GroundBoxHalfLength = 5;
	m_GroundHalfHeight = 0.05;
	m_BodyInitZ = m_GroundHalfHeight + 2.0;
	m_ballRadius = 0.3;

	m_bodies.reserve(m_constBody_size);
	m_collisionShapes.reserve(m_constBody_size);
}
SimpleBodiesGUI::~SimpleBodiesGUI() {

}
void SimpleBodiesGUI::initPhysics() {
	// create dynamic world
	createEmptyDynamicsWorld();

	// prepare drawer
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
	
	// create static plane
	{
		btVector3 gndBoxShape(m_GroundBoxHalfLength, m_GroundBoxHalfLength, m_GroundHalfHeight);
		std::shared_ptr<btCollisionShape> gndCollisionShape = std::make_shared<btBoxShape>(gndBoxShape);
		m_collisionShapes.push_back(gndCollisionShape);

		btTransform groundTrans;
		groundTrans.setIdentity();
		groundTrans.setOrigin({ 0,0,-m_GroundHalfHeight });

		btScalar groundMass(0);
		btVector3 groudnLocalInertia(0, 0, 0);
		if (groundMass != 0.0f)
			gndCollisionShape->calculateLocalInertia(groundMass, groudnLocalInertia);

		btDefaultMotionState* bodyMotion=new btDefaultMotionState(groundTrans);
		btRigidBody::btRigidBodyConstructionInfo bodyBuildInfo(groundMass, bodyMotion, gndCollisionShape.get(), groudnLocalInertia);

		auto body = std::make_shared<btRigidBody>(bodyBuildInfo);
		m_bodies.push_back(body);

		m_dynamicsWorld->addRigidBody(body.get());
	}

	btScalar curInitZ = m_BodyInitZ;

	// create dynamic sphere shape
	{
		std::shared_ptr<btCollisionShape> sphereCollisionShape = std::make_shared<btSphereShape>(m_ballRadius);
		m_collisionShapes.push_back(sphereCollisionShape);

		btScalar sphereMass(15.0);
		btVector3 localInertia;
		sphereCollisionShape->calculateLocalInertia(sphereMass, localInertia);

		//***************//
		btTransform bodyTransform;
		bodyTransform.setIdentity();
		bodyTransform.setOrigin({ 0, 0, curInitZ });

		btDefaultMotionState* bodyMotion = new btDefaultMotionState(bodyTransform);
		btRigidBody::btRigidBodyConstructionInfo bodyBuildInfo(sphereMass, bodyMotion, sphereCollisionShape.get(), localInertia);

		auto body = std::make_shared<btRigidBody>(bodyBuildInfo);
		body->setAngularVelocity({0,0,3});

		m_bodies.push_back(body);
		m_dynamicsWorld->addRigidBody(body.get());

		//***************//
		curInitZ = curInitZ + m_ballRadius * 2 * 1.2;
		btTransform bodyTransformA;
		bodyTransformA.setIdentity();
		bodyTransformA.setOrigin({ 0, 0.1, curInitZ });

		btDefaultMotionState* bodyMotionA = new btDefaultMotionState(bodyTransformA);
		btRigidBody::btRigidBodyConstructionInfo bodyBuildInfoA(sphereMass, bodyMotionA, sphereCollisionShape.get(), localInertia);

		body = std::make_shared<btRigidBody>(bodyBuildInfoA);

		m_bodies.push_back(body);
		m_dynamicsWorld->addRigidBody(body.get());

		//***************//
		curInitZ = curInitZ + m_ballRadius * 2 * 1.2;
		body = std::shared_ptr<btRigidBody>(createRigidBody(5.0, btTransform(btMatrix3x3::getIdentity(), btVector3(0, 0.2, curInitZ)), sphereCollisionShape.get()));
		body->setLinearVelocity({ 0,0,5 });

		m_bodies.push_back(body);
		m_dynamicsWorld->addRigidBody(body.get());
	}
	
	// set up axis and gravity
	setGravity();
	setGUI_UpAxis_Z();

	// auto add graphics
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void SimpleBodiesGUI::setGravity(const btVector3& gravity) {
	m_dynamicsWorld->setGravity(gravity);
}

void SimpleBodiesGUI::setGUI_UpAxis_Z() {
	m_guiHelper->setUpAxis(2);
}

void SimpleBodiesGUI::resetCamera() {
	btScalar scale(0.1);
	m_guiHelper->resetCamera(5, -45, -45, -m_GroundBoxHalfLength * scale, -m_GroundBoxHalfLength * scale, 3.0);
}

void SimpleBodiesGUI::exitPhysics() {

}