#include "ChainsExample.h"

CommonExampleInterface* StandaloneExampleCreateFunc(CommonExampleOptions& options) {
	return new ChainsExample(options.m_guiHelper);
}

ChainsExample::ChainsExample(GUIHelperInterface* helper) : CommonMultiBodyBase(helper){
	m_collisionShapes.reserve(m_const_size_100);
	m_bodies.reserve(m_const_size_100);

	m_GroundBoxHalfLength = 3;	// 10 by 10 
	m_GroundHalfHeight = 0.05;	// 0.1 height

	m_ballRadius = 0.2;
	m_ballMass = 3.0;

	m_roofInitZ = 6.0;
	m_roofBoxHalfLength = 0.5;
	m_roofBoxHalfHeight = 0.05;

	m_BodyInitZ = (m_roofInitZ - m_ballRadius * 2.0 * m_ballGapCoeff);
}

ChainsExample::~ChainsExample() {

}

void ChainsExample::setGUI_UpAxis_Z() {
	m_guiHelper->setUpAxis(2);
}
void ChainsExample::setGravity(const btVector3& gravity) {
	m_dynamicsWorld->setGravity(gravity);
}
void ChainsExample::resetCamera() {
	btScalar scale(0.5);
	m_guiHelper->resetCamera(6, -45, -45, -m_GroundBoxHalfLength * scale, -m_GroundBoxHalfLength * scale, 3.0);
}
void ChainsExample::renderScene() { 
	m_dynamicsWorld->debugDrawWorld();
	CommonMultiBodyBase::renderScene(); 
}
btRigidBodyPtr ChainsExample::createGround() {
	return createStaticPlane(btVector3(m_GroundBoxHalfLength, m_GroundBoxHalfLength, m_GroundHalfHeight), btQuaternion(0, 0, 0, 1), btVector3(0, 0, -5));
}

btRigidBodyPtr ChainsExample::createStaticPlane(const btVector3& halfExtendSize, const btQuaternion& rot, const btVector3& offset) {
	btCollisionShapePtr planeCollisionShape(new btBoxShape(halfExtendSize));
	auto staticPlane = createRigidBody(0, btTransform(rot, offset), planeCollisionShape.get());

	m_collisionShapes.push_back(planeCollisionShape);
	auto plane = btRigidBodyPtr(staticPlane);
	m_bodies.push_back(plane);

	return plane;
}

void ChainsExample::createRevoluteChains(const btVector3& chainsBasePos) {
	// number of links
	int numLinks = 10;

	// Create a btMultiBody with two links connected by a revolute joint
	btScalar mass = 1.0;
	btVector3 inertia(0, 0, 0);
	bool isFixedBase = true;		// fixed base
	bool canSleep = false;			// sleep

	btMultiBody* multiBody =
            new btMultiBody(numLinks, mass, inertia, isFixedBase, canSleep);
	m_multibodies.push_back(btMultiBodyPtr(multiBody));

	LOG_0 << "multiBody->getNumLinks():"
              << multiBody->getNumLinks()
              << "\n";

	// base world transform
        multiBody->setBaseWorldTransform(
            btTransform(btQuaternion(0, 0, 0, 1), chainsBasePos));

	// Create collision shapes for the baseand the links
	btCollisionShape* baseShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));
	btCollisionShape* linkShape1 = new btSphereShape(0.5);
	btCollisionShape* linkShape2 = new btCylinderShape(btVector3(0.5, 0.5, 0.5));
	m_collisionShapes.push_back(btCollisionShapePtr(baseShape));
	m_collisionShapes.push_back(btCollisionShapePtr(linkShape1));
	m_collisionShapes.push_back(btCollisionShapePtr(linkShape2));

	// Add a revolute joint between the base and the first link
	btVector3 hingeAxis(1, 0, 0);	// x-axis hinge constraint
	btVector3 offsetFromJoint(0, 0, -1.0);

	// add constraints
	{
		// Add a revolute joint between the base and the first link
		linkShape1->calculateLocalInertia(mass, inertia);
		for (auto i = 0; i < multiBody->getNumLinks(); ++i) {
			multiBody->setupRevolute(i, mass, inertia, i-1, btQuaternion(0, 0, 0, 1), hingeAxis, offsetFromJoint, offsetFromJoint, false);
		}
	}

	// add to world
	{
		// Finalize the creation of the btMultiBody
		multiBody->finalizeMultiDof();

		multiBody->setUseGyroTerm(true);

		// Add the btMultiBody to the dynamics world
		m_dynamicsWorld->addMultiBody(multiBody);
	}

	// add collider to btMultiBodyLink
	{
		// Create a btMultiBodyLinkCollider for the base and add it to the dynamics world
		btMultiBodyLinkCollider* baseCollider = new btMultiBodyLinkCollider(multiBody, -1);
		m_colliders.push_back(btMultiBodyLinkColliderPtr(baseCollider));

		baseCollider->setCollisionShape(baseShape);
		baseCollider->setWorldTransform(multiBody->getBaseWorldTransform());

		m_dynamicsWorld->addCollisionObject(baseCollider, 1, 1 + 2); // Set collision group and mask
		multiBody->setBaseCollider(baseCollider);

		// chains
		for (auto i = 0; i < multiBody->getNumLinks(); ++i) {
			// Create a btMultiBodyLinkCollider for the first link and add it to the dynamics world
			btMultiBodyLinkCollider* linkCollider1 = new btMultiBodyLinkCollider(multiBody, i);
			m_colliders.push_back(btMultiBodyLinkColliderPtr(linkCollider1));

			linkCollider1->setCollisionShape(linkShape1);
			linkCollider1->setWorldTransform(multiBody->getLink(i).m_cachedWorldTransform);

			m_dynamicsWorld->addCollisionObject(linkCollider1, 1, 1 + 2); // Set collision group and mask
			multiBody->getLink(i).m_collider = linkCollider1;
		}
	}

	// transfrom
	{
		std::vector<btQuaternion> world_to_local(multiBody->getNumLinks() + 1);
		std::vector<btVector3> local_origin(multiBody->getNumLinks() + 1);

		world_to_local[0] = multiBody->getWorldToBaseRot();
		local_origin[0] = multiBody->getBasePos();
		btVector3 eulerZYX;
		world_to_local[0].getEulerZYX(eulerZYX[0], eulerZYX[1], eulerZYX[2]);

		LOG_0 << "local_origin[" << 0 << "]:" << local_origin[0].x() << ", " << local_origin[0].y() << ", z:" << local_origin[0].z() << "\n";
		LOG_0 << "base Rot in world --> yaw:" << eulerZYX[0] << ", pitch:" << eulerZYX[1] << ", roll:" << eulerZYX[2] << "\n";

		for (int i = 0; i < multiBody->getNumLinks(); ++i)
		{
			btScalar stiffness = 100.0, damping = 10;

			auto& parentLink = multiBody->getLink(i - 1);
            btMultiBodyJointMotor* hingeConstraint = (btMultiBodyJointMotor*)parentLink.m_userPtr;
            LOG_0 << "hinge constraint pointer:" << hingeConstraint << "\n";
			
			world_to_local[i + 1] = multiBody->getParentToLocalRot(i) * world_to_local[i];
			world_to_local[i + 1].getEulerZYX(eulerZYX[0], eulerZYX[1], eulerZYX[2]);
			LOG_0 << "base Rot in world(" << i + 1 << ") --> yaw:" << eulerZYX[0] << ", pitch:" << eulerZYX[1] << ", roll:" << eulerZYX[2] << "\n";
			multiBody->getParentToLocalRot(i).getEulerZYX(eulerZYX[0], eulerZYX[1], eulerZYX[2]);
			LOG_0 << "multiBody->getParentToLocalRot(" << i << ") --> yaw:" << eulerZYX[0] << ", pitch:" << eulerZYX[1] << ", roll:" << eulerZYX[2] << "\n";
			local_origin[i + 1] = local_origin[i] + (quatRotate(world_to_local[i + 1].inverse(), multiBody->getRVector(i)));
			LOG_0 << "local_origin[" << i + 1 << "]:" << local_origin[i + 1].x() << ", " << local_origin[i + 1].y() << ", z:" << local_origin[i + 1].z() << "\n";
		}

		for (int i = 0; i < multiBody->getNumLinks(); ++i)
		{
			btVector3 posr = local_origin[i + 1];

			btScalar quat[4] = { -world_to_local[i + 1].x(), -world_to_local[i + 1].y(), -world_to_local[i + 1].z(), world_to_local[i + 1].w() };

			btMultiBodyLinkCollider* col = multiBody->getLink(i).m_collider;

			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(posr);
			tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
			col->setWorldTransform(tr);
			col->setFriction(0.9);
			LOG_0 << "collider[" << i + 1 << "]:" << posr.x() << ", " << posr.y() << ", z:" << posr.z() << "\n";
		}
	}
}

void ChainsExample::createPoint2Point_example(btScalar ballRadius,
                                              const btVector3& offset) {
  // get roof position
  auto roofPosition = m_roof->getWorldTransform().getOrigin();

  // z offset
  btVector3 defaultOffset = {0.0, 0.0, -(ballRadius * 2 * 1.5)};
  auto ballInitPosition = roofPosition + defaultOffset;
  LOG(INFO) << "ballInitPosition:" << ballInitPosition.x() << ", "
            << ballInitPosition.y() << ", " << ballInitPosition.z() << "\n";

  // create ball collision
  auto ballShape = new btSphereShape(ballRadius);
  m_collisionShapes.push_back(btCollisionShapePtr(ballShape));

  // create ball bodies
  const int ballSize = 3;
  std::vector<btRigidBodyPtr> balls;
  std::vector<btTransform> ballsTransform(ballSize);

  // inertia
  btVector3 inertia;
  ballShape->calculateLocalInertia(m_ballMass, inertia);

  // ball transform
  for (auto i = 0; i < ballSize; ++i) {
    auto& trans = ballsTransform.at(i);
    trans.setIdentity();
    trans.setOrigin(ballInitPosition);
    
    LOG(INFO) << "position[" << i << "]:" << ballInitPosition.x() << ", "
              << ballInitPosition.y() << ", " << ballInitPosition.z() << "\n";

	ballInitPosition.setZ(ballInitPosition.z() + defaultOffset.z());
  }

  // create balls
  for (auto i = 0; i < ballSize; ++i) {
    auto body = createRigidBody(m_ballMass, ballsTransform.at(i), ballShape);
    auto bodyPtr = btRigidBodyPtr(body);
    balls.push_back(bodyPtr);
    m_bodies.push_back(bodyPtr);
    m_dynamicsWorld->addRigidBody(body);
    LOG(INFO) << "rigid body pointer:" << body << "\n";
  }

  // add constraints
  for (auto i = 0; i < ballSize; ++i) {
    btPoint2PointConstraint* p2pConstraints = nullptr;
    if (i == 0) {
      p2pConstraints = new btPoint2PointConstraint(*m_roof.get(), *balls.at(0).get(),
                                      0.5 * defaultOffset, -0.5 * defaultOffset);

    } else {
      p2pConstraints = new btPoint2PointConstraint(
          *balls.at(i - 1).get(), *balls.at(i).get(), 0.5 * defaultOffset,
          -0.5 * defaultOffset);
    }

    std::string id = "P2P_" + std::to_string(i);
    m_constraintMap[id] = btTypedConstraintPtr(p2pConstraints);
    m_dynamicsWorld->addConstraint(p2pConstraints);
  }
}

void ChainsExample::createSpringDamper_Example(btScalar ballRadius, const btVector3& offsetFromRoof) {
	// get roof position
	auto roofPosition = m_roof->getWorldTransform().getOrigin();
	auto ballPosition = roofPosition + offsetFromRoof;
	ballPosition.setY(ballPosition.getZ());
;	LOG(INFO) << "roofPosition:" << roofPosition.z() << "\n";
	LOG(INFO) << "ballInitPosition:" << ballPosition.x()<<", "<< ballPosition.y()<<", "<< ballPosition.z() << "\n";

	// create rigid body
	auto cylinderShape = btCollisionShapePtr(new btCylinderShape(btVector3(m_ballRadius, 0.1, 0.1)));
	m_collisionShapes.push_back(cylinderShape);

	auto cylinder = createRigidBody(m_ballMass, btTransform(btQuaternion(0,0,0,1), ballPosition), cylinderShape.get());
	auto cylinderBody = btRigidBodyPtr(cylinder);
	m_bodies.push_back(cylinderBody);

	// create spring-damper
	btTransform transInA, transInB;
	transInA.setRotation(btQuaternion(btVector3(1, 0, 0), M_PI * 0.25));
	transInA.setOrigin(btVector3(0, 0, -m_roofBoxHalfHeight));

	transInB.setRotation(btQuaternion(btVector3(1, 0, 0), M_PI * 0.25));
	transInB.setOrigin(btVector3(0, -0.1, 0));

	//auto* spring = new btGeneric6DofSpringConstraint(*m_roof.get(), *cylinderBody.get(), transInA, transInB, true);
	auto* spring = new btGeneric6DofSpring2Constraint(*m_roof.get(), *cylinderBody.get(), transInA, transInB);
	auto springPtr = btTypedConstraintPtr(spring);
	m_constraintMap["SP_1"] = springPtr;

	m_dynamicsWorld->addConstraint(spring);
	spring->setDbgDrawSize(btScalar(5.f));

	int springAxis = 2;

	// disable other DoF
	for (auto i = 0; i < 6; ++i) {
		if (i != springAxis) {
			spring->enableSpring(i, false);
		}
		else {
			spring->enableSpring(i, true);
		}
	}

	spring->setLinearLowerLimit(btVector3(0, 0, -5));
	spring->setLinearUpperLimit(btVector3(0, 0, 5));
	spring->setAngularLowerLimit(btVector3(0, 0, 0));
	spring->setAngularUpperLimit(btVector3(0, 0, 0));

	spring->setStiffness(springAxis, 100.0);
	spring->setDamping(springAxis, 10);
	spring->setEquilibriumPoint();		// use this, do not set value of EquilibriumPoint manually.
}

void ChainsExample::initPhysics() {
	// create world
	createEmptyDynamicsWorld();

	// set gravity and up axis
	setGUI_UpAxis_Z();
	setGravity();

	// prepare drawer
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	// create ground
	m_ground = createGround();
	m_bodyMap["ground"] = m_ground;

	// create rood
	m_roof = createStaticPlane(btVector3(m_roofBoxHalfLength, m_roofBoxHalfLength, m_roofBoxHalfHeight), btQuaternion(0, 0, 0, 1), btVector3(0,0,m_roofInitZ));
	m_bodyMap["roof"] = m_roof;

	// create spring-damper example
	createSpringDamper_Example(m_ballRadius, btVector3(0.0, 0.0, -3.0));

	// create multibody example
    createRevoluteChains(btVector3(6, 0, 4));

	// create point to point example
	createPoint2Point_example(m_ballRadius, btVector3(0.0, -6.0, -2.0));

	// auto add graphics
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void ChainsExample::stepSimulation(float deltaTime) {
    auto multiBody = m_multibodies.front().get();
    btScalar stiffness = 100.0; // Set the desired stiffness value
    btScalar damping = 1.0;     // Set the desired damping value
    for (auto i = 0; i < multiBody->getNumLinks(); ++i) {
            btScalar jointPosition = multiBody->getJointPos(i);
            btScalar jointVelocity = multiBody->getJointVel(i);
            
            btScalar torque = -stiffness * jointPosition - damping * jointVelocity;
            LOG_0 << "joint:" << i << ", position:" << jointPosition
                  << ", velocity:" << jointVelocity << ", torque:" << torque << "\n";

            multiBody->addJointTorque(i, torque);
	}
	m_dynamicsWorld->stepSimulation(deltaTime);
}

void ChainsExample::exitPhysics() {
	//CommonMultiBodyBase::exitPhysics();
}