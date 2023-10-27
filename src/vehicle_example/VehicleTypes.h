#pragma once

#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "CommonInterfaces//CommonGUIHelperInterface.h"
#include "CommonInterfaces/CommonMultiBodyBase.h"
#include "CommonInterfaces/CommonRigidBodyBase.h"

#include "Vehicle_DW_DW.pb.h"

namespace tx_car {
	using btRigidBodyPtr = std::shared_ptr<btRigidBody>;
	using btCollisionShapePtr = std::shared_ptr<btCollisionShape>;
	using btMultiBodyPtr = std::shared_ptr<btMultiBody>;
	using btTypedConstraintPtr = std::shared_ptr<btTypedConstraint>;
}