#pragma once

#include <map>
#include <string>

#include "VehicleTypes.h"
#include "SimLog.h"

namespace tx_car {
class VehicleUtils {
public:
  static btVector3 paramVec3TobtVector3(const tx_car::RealVec3 &vec3In);
  static btVector4 paramVec4TobtVector4(const tx_car::RealVec4 &vec4In);
  static btQuaternion paramVec4TobtQuaternion(const tx_car::RealVec4 &vec4In);
  static void printbtVector3(const btVector3 &vec3,
                             const std::string &prefix = "");

public:
  static btRigidBody *
  createRigidBodyWithInertia(btScalar mass, const btVector3 &inertia,
                             const btTransform &startTransform,
                             btCollisionShape *shape,
                             const btVector4 &color = btVector4(1, 0, 0, 1));
};
} // namespace tx_car