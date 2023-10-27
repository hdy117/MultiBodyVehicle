#include "VehicleUtils.h"

namespace tx_car {
btVector3 VehicleUtils::paramVec3TobtVector3(const tx_car::RealVec3 &vec3In) {
  return btVector3(vec3In.x(), vec3In.y(), vec3In.z());
}

btVector4 VehicleUtils::paramVec4TobtVector4(const tx_car::RealVec4 &vec4In) {
  return btVector4(vec4In.x(), vec4In.y(), vec4In.z(), vec4In.w());
}

btQuaternion
VehicleUtils::paramVec4TobtQuaternion(const tx_car::RealVec4 &vec4In) {
  return btQuaternion(vec4In.x(), vec4In.y(), vec4In.z(), vec4In.w());
}

void VehicleUtils::printbtVector3(const btVector3 &vec3,
                                  const std::string &prefix) {
  LOG_INFO << prefix << ", btVector3:[" << vec3.x() << ", " << vec3.y() << ", "
        << vec3.z() << "].\n";
}

btRigidBody *VehicleUtils::createRigidBodyWithInertia(
    btScalar mass, const btVector3 &inertia, const btTransform &startTransform,
    btCollisionShape *shape, const btVector4 &color) {
  btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

  // rigidbody is dynamic if and only if mass is non zero, otherwise static
  bool isDynamic = (mass != 0.f);

  btVector3 localInertia(0, 0, 0);
  if (isDynamic)
    localInertia = inertia;

    // using motionstate is recommended, it provides interpolation capabilities,
    // and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
  btDefaultMotionState *myMotionState =
      new btDefaultMotionState(startTransform);

  btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape,
                                                 localInertia);

  btRigidBody *body = new btRigidBody(cInfo);
  // body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
  btRigidBody *body = new btRigidBody(mass, 0, shape, localInertia);
  body->setWorldTransform(startTransform);
#endif //

  body->setUserIndex(-1);
  return body;
}
} // namespace tx_car