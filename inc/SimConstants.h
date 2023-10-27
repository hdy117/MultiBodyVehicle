/**
 * @file SimConstants.h
 * @author DongYuanHu (dongyuanhu@tencent.com)
 * @brief
 * @version 0.1
 * @date 2023-08-01
 *
 *
 */
#pragma once

#ifndef _USE_MATH_DEFINES
#define_USE_MATH_DEFINES 1
#endif // !_USE_MATH_DEFINES

#include <cmath>

#define DEG_2_RAD (1.0 / 180.0 * M_PI)
#define RAD_2_DEG (1.0 / M_PI * 180.0)

namespace tx_car {
const int const_map1dSize = 100;
const float const_ControlArm_ChassisBody_Angle_Limit = 30 * DEG_2_RAD;
const float const_Upright_ControlArm_Angle_Limit = 10 * DEG_2_RAD;
const float const_ERP = 0.95; /*
						   ERP is a parameter that controls how quickly constraint errors are corrected during the simulation. 
						   Constraint errors occur when the constraint is violated, 
						   for example, when two connected rigid bodies move apart due to external forces or numerical inaccuracies.                        
						   The ERP value determines the fraction of the constraint error that will be corrected during each simulation step. 
						   A value of 1 means the error will be fully corrected in a single step, while smaller values will result in a slower error correction. 
						   Higher ERP values can make the constraint more responsive, but may also cause instability in the simulation.
						   */
const float const_CFM = 1e-3; /*CFM is a parameter that helps determine the softness of a constraint. 
							  It is used to mix constraint forces with the rigid body forces during the simulation. 
							  A higher CFM value results in a softer constraint, 
							  allowing some violation of the constraint under large forces. 
							  This can help prevent instability and jitter in the simulation.
							  A lower CFM value results in a stiffer constraint, 
							  which strictly enforces the constraint but may cause instability if the constraint is violated under large forces.
							  */
}