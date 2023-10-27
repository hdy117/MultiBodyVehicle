#include "param_writer.h"

namespace tx_car {

void ParamWriter_DWDW::write(tx_car::MBD_Vehicle_DW_DW &vehicle) {
  writeChassisBody(vehicle);
  writeFrontAxle(vehicle);
  writeRearAxle(vehicle);
}
void ParamWriter_DWDW::writeFrontAxle(tx_car::MBD_Vehicle_DW_DW &vehicle) {
	auto& frontAxle = *vehicle.mutable_front();
	
	// location
	auto posAxle = frontAxle.mutable_offsetinchassisbody();

	posAxle->set_x(1.688965);
	posAxle->set_y(0);
	posAxle->set_z(0);

	// steerable
    frontAxle.set_steerable(true);

	// axle inertia
	frontAxle.mutable_axleinertia()->set_val(0.4);

	// set toe and camer
    frontAxle.mutable_toe()->set_val(0.2);
    frontAxle.mutable_camber()->set_val(-0.14);

	// write DoubleWishBone
	writeDoubleWishBone_Front(*frontAxle.mutable_suspleft(), 1);
	writeDoubleWishBone_Front(*frontAxle.mutable_suspright(), -1);

	// write wheel
    writeWheels(vehicle);
}

void ParamWriter_DWDW::writeRearAxle(tx_car::MBD_Vehicle_DW_DW &vehicle) {
	auto& rearAxle = *vehicle.mutable_rear();

	// location
	auto posAxle = rearAxle.mutable_offsetinchassisbody();

	posAxle->set_x(-1.688965);
	posAxle->set_y(0);
	posAxle->set_z(0);

	// axle inertia
	rearAxle.mutable_axleinertia()->set_val(0.4);

	// write DoubleWishBone
	writeDoubleWishBone_Rear(*rearAxle.mutable_suspleft(), 1);
	writeDoubleWishBone_Rear(*rearAxle.mutable_suspright(), -1);
}

void ParamWriter_DWDW::writeDoubleWishBone_Front(tx_car::MBD_DoubleWishBoneSusp& dw, int flip) {
	// spindle
	{
		auto spindle = dw.mutable_spindle();

		spindle->mutable_mass()->set_val(14.705);

		spindle->mutable_offset()->set_x(0);
		spindle->mutable_offset()->set_y(0.91 * flip);
		spindle->mutable_offset()->set_z(-0.026);

		spindle->mutable_inertia()->set_x(0.04117);
		spindle->mutable_inertia()->set_y(0.07352);
		spindle->mutable_inertia()->set_z(0.04117);

		spindle->mutable_radius()->set_val(0.15); 
		spindle->mutable_width()->set_val(0.06); 
	}

	// upright
	{
		auto upright = dw.mutable_upright();

		upright->mutable_mass()->set_val(19.45);

		upright->mutable_radius()->set_val(0.025);

		upright->mutable_offset()->set_x(0);
		upright->mutable_offset()->set_y(0.751 * flip);
		upright->mutable_offset()->set_z(-0.026);

		upright->mutable_inertia()->set_x(0.1656);
		upright->mutable_inertia()->set_y(0.1934);
		upright->mutable_inertia()->set_z(0.04367);
	}

	float LCA_Z_ConnectorChassis = 0.1, UCA_Z_ConnectorChassis = 0.428;

	// upper control arm
	{
		auto arm = dw.mutable_uppercontrolarm();

		arm->mutable_mass()->set_val(5.813);

		arm->mutable_radius()->set_val(0.03);// origin is 0.002

		arm->mutable_offset()->set_x(0);
		arm->mutable_offset()->set_y(0.589 * flip);
        arm->mutable_offset()->set_z(UCA_Z_ConnectorChassis);

		arm->mutable_inertia()->set_x(0.03);
		arm->mutable_inertia()->set_y(0.03);
		arm->mutable_inertia()->set_z(0.06276);

		arm->mutable_offsetinchassisfront()->set_x(0.168);
		arm->mutable_offsetinchassisfront()->set_y(0.446 * flip);
                arm->mutable_offsetinchassisfront()->set_z(
                    UCA_Z_ConnectorChassis);

		arm->mutable_offsetinchassisback()->set_x(-0.168);
		arm->mutable_offsetinchassisback()->set_y(0.446 * flip);
                arm->mutable_offsetinchassisback()->set_z(
                    UCA_Z_ConnectorChassis);

		arm->mutable_offsetinupright()->set_x(0);
		arm->mutable_offsetinupright()->set_y(0.716 * flip);
        arm->mutable_offsetinupright()->set_z(UCA_Z_ConnectorChassis);
	}

	// lower control arm
	{
		auto arm = dw.mutable_lowercontrolarm();

		arm->mutable_mass()->set_val(23.965);

		arm->mutable_radius()->set_val(0.03);

		arm->mutable_offset()->set_x(0);
        arm->mutable_offset()->set_y(0.547 * flip);
        arm->mutable_offset()->set_z(LCA_Z_ConnectorChassis * 0.5 -
                                        0.118 * 0.5);

		arm->mutable_inertia()->set_x(0.4);
		arm->mutable_inertia()->set_y(0.4);
		arm->mutable_inertia()->set_z(0.8938);

		arm->mutable_offsetinchassisfront()->set_x(0.223);
		arm->mutable_offsetinchassisfront()->set_y(0.307 * flip);
                arm->mutable_offsetinchassisfront()->set_z(
                    LCA_Z_ConnectorChassis);

		arm->mutable_offsetinchassisback()->set_x(-0.223);
		arm->mutable_offsetinchassisback()->set_y(0.307 * flip);
                arm->mutable_offsetinchassisback()->set_z(
                    LCA_Z_ConnectorChassis);

		arm->mutable_offsetinupright()->set_x(0);
		arm->mutable_offsetinupright()->set_y(0.787 * flip);
		arm->mutable_offsetinupright()->set_z(-0.118);
	}

	// spring 
	{
		auto spring = dw.mutable_spring();
		
		spring->mutable_stiffness()->set_val(45000 * 100);
		spring->mutable_freelength()->set_val(0.3);

		spring->mutable_offsetinchassis()->set_x(0);//0.104
		spring->mutable_offsetinchassis()->set_y(0.45 * flip);
                spring->mutable_offsetinchassis()->set_z(
                    UCA_Z_ConnectorChassis + 0.1);

		spring->mutable_offsetinaxle()->set_x(0); // 0.097
        spring->mutable_offsetinaxle()->set_y(0.65 * flip);
        spring->mutable_offsetinaxle()->set_z(-0.047);
	}

	// damper 
	{
		auto damper = dw.mutable_damper();

		damper->mutable_damping()->set_val(19.015 * 1000.0);

		damper->mutable_offsetinchassis()->set_x(0.104);
		damper->mutable_offsetinchassis()->set_y(0.510 * flip);
		damper->mutable_offsetinchassis()->set_z(0.197);

		damper->mutable_offsetinaxle()->set_x(0.097);
        damper->mutable_offsetinaxle()->set_y(0.543 * flip);
        damper->mutable_offsetinaxle()->set_z(-0.047);
	}
}

void ParamWriter_DWDW::writeDoubleWishBone_Rear(tx_car::MBD_DoubleWishBoneSusp& dw, int flip) {
	// spindle
	{
		auto spindle = dw.mutable_spindle();

		spindle->mutable_mass()->set_val(15.91);

		spindle->mutable_offset()->set_x(0);
		spindle->mutable_offset()->set_y(0.91 * flip);
		spindle->mutable_offset()->set_z(-0.026);

		spindle->mutable_inertia()->set_x(0.04117);
		spindle->mutable_inertia()->set_y(0.07352);
		spindle->mutable_inertia()->set_z(0.04117);

		spindle->mutable_radius()->set_val(0.15);
        spindle->mutable_width()->set_val(0.06);
	}

	// upright
	{
		auto upright = dw.mutable_upright();

		upright->mutable_mass()->set_val(27.27);

		upright->mutable_radius()->set_val(0.025);

		upright->mutable_offset()->set_x(0.0);
		upright->mutable_offset()->set_y(0.751 * flip);
		upright->mutable_offset()->set_z(-0.026);

		upright->mutable_inertia()->set_x(0.1656);
		upright->mutable_inertia()->set_y(0.1934);
		upright->mutable_inertia()->set_z(0.04367);
	}

	float LCA_Z_ConnectorChassis = 0.1,
              UCA_Z_ConnectorChassis = 0.428;

	// upper control arm
	{
		auto arm = dw.mutable_uppercontrolarm();

		arm->mutable_mass()->set_val(5.45);

		arm->mutable_radius()->set_val(0.03); // origin is 0.002

		arm->mutable_offset()->set_x(0.0);
		arm->mutable_offset()->set_y(0.589 * flip);
        arm->mutable_offset()->set_z(UCA_Z_ConnectorChassis);

		arm->mutable_inertia()->set_x(0.03);
		arm->mutable_inertia()->set_y(0.03);
		arm->mutable_inertia()->set_z(0.06276);

		arm->mutable_offsetinchassisfront()->set_x(0.168);
		arm->mutable_offsetinchassisfront()->set_y(0.462 * flip);
                arm->mutable_offsetinchassisfront()->set_z(
                    UCA_Z_ConnectorChassis);

		arm->mutable_offsetinchassisback()->set_x(-0.168);
		arm->mutable_offsetinchassisback()->set_y(0.462 * flip);
                arm->mutable_offsetinchassisback()->set_z(
                    UCA_Z_ConnectorChassis);

		arm->mutable_offsetinupright()->set_x(0.0);
		arm->mutable_offsetinupright()->set_y(0.716 * flip);
        arm->mutable_offsetinupright()->set_z(UCA_Z_ConnectorChassis);
	}

	// lower control arm
	{
		auto arm = dw.mutable_lowercontrolarm();

		arm->mutable_mass()->set_val(16.36);

		arm->mutable_radius()->set_val(0.03);

		arm->mutable_offset()->set_x(0);
		arm->mutable_offset()->set_y(0.547 * flip);
        arm->mutable_offset()->set_z(LCA_Z_ConnectorChassis * 0.5 -
                                             0.118*0.5);

		arm->mutable_inertia()->set_x(0.4);
		arm->mutable_inertia()->set_y(0.4);
		arm->mutable_inertia()->set_z(0.8938);

		arm->mutable_offsetinchassisfront()->set_x(0.223);
		arm->mutable_offsetinchassisfront()->set_y(0.307 * flip);
                arm->mutable_offsetinchassisfront()->set_z(
                    LCA_Z_ConnectorChassis);

		arm->mutable_offsetinchassisback()->set_x(-0.223);
		arm->mutable_offsetinchassisback()->set_y(0.307 * flip);
                arm->mutable_offsetinchassisback()->set_z(
                    LCA_Z_ConnectorChassis);

		arm->mutable_offsetinupright()->set_x(0);
		arm->mutable_offsetinupright()->set_y(0.787 * flip);
		arm->mutable_offsetinupright()->set_z(-0.118);
	}

	// spring 
	{
		auto spring = dw.mutable_spring();

		spring->mutable_stiffness()->set_val(50000 * 100);
		spring->mutable_freelength()->set_val(0.3);

		spring->mutable_offsetinchassis()->set_x(0);//-0.104
		spring->mutable_offsetinchassis()->set_y(0.45 * flip);
                spring->mutable_offsetinchassis()->set_z(
                    UCA_Z_ConnectorChassis+0.1);

		spring->mutable_offsetinaxle()->set_x(0); //-0.097
        spring->mutable_offsetinaxle()->set_y(0.65 * flip);
        spring->mutable_offsetinaxle()->set_z(-0.047);
	}

	// damper 
	{
		auto damper = dw.mutable_damper();

		damper->mutable_damping()->set_val(19.015 * 1000.0);

		damper->mutable_offsetinchassis()->set_x(-0.104);
		damper->mutable_offsetinchassis()->set_y(0.502 * flip);
		damper->mutable_offsetinchassis()->set_z(0.256);

		damper->mutable_offsetinaxle()->set_x(-0.097);
        damper->mutable_offsetinaxle()->set_y(0.543 * flip);
        damper->mutable_offsetinaxle()->set_z(-0.047);
	}
}

void ParamWriter_DWDW::writeWheels(tx_car::MBD_Vehicle_DW_DW &vehicle) {
	auto& whl_FL = *vehicle.mutable_front()->mutable_wheelleft()->Add();
	auto& whl_FR = *vehicle.mutable_front()->mutable_wheelright()->Add();
	auto& whl_RL = *vehicle.mutable_rear()->mutable_wheelleft()->Add();
	auto& whl_RR = *vehicle.mutable_rear()->mutable_wheelright()->Add();

	auto setWheelInfo = [](tx_car::MBD_Wheel &wheelOut, bool isLeft, bool isSteerable, int flip) {
		wheelOut.mutable_radius()->set_val(0.268);
		wheelOut.mutable_diameter()->set_val(0.268 * 2);
		wheelOut.mutable_width()->set_val(0.22);
		wheelOut.mutable_aspectratio()->set_val(45);

		wheelOut.mutable_mass()->set_val(18.8);

		wheelOut.mutable_inertia()->set_x(0.4634);
		wheelOut.mutable_inertia()->set_y(0.6243);
		wheelOut.mutable_inertia()->set_z(0.4634);

		wheelOut.mutable_offsetinaxle()->set_x(0.036);
                wheelOut.mutable_offsetinaxle()->set_y(
                    (0.91 + 0.22 / 2 + 0.06/2 + 0.03) * flip);
        wheelOut.mutable_offsetinaxle()->set_z(-0.026);

		wheelOut.set_steerable(isSteerable);
        wheelOut.set_isleft(isLeft);
	};

	setWheelInfo(whl_FL, true, true, 1);
    setWheelInfo(whl_FR, false, true, -1);
    setWheelInfo(whl_RL, true, false, 1);
    setWheelInfo(whl_RR, false, false, -1);
}

void ParamWriter_DWDW::writeChassisBody(tx_car::MBD_Vehicle_DW_DW &vehicle) {
  auto &chassisBody = *vehicle.mutable_chassisbody();

  // location
  auto cgOffset = chassisBody.mutable_offsetofcg();
  cgOffset->set_x(0.056);
  cgOffset->set_y(0.0);
  cgOffset->set_z(0.3);

  // orentation
  auto cgOren = chassisBody.mutable_orientation();
  cgOren->set_x(0);
  cgOren->set_y(0);
  cgOren->set_z(0);
  cgOren->set_w(1.0);

  // mass
  chassisBody.mutable_mass()->set_val(2086.58);

  // inertia
  auto inertia = chassisBody.mutable_inertia();
  inertia->set_x(1078.52);
  inertia->set_y(2995.66);
  inertia->set_z(3570.2);

  // bounding box
  auto bbx = chassisBody.mutable_boundingbox();
  bbx->set_x(1.4);
  bbx->set_y(0.3);
  bbx->set_z(0.2);
}
} // namespace tx_car