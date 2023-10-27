/**
 * @file param_writer.h
 * @author DongYuanHu (dongyuanhu@tencent.com)
 * @brief
 * @version 0.1
 * @date 2023-08-01
 *
 *
 */

#pragma once

#include "SimCore.h"
#include "Vehicle_DW_DW.pb.h"
#include "proto_helper.h"

namespace tx_car {
class ParamWriter_DWDW {
public:
  ParamWriter_DWDW() {}
  void write(tx_car::MBD_Vehicle_DW_DW &vehicle);

private:
  void writeFrontAxle(tx_car::MBD_Vehicle_DW_DW &vehicle);
  void writeRearAxle(tx_car::MBD_Vehicle_DW_DW &vehicle);

  void writeDoubleWishBone_Front(tx_car::MBD_DoubleWishBoneSusp& dw, int flip = 1);
  void writeDoubleWishBone_Rear(tx_car::MBD_DoubleWishBoneSusp& dw, int flip = 1);

  void writeWheels(tx_car::MBD_Vehicle_DW_DW &vehicle);
  void writeChassisBody(tx_car::MBD_Vehicle_DW_DW &vehicle);
};
} // namespace tx_car