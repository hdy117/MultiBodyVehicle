#ifndef PB2JSON_FUNCTION_H
#define PB2JSON_FUNCTION_H

#include <google/protobuf/message.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>

#include "car_basic.pb.h"
#include "SimCore.h"
#include <string>

namespace tx_car {
// protobuf message to json string
SIM_API bool protoToJson(const google::protobuf::Message &message,
                         std::string &json_str);

// json string to protobuf message
SIM_API bool jsonToProto(const std::string &json_str,
                         google::protobuf::Message &message);

// protobuf message to json string
template <class MsgTy>
SIM_API bool protoToJson(const MsgTy &message, std::string &json_str,
                         bool always_print_primitive_fields = true) {
  google::protobuf::util::JsonPrintOptions options;
  options.add_whitespace = true;
  options.always_print_primitive_fields = always_print_primitive_fields;

  json_str.clear();
  return google::protobuf::util::MessageToJsonString(message, &json_str,
                                                     options)
      .ok();
}

// json string to protobuf message
template <class MsgTy>
SIM_API bool jsonToProto(const std::string &json_str, MsgTy &message) {
  return google::protobuf::util::JsonStringToMessage(json_str, &message).ok();
}

// set value of real scalar data
SIM_API void setRealScalar(tx_car::RealScalar *real_scalar,
                           const std::string &disp_name,
                           const std::string &comment, const std::string &unit,
                           double val);

// set value of string scalar data
SIM_API void setStringScalar(tx_car::StringVariable *string_variable,
                             const std::string &disp_name,
                             const std::string &comment,
                             const std::string &val);

// set value of real axis data
SIM_API void setRealAxis(tx_car::RealAxis *real_array,
                         const std::string &disp_name,
                         const std::string &comment, const std::string &unit,
                         double arr_in[], double size);

// set value of 1-d map info
SIM_API void set1DMapInfo(tx_car::Real1DMap *map, const std::string &disp_name,
                          const std::string &comment);

SIM_API void initAxis(real_T out[], const tx_car::RealAxis &in,
                      size_t size = tx_car::const_map1dSize);

// 1-d map checker
SIM_API bool map1DFormatChecker(const tx_car::Real1DMap &map_1d);

// 2-d map checker
SIM_API bool map2DFormatChecker(const tx_car::Real2DMap &map_2d);

// init map, used to set 1-d map used in simulink
SIM_API void initMap1D(const tx_car::Real1DMap &map_1d, real_T bp[],
                       real_T table[], uint32_t &max_index);

// print map 2d, map2d is column order
SIM_API void printMap2d(const tx_car::Real2DMap &map);

// transpose map 2d, map2d is column order
SIM_API tx_car::Real2DMap transposeMap2d(const tx_car::Real2DMap &map);

// get y by row and col index, map2d is column order
SIM_API double getValueFromMap2d(const tx_car::Real2DMap &map, size_t r,
                                 size_t c);

// set y by row and col index, map2d is column order
SIM_API void setValueOfMap2d(tx_car::Real2DMap &map, size_t r, size_t c,
                             double value);

// get one column of map 2d
SIM_API tx_car::RealAxis getColumnOfMap2d(const tx_car::Real2DMap &map,
                                          size_t c);

// get max in real axis
SIM_API double getMaxValueOf(const tx_car::RealAxis &axis);

// get min in real axis
SIM_API double getMinValueOf(const tx_car::RealAxis &axis);

// flip each element in axis and exchange element (i) with (n-i)
SIM_API void flipAndExchangeAxisData(tx_car::RealAxis &yAxis);
} // namespace tx_car

#endif