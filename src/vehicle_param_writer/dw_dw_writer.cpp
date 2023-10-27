#include "param_writer.h"

int main(int argc, char **argv) {
  tx_car::ParamWriter_DWDW writer_DWDW;
  tx_car::MBD_Vehicle_DW_DW vehicle_DWDW;
  writer_DWDW.write(vehicle_DWDW);
  std::string jsonConent;
  tx_car::protoToJson(vehicle_DWDW, jsonConent);
  tx_car::dumpToFile(jsonConent, "./param/vehicle_dwdw.json");
  return 0;
}
