/**
 * @file SimCore.h
 * @author DongYuanHu (dongyuanhu@tencent.com)
 * @brief designed as interface of this dll
 * @version 0.1
 * @date 2023-06-30
 *
 *
 */
#pragma once

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "SimBase.h"
#include "SimConstants.h"
#include "SimLog.h"

#ifndef real_T
#define real_T double
#endif // !real_T

namespace tx_car {
/* dump string to file*/
bool dumpToFile(const std::string &content, const std::string &file_path);

/* load string from file path*/
bool loadFromFile(std::string &content, const std::string &file_path);
} // namespace tx_car