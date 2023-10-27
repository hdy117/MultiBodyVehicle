#pragma once

#ifdef _WIN32
#ifdef SIM_BUILD_DLL
#define SIM_API __declspec(dllexport)
#else
#define SIM_API __declspec(dllimport)
#endif
#else
#define SIM_API
#endif

namespace tx_car {}