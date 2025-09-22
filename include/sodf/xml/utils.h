#ifndef UTILS_H_
#define UTILS_H_

#include <string>
#include <filesystem>

namespace sodf {
namespace xml {

inline std::string getDirectory(const std::string& filepath)
{
  return std::filesystem::path(filepath).parent_path().string();
}

// Helper function for unit conversion:
inline double convertVolumeToSI(double volume, const std::string& units)
{
  if (units == "uL")
    return volume * 1e-9;
  if (units == "mL")
    return volume * 1e-6;
  if (units == "L")
    return volume * 1e-3;
  return volume;  // assume already in m^3
}

}  // namespace xml
}  // namespace sodf
#endif  // UTILS_H_
