#pragma once
#include <cstddef>
#include <functional>

namespace sodf {
namespace database {

struct EntityID
{
  std::size_t index{ 0 };
  std::size_t generation{ 0 };

  bool operator==(const EntityID& o) const
  {
    return index == o.index && generation == o.generation;
  }
  bool operator!=(const EntityID& o) const
  {
    return !(*this == o);
  }
};

}  // namespace database
}  // namespace sodf

namespace std {
template <>
struct hash<sodf::database::EntityID>
{
  std::size_t operator()(const sodf::database::EntityID& e) const noexcept
  {
    std::size_t h1 = std::hash<std::size_t>{}(e.index);
    std::size_t h2 = std::hash<std::size_t>{}(e.generation);
    return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
  }
};
}  // namespace std
