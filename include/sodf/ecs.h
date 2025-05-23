#ifndef ECS_H_
#define ECS_H_

#include <iostream>
#include <vector>
#include <ginseng.hpp>

namespace sodf {

/// For storing a collection of components
template <typename K, typename V>
using FlatMap = std::vector<std::pair<K, V>>;

using EntityID = ginseng::database::ent_id;

using ObjectEntityMap = std::unordered_map<std::string, EntityID>;

// For mutable access
template <typename Key, typename Value>
Value* find_in_flat_map(std::vector<std::pair<Key, Value>>& flat_map, const Key& key)
{
  for (auto& [k, v] : flat_map)
  {
    if (k == key)
      return &v;
  }
  return nullptr;
}

// For const access
template <typename Key, typename Value>
const Value* find_in_flat_map(const std::vector<std::pair<Key, Value>>& flat_map, const Key& key)
{
  for (const auto& [k, v] : flat_map)
  {
    if (k == key)
      return &v;
  }
  return nullptr;
}

}  // namespace sodf

#endif  // ECS_H_
