#ifndef ECS_H_
#define ECS_H_

#include <iostream>
#include <vector>
#include "ginseng.hpp"

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

// For const access
template <typename Value>
const Value* find_in_flat_map(const std::vector<std::pair<std::string, Value>>& flat_map, std::string_view key)
{
  for (const auto& [k, v] : flat_map)
  {
    if (k == key)
      return &v;
  }
  return nullptr;
}

// For mutable access
template <typename Value>
Value* find_in_flat_map(std::vector<std::pair<std::string, Value>>& flat_map, std::string_view key)
{
  for (auto& [k, v] : flat_map)
  {
    if (k == key)
      return &v;
  }
  return nullptr;
}

template <typename ComponentT>
ComponentT* getOrCreateComponent(ginseng::database& db, EntityID eid)
{
  // Try to get pointer to the component
  ComponentT* comp = db.get_component<ComponentT*>(eid);
  if (!comp)
  {
    // Not found: add a default-constructed component
    db.add_component(eid, ComponentT{});
    // Now, get again (must exist now)
    comp = db.get_component<ComponentT*>(eid);
  }
  return comp;
}

}  // namespace sodf

#endif  // ECS_H_
