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

/**
 * @brief Retrieve or create a component for the given entity.
 *
 * Attempts to get a mutable pointer to the specified component type for the given entity.
 * If the component is not present, it adds a default-constructed instance to the entity
 * and returns a pointer to the newly added component.
 *
 * @tparam ComponentT The type of the component to retrieve or create.
 * @param db The ginseng database containing all entities and components.
 * @param eid The entity ID to query.
 * @return A mutable pointer to the component (existing or newly created), or nullptr in case of failure.
 */
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

/**
 * @brief Retrieve an element from a FlatMap (vector of key-value pairs) by key.
 *
 * Performs a linear search over the flat map to find the value associated with the given key.
 * Works with both custom and standard string keys. Returns a pointer to the value if found,
 * or nullptr if the key does not exist.
 *
 * @tparam Key The key type (e.g., std::string or std::string_view).
 * @tparam Value The value type stored in the map.
 * @param flat_map The flat map (vector of key-value pairs).
 * @param key The key to search for.
 * @return Pointer to the value if found, or nullptr otherwise.
 */
template <typename Key, typename Value>
Value* getComponentElement(std::vector<std::pair<Key, Value>>& flat_map, const Key& key)
{
  for (auto& [k, v] : flat_map)
  {
    if (k == key)
      return &v;
  }
  return nullptr;
}

template <typename Key, typename Value>
const Value* getComponentElement(const std::vector<std::pair<Key, Value>>& flat_map, const Key& key)
{
  for (const auto& [k, v] : flat_map)
  {
    if (k == key)
      return &v;
  }
  return nullptr;
}

template <typename Value>
Value* getComponentElement(std::vector<std::pair<std::string, Value>>& flat_map, std::string_view key)
{
  for (auto& [k, v] : flat_map)
  {
    if (k == key)
      return &v;
  }
  return nullptr;
}

template <typename Value>
const Value* getComponentElement(const std::vector<std::pair<std::string, Value>>& flat_map, std::string_view key)
{
  for (const auto& [k, v] : flat_map)
  {
    if (k == key)
      return &v;
  }
  return nullptr;
}

/**
 * @brief Retrieve an element from a component's internal map by key.
 *
 * Looks up a specific element within a component that exposes a `map` field
 * (typically a FlatMap). Returns a pointer to the value if the component and key exist,
 * or nullptr otherwise.
 *
 * @tparam ComponentT The type of the component (must have a `.map` field).
 * @param db The ECS database.
 * @param eid The entity ID owning the component.
 * @param key The lookup key (e.g., string identifier).
 * @return Pointer to the element value, or nullptr if not found.
 */
template <typename ComponentT>
auto* getComponentElement(ginseng::database& db, sodf::EntityID eid, const std::string& key)
{
  ComponentT* comp = db.get_component<ComponentT*>(eid);
  if (!comp)
    return static_cast<typename decltype(comp->map)::value_type::second_type*>(nullptr);

  return getComponentElement(comp->map, key);
}

template <typename ComponentT>
auto* getComponentElement(ginseng::database& db, sodf::EntityID eid, std::string_view key)
{
  ComponentT* comp = db.get_component<ComponentT*>(eid);
  if (!comp)
    return static_cast<typename decltype(comp->map)::value_type::second_type*>(nullptr);

  return getComponentElement(comp->map, key);
}

template <typename ComponentT>
const auto* getComponentElement(const ginseng::database& db, sodf::EntityID eid, std::string_view key)
{
  const ComponentT* comp = db.get_component<const ComponentT*>(eid);
  if (!comp)
    return static_cast<const typename decltype(comp->map)::value_type::second_type*>(nullptr);

  return getComponentElement(comp->map, key);
}

template <typename ComponentT>
const auto* getComponentElement(const ginseng::database& db, sodf::EntityID eid, const std::string& key)
{
  const ComponentT* comp = db.get_component<const ComponentT*>(eid);
  if (!comp)
    return static_cast<const typename decltype(comp->map)::value_type::second_type*>(nullptr);

  return getComponentElement(comp->map, key);
}
}  // namespace sodf

#endif  // ECS_H_
