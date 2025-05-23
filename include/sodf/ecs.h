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

// /// Wrapper for Ginseng’s ent_id
// class EntityID
// {
// public:
//   // Default constructor: invalid entity
//   EntityID() = default;

//   // Construct from ginseng::ent_id
//   explicit EntityID(ginseng::ent_id id) : id_(id)
//   {
//   }

//   // Accessors
//   ginseng::ent_id::index_type index() const
//   {
//     return id_.get_index();
//   }

//   const ginseng::ent_id& raw() const
//   {
//     return id_;
//   }

//   // Comparison operators
//   bool operator==(const EntityID& other) const
//   {
//     return id_ == other.id_;
//   }
//   bool operator!=(const EntityID& other) const
//   {
//     return !(*this == other);
//   }

//   // Stream output
//   friend std::ostream& operator<<(std::ostream& os, const EntityID& e)
//   {
//     return os << "EntityID(index=" << e.id_.get_index() << ")";
//   }

// private:
//   ginseng::ent_id id_;
// };

}  // namespace sodf

#endif  // ECS_H_
