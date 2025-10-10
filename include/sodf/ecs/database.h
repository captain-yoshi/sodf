#ifndef SODF_ECS_DATABASE_H_
#define SODF_ECS_DATABASE_H_

#include <sodf/ecs/registry_p.h>
#include <sodf/ecs/ginseng_backend.h>

namespace sodf {
namespace ecs {

template <class K, class V>
inline V* get_element(std::vector<std::pair<K, V>>& elements, const K& key)
{
  for (auto& kv : elements)
  {
    auto& k = kv.first;
    if (k == key)
      return &kv.second;
  }
  return nullptr;
}

template <class K, class V>
inline const V* get_element(const std::vector<std::pair<K, V>>& elements, const K& key)
{
  for (const auto& kv : elements)
  {
    const auto& k = kv.first;
    if (k == key)
      return &kv.second;
  }
  return nullptr;
}

template <class V>
inline V* get_element(std::vector<std::pair<std::string, V>>& elements, std::string_view key)
{
  for (auto& kv : elements)
  {
    auto& k = kv.first;
    if (k == key)
      return &kv.second;
  }
  return nullptr;
}

template <class V>
inline const V* get_element(const std::vector<std::pair<std::string, V>>& elements, std::string_view key)
{
  for (const auto& kv : elements)
  {
    const auto& k = kv.first;
    if (k == key)
      return &kv.second;
  }
  return nullptr;
}

// Public-friendly name
// Owns the backend so you can write `ecs::Database db;`
class Database : public Registry<GinsengBackend>
{
public:
  Database() : Registry<GinsengBackend>(backend_)
  {
  }

  GinsengBackend& backend()
  {
    return backend_;
  }
  const GinsengBackend& backend() const
  {
    return backend_;
  }

  template <class C, class Key>
  auto* get_element(entity_type e, const Key& key)
  {
    if (auto* c = this->template get<C>(e))
      return ecs::get_element(c->elements, key);

    using ElemContainer = decltype(std::declval<C&>().elements);
    using ElemPair = typename ElemContainer::value_type;
    using Elem = typename ElemPair::second_type;
    return static_cast<Elem*>(nullptr);
  }

  template <class C, class Key>
  const auto* get_element(entity_type e, const Key& key) const
  {
    if (const auto* c = this->template get_const<C>(e))
      return ecs::get_element(c->elements, key);

    using ElemContainer = decltype(std::declval<const C&>().elements);
    using ElemPair = typename ElemContainer::value_type;
    using Elem = typename ElemPair::second_type;
    return static_cast<const Elem*>(nullptr);
  }

  // Convenience overloads (forward to the template above)
  template <class C>
  auto* get_element(entity_type e, const char* key)
  {
    return get_element<C>(e, std::string_view(key));
  }
  template <class C>
  const auto* get_element(entity_type e, const char* key) const
  {
    return get_element<C>(e, std::string_view(key));
  }

private:
  GinsengBackend backend_;
};

using ObjectEntityMap = std::unordered_map<std::string, Database::entity_type>;

}  // namespace ecs
}  // namespace sodf

#endif  // SODF_ECS_DATABASE_H_
