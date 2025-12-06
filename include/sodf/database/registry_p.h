#pragma once
#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <utility>
#include <functional>
#include <string>

#include <sodf/components/data_type.h>

namespace sodf {
namespace database {

// Opaque entity id that SODF uses everywhere
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

// Forward-declare a backend tag type. You’ll implement specializations elsewhere.
template <class Backend>
struct ecs_traits;

// Thin wrapper that forwards to ecs_traits<Backend>

template <class Backend>
class Registry
{
public:
  using traits = ecs_traits<Backend>;
  using entity_type = EntityID;

  explicit Registry(Backend& impl) : impl_(impl)
  {
  }

  // entity lifecycle
  entity_type create()
  {
    return traits::create(impl_);
  }
  void destroy(entity_type e)
  {
    traits::destroy(impl_, e);
  }
  void clear()
  {
    traits::clear(impl_);
  }

  // components

  template <class C>
  C& add(entity_type e)
  {
    return traits::template add<C>(impl_, e);
  }
  template <class C>
  std::decay_t<C>& add(entity_type e, C&& value)
  {
    using T = std::decay_t<C>;
    T& ref = traits::template add<T>(impl_, e);  // default-construct in backend
    ref = std::forward<C>(value);                // assign your value
    return ref;
  }
  template <class C>
  bool has(entity_type e)
  {
    return traits::template has<C>(impl_, e);
  }
  template <class C>
  C* get(entity_type e)
  {
    return traits::template get<C>(impl_, e);
  }
  template <class C>
  const C* get_const(entity_type e) const
  {
    return traits::template get<C>(const_cast<Backend&>(impl_), e);
  }
  template <class C>
  C* get_or_add(entity_type e)
  {
    if (auto* p = get<C>(e))
      return p;
    add<C>(e);
    return get<C>(e);
  }
  template <class C>
  void remove(entity_type e)
  {
    traits::template remove<C>(impl_, e);
  }

  // iteration
  template <class Fn>
  void each(Fn&& fn)
  {
    traits::each_deduced(impl_, std::forward<Fn>(fn));
  }

  Backend& impl()
  {
    return impl_;
  }
  const Backend& impl() const
  {
    return impl_;
  }

private:
  Backend& impl_;
};

}  // namespace database
}  // namespace sodf

// Hash for EntityID so it can be used in unordered containers (diff/patch maps)
namespace std {
template <>
struct hash<sodf::database::EntityID>
{
  std::size_t operator()(const sodf::database::EntityID& e) const noexcept
  {
    // A simple, stable combine
    std::size_t h1 = std::hash<std::size_t>{}(e.index);
    std::size_t h2 = std::hash<std::size_t>{}(e.generation);
    return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
  }
};
}  // namespace std
