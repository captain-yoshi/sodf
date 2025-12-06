#ifndef SODF_DATABASE_DATABASE_H_
#define SODF_DATABASE_DATABASE_H_

#include <sodf/database/registry_p.h>
#include <sodf/database/ginseng_backend.h>

#include <unordered_map>
#include <typeindex>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include <type_traits>

namespace sodf {
namespace database {

class DatabaseDiff;  // forward declaration for multi-level diff bases

/// ---------------------------------------------------------------------------
/// ElementMap helpers
/// ---------------------------------------------------------------------------
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

/// ---------------------------------------------------------------------------
/// Database interface
/// ---------------------------------------------------------------------------
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
      return database::get_element(c->elements, key);

    using ElemContainer = decltype(std::declval<C&>().elements);
    using ElemPair = typename ElemContainer::value_type;
    using Elem = typename ElemPair::second_type;
    return static_cast<Elem*>(nullptr);
  }

  template <class C, class Key>
  const auto* get_element(entity_type e, const Key& key) const
  {
    if (const auto* c = this->template get_const<C>(e))
      return database::get_element(c->elements, key);

    using ElemContainer = decltype(std::declval<const C&>().elements);
    using ElemPair = typename ElemContainer::value_type;
    using Elem = typename ElemPair::second_type;
    return static_cast<const Elem*>(nullptr);
  }

  // Convenience overloads
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

/// ---------------------------------------------------------------------------
/// Internal diff machinery (not part of the public API)
/// ---------------------------------------------------------------------------
namespace detail {

// Traits to extract key/value types from a component's ElementMap
template <class C>
struct element_traits
{
  using container_t = decltype(std::declval<C&>().elements);
  using pair_t = typename container_t::value_type;
  using key_t = typename pair_t::first_type;
  using value_t = typename pair_t::second_type;
};

// Normalize keys for patch storage / lookup.
// Needed because GCC9/C++17 std::unordered_map doesn't support heterogeneous
// lookup for std::string keys with std::string_view.
template <class KeyT, class Key>
inline KeyT normalize_key(const Key& key)
{
  if constexpr (std::is_same_v<KeyT, std::string>)
  {
    if constexpr (std::is_same_v<std::decay_t<Key>, std::string_view>)
      return std::string(key);
    else if constexpr (std::is_same_v<std::decay_t<Key>, const char*>)
      return std::string(key);
    else
      return KeyT(key);
  }
  else
  {
    return KeyT(key);
  }
}

// Basic patch op
enum class PatchOpKind
{
  AddOrReplace,
  Remove
};

template <class V>
struct PatchOp
{
  PatchOpKind kind{ PatchOpKind::AddOrReplace };
  V value{};
};

// Type-erased base for component patches
struct IComponentPatch
{
  virtual ~IComponentPatch() = default;
};

// Concrete patch for a specific component type C
template <class C>
struct ComponentPatch : IComponentPatch
{
  using key_t = typename element_traits<C>::key_t;
  using value_t = typename element_traits<C>::value_t;

  // Per entity → per key → op
  std::unordered_map<EntityID, std::unordered_map<key_t, PatchOp<value_t>>> ops;
};

// A collection of patches across arbitrary component types
class DatabasePatch
{
public:
  DatabasePatch() = default;

  template <class C>
  ComponentPatch<C>& get_or_create()
  {
    const auto ti = std::type_index(typeid(C));
    auto it = patches_.find(ti);
    if (it != patches_.end())
      return *static_cast<ComponentPatch<C>*>(it->second.get());

    auto ptr = std::make_unique<ComponentPatch<C>>();
    auto* raw = ptr.get();
    patches_.emplace(ti, std::move(ptr));
    return *raw;
  }

  template <class C>
  const ComponentPatch<C>* get() const
  {
    const auto ti = std::type_index(typeid(C));
    auto it = patches_.find(ti);
    if (it == patches_.end())
      return nullptr;
    return static_cast<const ComponentPatch<C>*>(it->second.get());
  }

  bool empty() const
  {
    return patches_.empty();
  }

  void clear()
  {
    patches_.clear();
  }

  // Convenience authoring API:
  template <class C, class Key, class Value>
  void add_or_replace(Database::entity_type e, Key&& key, Value&& value)
  {
    auto& cp = get_or_create<C>();
    using value_t = typename element_traits<C>::value_t;
    using key_t = typename element_traits<C>::key_t;

    PatchOp<value_t> op;
    op.kind = PatchOpKind::AddOrReplace;
    op.value = std::forward<Value>(value);

    cp.ops[e][normalize_key<key_t>(key)] = std::move(op);
  }

  template <class C, class Key>
  void remove(Database::entity_type e, Key&& key)
  {
    auto& cp = get_or_create<C>();
    using value_t = typename element_traits<C>::value_t;
    using key_t = typename element_traits<C>::key_t;

    PatchOp<value_t> op;
    op.kind = PatchOpKind::Remove;

    cp.ops[e][normalize_key<key_t>(key)] = std::move(op);
  }

private:
  std::unordered_map<std::type_index, std::unique_ptr<IComponentPatch>> patches_;
};

/// ---------------------------------------------------------------------------
/// Diff base indirection (Database OR DatabaseDiff)
/// ---------------------------------------------------------------------------
class DiffBase
{
public:
  using entity_type = Database::entity_type;

  DiffBase(const Database& db) : db_(&db), diff_(nullptr)
  {
  }
  DiffBase(const DatabaseDiff& d) : db_(nullptr), diff_(&d)
  {
  }

  const Database& ultimate_database() const;

  template <class C>
  const C* get_const(entity_type e) const;

  template <class C, class Key>
  const auto* get_element(entity_type e, const Key& key) const;

private:
  const Database* db_;
  const DatabaseDiff* diff_;
};

/// ---------------------------------------------------------------------------
/// Read-only overlay view (internal).
/// ---------------------------------------------------------------------------
class PatchedDatabaseView
{
public:
  using entity_type = Database::entity_type;

  PatchedDatabaseView(const DiffBase& base, const DatabasePatch& patch) : base_(base), patch_(patch)
  {
  }

  template <class C>
  const C* get_const(entity_type e) const
  {
    return base_.template get_const<C>(e);
  }

  template <class C, class Key>
  const auto* get_element(entity_type e, const Key& key) const
  {
    using value_t = typename element_traits<C>::value_t;
    using key_t = typename element_traits<C>::key_t;

    if (const auto* cp = patch_.template get<C>())
    {
      auto itE = cp->ops.find(e);
      if (itE != cp->ops.end())
      {
        auto norm_key = normalize_key<key_t>(key);
        auto itK = itE->second.find(norm_key);
        if (itK != itE->second.end())
        {
          if (itK->second.kind == PatchOpKind::Remove)
            return static_cast<const value_t*>(nullptr);
          return &itK->second.value;
        }
      }
    }
    return base_.template get_element<C>(e, key);
  }

  template <class C>
  const auto* get_element(entity_type e, const char* key) const
  {
    return get_element<C>(e, std::string_view(key));
  }

private:
  const DiffBase& base_;
  const DatabasePatch& patch_;
};

}  // namespace detail

/// ---------------------------------------------------------------------------
/// Public façade: DatabaseDiff
/// ---------------------------------------------------------------------------
/// DatabaseDiff owns internal patch state and exposes a small authoring + read API.
/// It can be layered over either a Database or another DatabaseDiff.
class DatabaseDiff
{
public:
  using entity_type = Database::entity_type;

  explicit DatabaseDiff(const Database& base) : base_(base), patch_(), view_(base_, patch_)
  {
  }

  explicit DatabaseDiff(const DatabaseDiff& parent) : base_(parent), patch_(), view_(base_, patch_)
  {
  }

  // Keep the old-looking API: returns the ultimate Database.
  const Database& base() const
  {
    return base_.ultimate_database();
  }

  // If you ever need to explicitly access the root database.
  const Database& base_database() const
  {
    return base_.ultimate_database();
  }

  // -------------------------------------------------------------------------
  // Read API (patched)
  // -------------------------------------------------------------------------
  template <class C>
  const C* get_const(entity_type e) const
  {
    return view_.template get_const<C>(e);
  }

  template <class C, class Key>
  const auto* get_element(entity_type e, const Key& key) const
  {
    return view_.template get_element<C>(e, key);
  }

  template <class C>
  const auto* get_element(entity_type e, const char* key) const
  {
    return view_.template get_element<C>(e, key);
  }

  // -------------------------------------------------------------------------
  // Authoring API
  // -------------------------------------------------------------------------
  template <class C, class Key, class Value>
  void add_or_replace(entity_type e, Key&& key, Value&& value)
  {
    patch_.template add_or_replace<C>(e, std::forward<Key>(key), std::forward<Value>(value));
  }

  template <class C, class Key>
  void remove(entity_type e, Key&& key)
  {
    patch_.template remove<C>(e, std::forward<Key>(key));
  }

  bool empty() const
  {
    return patch_.empty();
  }

  void clear()
  {
    patch_.clear();
  }

private:
  detail::DiffBase base_;
  detail::DatabasePatch patch_;
  detail::PatchedDatabaseView view_;
};

/// ---------------------------------------------------------------------------
/// detail::DiffBase inline definitions (after DatabaseDiff is complete)
/// ---------------------------------------------------------------------------
namespace detail {

inline const Database& DiffBase::ultimate_database() const
{
  if (db_)
    return *db_;
  return diff_->base_database();
}

template <class C>
inline const C* DiffBase::get_const(entity_type e) const
{
  if (db_)
    return db_->template get_const<C>(e);
  return diff_->template get_const<C>(e);
}

template <class C, class Key>
inline const auto* DiffBase::get_element(entity_type e, const Key& key) const
{
  if (db_)
    return db_->template get_element<C>(e, key);
  return diff_->template get_element<C>(e, key);
}

}  // namespace detail

}  // namespace database
}  // namespace sodf

#endif  // SODF_DATABASE_DATABASE_H_
