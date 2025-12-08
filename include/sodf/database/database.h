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
#include <cstddef>
#include <optional>
#include <algorithm>

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
/// Internal traits declared EARLY so Database can gate overloads cleanly.
/// ---------------------------------------------------------------------------
namespace detail {

// Detect if component has an "elements" member
template <typename, typename = void>
struct has_elements : std::false_type
{
};
template <typename T>
struct has_elements<T, std::void_t<decltype(std::declval<T&>().elements)>> : std::true_type
{
};
template <typename T>
inline constexpr bool has_elements_v = has_elements<T>::value;

}  // namespace detail

/// ---------------------------------------------------------------------------
/// Database interface
/// ---------------------------------------------------------------------------
class Database : public Registry<GinsengBackend>
{
public:
  using entity_type = Registry<GinsengBackend>::entity_type;

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

  // -------------------------------------------------------------------------
  // Element-level convenience getters (ONLY for components with .elements)
  // -------------------------------------------------------------------------
  template <class C, class Key, typename std::enable_if<detail::has_elements_v<C>, int>::type = 0>
  auto* get_element(entity_type e, const Key& key)
  {
    if (auto* c = this->template get<C>(e))
      return database::get_element(c->elements, key);

    using ElemContainer = decltype(std::declval<C&>().elements);
    using ElemPair = typename ElemContainer::value_type;
    using Elem = typename ElemPair::second_type;
    return static_cast<Elem*>(nullptr);
  }

  template <class C, class Key, typename std::enable_if<detail::has_elements_v<C>, int>::type = 0>
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
  template <class C, typename std::enable_if<detail::has_elements_v<C>, int>::type = 0>
  auto* get_element(entity_type e, const char* key)
  {
    return get_element<C>(e, std::string_view(key));
  }

  template <class C, typename std::enable_if<detail::has_elements_v<C>, int>::type = 0>
  const auto* get_element(entity_type e, const char* key) const
  {
    return get_element<C>(e, std::string_view(key));
  }

private:
  GinsengBackend backend_;
};

using EntityID = Database::entity_type;
using ObjectEntityMap = std::unordered_map<std::string, EntityID>;

/// ---------------------------------------------------------------------------
/// Internal diff machinery (not part of the public API)
/// ---------------------------------------------------------------------------
namespace detail {

using EntityID = database::EntityID;

// Traits to extract key/value types from a component's ElementMap
template <class C, typename std::enable_if<has_elements_v<C>, int>::type = 0>
struct element_traits
{
  using container_t = decltype(std::declval<C&>().elements);
  using pair_t = typename container_t::value_type;
  using key_t = typename pair_t::first_type;
  using value_t = typename pair_t::second_type;

  // Store an owned key in patches if the component uses string_view keys.
  using key_storage_t = std::conditional_t<std::is_same_v<key_t, std::string_view>, std::string, key_t>;
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

// IMPORTANT: optional value to avoid default-constructing V
template <class V>
struct PatchOp
{
  PatchOpKind kind{ PatchOpKind::AddOrReplace };
  std::optional<V> value;
};

// Type-erased base for component patches
struct IComponentPatch
{
  virtual ~IComponentPatch() = default;
};

// ---------------------------------------------------------------------------
// Element-level patch (components with .elements)
// NOW ALSO supports whole-component ops for these components.
// ---------------------------------------------------------------------------
template <class C, typename std::enable_if<has_elements_v<C>, int>::type = 0>
struct ComponentPatchElements : IComponentPatch
{
  using key_t = typename element_traits<C>::key_t;
  using key_storage_t = typename element_traits<C>::key_storage_t;
  using value_t = typename element_traits<C>::value_t;

  // Per entity → per key → op (element-level ops)
  std::unordered_map<EntityID, std::unordered_map<key_storage_t, PatchOp<value_t>>> element_ops;

  // Per entity → op (whole-component override/remove)
  std::unordered_map<EntityID, PatchOp<C>> whole_ops;
};

// ---------------------------------------------------------------------------
// Component-level patch (components WITHOUT .elements)
// ---------------------------------------------------------------------------
template <class C, typename std::enable_if<!has_elements_v<C>, int>::type = 0>
struct ComponentPatchWhole : IComponentPatch
{
  // Per entity → op
  std::unordered_map<EntityID, PatchOp<C>> ops;
};

// A collection of patches across arbitrary component types
class DatabasePatch
{
public:
  DatabasePatch() = default;

  // ---------- elements components ----------
  template <class C, typename std::enable_if<has_elements_v<C>, int>::type = 0>
  ComponentPatchElements<C>& get_or_create()
  {
    const auto ti = std::type_index(typeid(C));
    auto it = patches_.find(ti);
    if (it != patches_.end())
      return *static_cast<ComponentPatchElements<C>*>(it->second.get());

    auto ptr = std::make_unique<ComponentPatchElements<C>>();
    auto* raw = ptr.get();
    patches_.emplace(ti, std::move(ptr));
    return *raw;
  }

  template <class C, typename std::enable_if<has_elements_v<C>, int>::type = 0>
  const ComponentPatchElements<C>* get() const
  {
    const auto ti = std::type_index(typeid(C));
    auto it = patches_.find(ti);
    if (it == patches_.end())
      return nullptr;
    return static_cast<const ComponentPatchElements<C>*>(it->second.get());
  }

  // ---------- whole components ----------
  template <class C, typename std::enable_if<!has_elements_v<C>, int>::type = 0>
  ComponentPatchWhole<C>& get_or_create()
  {
    const auto ti = std::type_index(typeid(C));
    auto it = patches_.find(ti);
    if (it != patches_.end())
      return *static_cast<ComponentPatchWhole<C>*>(it->second.get());

    auto ptr = std::make_unique<ComponentPatchWhole<C>>();
    auto* raw = ptr.get();
    patches_.emplace(ti, std::move(ptr));
    return *raw;
  }

  template <class C, typename std::enable_if<!has_elements_v<C>, int>::type = 0>
  const ComponentPatchWhole<C>* get() const
  {
    const auto ti = std::type_index(typeid(C));
    auto it = patches_.find(ti);
    if (it == patches_.end())
      return nullptr;
    return static_cast<const ComponentPatchWhole<C>*>(it->second.get());
  }

  bool empty() const
  {
    return patches_.empty();
  }

  void clear()
  {
    patches_.clear();
  }

  // -------------------------------------------------------------------------
  // Convenience authoring API (elements: per-element)
  // -------------------------------------------------------------------------
  template <class C, class Key, class Value, typename std::enable_if<has_elements_v<C>, int>::type = 0>
  void add_or_replace(EntityID e, Key&& key, Value&& value)
  {
    auto& cp = get_or_create<C>();
    using value_t = typename element_traits<C>::value_t;
    using key_storage_t = typename element_traits<C>::key_storage_t;

    PatchOp<value_t> op;
    op.kind = PatchOpKind::AddOrReplace;
    op.value = std::forward<Value>(value);

    cp.element_ops[e][normalize_key<key_storage_t>(key)] = std::move(op);
  }

  template <class C, class Key, typename std::enable_if<has_elements_v<C>, int>::type = 0>
  void remove(EntityID e, Key&& key)
  {
    auto& cp = get_or_create<C>();
    using value_t = typename element_traits<C>::value_t;
    using key_storage_t = typename element_traits<C>::key_storage_t;

    PatchOp<value_t> op;
    op.kind = PatchOpKind::Remove;
    op.value.reset();

    cp.element_ops[e][normalize_key<key_storage_t>(key)] = std::move(op);
  }

  // -------------------------------------------------------------------------
  // Convenience authoring API (elements: whole-component)
  // -------------------------------------------------------------------------
  template <class C, class Value, typename std::enable_if<has_elements_v<C>, int>::type = 0>
  void add_or_replace_component(EntityID e, Value&& value)
  {
    auto& cp = get_or_create<C>();

    PatchOp<C> op;
    op.kind = PatchOpKind::AddOrReplace;
    op.value = std::forward<Value>(value);

    cp.whole_ops[e] = std::move(op);
  }

  template <class C, typename std::enable_if<has_elements_v<C>, int>::type = 0>
  void remove_component(EntityID e)
  {
    auto& cp = get_or_create<C>();

    PatchOp<C> op;
    op.kind = PatchOpKind::Remove;
    op.value.reset();

    cp.whole_ops[e] = std::move(op);
  }

  // -------------------------------------------------------------------------
  // Convenience authoring API (whole component WITHOUT .elements)
  // -------------------------------------------------------------------------
  template <class C, class Value, typename std::enable_if<!has_elements_v<C>, int>::type = 0>
  void add_or_replace(EntityID e, Value&& value)
  {
    auto& cp = get_or_create<C>();

    PatchOp<C> op;
    op.kind = PatchOpKind::AddOrReplace;
    op.value = std::forward<Value>(value);

    cp.ops[e] = std::move(op);
  }

  template <class C, typename std::enable_if<!has_elements_v<C>, int>::type = 0>
  void remove_component(EntityID e)
  {
    auto& cp = get_or_create<C>();

    PatchOp<C> op;
    op.kind = PatchOpKind::Remove;
    op.value.reset();

    cp.ops[e] = std::move(op);
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

  template <class C, class Key, typename std::enable_if<has_elements_v<C>, int>::type = 0>
  const typename element_traits<C>::value_t* get_element(entity_type e, const Key& key) const;

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

  // -------------------------------------------------------------------------
  // Whole-component get_const with patch overlay
  // -------------------------------------------------------------------------
  template <class C>
  const C* get_const(entity_type e) const
  {
    // Elements components: check whole override/remove first
    if constexpr (has_elements_v<C>)
    {
      if (const auto* cp = patch_.template get<C>())
      {
        auto it = cp->whole_ops.find(e);
        if (it != cp->whole_ops.end())
        {
          if (it->second.kind == PatchOpKind::Remove)
            return static_cast<const C*>(nullptr);
          if (it->second.value)
            return &(*it->second.value);
        }
      }
      // No whole op -> fall through to base
      return base_.template get_const<C>(e);
    }
    else
    {
      // Non-elements components: existing whole patch channel
      if (const auto* cp = patch_.template get<C>())
      {
        auto it = cp->ops.find(e);
        if (it != cp->ops.end())
        {
          if (it->second.kind == PatchOpKind::Remove)
            return static_cast<const C*>(nullptr);
          if (it->second.value)
            return &(*it->second.value);
        }
      }
      return base_.template get_const<C>(e);
    }
  }

  // -------------------------------------------------------------------------
  // Element-level get_element with patch overlay
  // -------------------------------------------------------------------------
  template <class C, class Key, typename std::enable_if<has_elements_v<C>, int>::type = 0>
  const typename element_traits<C>::value_t* get_element(entity_type e, const Key& key) const
  {
    using value_t = typename element_traits<C>::value_t;
    using key_storage_t = typename element_traits<C>::key_storage_t;

    if (const auto* cp = patch_.template get<C>())
    {
      // 1) Whole-component override/remove takes precedence
      auto itW = cp->whole_ops.find(e);
      if (itW != cp->whole_ops.end())
      {
        if (itW->second.kind == PatchOpKind::Remove)
          return static_cast<const value_t*>(nullptr);
        if (itW->second.value)
        {
          // Look up inside the overridden component's elements
          return database::get_element(itW->second.value->elements, key);
        }
      }

      // 2) Element-level op overlay
      auto itE = cp->element_ops.find(e);
      if (itE != cp->element_ops.end())
      {
        auto norm_key = normalize_key<key_storage_t>(key);
        auto itK = itE->second.find(norm_key);
        if (itK != itE->second.end())
        {
          if (itK->second.kind == PatchOpKind::Remove)
            return static_cast<const value_t*>(nullptr);
          if (itK->second.value)
            return &(*itK->second.value);
          return static_cast<const value_t*>(nullptr);
        }
      }
    }

    return base_.template get_element<C>(e, key);
  }

  template <class C, typename std::enable_if<has_elements_v<C>, int>::type = 0>
  const typename element_traits<C>::value_t* get_element(entity_type e, const char* key) const
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
///
/// NOTE: A child DatabaseDiff must not outlive its parent diff (layered lifetime).
class DatabaseDiff
{
public:
  using entity_type = Database::entity_type;

  explicit DatabaseDiff(const Database& base) : base_(base), patch_(), view_(base_, patch_), revision_(0)
  {
  }

  explicit DatabaseDiff(const DatabaseDiff& parent) : base_(parent), patch_(), view_(base_, patch_), revision_(0)
  {
  }

  // Keep the old-looking API: returns the ultimate Database.
  const Database& base() const
  {
    return base_.ultimate_database();
  }

  const Database& base_database() const
  {
    return base_.ultimate_database();
  }

  std::size_t revision() const
  {
    return revision_;
  }

  // Read-only iteration view (for generic helpers)
  template <class Fn>
  void each(Fn&& fn) const
  {
    base_.ultimate_database().each(std::forward<Fn>(fn));
  }

  // -------------------------------------------------------------------------
  // Read API (patched)
  // -------------------------------------------------------------------------
  template <class C>
  const C* get_const(entity_type e) const
  {
    return view_.template get_const<C>(e);
  }

  // Element-level only
  template <class C, class Key, typename std::enable_if<detail::has_elements_v<C>, int>::type = 0>
  const typename detail::element_traits<C>::value_t* get_element(entity_type e, const Key& key) const
  {
    return view_.template get_element<C>(e, key);
  }

  template <class C, typename std::enable_if<detail::has_elements_v<C>, int>::type = 0>
  const typename detail::element_traits<C>::value_t* get_element(entity_type e, const char* key) const
  {
    return view_.template get_element<C>(e, key);
  }

  // -------------------------------------------------------------------------
  // Authoring API (elements: per-element)
  // -------------------------------------------------------------------------
  template <class C, class Key, class Value, typename std::enable_if<detail::has_elements_v<C>, int>::type = 0>
  void add_or_replace(entity_type e, Key&& key, Value&& value)
  {
    patch_.template add_or_replace<C>(e, std::forward<Key>(key), std::forward<Value>(value));
    ++revision_;
  }

  template <class C, class Key, typename std::enable_if<detail::has_elements_v<C>, int>::type = 0>
  void remove(entity_type e, Key&& key)
  {
    patch_.template remove<C>(e, std::forward<Key>(key));
    ++revision_;
  }

  // -------------------------------------------------------------------------
  // Authoring API (elements: whole-component)
  // -------------------------------------------------------------------------
  template <class C, class Value, typename std::enable_if<detail::has_elements_v<C>, int>::type = 0>
  void add_or_replace_component(entity_type e, Value&& value)
  {
    patch_.template add_or_replace_component<C>(e, std::forward<Value>(value));
    ++revision_;
  }

  template <class C, typename std::enable_if<detail::has_elements_v<C>, int>::type = 0>
  void remove_component(entity_type e)
  {
    patch_.template remove_component<C>(e);
    ++revision_;
  }

  // -------------------------------------------------------------------------
  // Authoring API (whole component WITHOUT .elements)
  // -------------------------------------------------------------------------
  template <class C, class Value, typename std::enable_if<!detail::has_elements_v<C>, int>::type = 0>
  void add_or_replace(entity_type e, Value&& value)
  {
    patch_.template add_or_replace<C>(e, std::forward<Value>(value));
    ++revision_;
  }

  template <class C, typename std::enable_if<!detail::has_elements_v<C>, int>::type = 0>
  void remove_component(entity_type e)
  {
    patch_.template remove_component<C>(e);
    ++revision_;
  }

  bool empty() const
  {
    return patch_.empty();
  }

  void clear()
  {
    patch_.clear();
    ++revision_;
  }

  // -------------------------------------------------------------------------
  // Internal-ish helpers (needed for materialization)
  // -------------------------------------------------------------------------

  // Enumerate ELEMENT patch ops for a specific ELEMENT component type on one entity.
  // Fn signature:
  //   void(const key_t& key, detail::PatchOpKind kind, const value_t* value_or_null)
  template <class C, class Fn, typename std::enable_if<detail::has_elements_v<C>, int>::type = 0>
  void for_each_patch_op(entity_type e, Fn&& fn) const
  {
    using key_t = typename detail::element_traits<C>::key_t;
    using key_storage_t = typename detail::element_traits<C>::key_storage_t;
    using value_t = typename detail::element_traits<C>::value_t;

    const auto* cp = patch_.template get<C>();
    if (!cp)
      return;

    auto itE = cp->element_ops.find(e);
    if (itE == cp->element_ops.end())
      return;

    auto to_key_t = [](const key_storage_t& ks) -> key_t {
      if constexpr (std::is_same_v<key_t, std::string_view>)
      {
        return std::string_view(ks);
      }
      else
      {
        return ks;
      }
    };

    for (const auto& kv : itE->second)
    {
      const key_storage_t& skey = kv.first;
      const auto& op = kv.second;

      key_t key = to_key_t(skey);

      if (op.kind == detail::PatchOpKind::Remove)
      {
        fn(key, detail::PatchOpKind::Remove, static_cast<const value_t*>(nullptr));
      }
      else
      {
        const value_t* vptr = op.value ? &(*op.value) : nullptr;
        fn(key, detail::PatchOpKind::AddOrReplace, vptr);
      }
    }
  }

  // Materialize an ELEMENT component by:
  //  1) If whole patch exists:
  //       - Remove -> false
  //       - Replace -> copy patched value
  //     else:
  //       - copy base if present, otherwise default-construct (create-if-missing)
  //  2) Apply element-level ops on top.
  //
  // This ENABLES create-if-missing semantics for elements-based components.
  template <class C, typename std::enable_if<detail::has_elements_v<C>, int>::type = 0>
  bool materialize_component(entity_type e, C& out) const
  {
    const auto* cp = patch_.template get<C>();

    // 1) Whole override/remove
    if (cp)
    {
      auto itW = cp->whole_ops.find(e);
      if (itW != cp->whole_ops.end())
      {
        if (itW->second.kind == detail::PatchOpKind::Remove)
          return false;
        if (itW->second.value)
          out = *itW->second.value;
        else
          out = C{};
      }
      else
      {
        const C* base_c = base_.template get_const<C>(e);
        if (base_c)
          out = *base_c;
        else
          out = C{};
      }
    }
    else
    {
      const C* base_c = base_.template get_const<C>(e);
      if (base_c)
        out = *base_c;
      else
        out = C{};
    }

    using key_t = typename detail::element_traits<C>::key_t;
    using value_t = typename detail::element_traits<C>::value_t;

    auto erase_key = [&](const key_t& k) {
      auto& elems = out.elements;
      elems.erase(std::remove_if(elems.begin(), elems.end(), [&](const auto& p) { return p.first == k; }), elems.end());
    };

    auto upsert_key = [&](const key_t& k, const value_t& v) {
      auto& elems = out.elements;
      auto it = std::find_if(elems.begin(), elems.end(), [&](const auto& p) { return p.first == k; });
      if (it != elems.end())
        it->second = v;
      else
        elems.emplace_back(k, v);
    };

    if (cp)
    {
      for_each_patch_op<C>(e, [&](const key_t& k, detail::PatchOpKind kind, const value_t* vptr) {
        if (kind == detail::PatchOpKind::Remove)
        {
          erase_key(k);
        }
        else if (vptr)
        {
          upsert_key(k, *vptr);
        }
      });
    }

    // Decide truthiness:
    // True if we have a base component OR any patch exists for this entity (whole or element).
    const bool has_base = (base_.template get_const<C>(e) != nullptr);
    bool has_any_patch = false;

    if (cp)
    {
      if (cp->whole_ops.find(e) != cp->whole_ops.end())
        has_any_patch = true;
      else
      {
        auto itE = cp->element_ops.find(e);
        if (itE != cp->element_ops.end() && !itE->second.empty())
          has_any_patch = true;
      }
    }

    return has_base || has_any_patch;
  }

  // Materialize a WHOLE component WITHOUT .elements:
  // - If patch removes -> return false
  // - If patch replaces -> out = patched
  // - Else -> out = base
  template <class C, typename std::enable_if<!detail::has_elements_v<C>, int>::type = 0>
  bool materialize_component(entity_type e, C& out) const
  {
    if (const auto* cp = patch_.template get<C>())
    {
      auto it = cp->ops.find(e);
      if (it != cp->ops.end())
      {
        if (it->second.kind == detail::PatchOpKind::Remove)
          return false;
        if (it->second.value)
        {
          out = *it->second.value;
          return true;
        }
      }
    }

    const C* base_c = base_.template get_const<C>(e);
    if (!base_c)
      return false;

    out = *base_c;
    return true;
  }

private:
  detail::DiffBase base_;
  detail::DatabasePatch patch_;
  detail::PatchedDatabaseView view_;
  std::size_t revision_;
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

template <class C, class Key, typename std::enable_if<has_elements_v<C>, int>::type>
inline const typename element_traits<C>::value_t* DiffBase::get_element(entity_type e, const Key& key) const
{
  if (db_)
    return db_->template get_element<C>(e, key);

  // IMPORTANT: explicit return type avoids GCC auto deduction bug
  return diff_->template get_element<C>(e, key);
}

}  // namespace detail

}  // namespace database
}  // namespace sodf

#endif  // SODF_DATABASE_DATABASE_H_
