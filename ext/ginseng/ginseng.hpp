#ifndef GINSENG_GINSENG_HPP
#define GINSENG_GINSENG_HPP

#include <algorithm>
#include <bitset>
#include <memory>
#include <type_traits>
#include <vector>

#include <cstddef>

namespace ginseng {

namespace _detail {

// Find Type

template <typename T, typename... Ts>
struct index_of;

template <typename T, typename... Ts>
constexpr auto index_of_v = index_of<T, Ts...>::value;

template <typename T, typename... Ts>
struct index_of<T, T, Ts...> : std::integral_constant<std::size_t, 0>
{
};

template <typename T, typename U, typename... Ts>
struct index_of<T, U, Ts...> : std::integral_constant<std::size_t, 1 + index_of_v<T, Ts...>>
{
};

// Type Guid

using type_guid = std::size_t;

inline type_guid get_next_type_guid() noexcept
{
  static type_guid x = 0;
  return ++x;
}

template <typename T>
type_guid get_type_guid()
{
  static const type_guid my_guid = get_next_type_guid();
  return my_guid;
}

// Dynamic Bitset

class dynamic_bitset
{
public:
  using size_type = std::size_t;

  static constexpr size_type word_size = 64;

  using bitset = std::bitset<word_size>;
  using bitset_array = bitset*;

  dynamic_bitset() : sdo(0), numbits(word_size)
  {
  }

  dynamic_bitset(const dynamic_bitset&) = delete;
  dynamic_bitset& operator=(const dynamic_bitset&) = delete;

  dynamic_bitset(dynamic_bitset&& other)
  {
    if (other.using_sdo())
    {
      new (&sdo) bitset(std::move(other.sdo));
      numbits = other.numbits;
    }
    else
    {
      dyna = other.dyna;
      numbits = other.numbits;
      other.numbits = word_size;
      new (&other.sdo) bitset(0);
    }
  }

  dynamic_bitset& operator=(dynamic_bitset&& other)
  {
    if (using_sdo())
    {
      sdo.~bitset();
    }
    else
    {
      delete[] dyna;
    }
    if (other.using_sdo())
    {
      new (&sdo) bitset(std::move(other.sdo));
    }
    else
    {
      dyna = other.dyna;
    }
    numbits = other.numbits;
    new (&other.sdo) bitset(0);
    other.numbits = word_size;
    return *this;
  }

  ~dynamic_bitset()
  {
    if (using_sdo())
    {
      sdo.~bitset();
    }
    else
    {
      delete[] dyna;
    }
  }

  size_type size() const
  {
    return numbits;
  }

  bool using_sdo() const
  {
    return numbits == word_size;
  }

  void resize(size_type i)
  {
    if (i > numbits)
    {
      auto count = (i + word_size - 1) / word_size;
      auto newptr = new bitset[count];
      auto newlen = count * word_size;
      auto bitarr = using_sdo() ? &sdo : dyna;
      std::copy(bitarr, bitarr + (numbits / word_size), newptr);
      if (using_sdo())
      {
        sdo.~bitset();
      }
      else
      {
        delete[] dyna;
      }
      dyna = newptr;
      numbits = newlen;
    }
  }

  bool get(size_type i) const
  {
    if (i >= numbits)
      return false;
    const auto& bits = using_sdo() ? sdo : dyna[i / word_size];
    return bits[i % word_size];
  }

  void set(size_type i)
  {
    if (using_sdo() && i < word_size)
    {
      sdo[i] = true;
    }
    else
    {
      resize(i + 1);
      auto& bits = using_sdo() ? sdo : dyna[i / word_size];
      bits[i % word_size] = true;
    }
  }

  void unset(size_type i)
  {
    if (i < numbits)
    {
      auto& bits = using_sdo() ? sdo : dyna[i / word_size];
      bits[i % word_size] = false;
    }
  }

  void zero()
  {
    auto bitarr = using_sdo() ? &sdo : dyna;
    std::fill(bitarr, bitarr + numbits / word_size, 0);
  }

private:
  union
  {
    bitset sdo;
    bitset_array dyna;
  };
  size_type numbits;
};

// Entity

struct entity
{
  using version_type = std::size_t;
  dynamic_bitset components = {};
  version_type version = 0;
};

// False Type

template <typename T>
struct false_t : std::false_type
{
};

// Queries

/*! Tag component
 *
 * When a tag component is stored in an entity, only the fact that it exists is recorded, no data is stored.
 */
template <typename T>
struct tag
{
};

/*! Require component
 *
 * When used as a visitor parameter, applies the matching logic for the parameter, but does not load the component.
 */
template <typename T>
struct require
{
};

/*! Deny component
 *
 * When used as a visitor parameter, entities that have the component will fail to match.
 */
template <typename T>
struct deny
{
};

/*! Optional component
 *
 * When used as a visitor parameter, loads the component if it exists, and does not cause matching to fail.
 *
 * Provides pointer-like access to the parameter.
 */
template <typename T>
class optional
{
public:
  optional() : com(nullptr)
  {
  }
  explicit optional(T& c) : com(&c)
  {
  }
  optional(const optional&) = default;
  optional(optional&&) = default;
  optional& operator=(const optional&) = default;
  optional& operator=(optional&&) = default;
  T* operator->() const
  {
    return com;
  }
  T& operator*() const
  {
    return *com;
  }
  explicit operator bool() const
  {
    return com;
  }
  T& get() const
  {
    return *com;
  }

private:
  T* com;
};

/*! Optional Tag component
 *
 * When used as a visitor parameter, checks if the tag exists, but does not cause matching to fail.
 */
template <typename T>
class optional<tag<T>>
{
public:
  optional() : tag(false)
  {
  }
  explicit optional(bool t) : tag(t)
  {
  }
  optional(const optional&) = default;
  optional(optional&&) = default;
  optional& operator=(const optional&) = default;
  optional& operator=(optional&&) = default;
  explicit operator bool() const
  {
    return tag;
  }

private:
  bool tag;
};

template <typename T>
class optional<require<T>>
{
public:
  static_assert(false_t<T>::value, "Optional require parameters not allowed.");
};

template <typename T>
class optional<deny<T>>
{
public:
  static_assert(false_t<T>::value, "Optional deny parameters not allowed.");
};

template <typename T>
class optional<optional<T>>
{
public:
  static_assert(false_t<T>::value, "Optional optional parameters not allowed.");
};

// Component Tags

namespace component_tags {

struct unit
{
};
struct positive
{
};
struct normal : positive
{
};
struct noload : positive, unit
{
};
struct tagged : positive, unit
{
};
struct meta
{
};
struct optional : meta
{
};
struct eid : meta
{
};
struct inverted : noload
{
};

}  // namespace component_tags

// Component Traits

template <typename DB, typename Component>
struct component_traits
{
  using category = component_tags::normal;
  using component = Component;
};

template <typename DB, typename Component>
struct component_traits<DB, require<Component>>
{
  using category = component_tags::noload;
  using component = Component;
};

template <typename DB, typename Component>
struct component_traits<DB, tag<Component>>
{
  using category = component_tags::tagged;
  using component = tag<Component>;
};

template <typename DB, typename Component>
struct component_traits<DB, optional<Component>>
{
  using category = component_tags::optional;
  using component = Component;
};

template <typename DB, typename Component>
struct component_traits<DB, deny<Component>>
{
  using category = component_tags::inverted;
  using component = Component;
};

template <typename DB>
struct component_traits<DB, typename DB::ent_id>
{
  using category = component_tags::eid;
  using component = void;
};

// First

template <typename T, typename... Ts>
struct first
{
  using type = T;
};

template <typename... Ts>
using first_t = typename first<Ts...>::type;

// Primary

template <typename T>
struct primary
{
  using type = T;
};

// GetPrimary

template <typename DB, typename... Components>
struct get_primary;

template <typename DB, typename... Components>
using get_primary_t = typename get_primary<DB, Components...>::type;

template <typename DB, typename HeadCom, typename... Components>
struct get_primary<DB, HeadCom, Components...>
{
  using category = typename component_traits<DB, HeadCom>::category;
  using type = std::conditional_t<std::is_same_v<category, component_tags::normal>, primary<HeadCom>,
                                  get_primary_t<DB, Components...>>;
};

template <typename DB>
struct get_primary<DB>
{
  using type = primary<void>;
};

// Database Traits

template <typename DB>
struct database_traits
{
  using ent_id = typename DB::ent_id;
  using com_id = typename DB::com_id;

  template <typename C>
  using component_traits = component_traits<DB, C>;

  template <typename... Components>
  using get_primary_t = get_primary_t<DB, Components...>;

  // VisitorKey

  template <typename Primary, typename... Coms>
  class visitor_key;

  template <typename Primary, typename... Coms>
  class visitor_key<primary<Primary>, Coms...>
  {
  public:
    template <typename Com>
    using tag_t = typename component_traits<Com>::category;

    template <typename Com>
    using com_t = typename component_traits<Com>::component;

    bool check(DB& db, ent_id eid) const
    {
      return (check<Coms>(db, eid, get_guid(index_of_v<com_t<Coms>, com_t<Coms>...>), tag_t<Coms>{}) && ...);
    }

    type_guid get_guid(std::size_t i) const
    {
      if (i < sizeof...(Coms))
      {
        return guids[i];
      }
      else
      {
        return 0;
      }
    }

  private:
    template <typename Com>
    static bool check(DB& db, ent_id eid, type_guid guid, component_tags::positive)
    {
      using component = typename component_traits<Com>::component;
      if constexpr (std::is_same_v<Primary, component>)
      {
        return true;
      }
      else
      {
        return db.template has_component<component>(eid, guid);
      }
    }

    template <typename Com>
    static bool check(DB& db, ent_id eid, type_guid guid, component_tags::inverted)
    {
      using component = typename component_traits<Com>::component;
      if constexpr (std::is_same_v<Primary, component>)
      {
        return false;
      }
      else
      {
        return !db.template has_component<component>(eid, guid);
      }
    }

    template <typename Com>
    static bool check([[maybe_unused]] DB& db, [[maybe_unused]] ent_id eid, [[maybe_unused]] type_guid guid,
                      component_tags::meta)
    {
      return true;
    }

    type_guid guids[sizeof...(Coms)] = { get_type_guid<com_t<Coms>>()... };
  };

  // VisitorTraits

  template <typename... Components>
  struct visitor_traits_impl
  {
    using ent_id = typename DB::ent_id;
    using com_id = typename DB::com_id;
    using primary_component = get_primary_t<Components...>;

    template <typename Com>
    using tag_t = typename component_traits<Com>::category;

    template <typename Com>
    using com_t = typename component_traits<Com>::component;

    template <typename T>
    type_guid get_guid() const
    {
      return key.get_guid(index_of_v<com_t<T>, com_t<Components>...>);
    }

    template <typename Visitor>
    auto apply(DB& db, ent_id eid, com_id primary_cid, Visitor&& visitor)
    {
      if (key.check(db, eid))
      {
        return std::forward<Visitor>(visitor)(get_com<Components>(tag_t<Components>{}, db, eid, primary_cid,
                                                                  get_guid<Components>(), primary_component{})...);
      }
    }

  private:
    template <typename Com, typename Primary>
    static Com& get_com(component_tags::normal, DB& db, const ent_id& eid, const com_id& primary_cid, type_guid guid,
                        primary<Primary>)
    {
      if constexpr (std::is_same_v<Com, Primary>)
      {
        return db.template get_component_by_id<Com>(primary_cid, guid);
      }
      else
      {
        return db.template get_component<Com>(eid, guid);
      }
    }

    template <typename Com, typename Primary>
    static Com get_com(component_tags::optional, DB& db, const ent_id& eid, const com_id& primary_cid, type_guid guid,
                       primary<Primary>)
    {
      using traits = component_traits<Com>;
      using inner_component = typename traits::component;
      using inner_traits = component_traits<inner_component>;
      using inner_category = typename inner_traits::category;
      return get_com_optional<inner_component>(inner_category{}, db, eid, primary_cid, guid, primary<Primary>{});
    }

    template <typename Com, typename Primary>
    static optional<Com> get_com_optional(component_tags::normal, DB& db, const ent_id& eid, const com_id& primary_cid,
                                          type_guid guid, primary<Primary>)
    {
      if constexpr (std::is_same_v<Com, Primary>)
      {
        return db.template get_component_by_id<Com>(primary_cid, guid);
      }
      else
      {
        if (db.template has_component<Com>(eid))
        {
          return optional<Com>(db.template get_component<Com>(eid, guid));
        }
        else
        {
          return optional<Com>();
        }
      }
    }

    template <typename Com, typename Primary>
    static optional<Com> get_com_optional(component_tags::tagged, DB& db, const ent_id& eid,
                                          [[maybe_unused]] const com_id& primary_cid, type_guid guid, primary<Primary>)
    {
      return optional<Com>(db.template has_component<Com>(eid, guid));
    }

    template <typename Com, typename Primary>
    static const ent_id& get_com(component_tags::eid, [[maybe_unused]] DB& db, const ent_id& eid,
                                 [[maybe_unused]] const com_id& primary_cid, [[maybe_unused]] type_guid guid,
                                 primary<Primary>)
    {
      return eid;
    }

    template <typename Com, typename Primary>
    static Com get_com(component_tags::unit, [[maybe_unused]] DB& db, [[maybe_unused]] const ent_id& eid,
                       [[maybe_unused]] const com_id& primary_cid, [[maybe_unused]] type_guid guid, primary<Primary>)
    {
      return {};
    }

    visitor_key<primary_component, Components...> key;
  };

  template <typename Visitor>
  struct visitor_traits : visitor_traits<decltype(&std::decay_t<Visitor>::operator())>
  {
  };

  template <typename R, typename... Ts>
  struct visitor_traits<R (&)(Ts...)> : visitor_traits_impl<std::decay_t<Ts>...>
  {
  };

  template <typename Visitor, typename R, typename... Ts>
  struct visitor_traits<R (Visitor::*)(Ts...)> : visitor_traits_impl<std::decay_t<Ts>...>
  {
  };

  template <typename Visitor, typename R, typename... Ts>
  struct visitor_traits<R (Visitor::*)(Ts...) const> : visitor_traits_impl<std::decay_t<Ts>...>
  {
  };

  template <typename Visitor, typename R, typename... Ts>
  struct visitor_traits<R (Visitor::*)(Ts...)&> : visitor_traits_impl<std::decay_t<Ts>...>
  {
  };

  template <typename Visitor, typename R, typename... Ts>
  struct visitor_traits<R (Visitor::*)(Ts...) const&> : visitor_traits_impl<std::decay_t<Ts>...>
  {
  };

  template <typename Visitor, typename R, typename... Ts>
  struct visitor_traits<R (Visitor::*)(Ts...) &&> : visitor_traits_impl<std::decay_t<Ts>...>
  {
  };
};

// Component Set

class component_set
{
public:
  using size_type = std::size_t;
  virtual ~component_set() = 0;
  virtual void remove(size_type entid) = 0;

  size_type get_count() const
  {
    return count;
  }

protected:
  void set_count(size_type new_count)
  {
    count = new_count;
  }

private:
  size_type count = 0;
};

inline component_set::~component_set() = default;

template <typename T>
class component_set_impl final : public component_set
{
public:
  virtual ~component_set_impl() override
  {
    for (auto i = size_type{ 0 }, sz = capacity(); i < sz; ++i)
    {
      if (is_valid(i))
      {
        auto bucket = get_bucket_index(i);
        auto rel_index = get_relative_index(i);
        buckets[bucket][rel_index].component.~T();
      }
    }
  }

  size_type assign(size_type entid, T com)
  {
    if (entid >= entid_to_comid.size())
    {
      entid_to_comid.resize((entid + 1) * 3 / 2);
    }

    auto index = free_head;
    auto bucket = get_bucket_index(index);
    auto rel_index = get_relative_index(index);
    storage* slot;

    if (index == back_index)
    {
      if (bucket == buckets.size())
      {
        buckets.push_back(std::make_unique<storage[]>(bucket_size));
        comid_to_entid.resize(comid_to_entid.size() + bucket_size, null_id);
      }

      slot = &buckets[bucket][rel_index];
      ++back_index;
      free_head = back_index;
    }
    else
    {
      slot = &buckets[bucket][rel_index];
      free_head = slot->next_free;
    }

    new (&slot->component) T(std::move(com));
    entid_to_comid[entid] = index;
    comid_to_entid[index] = entid;

    set_count(get_count() + 1);

    return index;
  }

  virtual void remove(size_type entid) override final
  {
    auto index = entid_to_comid[entid];
    auto bucket = get_bucket_index(index);
    auto rel_index = get_relative_index(index);
    auto& slot = buckets[bucket][rel_index];

    slot.component.~T();
    slot.next_free = free_head;
    free_head = index;
    comid_to_entid[index] = null_id;

    set_count(get_count() - 1);
  }

  bool is_valid(size_type comid) const
  {
    return comid_to_entid[comid] != null_id;
  }

  size_type get_comid(size_type entid) const
  {
    return entid_to_comid[entid];
  }

  T& get_com(size_type comid)
  {
    auto bucket = get_bucket_index(comid);
    auto rel_index = get_relative_index(comid);
    auto& slot = buckets[bucket][rel_index];
    return slot.component;
  }

  size_type get_entid(size_type comid) const
  {
    return comid_to_entid[comid];
  }

  size_type capacity() const
  {
    return back_index;
  }

private:
  union storage
  {
    size_type next_free;
    T component;

    storage()
    {
    }
    ~storage()
    {
    }
  };

  std::vector<size_type> entid_to_comid;
  std::vector<size_type> comid_to_entid;
  std::vector<std::unique_ptr<storage[]>> buckets;
  size_type free_head = 0;
  size_type back_index = 0;

  static constexpr size_type bucket_size = 4096 * 8;
  static constexpr size_type null_id = static_cast<size_type>(-1);

  static size_type get_bucket_index(size_type idx)
  {
    return idx / bucket_size;
  }

  static size_type get_relative_index(size_type idx)
  {
    return idx % bucket_size;
  }

  static size_type get_total_size(size_type num_buckets)
  {
    return num_buckets * bucket_size;
  }
};

template <typename T>
class component_set_impl<tag<T>> final : public component_set
{
public:
  virtual ~component_set_impl() = default;
  virtual void remove([[maybe_unused]] size_type entid) override final
  {
  }
};

// Opaque index

template <typename Tag, typename Friend, typename Index>
class opaque_index
{
public:
  opaque_index() = default;

  bool operator==(const opaque_index& other)
  {
    return index == other.index;
  }

  Index get_index() const
  {
    return index;
  }

private:
  friend Friend;

  opaque_index(Index i) : index(i)
  {
  }

  operator Index() const
  {
    return index;
  }

  opaque_index& operator++()
  {
    ++index;
    return *this;
  }

  Index index;
};

/*! Database
 *
 * An Entity component Database. Uses the given allocator to allocate
 * components, and may also use the same allocator for internal data.
 *
 * @warning
 * This container does not perform any synchronization. Therefore, it is not
 * considered "thread-safe".
 */
class database
{
public:
  // IDs

  /*! Entity ID.
   */
  class ent_id
  {
  public:
    friend class database;
    using index_type = std::vector<entity>::size_type;
    using version_type = entity::version_type;

    bool operator==(const ent_id& other) const
    {
      return index == other.index && version == other.version;
    }

    index_type get_index() const
    {
      return index;
    }

  private:
    ent_id(index_type i, version_type v) : index(i), version(v)
    {
    }

    index_type index = 0;
    version_type version = 0;
  };

  /*! Component ID.
   */
  using com_id = opaque_index<struct com_id_tag, database, component_set::size_type>;

  /*! Creates a new Entity.
   *
   * Creates a new Entity that has no components.
   *
   * @return ID of the new Entity.
   */
  ent_id create_entity()
  {
    ent_id::index_type index;

    if (free_entities.empty())
    {
      index = entities.size();
      entities.emplace_back();
    }
    else
    {
      index = free_entities.back();
      free_entities.pop_back();
    }

    entities[index].components.set(0);

    return { index, entities[index].version };
  }

  /*! Destroys an Entity.
   *
   * Destroys the given Entity and all associated components.
   *
   * If the Entity does not exist, no work is done.
   *
   * @param eid ID of the Entity to erase.
   */
  void destroy_entity(const ent_id& eid)
  {
    const auto index = eid.get_index();

    if (entities[index].version != eid.version)
    {
      return;
    }

    for (dynamic_bitset::size_type i = 1; i < entities[index].components.size(); ++i)
    {
      if (entities[index].components.get(i))
      {
        component_sets[i]->remove(index);
      }
    }

    entities[index].components.zero();
    ++entities[index].version;
    free_entities.push_back(index);
  }

  /*! Determines whether or not an entity exists.
   *
   * @param eid ID of the Entity to check.
   */
  bool exists(const ent_id& eid) const
  {
    return entities[eid.index].version == eid.version && entities[eid.index].components.get(0);
  }

  /*! Adds a component to an entity.
   *
   * If a component of the same type already exists for this entity,
   * the given component will be forward-assigned to it.
   *
   * Otherwise, moves or copies the given component and adds it to the entity.
   *
   * @param eid Entity to attach new component to.
   * @param com Component value.
   * @return ID of component.
   */
  template <typename T>
  com_id add_component(const ent_id& eid, T&& com)
  {
    using com_type = std::decay_t<T>;
    auto index = eid.get_index();
    auto guid = get_type_guid<com_type>();
    auto& ent_coms = entities[index].components;
    auto& com_set = get_or_create_com_set<com_type>();

    com_id cid;

    if (guid < ent_coms.size() && ent_coms.get(guid))
    {
      cid = com_set.get_comid(index);
      com_set.get_com(cid) = std::forward<T>(com);
    }
    else
    {
      cid = com_set.assign(index, std::forward<T>(com));
      ent_coms.set(guid);
    }

    return cid;
  }

  /*! Create new Tag component.
   *
   * Adds the tag to the entity if it does not already exist.
   *
   * @param eid Entity to attach new Tag component to.
   * @param com Tag value.
   */
  template <typename T>
  void add_component(ent_id eid, tag<T>)
  {
    auto index = eid.get_index();
    auto guid = get_type_guid<tag<T>>();
    auto& ent_coms = entities[index].components;

    get_or_create_com_set<tag<T>>();

    ent_coms.set(guid);
  }

  template <typename T>
  void add_component(ent_id eid, require<T> com) = delete;

  template <typename T>
  void add_component(ent_id eid, deny<T> com) = delete;

  template <typename T>
  void add_component(ent_id eid, optional<T> com) = delete;

  template <typename T>
  void add_component(ent_id eid, ent_id com) = delete;

  /*! Remove a component from an entity.
   *
   * Removes the component from the entity and destroys it.
   *
   * If the entity does not exist, no work is done.
   *
   * @warning
   * All ComIDs associated with the removed component will be invalidated.
   *
   * @tparam Com Type of the component to erase.
   *
   * @param eid ID of the entity.
   */
  template <typename Com>
  void remove_component(ent_id eid)
  {
    auto index = eid.get_index();

    if (entities[index].version != eid.version)
    {
      return;
    }

    auto guid = get_type_guid<Com>();
    auto& com_set = *get_com_set<Com>();
    com_set.remove(index);
    entities[index].components.unset(guid);
  }

  /*! Get a component.
   *
   * If Com is a non-pointer type, returns a reference to the component without performing safe checks for existence.
   *
   * Otherwise, if Com is a pointer type,
   * returns a pointer to the component of the pointed-to type that is associated with the given entity.
   *
   * If the entity has no associated component of the given type, returns nullptr.
   *
   * If the entity does not exist, returns nullptr.
   *
   * @tparam Com Type of the component to get.
   *
   * @param eid ID of the entity.
   * @return Either a reference to the component, or a pointer to the component, or nullptr.
   */
  template <typename Com>
  auto get_component(ent_id eid) -> std::conditional_t<std::is_pointer_v<Com>, Com, Com&>
  {
    auto index = eid.index;

    if constexpr (std::is_pointer_v<Com>)
    {
      using component_t = std::remove_pointer_t<Com>;

      if (entities[index].version != eid.version)
      {
        return nullptr;
      }

      auto guid = get_type_guid<component_t>();

      if (has_component<component_t>(eid, guid))
      {
        auto& com_set = *get_com_set<component_t>(guid);
        auto cid = com_set.get_comid(index);
        return &com_set.get_com(cid);
      }
      else
      {
        return nullptr;
      }
    }
    else
    {
      auto& com_set = *get_com_set<Com>();
      auto cid = com_set.get_comid(index);
      return com_set.get_com(cid);
    }
  }

  /*! Get a component by its ID.
   *
   * Gets a reference to the component of the given type that has the given ID.
   *
   * @warning
   * Behavior is undefined when no component is associated with the given ID.
   *
   * @tparam Com Type of the component to get.
   *
   * @param cid ID of the component.
   * @return Reference to the component.
   */
  template <typename Com>
  Com& get_component_by_id(com_id cid)
  {
    auto& com_set = *get_com_set<Com>();
    return com_set.get_com(cid);
  }

  /*! Checks if an entity has a component.
   *
   * Returns whether or not the entity has a component of the
   * associated type.
   *
   * If the entity does not exist, returns false.
   *
   * @tparam Com Type of the component to check.
   *
   * @param eid ID of the entity.
   * @return True if the component exists.
   */
  template <typename Com>
  bool has_component(ent_id eid)
  {
    auto index = eid.get_index();

    if (entities[index].version != eid.version)
    {
      return false;
    }

    return has_component<Com>(eid, get_type_guid<Com>());
  }

  /*! Visit the Database.
   *
   * Visit the Entities in the Database that match the given function's parameter types.
   *
   * The following parameter categories are accepted:
   *
   * - `T`, matches entities that have component `T`, and loads the component.
   * - `tag<T>`, matches entities that have component `tag<T>`.
   * - `require<T>`, matches entities that have component `T`, but does not load it.
   * - `optional<T>`, matches all entities, loads component `T` if it exists.
   * - `deny<T>`, matches entities that do *not* match component `T`.
   * - `ent_id`, matches all entities, provides the `ent_id` of the current entity.
   *
   * `T` and `optional<T>` parameters will refer to the entity's matching component.
   * `ent_id` parameters will contain the entity's `ent_id`.
   * Other parameters will be their default value.
   *
   * Entities that do not match all given parameter conditions will be skipped.
   *
   * @warning Entities are visited in no particular order, so creating and destroying
   *          entities or adding or removing components from within the visitor
   *          could result in weird behavior.
   *
   * @tparam Visitor Visitor function type.
   * @param visitor Visitor function.
   */
  template <typename Visitor>
  void visit(Visitor&& visitor)
  {
    using db_traits = database_traits<database>;
    using traits = typename db_traits::visitor_traits<Visitor>;
    using primary_component = typename traits::primary_component;

    return visit_helper(std::forward<Visitor>(visitor), primary_component{});
  }

  /*! Get the number of entities in the Database.
   *
   * @return Number of entities in the Database.
   */
  auto size() const
  {
    return entities.size() - free_entities.size();
  }

  /*! Get the number of components of a certain type in the Database.
   *
   * @return Number of components of type Com.
   */
  template <typename Com>
  component_set::size_type count() const
  {
    if (auto set = get_com_set<Com>())
    {
      return set->get_count();
    }
    else
    {
      return 0;
    }
  }

  /*! Converts an ent_id to a void* for storage purposes.
   *
   * @warning This is not a valid pointer and relies on widespread compiler-specific behavior.
   *          Do not ever dereference the pointer.
   *          Entity version checking is not preserved through conversion to and from pointers.
   *          Null pointers are a valid result.
   *
   * @param eid Entity ID to convert to a pointer.
   * @return A pointer which may be converted back into the same ID using from_ptr(ptr).
   */
  auto to_ptr(const ent_id& eid) const -> void*
  {
    static_assert(sizeof(void*) >= sizeof(ent_id::index_type), "Pointer conversion not possible");
    return reinterpret_cast<void*>(eid.get_index());
  }

  /*! Converts a void* to an ent_id. The pointer must have been returned from to_ptr(eid).
   *
   * @warning The entity must not have been deleted. Version checking is not applied.
   *
   * @param ptr Pointer which was obtained from to_ptr(eid).
   * @return The original ent_id that was passed to to_ptr(eid).
   */
  auto from_ptr(void* ptr) const -> ent_id
  {
    auto i = reinterpret_cast<ent_id::index_type>(ptr);
    return ent_id{ i, entities[i].version };
  }

private:
  friend struct database_traits<database>;

  template <typename Com>
  Com& get_component(ent_id eid, type_guid guid)
  {
    auto& com_set = *unsafe_get_com_set<Com>(guid);
    auto cid = com_set.get_comid(eid.get_index());
    return com_set.get_com(cid);
  }

  template <typename Com>
  Com& get_component_by_id(com_id cid, type_guid guid)
  {
    auto& com_set = *unsafe_get_com_set<Com>(guid);
    return com_set.get_com(cid);
  }

  template <typename Com>
  bool has_component(ent_id eid, type_guid guid)
  {
    auto& ent_coms = entities[eid.get_index()].components;
    return ent_coms.get(guid);
  }

  template <typename Com>
  component_set_impl<Com>* get_com_set()
  {
    return get_com_set<Com>(get_type_guid<Com>());
  }

  template <typename Com>
  const component_set_impl<Com>* get_com_set() const
  {
    return get_com_set<Com>(get_type_guid<Com>());
  }

  template <typename Com>
  component_set_impl<Com>* get_com_set(type_guid guid)
  {
    if (guid >= component_sets.size())
    {
      return nullptr;
    }
    return unsafe_get_com_set<Com>(guid);
  }

  template <typename Com>
  const component_set_impl<Com>* get_com_set(type_guid guid) const
  {
    if (guid >= component_sets.size())
    {
      return nullptr;
    }
    return unsafe_get_com_set<Com>(guid);
  }

  template <typename Com>
  component_set_impl<Com>* unsafe_get_com_set(type_guid guid)
  {
    auto& com_set = component_sets[guid];
    auto com_set_impl = static_cast<component_set_impl<Com>*>(com_set.get());
    return com_set_impl;
  }

  template <typename Com>
  const component_set_impl<Com>* unsafe_get_com_set(type_guid guid) const
  {
    auto& com_set = component_sets[guid];
    auto com_set_impl = static_cast<const component_set_impl<Com>*>(com_set.get());
    return com_set_impl;
  }

  template <typename Com>
  component_set_impl<Com>& get_or_create_com_set()
  {
    auto guid = get_type_guid<Com>();
    if (component_sets.size() <= guid)
    {
      component_sets.resize(guid + 1);
    }
    auto& com_set = component_sets[guid];
    if (!com_set)
    {
      com_set = std::make_unique<component_set_impl<Com>>();
    }
    auto com_set_impl = static_cast<component_set_impl<Com>*>(com_set.get());
    return *com_set_impl;
  }

  template <typename Visitor, typename Component>
  void visit_helper(Visitor&& visitor, primary<Component>)
  {
    using db_traits = database_traits<database>;
    using visitor_traits = typename db_traits::visitor_traits<Visitor>;

    auto traits = visitor_traits{};

    if (auto com_set_ptr = get_com_set<Component>(traits.template get_guid<Component>()))
    {
      auto& com_set = *com_set_ptr;

      for (com_id cid = 0, sz = com_set.capacity(); cid < sz; ++cid)
      {
        if (com_set.is_valid(cid))
        {
          auto i = com_set.get_entid(cid);
          traits.apply(*this, { i, entities[i].version }, cid, visitor);
        }
      }
    }
  }

  template <typename Visitor>
  void visit_helper(Visitor&& visitor, primary<void>)
  {
    using db_traits = database_traits<database>;
    using visitor_traits = typename db_traits::visitor_traits<Visitor>;

    auto traits = visitor_traits{};

    for (auto i = 0u; i < entities.size(); ++i)
    {
      if (entities[i].components.get(0))
      {
        traits.apply(*this, { i, entities[i].version }, {}, visitor);
      }
    }
  }

  std::vector<entity> entities;
  std::vector<ent_id::index_type> free_entities;
  std::vector<std::unique_ptr<component_set>> component_sets;
};

}  // namespace _detail

using _detail::database;
using _detail::deny;
using _detail::optional;
using _detail::require;
using _detail::tag;

}  // namespace ginseng

#endif  // GINSENG_GINSENG_HPP
