#pragma once
#include <ginseng.hpp>
#include <sodf/ecs/registry_p.h>

namespace sodf {
namespace ecs {

struct GinsengBackend
{
  ginseng::database db;

  // Adapters between façade EntityID and ginseng::database::ent_id
  static EntityID toEntityID(ginseng::database::ent_id id)
  {
    return { id.get_index(), id.get_version() };
  }
  static ginseng::database::ent_id toBackendID(const EntityID& e)
  {
    return ginseng::database::ent_id::compose(e.index, e.generation);
  }
};

namespace detail {
// Basic function-traits for lambdas (non-generic)
template <class T>
struct fn_traits : fn_traits<decltype(&T::operator())>
{
};
template <class C, class R, class... Args>
struct fn_traits<R (C::*)(Args...) const>
{
  using args = std::tuple<Args...>;
};
template <class C, class R, class... Args>
struct fn_traits<R (C::*)(Args...)>
{
  using args = std::tuple<Args...>;
};

template <class T>
using remove_cvref_t = std::remove_cv_t<std::remove_reference_t<T>>;
}  // namespace detail

template <>
struct ecs_traits<GinsengBackend>
{
  using backend_type = GinsengBackend;
  using entity_type = EntityID;

  // --- entity lifecycle ---
  static entity_type create(backend_type& b)
  {
    auto gid = b.db.create_entity();
    return backend_type::toEntityID(gid);
  }

  static void destroy(backend_type& b, entity_type e)
  {
    b.db.destroy_entity(backend_type::toBackendID(e));
  }

  static void clear(backend_type& b)
  {
    b.db = ginseng::database{};
  }

  // --- components ---
  template <class C>
  static bool has(backend_type& b, entity_type e)
  {
    return b.db.template has_component<C>(backend_type::toBackendID(e));
  }

  template <class C>
  static C* get(backend_type& b, entity_type e)
  {
    return b.db.template get_component<C*>(backend_type::toBackendID(e));
  }

  // default-construct if missing and return a reference
  template <class C>
  static C& add(backend_type& b, entity_type e)
  {
    auto ge = backend_type::toBackendID(e);
    if (auto* p = b.db.template get_component<C*>(ge))
    {
      return *p;
    }
    b.db.add_component(ge, C{});
    return b.db.template get_component<C>(ge);
  }

  template <class C>
  static void remove(backend_type& b, entity_type e)
  {
    b.db.template remove_component<C>(backend_type::toBackendID(e));
  }

  template <class... Cs, class Fn>
  static void each(backend_type& b, Fn&& fn)
  {
    auto wrapper = [&fn](ginseng::database::ent_id ge, Cs&... cs) {
      if constexpr (std::is_invocable_v<Fn&, entity_type, Cs&...>)
      {
        fn(GinsengBackend::toEntityID(ge), cs...);
      }
      else if constexpr (std::is_invocable_v<Fn&, ginseng::database::ent_id, Cs&...>)
      {
        fn(ge, cs...);
      }
      else
      {
        static_assert(std::is_invocable_v<Fn&, Cs&...>,
                      "Fn must be callable with (EntityID, Cs&...), (ent_id, Cs&...), or (Cs&...)");
        fn(cs...);
      }
    };
    b.db.visit(wrapper);  // concrete signature: (ent_id, Cs&...)
  }

  // New: deducing path (no template args at call site)
  template <class Fn>
  static void each_deduced(backend_type& b, Fn&& fn)
  {
    using F = std::decay_t<Fn>;
    using args_t = typename detail::fn_traits<F>::args;
    constexpr std::size_t N = std::tuple_size_v<args_t>;
    static_assert(N >= 1, "Visitor must take at least one parameter.");

    using A0 = std::tuple_element_t<0, args_t>;
    using A0d = detail::remove_cvref_t<A0>;

    // Case 1: user wrote lambda(EntityID, Cs&...)
    if constexpr (std::is_same_v<A0d, entity_type>)
    {
      using idx = std::make_index_sequence<N - 1>;
      forward_with_id_entity<F, args_t>(b, std::forward<Fn>(fn), idx{});
    }
    // Case 2: user wrote lambda(ginseng::ent_id, Cs&...)
    else if constexpr (std::is_same_v<A0d, ginseng::database::ent_id>)
    {
      // Directly visit with user fn (signature already concrete)
      b.db.visit(std::forward<Fn>(fn));
    }
    // Case 3: user wrote lambda(Cs&...)  (no id)
    else
    {
      using idx = std::make_index_sequence<N>;  // N component params in user's fn
      forward_without_id<F, args_t>(b, std::forward<Fn>(fn), idx{});
    }
  }

private:
  // With id first: (ent_id, Cs&...)
  template <class F, class Tuple, std::size_t... I>
  static void forward_with_id_entity(backend_type& b, F&& fn, std::integer_sequence<std::size_t, I...>)
  {
    auto wrapper = [fn = std::forward<F>(fn)](ginseng::database::ent_id ge,
                                              std::tuple_element_t<(I + 1), Tuple>... cs) mutable {
      fn(GinsengBackend::toEntityID(ge), cs...);
    };
    b.db.visit(wrapper);  // visit provides (ent_id, Cs&...)
  }

  template <class F, class Tuple, std::size_t... I>
  static void forward_without_id(backend_type& b, F&& fn, std::integer_sequence<std::size_t, I...>)
  {
    auto wrapper = [fn = std::forward<F>(fn)](ginseng::database::ent_id /*unused*/,
                                              std::tuple_element_t<I, Tuple>... cs) mutable {
      fn(cs...);  // user didn't ask for the id
    };
    b.db.visit(wrapper);  // visit provides (ent_id, Cs&...)
  }
};

}  // namespace ecs
}  // namespace sodf
