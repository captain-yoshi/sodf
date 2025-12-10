#ifndef SODF_ASSEMBLY_NAMESPACE_H_
#define SODF_ASSEMBLY_NAMESPACE_H_

#include <string>
#include <string_view>
#include <optional>
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <Eigen/Geometry>

namespace sodf {
namespace assembly {

// Namespaces are strict; they define the *kind*.
enum class NamespaceKind
{
  Insertion,     // "insertion"
  Shape,         // "shape"
  StackedShape,  // "stacked_shape"
  Link,          // "link"
  Origin,        // "origin"
  Frame,         // "frame" (optional user frames)
  Unknown
};

struct Ref
{
  std::string entity;    // object id (e.g., "table", "spreader-dock")
  NamespaceKind kind;    // kind determined by namespace segment
  std::string ns;        // raw namespace token (e.g., "insertion")
  std::string id;        // local id inside the namespace (may contain '/')
  std::string selector;  // optional "#selector" (e.g., "axis", "frame", "plane")
  std::string raw;       // original raw string for diagnostics
};

// ------------------ mapping token -> kind ------------------------------------

inline NamespaceKind ns_to_kind(std::string_view ns)
{
  if (ns == "insertion")
    return NamespaceKind::Insertion;
  if (ns == "shape")
    return NamespaceKind::Shape;
  if (ns == "stacked_shape")
    return NamespaceKind::StackedShape;
  if (ns == "link")
    return NamespaceKind::Link;
  if (ns == "origin")
    return NamespaceKind::Origin;
  if (ns == "frame")
    return NamespaceKind::Frame;
  return NamespaceKind::Unknown;
}

// ------------------ classification helpers -----------------------------------

// Global = "entity/ns/id[#sel]" → at least TWO slashes before optional '#'
inline bool is_global_ref(std::string_view s)
{
  const auto hash = s.find('#');
  const std::string_view core = (hash == std::string_view::npos) ? s : s.substr(0, hash);
  const auto a = core.find('/');
  if (a == std::string_view::npos)
    return false;
  const auto b = core.find('/', a + 1);
  return b != std::string_view::npos;
}

// Local = "ns/id[#sel]" (EXACTLY one slash before optional '#')
inline bool looks_local_ref(std::string_view s)
{
  const auto hash = s.find('#');
  const std::string_view core = (hash == std::string_view::npos) ? s : s.substr(0, hash);
  const auto a = core.find('/');
  if (a == std::string_view::npos)
    return false;
  return core.find('/', a + 1) == std::string_view::npos;
}

// ------------------ parsers --------------------------------------------------

// Parse "entity/ns/id[#selector]"  (GLOBAL)
inline Ref parse_global_ref(std::string_view s)
{
  Ref r;
  r.raw = std::string(s);

  // split selector
  const auto h = s.find('#');
  const std::string_view main = (h == std::string_view::npos) ? s : s.substr(0, h);
  r.selector = (h == std::string_view::npos) ? "" : std::string(s.substr(h + 1));

  // entity / ns / id
  const auto p1 = main.find('/');
  const auto p2 = (p1 == std::string_view::npos) ? std::string_view::npos : main.find('/', p1 + 1);
  if (p1 == std::string_view::npos || p2 == std::string_view::npos || p2 + 1 >= main.size())
    throw std::runtime_error("parse_global_ref: expected 'entity/ns/id[#selector]' in '" + std::string(s) + "'");

  r.entity = std::string(main.substr(0, p1));
  r.ns = std::string(main.substr(p1 + 1, p2 - (p1 + 1)));
  r.id = std::string(main.substr(p2 + 1));
  r.kind = ns_to_kind(r.ns);

  if (r.kind == NamespaceKind::Unknown)
    throw std::runtime_error("parse_global_ref: unknown namespace '" + r.ns + "' in '" + std::string(s) + "'");

  return r;
}

// Parse "ns/id[#selector]" with a default entity (LOCAL)
inline Ref parse_local_ref(std::string_view s, std::string_view default_entity)
{
  Ref r;
  r.raw = std::string(s);

  const auto h = s.find('#');
  const std::string_view main = (h == std::string_view::npos) ? s : s.substr(0, h);
  r.selector = (h == std::string_view::npos) ? "" : std::string(s.substr(h + 1));

  const auto p = main.find('/');
  if (p == std::string_view::npos || p + 1 >= main.size())
    throw std::runtime_error("parse_local_ref: expected 'ns/id[#selector]' in '" + std::string(s) + "'");

  r.entity = std::string(default_entity);
  r.ns = std::string(main.substr(0, p));
  r.id = std::string(main.substr(p + 1));
  r.kind = ns_to_kind(r.ns);

  if (r.kind == NamespaceKind::Unknown)
    throw std::runtime_error("parse_local_ref: unknown namespace '" + r.ns + "' in '" + std::string(s) + "'");

  return r;
}

// ------------------ scope enforcement helpers --------------------------------

inline const char* side_field_name(const char* side_name)
{
  return (std::string(side_name) == "host") ? "host_object" : "guest_object";
}

// Local refs require scope to exist.
inline void ensure_scope_for_local(const char* op_name, const char* side_name, std::string_view ref_str,
                                   std::string_view object_id)
{
  if (looks_local_ref(ref_str) && object_id.empty())
  {
    std::ostringstream oss;
    oss << op_name << ": '" << side_name << "' reference '" << ref_str
        << "' is local (e.g., 'insertion/A1#axis'), but OriginComponent." << side_field_name(side_name)
        << " is empty.\n"
        << "Add " << side_field_name("host") << "=\"...\" and " << side_field_name("guest")
        << "=\"...\" on <Origin>, or include <Scope host=\"...\" guest=\"...\"/>.";
    throw std::runtime_error(oss.str());
  }
}

// Global refs must match the declared host/guest object if provided.
inline void ensure_scope_for_global(const char* op_name, const char* side_name, const Ref& r, std::string_view object_id)
{
  // If the Origin side isn't set, we don't have a contract to validate against.
  if (object_id.empty())
    return;

  if (!r.entity.empty() && r.entity != object_id)
  {
    std::ostringstream oss;
    oss << op_name << ": '" << side_name << "' reference '" << r.raw << "' points to object '" << r.entity
        << "', but OriginComponent." << side_field_name(side_name) << "='" << object_id << "'.\n"
        << "This constraint mixes objects across scopes. Fix the ref or adjust the Origin scope.";
    throw std::runtime_error(oss.str());
  }
}

// ------------------ scoped ref builders --------------------------------------

// Core: parse host ref with scoping (global stays global but must match host_object if set)
template <class OriginComponentLike>
inline Ref make_host_ref(std::string_view ref, const OriginComponentLike& origin)
{
  if (ref.empty())
    throw std::runtime_error("make_host_ref: empty reference string");

  if (is_global_ref(ref))
  {
    Ref r = parse_global_ref(ref);
    ensure_scope_for_global("Origin constraint", "host", r, origin.host_object);
    return r;
  }

  ensure_scope_for_local("Origin constraint", "host", ref, origin.host_object);
  return parse_local_ref(ref, origin.host_object);
}

// Core: parse guest ref with scoping (global stays global but must match guest_object if set)
template <class OriginComponentLike>
inline Ref make_guest_ref(std::string_view ref, const OriginComponentLike& origin)
{
  if (ref.empty())
    throw std::runtime_error("make_guest_ref: empty reference string");

  if (is_global_ref(ref))
  {
    Ref r = parse_global_ref(ref);
    ensure_scope_for_global("Origin constraint", "guest", r, origin.guest_object);
    return r;
  }

  ensure_scope_for_local("Origin constraint", "guest", ref, origin.guest_object);
  return parse_local_ref(ref, origin.guest_object);
}

// ------------------ small pretty-print helpers -------------------------------

inline std::string to_string_ref(const Ref& r)
{
  std::ostringstream os;
  os << r.entity << "/" << r.ns << "/" << r.id;
  if (!r.selector.empty())
    os << "#" << r.selector;
  return os.str();
}

}  // namespace assembly
}  // namespace sodf

#endif  // SODF_ASSEMBLY_NAMESPACE_H_
