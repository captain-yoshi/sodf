#ifndef SODF_ASSEMBLY_NAMESPACE_H_
#define SODF_ASSEMBLY_NAMESPACE_H_

#pragma once
#include <string>
#include <string_view>
#include <optional>
#include <stdexcept>
#include <sstream>         // to_string_ref / error messages
#include <iostream>        // printTF
#include <iomanip>         // printTF
#include <Eigen/Geometry>  // printTF

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
  std::string ns;        // raw namespace token (e.g., "insert")
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

// Parse "ns/id[#selector]" with a default entity  (LOCAL)
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

// ------------------ scope enforcement + scoped helpers -----------------------

inline void ensure_scope_for_local(const char* op_name,
                                   const char* side_name,  // "host" / "guest"
                                   std::string_view ref_str,
                                   std::string_view object_id)  // host_object or guest_object
{
  if (looks_local_ref(ref_str) && object_id.empty())
  {
    std::ostringstream oss;
    oss << op_name << ": '" << side_name << "' reference '" << ref_str
        << "' is local (e.g., 'insert/A1#axis'), but OriginComponent."
        << (std::string(side_name) == "host" ? "host_object" : "guest_object") << " is empty.\n"
        << "Add host_object=\"<host object id>\" and guest_object=\"<guest object id>\" "
        << "on <Origin>, or include <Scope host=\"...\" guest=\"...\"/>.";
    throw std::runtime_error(oss.str());
  }
}

// Core: parse host ref with scoping (global stays global; local uses host_object)
template <class OriginComponentLike>
inline Ref make_host_ref(std::string_view ref, const OriginComponentLike& origin)
{
  if (ref.empty())
    throw std::runtime_error("make_host_ref: empty reference string");

  if (is_global_ref(ref))
    return parse_global_ref(ref);

  ensure_scope_for_local("Origin constraint", "host", ref, origin.host_object);
  return parse_local_ref(ref, origin.host_object);
}

// Core: parse guest ref with scoping (global stays global; local uses guest_object)
template <class OriginComponentLike>
inline Ref make_guest_ref(std::string_view ref, const OriginComponentLike& origin)
{
  if (ref.empty())
    throw std::runtime_error("make_guest_ref: empty reference string");

  if (is_global_ref(ref))
    return parse_global_ref(ref);

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

inline void printTF(const char* label, const Eigen::Isometry3d& T)
{
  const Eigen::Vector3d t = T.translation();
  const Eigen::AngleAxisd aa(T.linear());
  std::cout << std::fixed << std::setprecision(6) << "    " << label << " t = [" << t.x() << ", " << t.y() << ", "
            << t.z() << "]"
            << "  aa = (" << aa.angle() << ", [" << aa.axis().transpose() << "])\n";
}

}  // namespace assembly
}  // namespace sodf

#endif  // SODF_ASSEMBLY_NAMESPACE_H_
