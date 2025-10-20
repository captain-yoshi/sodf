#ifndef UTILS_H_
#define UTILS_H_

#include <string>
#include <filesystem>
#include <tinyxml2.h>
#include <iostream>

namespace sodf {
namespace xml {

inline std::string getDirectory(const std::string& filepath)
{
  return std::filesystem::path(filepath).parent_path().string();
}

static std::string document_xml_base_dir(const tinyxml2::XMLDocument* doc)
{
  if (!doc)
    return {};

  // Prefer <Root xml:base> on the scene document
  if (const auto* root = doc->FirstChildElement("Root"))
  {
    if (const char* rb = root->Attribute("xml:base"))
      return std::filesystem::path(rb).lexically_normal().string();
    return {};  // Root exists but has no xml:base → keep semantics unchanged
  }

  // No <Root>: fall back to the first top-level <Object xml:base>
  if (const auto* obj = doc->FirstChildElement("Object"))
  {
    if (const char* ob = obj->Attribute("xml:base"))
      return std::filesystem::path(ob).lexically_normal().string();
  }

  return {};
}

static std::string resolve_realtive_uri(const tinyxml2::XMLElement* anchor, const tinyxml2::XMLDocument* doc,
                                        const std::string& uri)
{
  auto has_scheme = [](const std::string& u) {
    auto p = u.find("://");
    return p != std::string::npos && p > 0;
  };
  auto abs_posix = [](const std::string& u) { return !u.empty() && u[0] == '/'; };
  if (uri.empty() || has_scheme(uri) || abs_posix(uri))
    return uri;

  // Prefer the SCENE base (Root xml:base or configured base_dir) first
  if (auto base = document_xml_base_dir(doc); !base.empty())
  {
    return (std::filesystem::path(base) / uri).lexically_normal().string();
  }

  // No way to resolve a relative URI -> throw (do NOT return it raw)
  const int line = anchor ? anchor->GetLineNum() : 0;
  const char* tag = anchor ? anchor->Name() : "<unknown>";
  throw std::runtime_error(std::string("Failed to resolve relative URI '") + uri + "' at <" + tag + "> (line " +
                           std::to_string(line) +
                           "): no xml:base on ancestors and no scene base_dir/Root xml:base available.");
}

// Helper function for unit conversion:
inline double convertVolumeToSI(double volume, const std::string& units)
{
  if (units == "uL")
    return volume * 1e-9;
  if (units == "mL")
    return volume * 1e-6;
  if (units == "L")
    return volume * 1e-3;
  return volume;  // assume already in m^3
}

static inline bool has_scheme(const std::string& u)
{
  auto p = u.find("://");
  return p != std::string::npos && p > 0;
}

static inline bool abs_posix(const std::string& u)
{
  return !u.empty() && u[0] == '/';
}

static inline bool is_file_uri(const std::string& u)
{
  return u.rfind("file://", 0) == 0;
}

static inline std::string strip_file_uri(const std::string& u)
{
  if (!is_file_uri(u))
    return u;
  return (u.size() >= 8 && u[7] == '/') ? u.substr(7) : ("/" + u.substr(7));
}

static const char* nearest_base(const tinyxml2::XMLElement* e)
{
  for (const tinyxml2::XMLNode* n = e; n; n = n->Parent())
  {
    if (auto* el = n->ToElement())
    {
      if (const char* b = el->Attribute("xml:base"))
        return b;
    }
  }
  return nullptr;
}

// Dump the entire document to stderr (for debugging)
static void dump_whole_doc(const tinyxml2::XMLElement* anchor_elem, const char* label = "DOC")
{
  if (!anchor_elem)
  {
    std::cerr << "[mesh] dump: no element\n";
    return;
  }
  const tinyxml2::XMLDocument* doc = anchor_elem->GetDocument();
  if (!doc)
  {
    std::cerr << "[mesh] dump: no document\n";
    return;
  }

  tinyxml2::XMLPrinter pr(nullptr, /*compact=*/false);
  doc->Print(&pr);  // or: doc->Accept(&pr);
  std::cerr << "[mesh] ---- " << label << " ----\n"
            << pr.CStr() << "\n"
            << "[mesh] ---- END " << label << " ----\n";
}

static void dump_whole_doc(const tinyxml2::XMLDocument* doc, const char* label = "DOC")
{
  if (!doc)
  {
    std::cerr << "[dump] no document\n";
    return;
  }
  tinyxml2::XMLPrinter pr(nullptr, /*compact=*/false);
  doc->Print(&pr);  // or: doc->Accept(&pr);
  std::cerr << "[dump] ---- " << label << " ----\n"
            << pr.CStr() << "\n"
            << "[dump] ---- END " << label << " ----\n";
}

}  // namespace xml
}  // namespace sodf

#endif  // UTILS_H_
