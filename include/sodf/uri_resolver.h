#pragma once
#include <string>

namespace sodf {

struct Resolved
{
  std::string local_path;   // path to feed your STL/mesh loader
  bool from_cache = false;  // true if downloaded from HTTP
  std::string source_root;  // which root matched (fs dir or URL)
};

/**
 * Resolve a resource URI to a local file path.
 *
 * Supported inputs:
 *  - sodf://vendor/object/.../file.ext  (searched under SODF_URI_PATH roots)
 *  - file:///abs/path or absolute/relative filesystem paths
 *  - http(s)://...  (downloaded to cache if SODF_WITH_CURL enabled)
 *
 * Search roots:
 *  - Read from env var SODF_URI_PATH (Unix ':' | Windows ';' separated)
 *  - Each root may be a filesystem directory OR an http(s) base URL
 *
 * @param uri             The URI/string from XML (e.g., "sodf://.../coarse.stl")
 * @param current_xml_dir Used to resolve *relative* non-sodf paths
 * @param env_roots       Optional override for SODF_URI_PATH (tests)
 * @throws std::runtime_error on failure (unset paths, not found, HTTP error, etc.)
 */
Resolved resolve_resource_uri(const std::string& uri, const std::string& current_xml_dir = "",
                              const char* env_roots = nullptr);

}  // namespace sodf
