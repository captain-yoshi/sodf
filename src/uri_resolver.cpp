#include <sodf/uri_resolver.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#define SODF_PATH_SEP ';'
#else
#define SODF_PATH_SEP ':'
#endif

#ifdef SODF_WITH_CURL
#include <curl/curl.h>
#endif

namespace {

// ---------- small helpers (internal) ----------

std::vector<std::string> split_roots(const std::string& roots)
{
  std::vector<std::string> out;
  std::string cur;
  for (char c : roots)
  {
    if (c == SODF_PATH_SEP)
    {
      if (!cur.empty())
        out.push_back(cur), cur.clear();
    }
    else
      cur.push_back(c);
  }
  if (!cur.empty())
    out.push_back(cur);
  return out;
}

bool is_http(const std::string& s)
{
  return s.rfind("http://", 0) == 0 || s.rfind("https://", 0) == 0;
}

std::string trim_slashes(const std::string& s)
{
  size_t b = 0, e = s.size();
  while (b < e && (s[b] == '/' || s[b] == '\\'))
    ++b;
  while (e > b && (s[e - 1] == '/' || s[e - 1] == '\\'))
    --e;
  return s.substr(b, e - b);
}

// prevent path traversal; keep generic separators
std::string sanitize_relpath(const std::string& rel)
{
  std::filesystem::path p = rel;
  std::filesystem::path clean;
  for (auto& part : p)
  {
    if (part == ".." || part == ".")
      continue;
    clean /= part;
  }
  return clean.generic_string();
}

std::string url_join(const std::string& base_url, const std::string& rel)
{
  std::string b = base_url;
  if (!b.empty() && b.back() == '/')
    b.pop_back();
  std::string r = trim_slashes(rel);
  return b + "/" + r;
}

#ifndef _WIN32
std::filesystem::path expand_user(const std::string& p)
{
  if (!p.empty() && p[0] == '~')
  {
    const char* home = std::getenv("HOME");
    if (home)
      return std::filesystem::path(home) / p.substr(1);
  }
  return std::filesystem::path(p);
}
#endif

std::filesystem::path cache_base_dir()
{
#ifndef _WIN32
  return expand_user("~/.cache/sodf");
#else
  if (const char* local = std::getenv("LOCALAPPDATA"))
  {
    return std::filesystem::path(local) / "sodf" / "cache";
  }
  return std::filesystem::temp_directory_path() / "sodf-cache";
#endif
}

std::filesystem::path cache_path_for_url(const std::string& full_url)
{
  auto pos = full_url.find("://");
  std::string after = (pos == std::string::npos) ? full_url : full_url.substr(pos + 3);
  auto slash = after.find('/');
  std::string host = slash == std::string::npos ? after : after.substr(0, slash);
  std::string rest = slash == std::string::npos ? "" : after.substr(slash + 1);

  std::filesystem::path p = cache_base_dir() / host / rest;
  std::filesystem::create_directories(p.parent_path());
  return p;
}

#ifdef SODF_WITH_CURL
size_t write_to_file_cb(void* ptr, size_t size, size_t nmemb, void* stream)
{
  std::ofstream* ofs = static_cast<std::ofstream*>(stream);
  ofs->write(static_cast<const char*>(ptr), size * nmemb);
  return size * nmemb;
}

std::filesystem::path http_fetch_to_cache(const std::string& url, long timeout_ms = 10000)
{
  auto dst = cache_path_for_url(url);

  // already cached → return (you can add validation later)
  if (std::filesystem::exists(dst))
    return dst;

  CURL* curl = curl_easy_init();
  if (!curl)
    throw std::runtime_error("curl_easy_init failed");

  std::ofstream ofs(dst, std::ios::binary);
  if (!ofs)
  {
    curl_easy_cleanup(curl);
    throw std::runtime_error("Failed to open cache file: " + dst.string());
  }

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl, CURLOPT_MAXREDIRS, 5L);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_to_file_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &ofs);
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "sodf-resolver/1.0");
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, timeout_ms);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, timeout_ms);

  CURLcode rc = curl_easy_perform(curl);
  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  curl_easy_cleanup(curl);
  ofs.close();

  if (rc != CURLE_OK || http_code < 200 || http_code >= 300)
  {
    std::error_code ec;
    std::filesystem::remove(dst, ec);
    throw std::runtime_error("HTTP fetch failed (" + std::to_string(http_code) + "): " + url);
  }
  return dst;
}
#endif  // SODF_WITH_CURL

}  // anonymous namespace

// ---------- public API ----------
namespace sodf {

Resolved resolve_resource_uri(const std::string& uri, const std::string& current_xml_dir, const char* env_roots)
{
  // Handle explicit http(s) URIs directly (download if CURL enabled)
  if (is_http(uri))
  {
#ifdef SODF_WITH_CURL
    auto cached = http_fetch_to_cache(uri);
    return { std::filesystem::weakly_canonical(cached).string(), true, uri };
#else
    throw std::runtime_error("HTTP URI provided but SODF_WITH_CURL not enabled: " + uri);
#endif
  }

  // Non-sodf: treat as file path (absolute or relative to current_xml_dir)
  if (uri.rfind("sodf://", 0) != 0)
  {
    std::filesystem::path p(uri);
    if (p.is_relative() && !current_xml_dir.empty())
    {
      p = std::filesystem::path(current_xml_dir) / p;
    }
    return { std::filesystem::weakly_canonical(p).string(), false, "" };
  }

  // sodf://... → search roots
  const char* roots_c = env_roots ? env_roots : std::getenv("SODF_URI_PATH");
  if (!roots_c || std::string(roots_c).empty())
  {
    throw std::runtime_error("SODF_URI_PATH not set (use ':' or ';' to separate multiple roots).");
  }

  std::string rel = sanitize_relpath(uri.substr(std::string("sodf://").size()));
  for (const auto& root : split_roots(roots_c))
  {
    if (root.empty())
      continue;

    if (is_http(root))
    {
#ifdef SODF_WITH_CURL
      std::string full_url = url_join(root, rel);
      try
      {
        auto cached = http_fetch_to_cache(full_url);
        if (std::filesystem::exists(cached))
        {
          return { std::filesystem::weakly_canonical(cached).string(), true, root };
        }
      }
      catch (...)
      {
        // fall through to next root
      }
#else
      // Skip HTTP roots if curl support is disabled
      continue;
#endif
    }
    else
    {
      std::filesystem::path candidate = std::filesystem::path(root) / rel;
      if (std::filesystem::exists(candidate))
      {
        return { std::filesystem::weakly_canonical(candidate).string(), false, root };
      }
    }
  }

  throw std::runtime_error("sodf:// resource not found in any root: " + uri);
}

}  // namespace sodf
