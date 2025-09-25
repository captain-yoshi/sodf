#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <sodf/uri_resolver.h>  // or .hpp, match your file

namespace fs = std::filesystem;

static std::string slurp(const fs::path& p)
{
  std::ifstream ifs(p, std::ios::binary);
  std::ostringstream oss;
  oss << ifs.rdbuf();
  return oss.str();
}

#if !defined(SODF_WITH_CURL)
TEST(UriResolver, HttpFetch_SkippedWithoutCurl)
{
  GTEST_SKIP() << "SODF_WITH_CURL not enabled; skipping HTTP resolver test.";
}
#else

TEST(URIResolver, DISABLED_HttpFetch_FromGitHubRaw)
{
  // 1) Raw folder that contains empty_mesh.stl
  //    (This is CORRECT for your repo layout.)
  const std::string http_root = "https://raw.githubusercontent.com/captain-yoshi/sodf/main/test/resources";

#ifdef _WIN32
  const char sep = ';';
#else
  const char sep = ':';
#endif

  // 2) Put a bogus local root first to exercise fallback-to-HTTP.
  std::string fake_local_root = (fs::temp_directory_path() / "sodf_no_such_root").string();
  std::error_code ec;
  fs::remove_all(fake_local_root, ec);

  // 3) Build the roots string and PASS IT DIRECTLY to the resolver
  //    (No reliance on setenv/_putenv_s order or visibility.)
  const std::string roots = fake_local_root + sep + http_root;

  // 4) Resolve via sodf:// and override env with our roots string.
  const std::string uri = "sodf://empty_mesh.stl";

  sodf::Resolved resolved;
  ASSERT_NO_THROW({
    resolved = sodf::resolve_resource_uri(uri,
                                          /*current_xml_dir=*/std::string{},
                                          /*env_roots       =*/roots.c_str());
  }) << "HTTP fetch likely failed (TLS/proxy?) and was swallowed during root iteration.";

  // 5) Validate results
  ASSERT_FALSE(resolved.local_path.empty());
  ASSERT_TRUE(fs::exists(resolved.local_path)) << "Resolved path does not exist: " << resolved.local_path;
  EXPECT_TRUE(resolved.from_cache);
  EXPECT_NE(resolved.source_root.find("https://raw.githubusercontent.com/"), std::string::npos);

  const std::string content = slurp(resolved.local_path);
  ASSERT_FALSE(content.empty());
  EXPECT_NE(content.find("solid"), std::string::npos);
  EXPECT_NE(content.find("endsolid"), std::string::npos);
}

#endif  // SODF_WITH_CURL

TEST(URIResolver, LocalFile_AbsolutePath_NoScheme)
{
  // Create a temp file
  fs::path dir = fs::temp_directory_path() / "sodf_local_file_test_abs";
  fs::create_directories(dir);
  fs::path mesh = dir / "local_mesh.stl";
  {
    std::ofstream(mesh.string()) << "solid local_abs\nendsolid local_abs\n";
  }

  // Call resolver with absolute path (no sodf://)
  auto resolved = sodf::resolve_resource_uri(mesh.string());
  EXPECT_TRUE(fs::exists(resolved.local_path));
  EXPECT_EQ(fs::weakly_canonical(mesh).string(), fs::weakly_canonical(resolved.local_path).string());
  EXPECT_FALSE(resolved.from_cache);
  EXPECT_TRUE(resolved.source_root.empty());

  // content sanity
  auto content = slurp(resolved.local_path);
  EXPECT_NE(content.find("solid"), std::string::npos);

  std::error_code ec;
  fs::remove_all(dir, ec);
}

TEST(URIResolver, LocalFile_RelativePath_WithCurrentDir)
{
  // Create a temp "scene" dir and file
  fs::path dir = fs::temp_directory_path() / "sodf_local_file_test_rel";
  fs::create_directories(dir);
  const std::string filename = "local_mesh_rel.stl";
  fs::path mesh = dir / filename;
  {
    std::ofstream(mesh.string()) << "solid local_rel\nendsolid local_rel\n";
  }

  // Pass relative URI and provide current_xml_dir so resolver can base it
  auto resolved = sodf::resolve_resource_uri(filename, /*current_xml_dir=*/dir.string());
  EXPECT_TRUE(fs::exists(resolved.local_path));
  EXPECT_EQ(fs::weakly_canonical(mesh).string(), fs::weakly_canonical(resolved.local_path).string());
  EXPECT_FALSE(resolved.from_cache);
  EXPECT_TRUE(resolved.source_root.empty());

  auto content = slurp(resolved.local_path);
  EXPECT_NE(content.find("endsolid"), std::string::npos);

  std::error_code ec;
  fs::remove_all(dir, ec);
}
