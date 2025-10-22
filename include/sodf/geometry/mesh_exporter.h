#ifndef SODF_GEOMETRY_MESH_EXPORTER_WRITE_H_
#define SODF_GEOMETRY_MESH_EXPORTER_WRITE_H_

#include <fstream>
#include <filesystem>
#include <string>
#include <vector>
#include <cstdlib>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sodf/geometry/shape.h>        // Shape, MeshRef, InlineMeshPtr, TriangleMesh
#include <sodf/geometry/mesh.h>         // meshifyPrimitive(...)
#include <sodf/geometry/mesh_loader.h>  // loadMeshWithAssimp(...)
#include <sodf/uri_resolver.h>          // resolve_resource_uri(...) (for reading)
                                        // we’ll add our own writer-side resolver

namespace sodf {
namespace geometry {

// --- small utils ------------------------------------------------------------

inline Eigen::Matrix3d basisFromSymRef(const Eigen::Vector3d& sym_in, const Eigen::Vector3d& ref_in)
{
  Eigen::Vector3d z = sym_in.normalized();
  Eigen::Vector3d x = (ref_in - z * (z.dot(ref_in))).normalized();
  if (!std::isfinite(x.squaredNorm()) || x.squaredNorm() < 1e-14)
  {
    Eigen::Vector3d tmp = (std::abs(z.z()) < 0.9) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
    x = (tmp - z * (z.dot(tmp))).normalized();
  }
  Eigen::Vector3d y = z.cross(x);
  Eigen::Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;
  return R;
}

// Turn any Shape into a TriangleMesh (bakes scale + axes for primitives).
inline bool exportTriangleMesh(const Shape& shape, int radial_res, int axial_res, TriangleMesh& out)
{
  out.V.clear();
  out.F.clear();

  if (shape.type == ShapeType::Mesh)
  {
    bool ok = false;
    std::visit(
        [&](auto&& alt) {
          using T = std::decay_t<decltype(alt)>;
          if constexpr (std::is_same_v<T, std::monostate>)
          {
            ok = false;
          }
          else if constexpr (std::is_same_v<T, MeshRef>)
          {
            ok = loadMeshWithAssimp(alt.uri, shape.scale, out);  // bakes scale
          }
          else if constexpr (std::is_same_v<T, InlineMeshPtr>)
          {
            if (!alt)
            {
              ok = false;
              return;
            }
            out = *alt;
            const Eigen::Vector3d s = shape.scale;
            for (auto& v : out.V)
            {
              v = v.cwiseProduct(s);
            }
            ok = (!out.V.empty() && !out.F.empty());
          }
        },
        shape.mesh);
    return ok;
  }

  // Primitive -> meshify
  std::vector<Eigen::Vector3d> V;
  std::vector<TriangleMesh::Face> F;
  if (!meshifyPrimitive(shape, V, F, radial_res, axial_res))
    return false;

  // bake scale
  const Eigen::Vector3d s = shape.scale;
  for (auto& p : V)
    p = p.cwiseProduct(s);

  // apply axes if present (sym=+Z, ref=+X in authoring space)
  if (shape.axes.size() >= 2)
  {
    const Eigen::Matrix3d R = basisFromSymRef(shape.axes[0], shape.axes[1]);
    for (auto& p : V)
      p = R * p;
  }

  out.V = std::move(V);
  out.F = std::move(F);
  return (!out.V.empty() && !out.F.empty());
}

// Binary STL writer
inline bool exportTriangleMeshToBinarySTL(const TriangleMesh& M, const std::string& path)
{
  if (M.V.empty() || M.F.empty())
    return false;

  std::ofstream ofs(path, std::ios::binary);
  if (!ofs)
    return false;

  // 80-byte header
  char header[80] = {};
  std::snprintf(header, sizeof(header), "sodf-export");
  ofs.write(header, 80);

  // number of triangles
  uint32_t triCount = static_cast<uint32_t>(M.F.size());
  ofs.write(reinterpret_cast<const char*>(&triCount), 4);

  auto write_f32 = [&](float f) { ofs.write(reinterpret_cast<const char*>(&f), 4); };

  for (const auto& f : M.F)
  {
    const Eigen::Vector3d& a = M.V[f[0]];
    const Eigen::Vector3d& b = M.V[f[1]];
    const Eigen::Vector3d& c = M.V[f[2]];

    Eigen::Vector3d n = (b - a).cross(c - a);
    double len = n.norm();
    if (len > 0)
      n /= len;
    else
      n.setZero();

    // normal
    write_f32(static_cast<float>(n.x()));
    write_f32(static_cast<float>(n.y()));
    write_f32(static_cast<float>(n.z()));
    // vertices
    write_f32(static_cast<float>(a.x()));
    write_f32(static_cast<float>(a.y()));
    write_f32(static_cast<float>(a.z()));
    write_f32(static_cast<float>(b.x()));
    write_f32(static_cast<float>(b.y()));
    write_f32(static_cast<float>(b.z()));
    write_f32(static_cast<float>(c.x()));
    write_f32(static_cast<float>(c.y()));
    write_f32(static_cast<float>(c.z()));
    // attribute byte count = 0
    uint16_t abc = 0;
    ofs.write(reinterpret_cast<const char*>(&abc), 2);
  }

  return ofs.good();
}

// Resolve an *output* path for writing (sodf:// goes to first filesystem root in SODF_URI_PATH)
inline std::optional<std::string> resolve_output_path_for_write(const std::string& out_uri,
                                                                const char* env_roots = nullptr)
{
  namespace fs = std::filesystem;

  auto is_http = [](const std::string& s) { return s.rfind("http://", 0) == 0 || s.rfind("https://", 0) == 0; };
  auto is_file_scheme = [](const std::string& s) { return s.rfind("file://", 0) == 0; };
  auto is_sodf = [](const std::string& s) { return s.rfind("sodf://", 0) == 0; };

  // Absolute/relative path
  if (!is_sodf(out_uri) && !is_file_scheme(out_uri))
  {
    fs::path p(out_uri);
    if (p.is_absolute() || !p.has_root_path())
    {  // accept both abs and rel
      return fs::weakly_canonical(p).string();
    }
  }

  // file:// scheme
  if (is_file_scheme(out_uri))
  {
    fs::path p(out_uri.substr(7));  // strip "file://"
    return fs::weakly_canonical(p).string();
  }

  // sodf:// → choose first FS root from SODF_URI_PATH
  if (is_sodf(out_uri))
  {
    // split SODF_URI_PATH
    const char* env = env_roots ? env_roots : std::getenv("SODF_URI_PATH");
#ifdef _WIN32
    const char sep = ';';
#else
    const char sep = ':';
#endif
    if (!env || !*env)
      return std::nullopt;

    std::string rel = out_uri.substr(7);  // after "sodf://"
    // Normalize 'rel' as a path (vendor/object/.../file.ext)
    for (char& ch : rel)
      if (ch == '\\')
        ch = '/';

    std::string roots(env);
    size_t start = 0;
    while (start <= roots.size())
    {
      size_t end = roots.find(sep, start);
      std::string root = roots.substr(start, (end == std::string::npos) ? std::string::npos : end - start);
      start = (end == std::string::npos) ? roots.size() + 1 : end + 1;

      if (root.empty() || is_http(root))
        continue;  // skip URL roots for writing
      fs::path base(root);
      fs::path full = fs::weakly_canonical(base / rel);
      return full.string();  // pick the first FS root
    }
    return std::nullopt;  // no filesystem root available for writing
  }

  return std::nullopt;
}

/**
 * @brief Export a Shape to an STL file at the given *destination URI*.
 *        - For primitives: meshify (radial/axial), bake scale + axes, write STL.
 *        - For Mesh shapes:
 *            * MeshRef  : load via Assimp (bakes scale) then write STL (re-export).
 *            * Inline   : copy (bakes scale) then write STL.
 *
 * @param shape       Input shape.
 * @param out_uri     Destination URI/path (supports sodf://, file://, abs/rel).
 * @param radial_res  Resolution for round primitives.
 * @param axial_res   Axial resolution for spherical segments.
 * @param out_path    (optional) filled with resolved filesystem path actually written.
 * @return true on success, false on failure.
 */
inline bool exportShapeToUriAsSTL(const Shape& shape, const std::string& out_uri, int radial_res, int axial_res,
                                  std::string* out_path = nullptr)
{
  auto path_opt = resolve_output_path_for_write(out_uri);
  if (!path_opt)
    return false;
  const std::string& path = *path_opt;

  // Ensure parent directory exists
  std::error_code ec;
  std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
  if (ec)
    return false;

  TriangleMesh M;
  if (!exportTriangleMesh(shape, radial_res, axial_res, M))
    return false;

  const bool ok = exportTriangleMeshToBinarySTL(M, path);
  if (ok && out_path)
    *out_path = path;
  return ok;
}

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_MESH_EXPORTER_H_
