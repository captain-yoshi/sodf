#ifndef SODF_GEOMETRY_MESH_LOADER_H_
#define SODF_GEOMETRY_MESH_LOADER_H_

#include <sodf/geometry/mesh_shape.h>
#include <sodf/uri_resolver.h>

#include <Eigen/Geometry>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace sodf {
namespace geometry {

inline bool loadMeshWithAssimp(const std::string& uri, const Eigen::Vector3d& scale, TriangleMesh& out)
{
  out.V.clear();
  out.F.clear();

  if (uri.empty())
    return false;

  sodf::Resolved resolved;
  try
  {
    resolved = sodf::resolve_resource_uri(uri);
  }
  catch (const std::runtime_error&)
  {
    return false;
  }
  if (resolved.local_path.empty())
    return false;
  const std::string& path = resolved.local_path;

  Assimp::Importer importer;
  const aiScene* scene =
      importer.ReadFile(path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_ImproveCacheLocality |
                                  aiProcess_OptimizeMeshes | aiProcess_ValidateDataStructure
                        // add if your coordinate system needs it:
                        // | aiProcess_FlipWindingOrder
      );

  if (!scene || !scene->mRootNode)
    return false;

  // Pre-reserve (best effort)
  size_t totalVerts = 0, totalFaces = 0;
  for (unsigned mi = 0; mi < scene->mNumMeshes; ++mi)
  {
    if (const aiMesh* m = scene->mMeshes[mi])
    {
      totalVerts += m->mNumVertices;
      totalFaces += m->mNumFaces;
    }
  }
  out.V.reserve(out.V.size() + totalVerts);
  out.F.reserve(out.F.size() + totalFaces);

  uint64_t vert_base = 0;  // use 64-bit to detect overflow before casting to uint32_t
  for (unsigned mi = 0; mi < scene->mNumMeshes; ++mi)
  {
    const aiMesh* m = scene->mMeshes[mi];
    if (!m || m->mNumVertices == 0)
      continue;

    // Vertices (+ per-instance scale)
    for (unsigned i = 0; i < m->mNumVertices; ++i)
    {
      const aiVector3D& v = m->mVertices[i];
      out.V.emplace_back(v.x * scale.x(), v.y * scale.y(), v.z * scale.z());
    }

    // Faces (triangulated)
    for (unsigned f = 0; f < m->mNumFaces; ++f)
    {
      const aiFace& face = m->mFaces[f];
      if (face.mNumIndices != 3)
        continue;  // should be 3 due to Triangulate

      // Check we fit into uint32
      if (vert_base + face.mIndices[0] > std::numeric_limits<uint32_t>::max() ||
          vert_base + face.mIndices[1] > std::numeric_limits<uint32_t>::max() ||
          vert_base + face.mIndices[2] > std::numeric_limits<uint32_t>::max())
      {
        // If you might have >4B vertices, switch to uint64_t indices globally.
        return false;
      }

      out.F.push_back({ static_cast<uint32_t>(vert_base + face.mIndices[0]),
                        static_cast<uint32_t>(vert_base + face.mIndices[1]),
                        static_cast<uint32_t>(vert_base + face.mIndices[2]) });
    }

    vert_base += m->mNumVertices;
  }

  return (!out.V.empty() && !out.F.empty());
}

}  // namespace geometry
}  // namespace sodf

#endif  // SODF_GEOMETRY_MESH_LOADER_H_
