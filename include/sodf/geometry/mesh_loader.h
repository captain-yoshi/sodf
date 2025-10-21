#ifndef SODF_GEOMETRY_MESH_LOADER_H_
#define SODF_GEOMETRY_MESH_LOADER_H_

#include <sodf/geometry/mesh.h>
#include <sodf/uri_resolver.h>

#include <Eigen/Geometry>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace sodf::geometry {

struct ResolvedMesh
{
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> triangles;
};

inline bool loadMeshWithAssimp(const std::string& uri, const Eigen::Vector3d& scale, ResolvedMesh& out)
{
  out.vertices.clear();
  out.triangles.clear();

  if (uri.empty())
    return false;

  sodf::Resolved resolved;
  try
  {
    resolved = sodf::resolve_resource_uri(uri);
  }
  catch (const std::runtime_error& e)
  {
    return false;
  }

  if (resolved.local_path.empty())
    return false;

  const std::string& path = resolved.local_path;

  Assimp::Importer importer;
  const aiScene* scene =
      importer.ReadFile(path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_ImproveCacheLocality |
                                  aiProcess_OptimizeMeshes | aiProcess_ValidateDataStructure);

  if (!scene || !scene->mRootNode)
    return false;

  // Flatten all meshes into one buffer:
  size_t vert_base = 0;
  for (unsigned int mi = 0; mi < scene->mNumMeshes; ++mi)
  {
    const aiMesh* m = scene->mMeshes[mi];
    if (!m || m->mNumVertices == 0)
      continue;

    // Append vertices
    out.vertices.reserve(out.vertices.size() + m->mNumVertices);
    for (unsigned int i = 0; i < m->mNumVertices; ++i)
    {
      const aiVector3D& v = m->mVertices[i];
      out.vertices.emplace_back(v.x * scale.x(), v.y * scale.y(), v.z * scale.z());
    }

    // Append triangles
    out.triangles.reserve(out.triangles.size() + m->mNumFaces);
    for (unsigned int f = 0; f < m->mNumFaces; ++f)
    {
      const aiFace& face = m->mFaces[f];
      if (face.mNumIndices != 3)
        continue;  // Triangulate flag should ensure 3
      out.triangles.emplace_back(int(vert_base + face.mIndices[0]), int(vert_base + face.mIndices[1]),
                                 int(vert_base + face.mIndices[2]));
    }

    vert_base += m->mNumVertices;
  }

  return (!out.vertices.empty() && !out.triangles.empty());
}

}  // namespace sodf::geometry

#endif  // SODF_GEOMETRY_MESH_LOADER_H_
