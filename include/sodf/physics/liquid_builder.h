#ifndef LIQUID_BUILDER_H_
#define LIQUID_BUILDER_H_

#include <sodf/geometry/mesh.h>
#include <sodf/geometry/mesh_loader.h>
#include <sodf/geometry/shape.h>
#include <sodf/physics/fluid_domain_shape.h>

namespace sodf {
namespace physics {

struct TruncationContext
{
  Eigen::Isometry3d T_wo = Eigen::Isometry3d::Identity();  // layer local → world
  Eigen::Vector3d g_world{ 0, 0, 1 };
  Eigen::Vector3d base_world{ 0, 0, 0 };  // h=0 point for this layer
};

inline std::unique_ptr<sodf::physics::FluidConvexMeshShape>
buildFluidDomainFromShape(const sodf::geometry::Shape& s, const sodf::physics::TruncationContext& ctx,
                          int radial_res = 64, int axial_res = 16)
{
  using sodf::physics::FluidConvexMeshShape;

  std::vector<Eigen::Vector3d> V;
  std::vector<std::array<int, 3>> Fidx;

  // 1) Primitive → meshify; 2) Mesh → vertices/triangles or load via Assimp
  const bool primitive = meshifyPrimitive(s, V, Fidx, radial_res, axial_res);
  if (!primitive)
  {
    if (s.type != sodf::geometry::ShapeType::Mesh)
      return nullptr;

    if (!s.vertices.empty() && !s.triangles.empty())
    {
      V = s.vertices;
      Fidx.reserve(s.triangles.size());
      for (auto t : s.triangles)
        Fidx.push_back({ int(t[0]), int(t[1]), int(t[2]) });
    }
    else
    {
      sodf::geometry::ResolvedMesh RM;
      if (!sodf::geometry::loadMeshWithAssimp(s.mesh_uri, s.scale, RM))
        return nullptr;
      V.swap(RM.vertices);
      Fidx.reserve(RM.triangles.size());
      for (const auto& t : RM.triangles)
        Fidx.push_back({ t.x(), t.y(), t.z() });
    }
  }

  // Convert to domain triangles
  std::vector<FluidConvexMeshShape::Tri> F;
  F.reserve(Fidx.size());
  for (auto& t : Fidx)
    F.push_back({ t[0], t[1], t[2] });

  auto domain = std::make_unique<FluidConvexMeshShape>(std::move(V), std::move(F));
  domain->setState(ctx.T_wo, ctx.g_world, ctx.base_world);
  return domain;
}

}  // namespace physics
}  // namespace sodf

#endif  // LIQUID_BUILDER_H_
