#ifndef SODF_PHYSICS_LIQUID_BUILDER_H_
#define SODF_PHYSICS_LIQUID_BUILDER_H_

#include <sodf/geometry/mesh.h>         // TriangleMesh + meshifyPrimitive
#include <sodf/geometry/mesh_loader.h>  // loadMeshWithAssimp(uri, scale, TriangleMesh&)
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
  using sodf::geometry::InlineMeshPtr;
  using sodf::geometry::MeshRef;
  using sodf::geometry::TriangleMesh;
  using sodf::physics::FluidConvexMeshShape;

  std::vector<Eigen::Vector3d> V;
  std::vector<TriangleMesh::Face> Fidx;  // uint32_t indices

  // 1) Primitive → mesh
  if (!meshifyPrimitive(s, V, Fidx, radial_res, axial_res))
  {
    // 2) Mesh source required
    if (s.type != sodf::geometry::ShapeType::Mesh)
      return nullptr;

    // 2a) Inline mesh (shared_ptr<const TriangleMesh>)
    if (const auto pm = std::get_if<InlineMeshPtr>(&s.mesh))
    {
      if (!*pm)
        return nullptr;
      const TriangleMesh& M = **pm;

      V.assign(M.V.begin(), M.V.end());
      Fidx.reserve(M.F.size());
      for (const auto& f : M.F)
        Fidx.push_back({ f[0], f[1], f[2] });

      // Per-instance scale for inline meshes
      if (s.scale != Eigen::Vector3d(1.0, 1.0, 1.0))
      {
        for (auto& p : V)
          p = p.cwiseProduct(s.scale);
      }
    }
    // 2b) External mesh (by URI)
    else if (const auto mref = std::get_if<MeshRef>(&s.mesh))
    {
      TriangleMesh M;
      if (!sodf::geometry::loadMeshWithAssimp(mref->uri, s.scale, M))
        return nullptr;

      V = std::move(M.V);
      Fidx.reserve(M.F.size());
      for (const auto& f : M.F)
        Fidx.push_back({ f[0], f[1], f[2] });
    }
    else
    {
      // Neither inline nor external provided
      return nullptr;
    }
  }

  // Convert to domain triangles (prefer making Tri = std::array<uint32_t,3>)
  std::vector<FluidConvexMeshShape::Tri> F;
  F.reserve(Fidx.size());
  for (const auto& t : Fidx)
  {
    F.push_back({ t[0], t[1], t[2] });
    // If Tri is still int[3], use:
    // F.push_back({ int(t[0]), int(t[1]), int(t[2]) });
  }

  auto domain = std::make_unique<FluidConvexMeshShape>(std::move(V), std::move(F));
  domain->setState(ctx.T_wo, ctx.g_world, ctx.base_world);
  return domain;
}

}  // namespace physics
}  // namespace sodf

#endif  // SODF_PHYSICS_LIQUID_BUILDER_H_
