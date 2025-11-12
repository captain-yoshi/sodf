#include <sodf/physics/domain_shape_conversion.h>
#include <sodf/physics/fluid_domain_shape.h>
#include <sodf/geometry/mesh_loader.h>

namespace sodf {
namespace physics {

static inline bool positive(double x)
{
  return x > 0.0 && std::isfinite(x);
}
static inline bool nonneg(double x)
{
  return x >= 0.0 && std::isfinite(x);
}

DomainShapeBasePtr shapeMeshToDomainShape(const geometry::Shape& meshShape, const geometry::TriangleMesh& mesh,
                                          sodf::physics::DomainType dtype)
{
  if (meshShape.type != geometry::ShapeType::Mesh)
    return nullptr;

  // Copy vertices/faces (apply per-instance scale to vertices)
  std::vector<Eigen::Vector3d> V(mesh.V.begin(), mesh.V.end());
  if (meshShape.scale != Eigen::Vector3d(1.0, 1.0, 1.0))
  {
    for (auto& p : V)
      p = p.cwiseProduct(meshShape.scale);
  }

  std::vector<FluidConvexMeshShape::Tri> F;
  F.reserve(mesh.F.size());
  for (const auto& f : mesh.F)
    F.push_back({ static_cast<uint32_t>(f[0]), static_cast<uint32_t>(f[1]), static_cast<uint32_t>(f[2]) });

  if (!geometry::hasTriangles(V, mesh.F))
    return nullptr;

  // Local-vertices constructor; gravity/base will be provided later via FillEnv
  return std::make_shared<FluidConvexMeshShape>(std::move(V), std::move(F));
}

DomainShapeBasePtr shapeMeshToDomainShape(const geometry::Shape& meshShape, sodf::physics::DomainType dtype)
{
  if (meshShape.type != geometry::ShapeType::Mesh)
    return nullptr;

  std::vector<Eigen::Vector3d> V;
  std::vector<geometry::TriangleMesh::Face> Fidx;

  // Inline mesh
  if (const auto pm = std::get_if<geometry::InlineMeshPtr>(&meshShape.mesh))
  {
    if (!*pm)
      return nullptr;

    const geometry::TriangleMesh& M = **pm;
    V.assign(M.V.begin(), M.V.end());
    Fidx.assign(M.F.begin(), M.F.end());

    // Apply per-instance scale to inline meshes
    if (meshShape.scale != Eigen::Vector3d(1.0, 1.0, 1.0))
    {
      for (auto& p : V)
        p = p.cwiseProduct(meshShape.scale);
    }
  }
  // External URI
  else if (const auto mref = std::get_if<geometry::MeshRef>(&meshShape.mesh))
  {
    geometry::TriangleMesh M;
    // Loader applies scale for us
    if (!sodf::geometry::loadMeshWithAssimp(mref->uri, meshShape.scale, M))
      return nullptr;

    V = std::move(M.V);
    Fidx = std::move(M.F);
  }
  else
  {
    return nullptr;  // neither inline nor external
  }

  if (!geometry::hasTriangles(V, Fidx))
    return nullptr;

  // Convert faces to the domain’s Tri type
  std::vector<FluidConvexMeshShape::Tri> F;
  F.reserve(Fidx.size());
  for (const auto& t : Fidx)
    F.push_back({ t[0], t[1], t[2] });

  return std::make_shared<FluidConvexMeshShape>(std::move(V), std::move(F));
}

sodf::physics::DomainShapeBasePtr shapeToDomainShape(const sodf::geometry::Shape& s, sodf::physics::DomainType dtype)
{
  using namespace sodf::geometry;
  using sodf::physics::DomainShapeBasePtr;
  using sodf::physics::DomainType;

  // Only Fluids for now
  if (dtype != DomainType::Fluid)
    return nullptr;

  switch (s.type)
  {
    case ShapeType::Box:
    {
      // Canonical: {x, y, z}; fluid box wants (width, length, height)
      // width  = Y, length = Z, height = X
      if (!dim(s).has(DimRole::Y) || !dim(s).has(DimRole::Z) || !dim(s).has(DimRole::Height))
        return nullptr;

      const double w = dim(s).at(DimRole::Y);
      const double l = dim(s).at(DimRole::Z);
      const double h = dim(s).at(DimRole::Height);
      if (!positive(w) || !positive(l) || !positive(h))
        return nullptr;

      return std::make_shared<sodf::physics::FluidBoxShape>(w, l, h);
    }

    case ShapeType::Cylinder:
    {
      // Canonical order: {height_x, radius}
      if (!dim(s).has(DimRole::Height) || !dim(s).has(DimRole::Radius))
        return nullptr;

      const double r = dim(s).at(DimRole::Radius);
      const double h = dim(s).at(DimRole::Height);
      if (!positive(r) || !positive(h))
        return nullptr;

      return std::make_shared<sodf::physics::FluidCylinderShape>(r, h);
    }

    case ShapeType::Cone:
    {
      // Canonical order: {height_x, base_radius, top_radius}
      if (!dim(s).has(DimRole::Height) || !dim(s).has(DimRole::BaseRadius) || !dim(s).has(DimRole::TopRadius))
        return nullptr;

      const double h = dim(s).at(DimRole::Height);
      const double r0 = dim(s).at(DimRole::BaseRadius);
      const double r1 = dim(s).at(DimRole::TopRadius);
      if (!nonneg(r0) || !nonneg(r1) || !positive(h))
        return nullptr;

      return std::make_shared<sodf::physics::FluidConeShape>(r0, r1, h);
    }

    case ShapeType::SphericalSegment:
    {
      // Canonical order: {height_x, base_radius, top_radius}
      if (!dim(s).has(DimRole::Height) || !dim(s).has(DimRole::BaseRadius) || !dim(s).has(DimRole::TopRadius))
        return nullptr;

      const double H = dim(s).at(DimRole::Height);
      const double a1 = dim(s).at(DimRole::BaseRadius);
      const double a2 = dim(s).at(DimRole::TopRadius);
      if (!nonneg(a1) || !nonneg(a2) || !positive(H))
        return nullptr;

      try
      {
        return std::make_shared<sodf::physics::FluidSphericalSegmentShape>(a1, a2, H);
      }
      catch (...)
      {
        return nullptr;  // geometric consistency failed
      }
    }

    case ShapeType::Mesh:
      return shapeMeshToDomainShape(s, dtype);

    default:
      break;
  }

  // Unsupported for Fluid domains
  return nullptr;
}

}  // namespace physics
}  // namespace sodf
