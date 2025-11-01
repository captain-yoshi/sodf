#include <cmath>
#include <vector>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtest/gtest.h>

#include <sodf/physics/fluid_domain_shape.h>

#include <sodf/geometry/mesh_shape.h>
#include <sodf/geometry/mesh.h>

using namespace sodf::physics;
using namespace sodf::geometry;

using Index = sodf::geometry::TriangleMesh::Index;

constexpr double VOLUME_EPS = 1e-9;  // looser than analytic tests; meshes can approximate
constexpr double HEIGHT_EPS = 1e-8;

// --------- Helpers to build convex meshes ---------

static void makeBoxMesh(double W, double L, double H, std::vector<Eigen::Vector3d>& V,
                        std::vector<FluidConvexMeshShape::Tri>& F)
{
  V = {
    { 0.0, 0.0, 0.0 }, { W, 0.0, 0.0 }, { W, L, 0.0 }, { 0.0, L, 0.0 },  // bottom  (0,1,2,3)
    { 0.0, 0.0, H },   { W, 0.0, H },   { W, L, H },   { 0.0, L, H }     // top     (4,5,6,7)
  };

  F.clear();
  F.reserve(12);

  auto tri = [&F](Index a, Index b, Index c) { F.emplace_back(TriangleMesh::Face{ a, b, c }); };

  // bottom (0,1,2,3)  CCW seen from below -> outward is -Z; volumes are |…| later, so OK
  tri(Index{ 0 }, Index{ 1 }, Index{ 2 });
  tri(Index{ 0 }, Index{ 2 }, Index{ 3 });

  // top (4,5,6,7)     CCW seen from above -> outward is +Z
  tri(Index{ 4 }, Index{ 6 }, Index{ 5 });
  tri(Index{ 4 }, Index{ 7 }, Index{ 6 });

  // sides
  tri(Index{ 0 }, Index{ 4 }, Index{ 5 });
  tri(Index{ 0 }, Index{ 5 }, Index{ 1 });

  tri(Index{ 1 }, Index{ 5 }, Index{ 6 });
  tri(Index{ 1 }, Index{ 6 }, Index{ 2 });

  tri(Index{ 2 }, Index{ 6 }, Index{ 7 });
  tri(Index{ 2 }, Index{ 7 }, Index{ 3 });

  tri(Index{ 3 }, Index{ 7 }, Index{ 4 });
  tri(Index{ 3 }, Index{ 4 }, Index{ 0 });
}

// Regular N-gon area with circumradius r
static double regularNgonArea(int N, double r)
{
  return 0.5 * N * r * r * std::sin(2.0 * M_PI / N);
}

static void makePrismMesh_RegularNgon(Index N, double r, double H, std::vector<Eigen::Vector3d>& V,
                                      std::vector<FluidConvexMeshShape::Tri>& F)
{
  V.clear();
  F.clear();
  V.reserve(2 * N);
  // bottom ring z=0, top ring z=H
  for (int i = 0; i < N; ++i)
  {
    double ang = 2.0 * M_PI * i / N;
    V.emplace_back(r * std::cos(ang), r * std::sin(ang), 0.0);
  }
  for (int i = 0; i < N; ++i)
  {
    double ang = 2.0 * M_PI * i / N;
    V.emplace_back(r * std::cos(ang), r * std::sin(ang), H);
  }
  auto addTri = [&](Index a, Index b, Index c) { F.push_back({ a, b, c }); };

  // side quads as two tris
  for (int i = 0; i < N; ++i)
  {
    int i0 = i, i1 = (i + 1) % N;
    int j0 = i + N, j1 = ((i + 1) % N) + N;
    addTri(i0, i1, j1);
    addTri(i0, j1, j0);
  }
  // bottom fan (0..N-1) – pick vertex 0 as fan center
  for (int i = 1; i + 1 < N; ++i)
    addTri(0, i + 1, i);
  // top fan (N..2N-1) – CCW seen from above
  for (int i = 1; i + 1 < N; ++i)
    addTri(N, N + i, N + i + 1);
}

static void makeFrustumMesh_RegularNgon(Index N, double r0, double r1, double H, std::vector<Eigen::Vector3d>& V,
                                        std::vector<FluidConvexMeshShape::Tri>& F)
{
  V.clear();
  F.clear();
  V.reserve(2 * N);
  for (int i = 0; i < N; ++i)
  {
    double ang = 2.0 * M_PI * i / N;
    V.emplace_back(r0 * std::cos(ang), r0 * std::sin(ang), 0.0);  // bottom
  }
  for (int i = 0; i < N; ++i)
  {
    double ang = 2.0 * M_PI * i / N;
    V.emplace_back(r1 * std::cos(ang), r1 * std::sin(ang), H);  // top
  }
  auto addTri = [&](Index a, Index b, Index c) { F.push_back({ a, b, c }); };

  for (Index i = 0; i < N; ++i)
  {
    Index i0 = i, i1 = (i + 1) % N;
    Index j0 = i + N, j1 = ((i + 1) % N) + N;
    addTri(i0, i1, j1);
    addTri(i0, j1, j0);
  }
  // bottom fan
  for (Index i = 1; i + 1 < N; ++i)
    F.push_back({ 0, i + 1, i });
  // top fan
  for (Index i = 1; i + 1 < N; ++i)
    F.push_back({ N, N + i, N + i + 1 });
}

// -------------------- Tests --------------------

TEST(ConvexMesh_Box, MatchesBoxPrimitive)
{
  const double W = 0.5, L = 0.3, H = 0.8;
  std::vector<Eigen::Vector3d> V;
  std::vector<FluidConvexMeshShape::Tri> F;
  makeBoxMesh(W, L, H, V, F);

  FluidConvexMeshShape mesh(V, F);
  FluidBoxShape box(W, L, H);

  // capacity & height
  EXPECT_NEAR(box.getMaxFillVolume(), mesh.getMaxFillVolume(), VOLUME_EPS);
  EXPECT_NEAR(box.getMaxFillHeight(), mesh.getMaxFillHeight(), HEIGHT_EPS);

  // some sample heights
  for (double h : { 0.0, 0.2, 0.4, 0.8 })
  {
    EXPECT_NEAR(box.getFillVolume(h), mesh.getFillVolume(h), VOLUME_EPS);
    if (h > 0)
    {
      double v = box.getFillVolume(h);
      EXPECT_NEAR(h, mesh.getFillHeight(v), HEIGHT_EPS);
    }
  }
}

TEST(ConvexMesh_CylinderNgon, MatchesAnalyticNgonPrism)
{
  const int N = 128;  // polygon resolution (higher = closer to circle)
  const double r = 0.5;
  const double H = 0.8;

  std::vector<Eigen::Vector3d> V;
  std::vector<FluidConvexMeshShape::Tri> F;
  makePrismMesh_RegularNgon(N, r, H, V, F);
  FluidConvexMeshShape mesh(V, F);

  const double A = regularNgonArea(N, r);
  const double Vcap = A * H;

  EXPECT_NEAR(Vcap, mesh.getMaxFillVolume(), 1e-9);
  EXPECT_NEAR(H, mesh.getMaxFillHeight(), HEIGHT_EPS);

  // For a vertical prism, volume is linear in h
  for (double h : { 0.0, 0.2, 0.4, 0.8 })
  {
    EXPECT_NEAR(A * h, mesh.getFillVolume(h), 1e-9);
    if (h > 0)
    {
      EXPECT_NEAR(h, mesh.getFillHeight(A * h), HEIGHT_EPS);
    }
  }

  // Compare approximately to circular cylinder primitive
  FluidCylinderShape cyl(r, H);
  // area difference between circle and N-gon yields small mismatch; allow relative tolerance
  EXPECT_NEAR(cyl.getMaxFillVolume(), mesh.getMaxFillVolume(), 5e-3);
  EXPECT_NEAR(cyl.getFillVolume(0.4), mesh.getFillVolume(0.4), 5e-3);
}

TEST(ConvexMesh_FrustumNgon, MatchesAnalyticFrustumOfSimilarPolygons)
{
  const int N = 128;
  const double r0 = 0.5;  // bottom circumradius
  const double r1 = 0.3;  // top circumradius
  const double H = 0.8;

  std::vector<Eigen::Vector3d> V;
  std::vector<FluidConvexMeshShape::Tri> F;
  makeFrustumMesh_RegularNgon(N, r0, r1, H, V, F);
  FluidConvexMeshShape mesh(V, F);

  const double A0 = regularNgonArea(N, r0);
  const double A1 = regularNgonArea(N, r1);
  // Frustum volume for similar polygons: V = h/3 * (A0 + A1 + sqrt(A0*A1))
  const double Vfr = H / 3.0 * (A0 + A1 + std::sqrt(A0 * A1));

  EXPECT_NEAR(Vfr, mesh.getMaxFillVolume(), 1e-9);
  EXPECT_NEAR(H, mesh.getMaxFillHeight(), HEIGHT_EPS);

  // Spot checks vs cone primitive (circle) within a few permil
  FluidConeShape cone(r0, r1, H);
  EXPECT_NEAR(cone.getMaxFillVolume(), mesh.getMaxFillVolume(), 7e-3);
  EXPECT_NEAR(cone.getFillVolume(0.4), mesh.getFillVolume(0.4), 7e-3);
  // Inversion consistency
  double v_half = mesh.getFillVolume(0.5 * H);
  EXPECT_NEAR(0.5 * H, mesh.getFillHeight(v_half), HEIGHT_EPS);
}

TEST(ConvexMesh_Orientation_ZRotate, InvarianceUnderYaw)
{
  // Box: rotate about Z (yaw) → gravity unchanged, results identical
  std::vector<Eigen::Vector3d> V;
  std::vector<FluidConvexMeshShape::Tri> F;
  makeBoxMesh(0.5, 0.3, 0.8, V, F);
  FluidConvexMeshShape mesh(V, F);

  const double cap = mesh.getMaxFillVolume();
  const double H = mesh.getMaxFillHeight();

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(M_PI / 3.0, Eigen::Vector3d::UnitZ())).toRotationMatrix();

  sodf::physics::FillEnv env;
  env.g_down_world = Eigen::Vector3d(0, 0, -1);
  env.T_world_domain = T;
  env.p_base_world = Eigen::Vector3d::Zero();

  EXPECT_NEAR(cap, mesh.getMaxFillVolume(), VOLUME_EPS);
  EXPECT_NEAR(H, mesh.getMaxFillHeight(), HEIGHT_EPS);

  for (double h : { 0.2, 0.6 })
  {
    const double v_default = mesh.getFillVolume(h);
    const double v_env = mesh.getFillVolumeWithEnv(h, env);

    EXPECT_NEAR(v_default, v_env, VOLUME_EPS);
    EXPECT_NEAR(h, mesh.getFillHeightWithEnv(v_default, env), HEIGHT_EPS);
  }
}

TEST(ConvexMesh_Orientation_TiltWithUpdatedBase, ConsistentHeights)
{
  // Tilt the box 30° around X; update base point to the lowest vertex so z>=0
  std::vector<Eigen::Vector3d> V;
  std::vector<FluidConvexMeshShape::Tri> F;
  const double W = 0.5, L = 0.3, H = 0.8;
  makeBoxMesh(W, L, H, V, F);

  FluidConvexMeshShape mesh(V, F);
  const double cap = mesh.getMaxFillVolume();

  // Compute lowest vertex after tilt to set base
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitX())).toRotationMatrix();

  double minz = std::numeric_limits<double>::infinity();
  for (auto v : V)
  {
    Eigen::Vector3d w = T * v;
    minz = std::min(minz, w.z());
  }
  Eigen::Vector3d base = { 0, 0, minz };  // place base plane at lowest vertex z

  sodf::physics::FillEnv env;
  env.g_down_world = Eigen::Vector3d(0, 0, -1);
  env.T_world_domain = T;
  env.p_base_world = base;

  // Capacity must be unchanged
  EXPECT_NEAR(cap, mesh.getMaxFillVolume(), VOLUME_EPS);

  // Endpoints
  EXPECT_NEAR(0.0, mesh.getFillVolumeWithEnv(0.0, env), VOLUME_EPS);
  EXPECT_NEAR(mesh.getMaxFillVolume(),
              mesh.getFillVolumeWithEnv(mesh.getFillHeightWithEnv(mesh.getMaxFillVolume(), env), env), VOLUME_EPS);

  // Inversion self-consistency at multiple fractions
  for (double f : { 0.1, 0.33, 0.57, 0.9 })
  {
    const double v = f * mesh.getMaxFillVolume();
    const double h = mesh.getFillHeightWithEnv(v, env);
    EXPECT_GE(h, 0.0);
    EXPECT_NEAR(v, mesh.getFillVolumeWithEnv(h, env), 1e-8);
  }
}

TEST(ConvexMesh_EdgeCases, GuardsAndInvalid)
{
  // Degenerate/invalid mesh (too few verts) should throw in ctor
  std::vector<Eigen::Vector3d> Vbad = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } };  // only 3 verts
  std::vector<FluidConvexMeshShape::Tri> Fbad = { { 0, 1, 2 } };
  EXPECT_THROW(FluidConvexMeshShape(Vbad, Fbad), std::invalid_argument);

  // Valid tiny box: inputs outside [0,H] clamp to 0 or capacity
  std::vector<Eigen::Vector3d> V;
  std::vector<FluidConvexMeshShape::Tri> F;
  makeBoxMesh(0.1, 0.1, 0.2, V, F);
  FluidConvexMeshShape mesh(V, F);

  EXPECT_EQ(0.0, mesh.getFillVolume(-1.0));
  EXPECT_NEAR(mesh.getMaxFillVolume(), mesh.getFillVolume(1e9), 1e-12);

  EXPECT_EQ(0.0, mesh.getFillHeight(-1.0));
  EXPECT_NEAR(mesh.getMaxFillHeight(), mesh.getFillHeight(mesh.getMaxFillVolume() + 1.0), 1e-12);
}

TEST(ConvexMesh_Clipper, VertexEdgeFaceCoincidence)
{
  using V3 = Eigen::Vector3d;
  auto Vtet = [](V3 a, V3 b, V3 c, V3 d) { return std::abs(sodf::geometry::signed_tetra_volume(a, b, c, d)); };

  // A standard tetra resting on z=0 face; full volume is 1/6
  V3 a{ 0, 0, 0 }, b{ 1, 0, 0 }, c{ 0, 1, 0 }, d{ 0, 0, 1 };
  const double Vfull = Vtet(a, b, c, d);

  auto clip = [&](double h) { return sodf::geometry::clipped_tetra_volume_zleq(a, b, c, d, h, 1e-12); };

  // Face-coincidence: h=0 keeps zero volume
  EXPECT_DOUBLE_EQ(0.0, clip(0.0));

  // Vertex-coincidence: h=1 passes the whole tetra
  EXPECT_NEAR(Vfull, clip(1.0), 1e-15);

  // Edge-coincidence: choose h that passes through midpoint of segment d–a (z=0.5)
  EXPECT_GT(clip(0.5), 0.0);
  EXPECT_LT(clip(0.5), Vfull);

  // Tiny epsilon around a vertex/edge to hit |dz|<kEps path
  const double h_eps = 1.0 + 1e-14;
  EXPECT_NEAR(Vfull, clip(h_eps), 1e-15);
}

TEST(ConvexMesh_Box, KnotHeightsAtVertexZ)
{
  std::vector<Eigen::Vector3d> V;
  std::vector<FluidConvexMeshShape::Tri> F;
  makeBoxMesh(0.5, 0.3, 0.8, V, F);
  FluidConvexMeshShape mesh(V, F);

  // z-levels present in the mesh (and interior)
  std::vector<double> Z = { 0.0, 0.8 };
  for (double z : Z)
  {
    double v = mesh.getFillVolume(z);
    // Right/left limits (step just below/above)
    EXPECT_LE(mesh.getFillVolume(std::nextafter(z, 0.0)), v + 1e-15);
    EXPECT_GE(mesh.getFillVolume(std::nextafter(z, 1.0)), v - 1e-15);
  }

  // Monotone and 1-Lipschitz in the ‘height→volume’ sense for a prism
  EXPECT_LE(mesh.getFillVolume(0.2), mesh.getFillVolume(0.4));
}

TEST(ConvexMesh_Generic, HeightVolumeRoundtripDense)
{
  std::vector<Eigen::Vector3d> V;
  std::vector<FluidConvexMeshShape::Tri> F;
  makePrismMesh_RegularNgon(24, 0.37, 0.91, V, F);  // arbitrary convex, non-box
  FluidConvexMeshShape mesh(V, F);

  const double H = mesh.getMaxFillHeight();
  for (int i = 0; i <= 50; ++i)
  {
    double h = H * (i / 50.0);
    double v = mesh.getFillVolume(h);
    double h2 = mesh.getFillHeight(v);
    EXPECT_NEAR(h, h2, 1e-9) << "i=" << i;
  }
}

TEST(ConvexMesh_StateAPIs, GravityNormalizationAndRigidInvariance)
{
  std::vector<Eigen::Vector3d> V;
  std::vector<FluidConvexMeshShape::Tri> F;
  makeBoxMesh(0.5, 0.3, 0.8, V, F);
  FluidConvexMeshShape mesh(V, F);

  const double cap0 = mesh.getMaxFillVolume();
  const double H0 = mesh.getMaxFillHeight();

  // Build an arbitrary rigid transform
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(1.0, -2.0, 3.0);
  T.linear() = (Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(1.1, Eigen::Vector3d::UnitZ()))
                   .toRotationMatrix();

  // env A: standard down, with T, base at origin
  sodf::physics::FillEnv envA;
  envA.g_down_world = Eigen::Vector3d(0, 0, -1);
  envA.T_world_domain = T;
  envA.p_base_world = Eigen::Vector3d::Zero();
  EXPECT_NEAR(cap0, mesh.getMaxFillVolume(), 1e-15);

  // env B: non-unit, skewed gravity → different up-axis
  sodf::physics::FillEnv envB = envA;
  envB.g_down_world = Eigen::Vector3d(-2, 0, 5);
  const double H1 = mesh.getFillHeightWithEnv(cap0, envB);  // "max height" in envB
  EXPECT_GT(H1, 0.0);

  // Base shift changes heights but not capacity
  const double v_mid = 0.5 * cap0;
  const double h_mid_before = mesh.getFillHeightWithEnv(v_mid, envB);
  envB.p_base_world = Eigen::Vector3d(10, 10, 10);
  const double h_mid_after = mesh.getFillHeightWithEnv(v_mid, envB);
  EXPECT_NEAR(cap0, mesh.getMaxFillVolume(), 1e-15);
  EXPECT_NE(h_mid_before, h_mid_after);
}

TEST(ConvexMesh_BasePlacement, BaseBelowOrAboveMesh)
{
  std::vector<Eigen::Vector3d> V;
  std::vector<FluidConvexMeshShape::Tri> F;
  makeBoxMesh(0.5, 0.3, 0.8, V, F);

  FluidConvexMeshShape mesh(V, F);
  const double cap = mesh.getMaxFillVolume();
  // Base far below → "max height" with env equals geometric height (0.8)
  sodf::physics::FillEnv env1;
  env1.g_down_world = Eigen::Vector3d(0, 0, -1);
  env1.T_world_domain = Eigen::Isometry3d::Identity();
  env1.p_base_world = Eigen::Vector3d(0, 0, -100);
  EXPECT_NEAR(0.8, mesh.getFillHeightWithEnv(cap, env1), 1e-12);

  // Base cutting through mesh bottom (some z<0). Volume at small positive h should be smaller than base-at-bottom case
  sodf::physics::FillEnv env2 = env1;
  env2.p_base_world = Eigen::Vector3d(0, 0, 0.05);
  EXPECT_GT(mesh.getFillVolumeWithEnv(0.05, env2), mesh.getFillVolumeWithEnv(0.05, env1));

  // Base just epsilon below bottom to trigger z_min_→0.0 snapping
  sodf::physics::FillEnv env3 = env1;
  env3.p_base_world = Eigen::Vector3d(0, 0, -1e-12);
  EXPECT_NEAR(mesh.getFillHeightWithEnv(cap, env1), mesh.getFillHeightWithEnv(cap, env3), 2e-12);
}

TEST(ConvexMesh_EdgeCases, MoreCtorGuards)
{
  std::vector<Eigen::Vector3d> Vok = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
  std::vector<FluidConvexMeshShape::Tri> Fempty;
  EXPECT_THROW(FluidConvexMeshShape(Vok, Fempty), std::invalid_argument);

  // Out-of-range indices (behavior: expect throw if you add such a guard in ctor; if not, keep as EXPECT_NO_THROW)
  std::vector<FluidConvexMeshShape::Tri> Fbad = { { 0, 1, 99 } };
  // EXPECT_THROW(FluidConvexMeshShape(Vok, Fbad, {0,0,1}, {0,0,0}), std::invalid_argument);
  (void)Fbad;  // enable once guard exists
}

TEST(ConvexMesh_Capacity, RigidInvariantLocalBuild)
{
  std::vector<Eigen::Vector3d> V;
  std::vector<FluidConvexMeshShape::Tri> F;
  const double W = 0.2, L = 0.4, H = 0.6;
  makeBoxMesh(W, L, H, V, F);

  // Rotate the vertices beforehand (so "local" is not axis-aligned)
  Eigen::AngleAxisd R(M_PI / 4.0, Eigen::Vector3d::UnitY());
  for (auto& p : V)
    p = R * p;

  FluidConvexMeshShape mesh(V, F);
  EXPECT_NEAR(W * L * H, mesh.getMaxFillVolume(), 1e-12);
}

TEST(ConvexMesh_Numerics, ExtremeAspectRatios)
{
  // Ultra-thin plate
  std::vector<Eigen::Vector3d> V1;
  std::vector<FluidConvexMeshShape::Tri> F1;
  makeBoxMesh(1.0, 1.0, 1e-6, V1, F1);
  FluidConvexMeshShape m1(V1, F1);
  EXPECT_NEAR(1e-6, m1.getMaxFillVolume(), 1e-12);

  // Very tall, skinny prism
  std::vector<Eigen::Vector3d> V2;
  std::vector<FluidConvexMeshShape::Tri> F2;
  makeBoxMesh(1e-6, 1e-6, 10.0, V2, F2);
  FluidConvexMeshShape m2(V2, F2);
  EXPECT_NEAR(1e-12 * 10.0, m2.getMaxFillVolume(), 1e-18);

  // Round-trip still behaves
  double v = 0.5 * m2.getMaxFillVolume();
  EXPECT_NEAR(v, m2.getFillVolume(m2.getFillHeight(v)), 1e-12);
}

TEST(StackedDomains, OverflowUnderflowGuards)
{
  using namespace sodf::physics;
  std::vector<DomainShapeBasePtr> layers;
  layers.emplace_back(std::make_shared<FluidBoxShape>(0.5, 0.3, 0.2));
  layers.emplace_back(std::make_shared<FluidCylinderShape>(0.1, 0.4));

  const double TOL = 1e-12;

  // Height greater than total -> getFillVolume returns 0 per contract
  EXPECT_EQ(0.0, getFillVolume(layers, 1e9, TOL));

  // Volume greater than total -> getFillHeight returns 0 per contract
  EXPECT_EQ(0.0, getFillHeight(layers, 1e9, TOL));
}

TEST(ConeFrustum_Specials, CylinderAndPointedCone)
{
  using namespace sodf::physics;
  // r0==r1 → cylinder equivalence
  FluidConeShape fr_cyl(0.3, 0.3, 1.2);
  FluidCylinderShape cyl(0.3, 1.2);
  EXPECT_NEAR(cyl.getMaxFillVolume(), fr_cyl.getMaxFillVolume(), 1e-12);
  EXPECT_NEAR(cyl.getFillVolume(0.6), fr_cyl.getFillVolume(0.6), 1e-12);
  EXPECT_NEAR(0.6, fr_cyl.getFillHeight(cyl.getFillVolume(0.6)), 1e-10);

  // Proper cone (top radius 0)
  FluidConeShape cone(0.4, 0.0, 1.0);
  double v = cone.getFillVolume(0.25);
  EXPECT_NEAR(0.25, cone.getFillHeight(v), 1e-10);
}
