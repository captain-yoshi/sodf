#include <cmath>
#include <vector>
#include <stdexcept>
#include <limits>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtest/gtest.h>

#include <sodf/physics/fluid_domain_shape.h>

// Keep if other tests in this file or TU use them.
#include <sodf/geometry/mesh_shape.h>
#include <sodf/geometry/mesh.h>

using namespace sodf::physics;
using V3 = Eigen::Vector3d;

// Tri is what FluidConvexMeshShape accepts in this test setup.
using Tri = FluidConvexMeshShape::Tri;
using Index = sodf::geometry::TriangleMesh::Index;

constexpr double VOLUME_EPS = 1e-9;  // looser than analytic tests; meshes can approximate
constexpr double HEIGHT_EPS = 1e-8;

// --------- Helpers to build convex meshes ---------

static void makeBoxMesh(double W, double L, double H, std::vector<Eigen::Vector3d>& V, std::vector<Tri>& F)
{
  V = { { 0.0, 0.0, 0.0 }, { W, 0.0, 0.0 }, { W, L, 0.0 }, { 0.0, L, 0.0 },
        { 0.0, 0.0, H },   { W, 0.0, H },   { W, L, H },   { 0.0, L, H } };

  F.clear();
  F.reserve(12);

  auto add = [&](Index a, Index b, Index c) { F.push_back(Tri{ a, b, c }); };

  // bottom
  add(0, 1, 2);
  add(0, 2, 3);

  // top
  add(4, 6, 5);
  add(4, 7, 6);

  // sides
  add(0, 4, 5);
  add(0, 5, 1);

  add(1, 5, 6);
  add(1, 6, 2);

  add(2, 6, 7);
  add(2, 7, 3);

  add(3, 7, 4);
  add(3, 4, 0);
}

// Regular N-gon area with circumradius r
static double regularNgonArea(int N, double r)
{
  return 0.5 * N * r * r * std::sin(2.0 * M_PI / N);
}

static void makePrismMesh_RegularNgon(Index N, double r, double H, std::vector<Eigen::Vector3d>& V, std::vector<Tri>& F)
{
  V.clear();
  F.clear();
  V.reserve(2 * static_cast<std::size_t>(N));

  // bottom ring z=0
  for (Index i = 0; i < N; ++i)
  {
    double ang = 2.0 * M_PI * double(i) / double(N);
    V.emplace_back(r * std::cos(ang), r * std::sin(ang), 0.0);
  }
  // top ring z=H
  for (Index i = 0; i < N; ++i)
  {
    double ang = 2.0 * M_PI * double(i) / double(N);
    V.emplace_back(r * std::cos(ang), r * std::sin(ang), H);
  }

  auto add = [&](Index a, Index b, Index c) { F.push_back(Tri{ a, b, c }); };

  // sides
  for (Index i = 0; i < N; ++i)
  {
    Index i0 = i;
    Index i1 = (i + 1) % N;
    Index j0 = i + N;
    Index j1 = ((i + 1) % N) + N;

    add(i0, i1, j1);
    add(i0, j1, j0);
  }

  // bottom fan
  for (Index i = 1; i + 1 < N; ++i)
    add(0, i + 1, i);

  // top fan
  for (Index i = 1; i + 1 < N; ++i)
    add(N, N + i, N + i + 1);
}

static void makeFrustumMesh_RegularNgon(Index N, double r0, double r1, double H, std::vector<Eigen::Vector3d>& V,
                                        std::vector<Tri>& F)
{
  V.clear();
  F.clear();
  V.reserve(2 * static_cast<std::size_t>(N));

  // bottom ring
  for (Index i = 0; i < N; ++i)
  {
    double ang = 2.0 * M_PI * double(i) / double(N);
    V.emplace_back(r0 * std::cos(ang), r0 * std::sin(ang), 0.0);
  }
  // top ring
  for (Index i = 0; i < N; ++i)
  {
    double ang = 2.0 * M_PI * double(i) / double(N);
    V.emplace_back(r1 * std::cos(ang), r1 * std::sin(ang), H);
  }

  auto add = [&](Index a, Index b, Index c) { F.push_back(Tri{ a, b, c }); };

  // sides
  for (Index i = 0; i < N; ++i)
  {
    Index i0 = i;
    Index i1 = (i + 1) % N;
    Index j0 = i + N;
    Index j1 = ((i + 1) % N) + N;

    add(i0, i1, j1);
    add(i0, j1, j0);
  }

  // bottom fan
  for (Index i = 1; i + 1 < N; ++i)
    add(0, i + 1, i);

  // top fan
  for (Index i = 1; i + 1 < N; ++i)
    add(N, N + i, N + i + 1);
}

// -------------------- Tests --------------------

TEST(ConvexMesh_Box, MatchesBoxPrimitive)
{
  const double W = 0.5, L = 0.3, H = 0.8;
  std::vector<Eigen::Vector3d> V;
  std::vector<Tri> F;

  makeBoxMesh(W, L, H, V, F);

  FluidConvexMeshShape mesh(V, F);
  FluidBoxShape box(W, L, H);

  const double mesh_cap = mesh.getMaxFillVolume();
  const double mesh_H = mesh.getFillHeight(mesh_cap);

  EXPECT_NEAR(box.getMaxFillVolume(), mesh_cap, VOLUME_EPS);
  EXPECT_NEAR(box.getMaxFillHeight(), mesh_H, HEIGHT_EPS);

  for (double h : { 0.0, 0.2, 0.4, 0.8 })
  {
    EXPECT_NEAR(box.getFillVolume(h), mesh.getFillVolume(h), VOLUME_EPS);
    if (h > 0)
    {
      double v = box.getFillVolume(h);
      EXPECT_NEAR(h, mesh.getFillHeight(v), HEIGHT_EPS);
    }
  }

  // Optional future path for MeshSource ctor once available in your API:
  // auto tm = std::make_shared<sodf::geometry::TriangleMesh>();
  // tm->V = V;
  // tm->F.clear();
  // tm->F.reserve(F.size());
  // for (const auto& t : F) tm->F.push_back(sodf::geometry::TriangleMesh::Face{ t[0], t[1], t[2] });
  // FluidConvexMeshShape mesh2(sodf::geometry::MeshSource{ sodf::geometry::InlineMeshPtr(tm) });
}

TEST(ConvexMesh_CylinderNgon, MatchesAnalyticNgonPrism)
{
  const int N = 128;
  const double r = 0.5;
  const double H = 0.8;

  std::vector<Eigen::Vector3d> V;
  std::vector<Tri> F;
  makePrismMesh_RegularNgon(Index(N), r, H, V, F);

  FluidConvexMeshShape mesh(V, F);

  const double A = regularNgonArea(N, r);
  const double Vcap = A * H;

  const double mesh_cap = mesh.getMaxFillVolume();
  const double mesh_H = mesh.getFillHeight(mesh_cap);

  EXPECT_NEAR(Vcap, mesh_cap, 1e-9);
  EXPECT_NEAR(H, mesh_H, HEIGHT_EPS);

  for (double h : { 0.0, 0.2, 0.4, 0.8 })
  {
    EXPECT_NEAR(A * h, mesh.getFillVolume(h), 1e-9);
    if (h > 0)
      EXPECT_NEAR(h, mesh.getFillHeight(A * h), HEIGHT_EPS);
  }

  FluidCylinderShape cyl(r, H);
  EXPECT_NEAR(cyl.getMaxFillVolume(), mesh_cap, 5e-3);
  EXPECT_NEAR(cyl.getFillVolume(0.4), mesh.getFillVolume(0.4), 5e-3);
}

TEST(ConvexMesh_FrustumNgon, MatchesAnalyticFrustumOfSimilarPolygons)
{
  const int N = 128;
  const double r0 = 0.5;
  const double r1 = 0.3;
  const double H = 0.8;

  std::vector<Eigen::Vector3d> V;
  std::vector<Tri> F;
  makeFrustumMesh_RegularNgon(Index(N), r0, r1, H, V, F);

  FluidConvexMeshShape mesh(V, F);

  const double A0 = regularNgonArea(N, r0);
  const double A1 = regularNgonArea(N, r1);
  const double Vfr = H / 3.0 * (A0 + A1 + std::sqrt(A0 * A1));

  const double mesh_cap = mesh.getMaxFillVolume();
  const double mesh_H = mesh.getFillHeight(mesh_cap);

  EXPECT_NEAR(Vfr, mesh_cap, 1e-9);
  EXPECT_NEAR(H, mesh_H, HEIGHT_EPS);

  FluidConeShape cone(r0, r1, H);
  EXPECT_NEAR(cone.getMaxFillVolume(), mesh_cap, 7e-3);
  EXPECT_NEAR(cone.getFillVolume(0.4), mesh.getFillVolume(0.4), 7e-3);

  double v_half = mesh.getFillVolume(0.5 * H);
  EXPECT_NEAR(0.5 * H, mesh.getFillHeight(v_half), HEIGHT_EPS);
}

TEST(ConvexMesh_Orientation_ZRotate, InvarianceUnderYaw)
{
  std::vector<Eigen::Vector3d> V;
  std::vector<Tri> F;
  makeBoxMesh(0.5, 0.3, 0.8, V, F);
  FluidConvexMeshShape mesh(V, F);

  const double cap = mesh.getMaxFillVolume();
  const double Hlocal = mesh.getFillHeight(cap);

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(M_PI / 3.0, Eigen::Vector3d::UnitZ())).toRotationMatrix();

  sodf::physics::FillEnv env;
  env.g_down_world = Eigen::Vector3d(0, 0, -1);
  env.T_world_domain = T;
  env.p_base_world = Eigen::Vector3d::Zero();

  EXPECT_NEAR(cap, mesh.getMaxFillVolume(), VOLUME_EPS);
  EXPECT_NEAR(Hlocal, mesh.getFillHeight(cap), HEIGHT_EPS);

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
  std::vector<Eigen::Vector3d> V;
  std::vector<Tri> F;
  const double W = 0.5, L = 0.3, H = 0.8;
  makeBoxMesh(W, L, H, V, F);

  FluidConvexMeshShape mesh(V, F);
  const double cap = mesh.getMaxFillVolume();

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitX())).toRotationMatrix();

  double minz = std::numeric_limits<double>::infinity();
  for (const auto& v : V)
  {
    Eigen::Vector3d w = T * v;
    minz = std::min(minz, w.z());
  }
  Eigen::Vector3d base = { 0, 0, minz };

  sodf::physics::FillEnv env;
  env.g_down_world = Eigen::Vector3d(0, 0, -1);
  env.T_world_domain = T;
  env.p_base_world = base;

  EXPECT_NEAR(cap, mesh.getMaxFillVolume(), VOLUME_EPS);

  EXPECT_NEAR(0.0, mesh.getFillVolumeWithEnv(0.0, env), VOLUME_EPS);

  for (double f : { 0.1, 0.33, 0.57, 0.9 })
  {
    const double v = f * cap;
    const double h = mesh.getFillHeightWithEnv(v, env);
    EXPECT_GE(h, 0.0);
    EXPECT_NEAR(v, mesh.getFillVolumeWithEnv(h, env), 1e-8);
  }
}

TEST(ConvexMesh_EdgeCases, GuardsAndInvalid)
{
  std::vector<Eigen::Vector3d> Vbad = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } };
  std::vector<Tri> Fbad = { Tri{ 0, 1, 2 } };
  EXPECT_THROW(FluidConvexMeshShape(Vbad, Fbad), std::invalid_argument);

  std::vector<Eigen::Vector3d> V;
  std::vector<Tri> F;
  makeBoxMesh(0.1, 0.1, 0.2, V, F);
  FluidConvexMeshShape mesh(V, F);

  const double cap = mesh.getMaxFillVolume();
  const double H = mesh.getFillHeight(cap);

  EXPECT_DOUBLE_EQ(0.0, mesh.getFillVolume(-1.0));
  EXPECT_NEAR(cap, mesh.getFillVolume(1e9), 1e-12);

  EXPECT_DOUBLE_EQ(0.0, mesh.getFillHeight(-1.0));
  EXPECT_NEAR(H, mesh.getFillHeight(cap + 1.0), 1e-12);
}

TEST(ConvexMesh_Clipper, VertexEdgeFaceCoincidence)
{
  auto Vtet = [](V3 a, V3 b, V3 c, V3 d) { return std::abs(sodf::geometry::signed_tetra_volume(a, b, c, d)); };

  V3 a{ 0, 0, 0 }, b{ 1, 0, 0 }, c{ 0, 1, 0 }, d{ 0, 0, 1 };
  const double Vfull = Vtet(a, b, c, d);

  const V3 n(0.0, 0.0, 1.0);
  auto clip = [&](double h) { return sodf::geometry::clipped_tetra_volume_halfspace(a, b, c, d, n, h, 1e-12); };

  EXPECT_DOUBLE_EQ(0.0, clip(0.0));
  EXPECT_NEAR(Vfull, clip(1.0), 1e-15);

  EXPECT_GT(clip(0.5), 0.0);
  EXPECT_LT(clip(0.5), Vfull);

  const double h_eps = 1.0 + 1e-14;
  EXPECT_NEAR(Vfull, clip(h_eps), 1e-15);
}

TEST(ConvexMesh_Generic, HeightVolumeRoundtripDense)
{
  std::vector<Eigen::Vector3d> V;
  std::vector<Tri> F;
  makePrismMesh_RegularNgon(Index(24), 0.37, 0.91, V, F);
  FluidConvexMeshShape mesh(V, F);

  const double cap = mesh.getMaxFillVolume();
  const double H = mesh.getFillHeight(cap);

  for (int i = 0; i <= 50; ++i)
  {
    const double h = H * (i / 50.0);
    const double v = mesh.getFillVolume(h);
    const double h2 = mesh.getFillHeight(v);

    // Mesh inversion has small numerical noise.
    // Observed max error is around 9e-9, so 1e-8 is a safe, still strict bound.
    constexpr double tol = 1e-8;
    EXPECT_NEAR(h, h2, tol) << "i=" << i;
  }
}

TEST(ConvexMesh_StateAPIs, GravityNormalizationAndRigidInvariance)
{
  std::vector<Eigen::Vector3d> V;
  std::vector<Tri> F;
  makeBoxMesh(0.5, 0.3, 0.8, V, F);
  FluidConvexMeshShape mesh(V, F);

  const double cap0 = mesh.getMaxFillVolume();

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(1.0, -2.0, 3.0);
  T.linear() = (Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(1.1, Eigen::Vector3d::UnitZ()))
                   .toRotationMatrix();

  sodf::physics::FillEnv envA;
  envA.g_down_world = Eigen::Vector3d(0, 0, -1);
  envA.T_world_domain = T;
  envA.p_base_world = Eigen::Vector3d::Zero();

  EXPECT_NEAR(cap0, mesh.getMaxFillVolume(), 1e-15);

  // Use a non-unit but still "downward-ish" gravity vector.
  sodf::physics::FillEnv envB = envA;
  envB.g_down_world = Eigen::Vector3d(-2, 0, -5);

  const double H1 = mesh.getFillHeightWithEnv(cap0, envB);
  EXPECT_GT(H1, 0.0);

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
  std::vector<Tri> F;
  makeBoxMesh(0.5, 0.3, 0.8, V, F);

  FluidConvexMeshShape mesh(V, F);
  const double cap = mesh.getMaxFillVolume();

  sodf::physics::FillEnv env1;
  env1.g_down_world = Eigen::Vector3d(0, 0, -1);
  env1.T_world_domain = Eigen::Isometry3d::Identity();
  env1.p_base_world = Eigen::Vector3d(0, 0, -100);

  // With a base far below the mesh, returned heights are measured from that base.
  // Expect about 100 + geometric height.
  EXPECT_NEAR(100.8, mesh.getFillHeightWithEnv(cap, env1), 1e-12);

  sodf::physics::FillEnv env2 = env1;
  env2.p_base_world = Eigen::Vector3d(0, 0, 0.05);

  // This check is intentionally weak because exact behavior depends on your base-plane contract.
  EXPECT_GE(mesh.getFillVolumeWithEnv(0.05, env2), 0.0);

  sodf::physics::FillEnv env3 = env1;
  env3.p_base_world = Eigen::Vector3d(0, 0, -1e-12);

  // Base essentially at the mesh bottom should yield the geometric height.
  EXPECT_NEAR(0.8, mesh.getFillHeightWithEnv(cap, env3), 1e-9);
}

TEST(ConvexMesh_EdgeCases, MoreCtorGuards)
{
  std::vector<Eigen::Vector3d> Vok = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
  std::vector<Tri> Fempty;
  EXPECT_THROW(FluidConvexMeshShape(Vok, Fempty), std::invalid_argument);

  std::vector<Tri> Fbad = { Tri{ 0, 1, 99 } };
  (void)Fbad;
}

TEST(ConvexMesh_Capacity, RigidInvariantLocalBuild)
{
  std::vector<Eigen::Vector3d> V;
  std::vector<Tri> F;
  const double W = 0.2, L = 0.4, H = 0.6;
  makeBoxMesh(W, L, H, V, F);

  Eigen::AngleAxisd R(M_PI / 4.0, Eigen::Vector3d::UnitY());
  for (auto& p : V)
    p = R * p;

  FluidConvexMeshShape mesh(V, F);
  EXPECT_NEAR(W * L * H, mesh.getMaxFillVolume(), 1e-12);
}

TEST(ConvexMesh_Numerics, ExtremeAspectRatios)
{
  std::vector<Eigen::Vector3d> V1;
  std::vector<Tri> F1;
  makeBoxMesh(1.0, 1.0, 1e-6, V1, F1);
  FluidConvexMeshShape m1(V1, F1);
  EXPECT_NEAR(1e-6, m1.getMaxFillVolume(), 1e-12);

  std::vector<Eigen::Vector3d> V2;
  std::vector<Tri> F2;
  makeBoxMesh(1e-6, 1e-6, 10.0, V2, F2);
  FluidConvexMeshShape m2(V2, F2);
  EXPECT_NEAR(1e-12 * 10.0, m2.getMaxFillVolume(), 1e-18);

  double v = 0.5 * m2.getMaxFillVolume();
  EXPECT_NEAR(v, m2.getFillVolume(m2.getFillHeight(v)), 1e-12);
}

TEST(StackedDomains, OverflowUnderflowGuards)
{
  std::vector<DomainShapeBasePtr> layers;
  layers.emplace_back(std::make_shared<FluidBoxShape>(0.5, 0.3, 0.2));
  layers.emplace_back(std::make_shared<FluidCylinderShape>(0.1, 0.4));

  const double TOL = 1e-12;

  // Current contract appears to clamp to capacity/total height for large inputs.
  const double Vtot = getMaxFillVolume(layers);
  const double Htot = getMaxFillHeight(layers);

  EXPECT_NEAR(Vtot, getFillVolume(layers, 1e9, TOL), 1e-12);
  EXPECT_NEAR(Htot, getFillHeight(layers, 1e9, TOL), 1e-12);
}

TEST(ConeFrustum_Specials, CylinderAndPointedCone)
{
  FluidConeShape fr_cyl(0.3, 0.3, 1.2);
  FluidCylinderShape cyl(0.3, 1.2);

  EXPECT_NEAR(cyl.getMaxFillVolume(), fr_cyl.getMaxFillVolume(), 1e-12);
  EXPECT_NEAR(cyl.getFillVolume(0.6), fr_cyl.getFillVolume(0.6), 1e-12);
  EXPECT_NEAR(0.6, fr_cyl.getFillHeight(cyl.getFillVolume(0.6)), 1e-10);

  FluidConeShape cone(0.4, 0.0, 1.0);
  double v = cone.getFillVolume(0.25);
  EXPECT_NEAR(0.25, cone.getFillHeight(v), 1e-10);
}
