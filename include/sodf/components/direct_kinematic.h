#ifndef SODF_COMPONENTS_DIRECT_KINEMATIC_H_
#define SODF_COMPONENTS_DIRECT_KINEMATIC_H_

#include <memory>
#include <kdl/treefksolverpos_recursive.hpp>

namespace sodf {
namespace components {

struct DirectKinematic
{
  std::shared_ptr<KDL::Tree> tree;
  std::shared_ptr<KDL::TreeFkSolverPos_recursive> fk_solver;
  std::shared_ptr<KDL::JntArray> joints;
};

}  // namespace components
}  // namespace sodf

#endif
