#include <sodf/elements/insertion.h>

namespace sodf {
namespace elements {

Insertion::Insertion(const geometry::Transform& insertion_tf, double pre_insertion_distance)
  : insertion_tf_(insertion_tf), pre_insertion_distance_(pre_insertion_distance)
{
}

const geometry::Transform& Insertion::tf() const
{
  return insertion_tf_;
}

double Insertion::preInsertionDistance()
{
  return pre_insertion_distance_;
}

}  // namespace elements
}  // namespace sodf
