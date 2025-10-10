#ifndef SODF_ECS_COMPONENTS_H_
#define SODF_ECS_COMPONENTS_H_

#include <vector>
#include <utility>
#include <string>
#include <string_view>

namespace sodf {
namespace components {

// Flat map commonly used by components
template <class K, class V>
using ElementMap = std::vector<std::pair<K, V>>;

}  // namespace components
}  // namespace sodf

#endif  // SODF_ECS_COMPONENTS_H_
