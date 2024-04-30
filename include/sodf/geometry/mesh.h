#ifndef MESH_H_
#define MESH_H_

#include <string>

namespace sodf {
namespace geometry {

class Mesh
{
public:
  Mesh(const std::string& url = "") : url_(url){};

  const std::string& url() const
  {
    return url_;
  };

private:
  const std::string url_;
};

}  // namespace geometry
}  // namespace sodf

#endif  // MESH_H_
