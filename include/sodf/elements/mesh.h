#ifndef MESH_H_
#define MESH_H_

#include <sodf/geometry/transform.h>
#include <sodf/element.h>

#include <geometry_msgs/Inertia.h>

namespace sodf {
namespace elements {

class Mesh : public Element
{
public:
  using pointer = std::unique_ptr<Mesh>;

  Mesh(const geometry::Transform& tf, const std::string& url,
       const geometry_msgs::Inertia& inertia = geometry_msgs::Inertia())
    : tf_(tf), url_(url), inertia_(inertia), Element(){};

  const std::string& url() const
  {
    return url_;
  };

  const geometry::Transform& tf() const
  {
    return tf_;
  }

  virtual const geometry::Transform* getTransform() const override
  {
    return &tf_;
  };

  virtual bool isMesh() const override
  {
    return true;
  };

  const geometry_msgs::Inertia inertia() const
  {
    return inertia_;
  }

private:
  const geometry::Transform tf_;
  const std::string url_;

  const geometry_msgs::Inertia inertia_;
};

}  // namespace elements
}  // namespace sodf

#endif  // MESH_H_
