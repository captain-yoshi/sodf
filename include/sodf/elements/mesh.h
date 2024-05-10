#ifndef MESH_H_
#define MESH_H_

#include <sodf/geometry/transform.h>
#include <sodf/element.h>

namespace sodf {
namespace elements {

class Mesh : public Element
{
public:
  using pointer = std::unique_ptr<Mesh>;

  Mesh(const geometry::Transform& tf, const std::string& url) : tf_(tf), url_(url), Element(){};

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

private:
  const geometry::Transform tf_;
  const std::string url_;
};

}  // namespace elements
}  // namespace sodf

#endif  // MESH_H_
