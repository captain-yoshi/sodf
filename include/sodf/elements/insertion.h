#ifndef INSERTION_H_
#define INSERTION_H_

#include <memory>
#include <map>

#include <sodf/geometry/transform.h>
#include <sodf/element.h>

namespace sodf {
namespace elements {

class Insertion : public Element
{
public:
  Insertion(const geometry::Transform& insertion_frame, double pre_insertion_distance = 0.0);

  virtual ~Insertion(){};

  const geometry::Transform& tf() const;
  double preInsertionDistance();

  virtual const geometry::Transform* getTransform() const override
  {
    return &tf();
  };

protected:
  const geometry::Transform insertion_tf_;  // X axis pointing towards the bottom of the container, wrt. object origin

  double pre_insertion_distance_;  // Along X axis wrt insertion_tf
};

}  // namespace elements
}  // namespace sodf

#endif  // INSERTION_H_
