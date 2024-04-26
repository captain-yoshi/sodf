#ifndef ELEMENT_H_
#define ELEMENT_H_

#include <memory>

namespace sodf {

class Element;

using ElementPtr = std::shared_ptr<Element>;
using ElementUniquePtr = std::unique_ptr<Element>;

class Element
{
public:
  using pointer = std::unique_ptr<Element>;
  virtual ~Element() = default;

protected:
  Element(){};
};

}  // namespace sodf

#endif  // ELEMENT_H_
