#pragma once

#include <string>
#include <string_view>

namespace sodf {

inline void splitObjectElement(std::string_view id, std::string_view& object, std::string_view& element,
                               std::string_view delimiter = "/")
{
  const std::size_t pos = id.find(delimiter);

  if (pos == std::string_view::npos)
  {
    object = id;
    element = std::string_view{};
    return;
  }

  object = id.substr(0, pos);

  const std::size_t elem_start = pos + delimiter.size();

  if (elem_start >= id.size())
    element = std::string_view{};
  else
    element = id.substr(elem_start);
}

inline void splitObjectElement(const std::string& id, std::string& object, std::string& element,
                               std::string_view delimiter = "/")
{
  std::string_view obj_sv;
  std::string_view elem_sv;

  splitObjectElement(std::string_view(id), obj_sv, elem_sv, delimiter);

  object.assign(obj_sv.begin(), obj_sv.end());
  element.assign(elem_sv.begin(), elem_sv.end());
}

}  // namespace sodf
