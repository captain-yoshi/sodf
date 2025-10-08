#pragma once
#include <string>
#include <string_view>
#include <vector>
#include <optional>
#include <tinyxml2.h>

namespace sodf {
namespace xml {

// Attribute predicate on a single step.
struct AttrPred
{
  enum class Op
  {
    EXISTS,
    EQUALS,
    ANY_EQUALS,
    CONTAINS
  };
  std::string name;  // "*" allowed
  Op op = Op::EXISTS;
  std::string value;  // for EQUALS/ANY_EQUALS/CONTAINS
};

struct Step
{
  std::string tag;              // element/tag name
  std::optional<int> index;     // [0]-based index selector
  std::vector<AttrPred> preds;  // @id=foo, @*=bar, @name*=gl*b
};

struct Selector
{
  std::vector<Step> steps;
};

// Parse a path like: "Thing[@id=link/base]/Dims"
Selector parse_selector(std::string path);

// Evaluate a single step under a base node (public for reuse/testing).
std::vector<tinyxml2::XMLElement*> eval_step(tinyxml2::XMLElement* base, const Step& st);

// Match the first step against an element (public for reuse/testing).
bool step_matches(const tinyxml2::XMLElement* e, const Step& st);

// Convenience: select elements under a given root element that match the full selector.
std::vector<tinyxml2::XMLElement*> select_elements(tinyxml2::XMLElement* root, const Selector& sel);

// Convenience: resolve "path.attr" from `root`. Returns the attribute as raw text if any hit exists.
// If multiple hits exist, returns the first.
std::optional<std::string> resolve_attr(tinyxml2::XMLElement* root, const std::string& spec);

// Utility: simple trim
inline std::string trim_copy(std::string s)
{
  auto issp = [](unsigned char c) { return std::isspace(c); };
  while (!s.empty() && issp(s.front()))
    s.erase(s.begin());
  while (!s.empty() && issp(s.back()))
    s.pop_back();
  return s;
}

}  // namespace xml
}  // namespace sodf
