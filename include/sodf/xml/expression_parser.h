#ifndef SODF_XML_EXPRESSION_PARSER_H_
#define SODF_XML_EXPRESSION_PARSER_H_

#include <tinyxml2.h>
#include <string>

#include <optional>

namespace sodf {
namespace xml {

constexpr int MAX_RECURSION_DEPTH = 32;

struct ExprEvalContext
{
  const tinyxml2::XMLDocument* doc;         // whole doc
  const tinyxml2::XMLElement* object_root;  // current <Object id="...">
  const tinyxml2::XMLElement* scope;        // element that owns the attribute being parsed
};

/// Numbers
double evalNumberAttribute(const tinyxml2::XMLElement* elem, const char* attr, double fallback);
double evalNumberAttributeRequired(const tinyxml2::XMLElement* elem, const char* attr);
bool tryEvalNumberAttribute(const tinyxml2::XMLElement* elem, const char* attr, double* out);

/// Text
std::string evalTextNode(const tinyxml2::XMLElement* elem);
std::string evalTextAttribute(const tinyxml2::XMLElement* elem, const char* attr, const std::string& fallback);
std::string evalTextAttributeRequired(const tinyxml2::XMLElement* elem, const char* attr);
bool tryEvalTextAttribute(const tinyxml2::XMLElement* elem, const char* attr, std::string* out);

/// Bool
bool evalBoolAttributeRequired(const tinyxml2::XMLElement* elem, const char* attr);
bool evalBoolAttribute(const tinyxml2::XMLElement* elem, const char* attr, bool fallback);

/// Uint
uint32_t evalUIntAttributeRequired(const tinyxml2::XMLElement* elem, const char* attr);

}  // namespace xml
}  // namespace sodf

#endif  // SODF_XML_EXPRESSION_PARSER_H_
