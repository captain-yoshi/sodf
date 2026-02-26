#ifndef SODF_XML_EXPRESSION_PARSER_H_
#define SODF_XML_EXPRESSION_PARSER_H_

#include <tinyxml2.h>
#include <string>

#include <optional>
#include <unordered_map>
#include <stdexcept>

namespace sodf {
namespace xml {

constexpr int MAX_RECURSION_DEPTH = 32;

struct XMLParseContext
{
  tinyxml2::XMLDocument* doc;               // whole doc
  const tinyxml2::XMLElement* object_root;  // current <Object id="...">

  std::string filename;
};

[[noreturn]] inline void throw_xml_error(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx,
                                         const std::string& msg)
{
  const int line = elem ? elem->GetLineNum() : 0;
  const std::string file = ctx.filename.empty() ? "<unknown>" : ctx.filename;

  throw std::runtime_error(file + ":" + std::to_string(line) + ": " + msg);
}

/// Math
double evalMathWithVariables(const char* expr, const std::unordered_map<std::string, double>& vars);

/// Numbers
double evalNumberAttribute(const tinyxml2::XMLElement* elem, const char* attr, double fallback,
                           const XMLParseContext& ctx);
double evalNumberAttributeRequired(const tinyxml2::XMLElement* elem, const char* attr, const XMLParseContext& ctx);
bool tryEvalNumberAttribute(const tinyxml2::XMLElement* elem, const char* attr, double* out, const XMLParseContext& ctx);

/// Text
std::string evalTextNode(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx);
std::string evalTextAttribute(const tinyxml2::XMLElement* elem, const char* attr, const std::string& fallback,
                              const XMLParseContext& ctx);
std::string evalTextAttributeRequired(const tinyxml2::XMLElement* elem, const char* attr, const XMLParseContext& ctx);
bool tryEvalTextAttribute(const tinyxml2::XMLElement* elem, const char* attr, std::string* out,
                          const XMLParseContext& ctx);

/// Bool
bool evalBoolAttributeRequired(const tinyxml2::XMLElement* elem, const char* attr, const XMLParseContext& ctx);
bool evalBoolAttribute(const tinyxml2::XMLElement* elem, const char* attr, bool fallback, const XMLParseContext& ctx);

/// Uint
uint32_t evalUIntAttributeRequired(const tinyxml2::XMLElement* elem, const char* attr, const XMLParseContext& ctx);

}  // namespace xml
}  // namespace sodf

#endif  // SODF_XML_EXPRESSION_PARSER_H_
