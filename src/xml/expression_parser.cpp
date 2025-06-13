#include "sodf/xml/expression_parser.h"

#include "muParser.h"

namespace sodf {
namespace xml {

double parseExpression(const char* expr)
{
  if (!expr)
    throw std::runtime_error("Expression string is null.");

  try
  {
    mu::Parser parser;
    parser.DefineConst("pi", M_PI);
    parser.DefineConst("inf", std::numeric_limits<double>::infinity());
    parser.SetExpr(expr);
    return parser.Eval();
  }
  catch (mu::Parser::exception_type& e)
  {
    throw std::runtime_error(std::string("Error parsing expression: ") + e.GetMsg());
  }
}

double parseRequiredDoubleExpression(const tinyxml2::XMLElement* elem, const char* attr_name)
{
  const char* expr = elem->Attribute(attr_name);
  if (!expr)
    throw std::runtime_error(std::string("Missing attribute '") + attr_name + "' in <Shape> at line " +
                             std::to_string(elem->GetLineNum()));
  return parseExpression(expr);
}

double parseDoubleExpression(const tinyxml2::XMLElement* elem, const char* attr_name, double default_value)
{
  const char* expr = elem->Attribute(attr_name);
  if (!expr)
    return default_value;
  return parseExpression(expr);
}

bool queryDoubleExpression(const tinyxml2::XMLElement* elem, const char* attr_name, double* out_value)
{
  const char* expr = elem->Attribute(attr_name);
  if (!expr)
    return false;
  *out_value = parseExpression(expr);
  return true;
}

}  // namespace xml
}  // namespace sodf
