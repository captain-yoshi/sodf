#ifndef SODF_XML_EXPRESSION_PARSER_H_
#define SODF_XML_EXPRESSION_PARSER_H_

#include <tinyxml2.h>
#include <string>

namespace sodf {
namespace xml {

std::string parseText(const tinyxml2::XMLElement* elem);

double parseRequiredDoubleExpression(const tinyxml2::XMLElement* elem, const char* attr_name);
double parseDoubleExpression(const tinyxml2::XMLElement* elem, const char* attr_name, double default_value);
bool queryDoubleExpression(const tinyxml2::XMLElement* elem, const char* attr_name, double* out_value);

}  // namespace xml
}  // namespace sodf

#endif  // SODF_XML_EXPRESSION_PARSER_H_
