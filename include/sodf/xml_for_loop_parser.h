#ifndef SODF_XML_FOR_LOOP_PARSER_H_
#define SODF_XML_FOR_LOOP_PARSER_H_

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <tinyxml2.h>

namespace sodf {

// --- Helpers ---

// Split string (by single char delimiter)
std::vector<std::string> split(const std::string& s, char sep);

// Trim leading/trailing whitespace
std::string trim(const std::string& s);
// --- Loop variable specification ---
struct LoopVarSpec
{
  std::string name;
  enum
  {
    Numeric,
    Char
  } type;
  std::vector<std::string> values;
};

std::vector<std::string> expandCharRange(const std::string& start, const std::string& end, int step, bool overflow);

// Parse one variable's spec
LoopVarSpec parseLoopVarSpec(const std::string& name, const std::string& val);

// Parse zipped group(s), e.g. "[row_name,row]" -> {"row_name","row"}
std::vector<std::string> parseZipped(const std::string& zipped_val);

// Recursively expand non-zipped vars
void expandCartesian(const std::vector<LoopVarSpec>& vars, size_t idx, std::map<std::string, std::string>& ctx,
                     const std::function<void(const std::map<std::string, std::string>&)>& body);

// Top-level ForLoop handler
void expandForLoop(const tinyxml2::XMLElement* forElem,
                   const std::function<void(const std::map<std::string, std::string>&)>& body);

// Substitute {var} in input string with values from ctx map
std::string substituteVars(const std::string& input, const std::map<std::string, std::string>& ctx);

// Clone an element and substitute {vars} in all attributes/children
tinyxml2::XMLElement* cloneAndSubstitute(const tinyxml2::XMLElement* elem, tinyxml2::XMLDocument* doc,
                                         const std::map<std::string, std::string>& ctx);

}  // namespace sodf

#endif  // SODF_XML_FOR_LOOP_PARSER_H_
