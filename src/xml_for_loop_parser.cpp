#include <sodf/xml_for_loop_parser.h>
#include <cctype>

namespace sodf {

std::vector<std::string> split(const std::string& s, char sep)
{
  std::vector<std::string> tokens;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, sep))
    tokens.push_back(item);
  return tokens;
}

std::string trim(const std::string& s)
{
  auto start = s.begin(), end = s.end();
  while (start != end && std::isspace(*start))
    ++start;
  while (end != start && std::isspace(*(end - 1)))
    --end;
  return std::string(start, end);
}

std::vector<std::string> expandCharTriplet(const std::string& start, const std::string& end, int step)
{
  std::vector<std::string> range;
  std::string current = start;
  int iter_limit = 10000;
  bool reached_end = false;

  auto is_upper = [](char c) { return std::isupper(static_cast<unsigned char>(c)); };
  auto is_lower = [](char c) { return std::islower(static_cast<unsigned char>(c)); };
  if (start.empty() || end.empty() || (is_upper(start[0]) && is_lower(end[0])) ||
      (is_lower(start[0]) && is_upper(end[0])))
    throw std::runtime_error("expandCharTriplet: Start and end must be both upper or lower case.");
  if (start.length() != end.length())
    throw std::runtime_error("expandCharTriplet: Start and end must be the same length for non-overflow triplets.");

  for (int i = 0; i < iter_limit; ++i)
  {
    range.push_back(current);
    if (current == end)
    {
      reached_end = true;
      break;
    }
    // increment lexicographically
    std::string next = current;
    for (int s = 0; s < step; ++s)
    {
      int idx = next.size() - 1;
      while (idx >= 0 && next[idx] == (is_upper(next[0]) ? 'Z' : 'z'))
      {
        next[idx] = is_upper(next[0]) ? 'A' : 'a';
        idx--;
      }
      if (idx < 0)  // can't overflow, so fail
        throw std::runtime_error("expandCharTriplet: Overflow not supported in this mode");
      next[idx]++;
    }
    current = next;
    // If we ever overshoot, stop early (shouldn't happen, but for safety)
    if ((current.length() == end.length() && current > end) || current.length() > end.length())
      break;
  }
  if (!reached_end)
    throw std::runtime_error("expandCharTriplet: Could not reach end label '" + end + "' (bad step or out of range).");
  return range;
}

std::vector<std::string> parseCharMultiTriplet(const std::string& val)
{
  std::vector<std::string> values;
  auto tokens = sodf::split(val, ':');
  if (tokens.size() % 3 != 0)
    throw std::runtime_error("parseCharMultiTriplet: Must have triplets of start:end:step.");
  for (size_t i = 0; i < tokens.size(); i += 3)
  {
    std::string start = tokens[i];
    std::string end = tokens[i + 1];
    int step = std::stoi(tokens[i + 2]);
    auto part = expandCharTriplet(start, end, step);
    values.insert(values.end(), part.begin(), part.end());
  }
  return values;
}

LoopVarSpec parseLoopVarSpec(const std::string& name, const std::string& val)
{
  LoopVarSpec spec;
  spec.name = name;
  // Character range: e.g. "A:G:1:AA"
  if (std::isalpha(val[0]))
  {
    spec.type = LoopVarSpec::Char;
    spec.values = parseCharMultiTriplet(val);
  }
  else
  {  // Numeric: e.g. "0:10:2" or "0;10;2"
    char sep = val.find(';') != std::string::npos ? ';' : ':';
    auto tokens = split(val, sep);
    int start = std::stoi(tokens[0]);
    int end = std::stoi(tokens[1]);
    int step = tokens.size() > 2 ? std::stoi(tokens[2]) : 1;
    spec.type = LoopVarSpec::Numeric;
    for (int i = start; i <= end; i += step)
      spec.values.push_back(std::to_string(i));
  }
  return spec;
}

std::vector<std::string> parseZipped(const std::string& zipped_val)
{
  std::vector<std::string> vars;
  std::string s = zipped_val;
  // Remove brackets and spaces
  s.erase(std::remove_if(s.begin(), s.end(), [](char c) { return c == '[' || c == ']' || std::isspace(c); }), s.end());
  vars = split(s, ',');
  return vars;
}

void expandCartesian(const std::vector<LoopVarSpec>& vars, size_t idx, std::map<std::string, std::string>& ctx,
                     const std::function<void(const std::map<std::string, std::string>&)>& body)
{
  if (idx == vars.size())
  {
    body(ctx);
    return;
  }
  const auto& spec = vars[idx];
  for (const auto& val : spec.values)
  {
    ctx[spec.name] = val;
    expandCartesian(vars, idx + 1, ctx, body);
  }
}

void expandForLoop(const tinyxml2::XMLElement* forElem,
                   const std::function<void(const std::map<std::string, std::string>&)>& body)
{
  // 1. Ordered variable specs
  std::vector<LoopVarSpec> vars;
  std::vector<std::string> attrOrder;
  for (const tinyxml2::XMLAttribute* attr = forElem->FirstAttribute(); attr; attr = attr->Next())
  {
    std::string name = attr->Name();
    if (name == "zipped")
      continue;
    attrOrder.push_back(name);
  }

  for (const auto& name : attrOrder)
  {
    vars.push_back(parseLoopVarSpec(name, forElem->Attribute(name.c_str())));
  }

  // 2. Zipped parsing
  std::vector<std::vector<std::string>> zippedValues;
  std::vector<size_t> zippedIndices;
  if (const char* zipped = forElem->Attribute("zipped"))
  {
    auto zippedGroup = parseZipped(zipped);
    zippedIndices.reserve(zippedGroup.size());
    for (const auto& v : zippedGroup)
    {
      auto it = std::find_if(vars.begin(), vars.end(), [&](const LoopVarSpec& s) { return s.name == v; });
      if (it == vars.end())
        throw std::runtime_error("Zipped var not found: " + v);
      zippedIndices.push_back(std::distance(vars.begin(), it));
    }
    // Precompute zipped value sets
    size_t zipLen = vars[zippedIndices[0]].values.size();
    for (size_t idx : zippedIndices)
    {
      if (vars[idx].values.size() != zipLen)
      {
        throw std::runtime_error("Zipped groups must be same length! Attribute '" + vars[idx].name + "' has " +
                                 std::to_string(vars[idx].values.size()) + " values, expected " +
                                 std::to_string(zipLen));
      }
    }

    // Loop over zipped
    for (size_t i = 0; i < zipLen; ++i)
    {
      std::map<std::string, std::string> ctx;
      // Fill zipped values
      for (size_t j = 0; j < zippedIndices.size(); ++j)
        ctx[vars[zippedIndices[j]].name] = vars[zippedIndices[j]].values[i];
      // Expand the non-zipped vars
      std::vector<LoopVarSpec> nonZipped;
      for (size_t j = 0; j < vars.size(); ++j)
        if (std::find(zippedIndices.begin(), zippedIndices.end(), j) == zippedIndices.end())
          nonZipped.push_back(vars[j]);
      if (!nonZipped.empty())
        expandCartesian(nonZipped, 0, ctx, body);
      else
        body(ctx);
    }
  }
  else
  {
    // No zipped: expand all combinations
    std::map<std::string, std::string> ctx;
    expandCartesian(vars, 0, ctx, body);
  }
}
std::string substituteVars(const std::string& input, const std::map<std::string, std::string>& ctx)
{
  std::string result = input;
  size_t pos = 0;
  while ((pos = result.find('{', pos)) != std::string::npos)
  {
    size_t end = result.find('}', pos + 1);
    if (end == std::string::npos)
      break;  // Malformed: no closing brace
    std::string var = result.substr(pos + 1, end - pos - 1);
    auto it = ctx.find(var);
    if (it != ctx.end())
    {
      result.replace(pos, end - pos + 1, it->second);
      pos += it->second.size();  // Move past replacement
    }
    else
    {
      pos = end + 1;  // Skip this brace, var not found
    }
  }
  return result;
}

tinyxml2::XMLElement* cloneAndSubstitute(const tinyxml2::XMLElement* elem, tinyxml2::XMLDocument* doc,
                                         const std::map<std::string, std::string>& ctx)
{
  auto* cloned = doc->NewElement(elem->Name());
  // Copy and substitute attributes
  for (const tinyxml2::XMLAttribute* attr = elem->FirstAttribute(); attr; attr = attr->Next())
  {
    cloned->SetAttribute(attr->Name(), substituteVars(attr->Value(), ctx).c_str());
  }
  // Copy and substitute child elements or text
  for (const tinyxml2::XMLNode* node = elem->FirstChild(); node; node = node->NextSibling())
  {
    if (node->ToElement())
    {
      cloned->InsertEndChild(cloneAndSubstitute(node->ToElement(), doc, ctx));
    }
    else if (node->ToText())
    {
      std::string text = substituteVars(node->ToText()->Value(), ctx);
      cloned->InsertEndChild(doc->NewText(text.c_str()));
    }
  }
  return cloned;
}

}  // namespace sodf
