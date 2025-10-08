#include "sodf/xml/selector.h"
#include <algorithm>
#include <cstring>

namespace sodf {
namespace xml {

static bool match_contains_pattern(const char* hay, const std::string& pat)
{
  if (!hay)
    return false;
  if (pat.empty())
    return true;
  std::vector<std::string> chunks;
  for (size_t i = 0; i < pat.size();)
  {
    size_t j = i;
    while (j < pat.size() && pat[j] != '*')
      ++j;
    chunks.emplace_back(pat.substr(i, j - i));
    i = (j < pat.size() ? j + 1 : j);
  }
  if (chunks.size() == 1 && chunks[0].empty())
    return true;
  const bool anchored_prefix = !pat.empty() && pat.front() != '*';
  const bool anchored_suffix = !pat.empty() && pat.back() != '*';

  const char* s = hay;
  if (anchored_prefix && !chunks.front().empty())
  {
    const auto& c0 = chunks.front();
    if (std::strncmp(s, c0.c_str(), c0.size()) != 0)
      return false;
    s += c0.size();
    chunks.erase(chunks.begin());
  }
  for (const auto& c : chunks)
  {
    if (c.empty())
      continue;
    const char* f = std::strstr(s, c.c_str());
    if (!f)
      return false;
    s = f + c.size();
  }
  if (anchored_suffix)
  {
    const std::string& last = chunks.empty() ? std::string() : chunks.back();
    if (!last.empty())
    {
      size_t L = std::strlen(hay);
      if (L < last.size())
        return false;
      if (std::strncmp(hay + (L - last.size()), last.c_str(), last.size()) != 0)
        return false;
    }
  }
  return true;
}

bool step_matches(const tinyxml2::XMLElement* e, const Step& st)
{
  if (!e || st.tag != e->Name())
    return false;
  for (const auto& p : st.preds)
  {
    if (p.op == AttrPred::Op::EXISTS)
    {
      if (p.name == "*")
      {
        if (!e->FirstAttribute())
          return false;
      }
      else
      {
        if (!e->FindAttribute(p.name.c_str()))
          return false;
      }
    }
    else if (p.op == AttrPred::Op::EQUALS)
    {
      const char* v = e->Attribute(p.name.c_str());
      if (!v || std::string(v) != p.value)
        return false;
    }
    else if (p.op == AttrPred::Op::ANY_EQUALS)
    {
      bool ok = false;
      for (auto* a = e->FirstAttribute(); a; a = a->Next())
        if (a->Value() && p.value == a->Value())
        {
          ok = true;
          break;
        }
      if (!ok)
        return false;
    }
    else
    {  // CONTAINS
      if (p.name == "*")
      {
        bool ok = false;
        for (auto* a = e->FirstAttribute(); a; a = a->Next())
          if (match_contains_pattern(a->Value(), p.value))
          {
            ok = true;
            break;
          }
        if (!ok)
          return false;
      }
      else
      {
        const char* v = e->Attribute(p.name.c_str());
        if (!v || !match_contains_pattern(v, p.value))
          return false;
      }
    }
  }
  return true;
}

static std::vector<tinyxml2::XMLElement*> children_by_tag(tinyxml2::XMLElement* parent, const std::string& tag)
{
  std::vector<tinyxml2::XMLElement*> out;
  for (auto* c = parent->FirstChildElement(tag.c_str()); c; c = c->NextSiblingElement(tag.c_str()))
    out.push_back(c);
  return out;
}

std::vector<tinyxml2::XMLElement*> eval_step(tinyxml2::XMLElement* base, const Step& st)
{
  std::vector<tinyxml2::XMLElement*> cands = children_by_tag(base, st.tag);
  std::vector<tinyxml2::XMLElement*> out;
  for (auto* e : cands)
    if (step_matches(e, st))
      out.push_back(e);
  if (st.index)
  {
    if (*st.index < 0 || (size_t)*st.index >= out.size())
      return {};
    return { out[(size_t)*st.index] };
  }
  return out;
}

static Step parse_step(std::string s)
{
  Step st;
  s = trim_copy(s);
  size_t i = s.find('[');
  st.tag = trim_copy(i == std::string::npos ? s : s.substr(0, i));
  while (i != std::string::npos)
  {
    size_t j = s.find(']', i + 1);
    if (j == std::string::npos)
      break;
    std::string pred = trim_copy(s.substr(i + 1, j - (i + 1)));

    if (!pred.empty() && std::all_of(pred.begin(), pred.end(), ::isdigit))
    {
      st.index = std::stoi(pred);
    }
    else if (!pred.empty() && pred[0] == '@')
    {
      std::string body = pred.substr(1);
      AttrPred ap;
      if (body == "*")
      {
        ap.name = "*";
        ap.op = AttrPred::Op::EXISTS;
      }
      else
      {
        size_t pos_star = body.find("*=");
        size_t pos_eq = body.find('=');
        if (pos_star != std::string::npos)
        {
          ap.name = trim_copy(body.substr(0, pos_star));
          ap.op = AttrPred::Op::CONTAINS;
          ap.value = trim_copy(body.substr(pos_star + 2));
        }
        else if (pos_eq != std::string::npos)
        {
          ap.name = trim_copy(body.substr(0, pos_eq));
          ap.op = (ap.name == "*") ? AttrPred::Op::ANY_EQUALS : AttrPred::Op::EQUALS;
          ap.value = trim_copy(body.substr(pos_eq + 1));
        }
        else
        {
          ap.name = trim_copy(body);
          ap.op = AttrPred::Op::EXISTS;
        }
      }
      if (!ap.value.empty() && ((ap.value.front() == '\'' && ap.value.back() == '\'') ||
                                (ap.value.front() == '"' && ap.value.back() == '"')))
      {
        ap.value = ap.value.substr(1, ap.value.size() - 2);
      }
      st.preds.push_back(std::move(ap));
    }
    i = s.find('[', j + 1);
  }
  return st;
}

Selector parse_selector(std::string path)
{
  Selector sel;
  path = trim_copy(path);
  if (path.empty())
    return sel;
  if (!path.empty() && path.front() == '/')
    path.erase(path.begin());

  std::string cur;
  int bracket_depth = 0;
  char quote = 0;

  auto flush = [&] {
    std::string t = trim_copy(cur);
    if (!t.empty())
      sel.steps.push_back(parse_step(t));
    cur.clear();
  };

  for (size_t i = 0; i < path.size(); ++i)
  {
    char c = path[i];
    if (quote)
    {
      cur.push_back(c);
      if (c == quote)
        quote = 0;
      continue;
    }
    if (c == '\'' || c == '"')
    {
      quote = c;
      cur.push_back(c);
      continue;
    }
    if (c == '[')
    {
      ++bracket_depth;
      cur.push_back(c);
      continue;
    }
    if (c == ']')
    {
      if (bracket_depth > 0)
        --bracket_depth;
      cur.push_back(c);
      continue;
    }
    if (c == '/' && bracket_depth == 0)
    {
      flush();
      continue;
    }
    cur.push_back(c);
  }
  flush();
  return sel;
}

std::vector<tinyxml2::XMLElement*> select_elements(tinyxml2::XMLElement* root, const Selector& sel)
{
  std::vector<tinyxml2::XMLElement*> hits;
  if (!root || sel.steps.empty())
    return hits;

  // First step must start at `root`
  if (!step_matches(root, sel.steps[0]))
    return hits;

  std::vector<tinyxml2::XMLElement*> frontier{ root };
  for (size_t k = 1; k < sel.steps.size(); ++k)
  {
    std::vector<tinyxml2::XMLElement*> next;
    for (auto* base : frontier)
    {
      auto partial = eval_step(base, sel.steps[k]);
      next.insert(next.end(), partial.begin(), partial.end());
    }
    frontier.swap(next);
    if (frontier.empty())
      break;
  }
  hits = std::move(frontier);
  return hits;
}

static bool split_attr_path(const std::string& spec, std::string& path_out, std::string& attr_out)
{
  auto dot = spec.rfind('.');
  if (dot == std::string::npos)
    return false;
  path_out = spec.substr(0, dot);
  attr_out = trim_copy(spec.substr(dot + 1));
  return !path_out.empty() && !attr_out.empty();
}

std::optional<std::string> resolve_attr(tinyxml2::XMLElement* root, const std::string& spec)
{
  std::string path, attr;
  if (!split_attr_path(spec, path, attr))
    return std::nullopt;
  Selector sel = parse_selector(path);
  auto hits = select_elements(root, sel);
  if (hits.empty())
    return std::nullopt;
  const char* v = hits.front()->Attribute(attr.c_str());
  if (!v)
    return std::nullopt;
  return std::string(v);
}

}  // namespace xml
}  // namespace sodf
