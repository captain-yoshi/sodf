#include "sodf/xml/expression_parser.h"
#include <string_view>

#include "muParser.h"

namespace sodf {
namespace xml {

std::optional<double> resolve_numeric_ref(std::string_view ref, const ExprEvalContext& ctx);
double parseExpression(const char* expr);
static std::optional<std::string> resolve_ref_raw(std::string_view ref, const ExprEvalContext& ctx);
double evalBareMath(const char* expr);
ExprEvalContext make_context(const tinyxml2::XMLElement* scope);

// Expand ${...} references as raw text, repeatedly, up to max_depth.
// No math evaluation here.
std::string eval_text_with_refs(const char* raw, const ExprEvalContext& ctx, int max_depth = MAX_RECURSION_DEPTH)
{
  if (!raw)
    return {};
  std::string s(raw);

  for (int depth = 0; depth < max_depth; ++depth)
  {
    bool changed = false;
    size_t pos = 0;

    while (true)
    {
      size_t l = s.find("${", pos);
      if (l == std::string::npos)
        break;
      size_t r = s.find('}', l + 2);
      if (r == std::string::npos)
        throw std::runtime_error("Unclosed ${...} in text expression");

      std::string ref = s.substr(l + 2, r - (l + 2));
      auto rawv = resolve_ref_raw(ref, ctx);  // <-- the raw resolver you already have
      if (!rawv.has_value())
        throw std::runtime_error("Cannot resolve reference: " + ref);

      s.replace(l, r - l + 1, *rawv);
      pos = l + rawv->size();
      changed = true;
    }

    if (!changed)
    {
      if (s.find("${") != std::string::npos)
        throw std::runtime_error("Unresolved ${...} after expansion");
      return s;  // fully expanded, as text
    }
  }

  throw std::runtime_error("Recursive reference detected (text), expansion too deep.");
}

static std::string_view trim_sv(std::string_view v)
{
  auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
  size_t a = 0, b = v.size();
  while (a < b && is_space(v[a]))
    ++a;
  while (b > a && is_space(v[b - 1]))
    --b;
  return v.substr(a, b - a);
}

static const tinyxml2::XMLElement* parent_element(const tinyxml2::XMLElement* e)
{
  if (!e)
    return nullptr;
  if (auto* p = e->Parent())
    return p->ToElement();
  return nullptr;
}

static bool parse_indexed_name(std::string_view token, std::string_view& name, int& idx /* 0-based or -1 */)
{
  size_t lb = token.find('[');
  if (lb == std::string::npos)
  {
    name = trim_sv(token);
    idx = -1;
    return true;
  }
  size_t rb = token.find(']', lb + 1);
  if (rb == std::string::npos)
    return false;
  name = trim_sv(token.substr(0, lb));
  auto idx_sv = trim_sv(token.substr(lb + 1, rb - lb - 1));
  long long tmp = -1;
  try
  {
    tmp = std::stoll(std::string(idx_sv));
  }
  catch (...)
  {
    return false;
  }
  if (tmp < 0 || tmp > std::numeric_limits<int>::max())
    return false;
  idx = (int)tmp;  // 0-based
  if (rb + 1 != token.size())
    return false;
  return true;
}

static const tinyxml2::XMLElement* nth_child_named(const tinyxml2::XMLElement* parent, std::string_view name,
                                                   int zero_based_idx /* -1 = first (i.e., 0) */)
{
  if (!parent)
    return nullptr;
  int k = 0;
  for (auto* e = parent->FirstChildElement(std::string(name).c_str()); e;
       e = e->NextSiblingElement(std::string(name).c_str()))
  {
    if (zero_based_idx < 0 || k == zero_based_idx)
      return e;
    ++k;
  }
  return nullptr;
}

static const tinyxml2::XMLElement* dfs_find_id(const tinyxml2::XMLElement* root, std::string_view id)
{
  if (!root)
    return nullptr;
  std::vector<const tinyxml2::XMLElement*> st{ root };
  while (!st.empty())
  {
    auto* cur = st.back();
    st.pop_back();
    if (const char* v = cur->Attribute("id"))
    {
      if (id == v)
        return cur;
    }
    for (auto* ch = cur->LastChildElement(); ch; ch = ch->PreviousSiblingElement())
      st.push_back(ch);
  }
  return nullptr;
}

static std::vector<std::string_view> split_slashes(std::string_view p)
{
  std::vector<std::string_view> out;
  size_t start = 0, n = p.size();
  for (size_t i = 0; i <= n; ++i)
  {
    if (i == n || p[i] == '/')
    {
      out.push_back(p.substr(start, i - start));
      start = i + 1;
    }
  }
  return out;
}

double eval_with_refs(const char* raw_expr, const ExprEvalContext& ctx, int max_depth = MAX_RECURSION_DEPTH)
{
  if (!raw_expr)
    throw std::runtime_error("Null expression.");
  std::string expr(raw_expr);

  for (int depth = 0; depth < max_depth; ++depth)
  {
    bool changed = false;
    size_t pos = 0;

    while (true)
    {
      size_t l = expr.find("${", pos);
      if (l == std::string::npos)
        break;
      size_t r = expr.find('}', l + 2);
      if (r == std::string::npos)
        throw std::runtime_error("Unclosed ${...} in expression");

      std::string ref = expr.substr(l + 2, r - (l + 2));
      auto raw = resolve_ref_raw(ref, ctx);
      if (!raw.has_value())
        throw std::runtime_error("Cannot resolve reference: " + ref);

      // Replace ${...} with (raw) to preserve precedence
      std::string replacement = "(" + *raw + ")";
      expr.replace(l, r - l + 1, replacement);
      pos = l + replacement.size();
      changed = true;
    }

    if (!changed)
    {
      // No ${...} left → parse math
      if (expr.find("${") != std::string::npos)
        throw std::runtime_error("Unresolved ${...} after expansion");
      return evalBareMath(expr.c_str());
    }
  }

  // Depth exceeded → likely a cycle (A->B->A…)
  throw std::runtime_error("Recursive reference detected or expansion too deep (> " + std::to_string(max_depth) + ").");
}

double eval_with_refs(const char* expr, const tinyxml2::XMLElement* scope, int max_depth = MAX_RECURSION_DEPTH)
{
  ExprEvalContext ctx = make_context(scope);
  return eval_with_refs(expr, ctx, max_depth);
}

// Consume an ID right after '#'. Supports #'...'  #"..."  #( ... )  or unquoted.
// For unquoted, we now stop at the first '/' OR at the first '.' (to allow #ID.attr).
static bool consume_id_after_hash(std::string_view& ref, std::string& out_id)
{
  if (ref.empty() || ref.front() != '#')
    return false;
  ref.remove_prefix(1);  // drop '#'
  if (ref.empty())
    return false;

  char c = ref.front();
  if (c == '\'' || c == '"')
  {
    // #'...' or #"..."
    ref.remove_prefix(1);
    size_t end = 0;
    while (end < ref.size() && ref[end] != c)
      ++end;
    if (end == ref.size())
      throw std::runtime_error("Unclosed quoted id after #");
    out_id.assign(ref.substr(0, end));
    ref.remove_prefix(end + 1);  // remainder may start with '.' or '/'
    return true;
  }
  else if (c == '(')
  {
    // #( ... )
    ref.remove_prefix(1);
    size_t end = ref.find(')');
    if (end == std::string::npos)
      throw std::runtime_error("Unclosed #(id) after #");
    out_id.assign(ref.substr(0, end));
    ref.remove_prefix(end + 1);  // remainder may start with '.' or '/'
    return true;
  }
  else
  {
    // Unquoted: stop at first '/' or '.' (dot means a following final .attr)
    size_t slash = ref.find('/');
    size_t dot = ref.find('.');
    bool cut_at_dot = (dot != std::string::npos) && (slash == std::string::npos || dot < slash);

    if (cut_at_dot)
    {
      if (dot == 0)
        throw std::runtime_error("Empty id before '.attr' after #");
      out_id.assign(ref.substr(0, dot));
      ref.remove_prefix(dot);  // keep ".attr..." as remainder
      return true;
    }
    else
    {
      out_id.assign(ref.substr(0, slash));
      if (slash == std::string::npos)
      {
        ref = std::string_view{};  // no remainder
      }
      else
      {
        ref.remove_prefix(slash);  // keep "/..." as remainder
      }
      return true;
    }
  }
}

static std::optional<std::string> resolve_ref_raw(std::string_view ref, const ExprEvalContext& ctx)
{
  ref = trim_sv(ref);
  if (ref.empty())
    return std::nullopt;

  const tinyxml2::XMLElement* cur = nullptr;

  // anchor
  if (ref.size() >= 2 && ref.substr(0, 2) == "//")
  {
    ref.remove_prefix(2);
    cur = ctx.doc ? ctx.doc->RootElement() : nullptr;
  }
  else if (!ref.empty() && ref.front() == '/')
  {
    ref.remove_prefix(1);
    cur = ctx.object_root;
  }
  else if (!ref.empty() && ref.front() == '#')
  {
    std::string want;
    if (!consume_id_after_hash(ref, want))
      return std::nullopt;
    const tinyxml2::XMLElement* root = ctx.doc ? ctx.doc->RootElement() : nullptr;
    if (!root)
      return std::nullopt;
    cur = dfs_find_id(root, want);
    if (!cur)
      return std::nullopt;
    // ref now empty or begins with '/' for further descent
  }
  else
  {
    cur = ctx.scope;
  }
  if (!cur)
    return std::nullopt;

  // walk
  auto tokens = split_slashes(ref);
  for (size_t ti = 0; ti < tokens.size(); ++ti)
  {
    auto tok = trim_sv(tokens[ti]);
    if (tok.empty())
      continue;
    const bool last = (ti + 1 == tokens.size());

    if (tok == ".")
      continue;
    if (tok == "..")
    {
      cur = parent_element(cur);
      if (!cur)
        return std::nullopt;
      continue;
    }

    // final-dot attribute on last token: "Name.attr" or ".attr"
    if (last)
    {
      size_t dot = tok.rfind('.');
      if (dot != std::string::npos && (dot + 1) < tok.size())
      {
        std::string_view left = trim_sv(tok.substr(0, dot));
        std::string_view attr = trim_sv(tok.substr(dot + 1));

        // optional descend one step
        if (!left.empty() && !(left.size() == 1 && left[0] == '.'))
        {
          std::string_view name;
          int idx = -1;
          if (!parse_indexed_name(left, name, idx))
            throw std::runtime_error("Invalid token before .attr: " + std::string(left));
          const tinyxml2::XMLElement* next = nth_child_named(cur, name, idx);
          if (!next)
            return std::nullopt;
          cur = next;
        }

        if (attr.empty())
          throw std::runtime_error("Empty .attr on last segment");
        if (const char* aval = cur->Attribute(std::string(attr).c_str()))
        {
          return std::string(aval);  // raw text (may contain ${...} and math)
        }
        return std::nullopt;
      }
      // if last and not .attr → require .attr
      throw std::runtime_error("Terminal element requires `.attr` (e.g., `.../Tag.attr`).");
    }

    // mid-path #id: search within current subtree
    if (tok.front() == '#')
    {
      std::string_view id_sv = trim_sv(tok.substr(1));
      if (id_sv.empty())
        throw std::runtime_error("Empty #id in ${...}");
      const tinyxml2::XMLElement* found = dfs_find_id(cur, id_sv);
      if (!found)
        return std::nullopt;
      cur = found;
      continue;
    }

    // element or element[i]
    std::string_view name;
    int idx = -1;
    if (!parse_indexed_name(tok, name, idx))
      throw std::runtime_error("Invalid token in ${...} path: " + std::string(tok));
    if (name.empty())
      throw std::runtime_error("Empty element name in ${...} path.");

    const tinyxml2::XMLElement* next = nth_child_named(cur, name, idx);
    if (!next)
      return std::nullopt;
    cur = next;
  }

  // should not reach here (last must be .attr)
  return std::nullopt;
}

std::optional<double> resolve_numeric_ref(std::string_view ref, const ExprEvalContext& ctx)
{
  ref = trim_sv(ref);
  if (ref.empty())
    return std::nullopt;

  const tinyxml2::XMLElement* cur = nullptr;

  // --- anchor selection
  if (ref.size() >= 2 && ref.substr(0, 2) == "//")
  {
    ref.remove_prefix(2);
    cur = ctx.doc ? ctx.doc->RootElement() : nullptr;
  }
  else if (!ref.empty() && ref.front() == '/')
  {
    ref.remove_prefix(1);
    cur = ctx.object_root;
  }
  else if (!ref.empty() && ref.front() == '#')
  {
    std::string want;
    if (!consume_id_after_hash(ref, want))
      return std::nullopt;
    const tinyxml2::XMLElement* root = ctx.doc ? ctx.doc->RootElement() : nullptr;
    if (!root)
      return std::nullopt;
    cur = dfs_find_id(root, want);
    if (!cur)
      return std::nullopt;
    // `ref` now either empty or begins with '/' to continue walking under that element
  }
  else
  {
    cur = ctx.scope;
  }
  if (!cur)
    return std::nullopt;

  // --- walk slash-separated tokens
  auto tokens = split_slashes(ref);
  for (size_t ti = 0; ti < tokens.size(); ++ti)
  {
    auto tok = trim_sv(tokens[ti]);
    if (tok.empty())
      continue;

    const bool last = (ti + 1 == tokens.size());

    // FINAL-DOT attribute on the last token: "Name.attr" or ".attr" (current element)
    if (last)
    {
      size_t dot = tok.rfind('.');
      if (dot != std::string::npos && (dot + 1) < tok.size())
      {
        std::string_view left = trim_sv(tok.substr(0, dot));
        std::string_view attr = trim_sv(tok.substr(dot + 1));

        // Optional: descend one step into 'left' if provided
        if (!left.empty() && left != std::string_view("."))
        {
          std::string_view name;
          int idx = -1;
          if (!parse_indexed_name(left, name, idx))
            throw std::runtime_error("Invalid token before .attr: " + std::string(left));
          const tinyxml2::XMLElement* next = nth_child_named(cur, name, idx);
          if (!next)
            return std::nullopt;
          cur = next;
        }

        if (attr.empty())
          throw std::runtime_error("Empty .attr on last segment");
        const char* aval = cur->Attribute(std::string(attr).c_str());
        if (!aval)
          return std::nullopt;
        return eval_with_refs(aval, ctx);
      }
    }

    // No '@' allowed anymore
    if (tok.front() == '@')
    {
      throw std::runtime_error("`@attr` is not supported. Use final `.attr` (e.g., `.../Tag.attr`).");
    }

    if (tok == ".")
      continue;
    if (tok == "..")
    {
      cur = parent_element(cur);
      if (!cur)
        return std::nullopt;
      continue;
    }

    // mid-path #id (within current subtree)
    if (tok.front() == '#')
    {
      std::string_view id_sv = trim_sv(tok.substr(1));
      if (id_sv.empty())
        throw std::runtime_error("Empty #id in ${...}");
      const tinyxml2::XMLElement* found = dfs_find_id(cur, id_sv);
      if (!found)
        return std::nullopt;
      cur = found;
      continue;
    }

    // Element or Element[i]
    std::string_view name;
    int idx = -1;
    if (!parse_indexed_name(tok, name, idx))
      throw std::runtime_error("Invalid token in ${...} path: " + std::string(tok));
    if (name.empty())
      throw std::runtime_error("Empty element name in ${...} path.");

    const tinyxml2::XMLElement* next = nth_child_named(cur, name, idx);
    if (!next)
      return std::nullopt;
    cur = next;

    if (last)
    {
      // Strict: terminal element must specify attribute via final .attr
      throw std::runtime_error("Terminal element requires `.attr` (e.g., `.../Tag.attr`).");
    }
  }

  return std::nullopt;
}

const tinyxml2::XMLElement* find_enclosing_object(const tinyxml2::XMLElement* cur)
{
  const tinyxml2::XMLElement* e = cur;
  while (e)
  {
    if (strcmp(e->Name(), "Object") == 0)
      return e;
    const tinyxml2::XMLNode* p = e->Parent();
    e = p ? p->ToElement() : nullptr;
  }
  return nullptr;
}

ExprEvalContext make_context(const tinyxml2::XMLElement* scope)
{
  const tinyxml2::XMLDocument* doc = scope ? scope->GetDocument() : nullptr;
  const tinyxml2::XMLElement* object_root = scope ? find_enclosing_object(scope) : nullptr;
  return { doc, object_root, scope };
}

double evalBareMath(const char* expr)
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

double evalNumberExpr(const char* expr, const tinyxml2::XMLElement* scope)
{
  if (!expr)
    throw std::runtime_error("Null expression.");
  // Fast path: no ${...}, evaluate directly
  if (std::strstr(expr, "${") == nullptr)
    return evalBareMath(expr);
  return eval_with_refs(expr, scope);  // your multi-pass expander + muParser
}

double evalNumberAttributeRequired(const tinyxml2::XMLElement* elem, const char* attr)
{
  const char* expr = elem ? elem->Attribute(attr) : nullptr;
  if (!expr)
  {
    throw std::runtime_error(std::string("Missing required attribute '") + attr + "' on <" +
                             (elem ? elem->Name() : "?") + "> at line " + std::to_string(elem ? elem->GetLineNum() : 0));
  }
  return evalNumberExpr(expr, elem);
}

double evalNumberAttribute(const tinyxml2::XMLElement* elem, const char* attr, double fallback)
{
  const char* expr = elem ? elem->Attribute(attr) : nullptr;
  if (!expr)
  {
    return fallback;
  }
  return evalNumberExpr(expr, elem);
}

bool tryEvalNumberAttribute(const tinyxml2::XMLElement* elem, const char* attr, double* out)
{
  if (!elem || !attr || !out)
    return false;
  const char* expr = elem->Attribute(attr);
  if (!expr)
    return false;
  *out = evalNumberExpr(expr, elem);
  return true;
}

std::string evalTextExpr(const char* raw, const tinyxml2::XMLElement* scope, int max_depth = 32)
{
  if (!raw)
    return {};
  // Fast path: no ${...}
  if (std::strstr(raw, "${") == nullptr)
    return std::string(raw);

  // Build context
  ExprEvalContext ctx = make_context(scope);

  std::string s(raw);
  for (int depth = 0; depth < max_depth; ++depth)
  {
    bool changed = false;
    size_t pos = 0;
    while (true)
    {
      size_t l = s.find("${", pos);
      if (l == std::string::npos)
        break;
      size_t r = s.find('}', l + 2);
      if (r == std::string::npos)
        throw std::runtime_error("Unclosed ${...} in text expression");

      std::string ref = s.substr(l + 2, r - (l + 2));
      auto rawv = resolve_ref_raw(ref, ctx);  // returns raw attribute string
      if (!rawv.has_value())
        throw std::runtime_error("Cannot resolve reference: " + ref);

      s.replace(l, r - l + 1, *rawv);  // insert raw text (no parentheses)
      pos = l + rawv->size();
      changed = true;
    }
    if (!changed)
    {
      if (s.find("${") != std::string::npos)
        throw std::runtime_error("Unresolved ${...} after text expansion");
      return s;
    }
  }
  throw std::runtime_error("Recursive reference detected (text) or expansion too deep.");
}

std::string evalTextNode(const tinyxml2::XMLElement* elem)
{
  const char* raw = (elem && elem->GetText()) ? elem->GetText() : "";
  return evalTextExpr(raw, elem);
}

std::string evalTextAttributeRequired(const tinyxml2::XMLElement* elem, const char* attr)
{
  const char* raw = elem ? elem->Attribute(attr) : nullptr;
  if (!raw)
    throw std::runtime_error(std::string("Missing required attribute '") + attr + "' on <" +
                             (elem ? elem->Name() : "?") + "> at line " + std::to_string(elem ? elem->GetLineNum() : 0));
  return evalTextExpr(raw, elem);
}

std::string evalTextAttribute(const tinyxml2::XMLElement* elem, const char* attr, const std::string& fallback)
{
  const char* raw = elem ? elem->Attribute(attr) : nullptr;
  return raw ? evalTextExpr(raw, elem) : fallback;
}

bool tryEvalTextAttribute(const tinyxml2::XMLElement* elem, const char* attr, std::string* out)
{
  if (!elem || !attr || !out)
    return false;
  const char* raw = elem->Attribute(attr);
  if (!raw)
    return false;
  *out = evalTextExpr(raw, elem);
  return true;
}

bool evalBoolAttributeRequired(const tinyxml2::XMLElement* elem, const char* attr)
{
  std::string s = evalTextAttributeRequired(elem, attr);
  if (s == "true")
    return true;
  if (s == "false")
    return false;
  throw std::runtime_error(std::string("Attribute '") + attr + "' must be 'true' or 'false' at line " +
                           std::to_string(elem->GetLineNum()));
}

bool evalBoolAttribute(const tinyxml2::XMLElement* elem, const char* attr, bool fallback)
{
  std::string s;
  if (!tryEvalTextAttribute(elem, attr, &s))
    return fallback;
  if (s == "true")
    return true;
  if (s == "false")
    return false;
  throw std::runtime_error(std::string("Attribute '") + attr + "' must be 'true' or 'false' at line " +
                           std::to_string(elem->GetLineNum()));
}

uint32_t evalUIntAttributeRequired(const tinyxml2::XMLElement* elem, const char* attr)
{
  double v = evalNumberAttributeRequired(elem, attr);
  if (v < 0.0 || v > static_cast<double>(std::numeric_limits<uint32_t>::max()))
    throw std::runtime_error(std::string("Attribute '") + attr + "' out of range at line " +
                             std::to_string(elem->GetLineNum()));
  double r = std::round(v);
  if (std::abs(v - r) > 1e-9)
    throw std::runtime_error(std::string("Attribute '") + attr + "' must be an integer at line " +
                             std::to_string(elem->GetLineNum()));
  return static_cast<uint32_t>(r);
}

}  // namespace xml
}  // namespace sodf
