#include "sodf/xml/expression_parser.h"
#include <string_view>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <sodf/xml/selector.h>

#include "muParser.h"

namespace sodf {
namespace xml {

std::optional<double> resolve_numeric_ref(std::string_view ref, const ExprEvalContext& ctx);
double parseExpression(const char* expr);
static std::optional<std::string> resolve_ref_raw(std::string_view ref, const ExprEvalContext& ctx);
double evalBareMath(const char* expr);
ExprEvalContext make_context(const tinyxml2::XMLElement* scope);

namespace {
using Real = mu::value_type;

// High-precision constants computed in long double
constexpr long double kPiL = 3.141592653589793238462643383279502884L;
constexpr long double kTauL = 2.0L * kPiL;
constexpr long double kDeg2RadL = kPiL / 180.0L;
constexpr long double kRad2DegL = 180.0L / kPiL;

// Radian trig (compute in long double; return Real)
Real wrap_sin(Real x)
{
  return static_cast<Real>(::sin((long double)x));
}
Real wrap_cos(Real x)
{
  return static_cast<Real>(::cos((long double)x));
}
Real wrap_tan(Real x)
{
  return static_cast<Real>(::tan((long double)x));
}
Real wrap_asin(Real x)
{
  return static_cast<Real>(::asin((long double)x));
}
Real wrap_acos(Real x)
{
  return static_cast<Real>(::acos((long double)x));
}

// Degree trig
Real wrap_sind(Real x)
{
  return static_cast<Real>(::sin((long double)x * kDeg2RadL));
}
Real wrap_cosd(Real x)
{
  return static_cast<Real>(::cos((long double)x * kDeg2RadL));
}
Real wrap_tand(Real x)
{
  return static_cast<Real>(::tan((long double)x * kDeg2RadL));
}
Real wrap_asind(Real x)
{
  return static_cast<Real>(::asin((long double)x) * kRad2DegL);
}
Real wrap_acosd(Real x)
{
  return static_cast<Real>(::acos((long double)x) * kRad2DegL);
}

// Two-arg
Real wrap_atan2(Real y, Real x)
{
  return static_cast<Real>(::atan2((long double)y, (long double)x));
}
Real wrap_atan2d(Real y, Real x)
{
  return static_cast<Real>(::atan2((long double)y, (long double)x) * kRad2DegL);
}

// Strict integer checks for custom bitwise ops
static bool is_intlike(mu::value_type x, mu::value_type tol = 1e-9)
{
  if (!std::isfinite(x))
    return false;
  // nearest integer as mu::value_type, then compare within tol
  long long r = static_cast<long long>(std::llround(x));
  return std::fabs(x - static_cast<mu::value_type>(r)) <= tol;
}

static long long to_int_checked(mu::value_type x)
{
  if (!is_intlike(x))
    throw mu::Parser::exception_type("bitwise operator requires integer operands");
  return static_cast<long long>(std::llround(x));
}

// Bitwise AND (Integers only)
static mu::value_type wrap_band(mu::value_type a, mu::value_type b)
{
  const long long ia = to_int_checked(a);
  const long long ib = to_int_checked(b);
  return static_cast<mu::value_type>(ia & ib);
}

static mu::value_type wrap_bor(mu::value_type a, mu::value_type b)
{
  return static_cast<mu::value_type>(to_int_checked(a) | to_int_checked(b));
}

// Modulo operator
static mu::value_type wrap_imod(mu::value_type a, mu::value_type b)
{
  const long long ia = to_int_checked(a);
  const long long ib = to_int_checked(b);
  if (ib == 0)
    throw mu::Parser::exception_type("modulo by zero");
  return static_cast<mu::value_type>(ia % ib);
}

}  // anonymous namespace

void register_math_symbols(mu::Parser& parser)
{
  using Real = mu::value_type;

  const Real pi = static_cast<Real>(kPiL);
  const Real tau = static_cast<Real>(kTauL);
  const Real inf = std::numeric_limits<Real>::infinity();
  const Real nan = std::numeric_limits<Real>::quiet_NaN();

  parser.DefineConst("pi", pi);
  parser.DefineConst("tau", tau);

  parser.DefineConst("inf", inf);
  parser.DefineConst("nan", nan);

  // radian trig
  parser.DefineFun("sin", mu::fun_type1(&wrap_sin));
  parser.DefineFun("cos", mu::fun_type1(&wrap_cos));
  parser.DefineFun("tan", mu::fun_type1(&wrap_tan));
  parser.DefineFun("asin", mu::fun_type1(&wrap_asin));
  parser.DefineFun("acos", mu::fun_type1(&wrap_acos));
  parser.DefineFun("atan2", mu::fun_type2(&wrap_atan2));

  // degree trig
  parser.DefineFun("sind", mu::fun_type1(&wrap_sind));
  parser.DefineFun("cosd", mu::fun_type1(&wrap_cosd));
  parser.DefineFun("tand", mu::fun_type1(&wrap_tand));
  parser.DefineFun("asind", mu::fun_type1(&wrap_asind));
  parser.DefineFun("acosd", mu::fun_type1(&wrap_acosd));
  parser.DefineFun("atan2d", mu::fun_type2(&wrap_atan2d));

  // bitwise operators
  constexpr int PRIO_AND = 4;
  constexpr int PRIO_OR = 4;

  parser.DefineOprt("&", mu::fun_type2(&wrap_band), PRIO_AND, mu::oaLEFT, true);
  parser.DefineOprt("|", mu::fun_type2(&wrap_bor), PRIO_OR, mu::oaLEFT, true);

  // arithmetic remainder
  parser.DefineOprt("%", mu::fun_type2(&wrap_imod), mu::prMUL_DIV, mu::oaLEFT, true);
}

static mu::Parser& get_math_parser()
{
  // One parser per thread, initialized once
  thread_local mu::Parser parser = [] {
    mu::Parser p;
    register_math_symbols(p);
    return p;
  }();
  return parser;
}

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
      auto rawv = resolve_ref_raw(ref, ctx);
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
  int bracket_depth = 0;
  char quote = 0;

  auto flush = [&](size_t i) {
    if (i >= start)
      out.push_back(p.substr(start, i - start));
    start = i + 1;
  };

  for (size_t i = 0; i < n; ++i)
  {
    char c = p[i];
    if (quote)
    {
      if (c == quote)
        quote = 0;
      continue;
    }
    if (c == '\'' || c == '"')
    {
      quote = c;
      continue;
    }
    if (c == '[')
    {
      ++bracket_depth;
      continue;
    }
    if (c == ']')
    {
      if (bracket_depth > 0)
        --bracket_depth;
      continue;
    }

    if (c == '/' && bracket_depth == 0 && quote == 0)
    {
      flush(i);
    }
  }
  // tail
  if (start <= n)
    out.push_back(p.substr(start, n - start));
  return out;
}

// Helper: are we a top-level element (whose parent is the document root element)?
static bool is_top_level_under_doc_root(const tinyxml2::XMLElement* cur,
                                        const tinyxml2::XMLElement** out_parent_root = nullptr)
{
  if (!cur)
    return false;
  const tinyxml2::XMLNode* p = cur->Parent();
  const tinyxml2::XMLElement* parent_el = p ? p->ToElement() : nullptr;
  if (!parent_el)
    return false;  // cur itself is very likely the true root element.
  const tinyxml2::XMLNode* pp = parent_el->Parent();
  bool top = pp && (pp->ToElement() == nullptr);  // parent_el's parent is XMLDocument
  if (top && out_parent_root)
    *out_parent_root = parent_el;
  return top;
}

// --- STRICT: guard against predicates with missing keys or values.
// Throws on:
//   [@]         -> missing key
//   [@=X]       -> missing key
//   [@*=foo]    -> missing key
//   [=foo]      -> missing key (no '@')
//   [*=foo]     -> missing key (no '@')
//   [@id=]      -> missing value
//   [@id*=]     -> missing value
static void validate_attr_predicates_have_keys_and_values(std::string_view token)
{
  auto is_space = [](char c) { return std::isspace(static_cast<unsigned char>(c)) != 0; };
  auto trim = [&](std::string_view s) {
    size_t a = 0, b = s.size();
    while (a < b && is_space(s[a]))
      ++a;
    while (b > a && is_space(s[b - 1]))
      --b;
    return s.substr(a, b - a);
  };

  size_t i = 0;
  while (i < token.size())
  {
    if (token[i] != '[')
    {
      ++i;
      continue;
    }

    // Find the matching ']'
    size_t j = i + 1;
    int depth = 1;
    char quote = 0;
    for (; j < token.size(); ++j)
    {
      char c = token[j];
      if (quote)
      {
        if (c == quote)
          quote = 0;
        continue;
      }
      if (c == '\'' || c == '"')
      {
        quote = c;
        continue;
      }
      if (c == '[')
      {
        ++depth;
        continue;
      }
      if (c == ']')
      {
        if (--depth == 0)
          break;
      }
    }
    if (j >= token.size())
      break;  // malformed; let downstream parse complain

    std::string_view body = trim(token.substr(i + 1, j - (i + 1)));
    if (!body.empty())
    {
      if (body.front() == '@')
      {
        // Parse @key[(*=|=)value] or @key (exists) or @* forms
        std::string_view rest = trim(body.substr(1));  // after '@'

        // Find operator (*= or =) while ignoring quotes
        size_t op_pos = std::string::npos;
        bool contains_op = false;
        {  // scan rest, track quotes to find unquoted '='
          char q = 0;
          for (size_t k = 0; k < rest.size(); ++k)
          {
            char c = rest[k];
            if (q)
            {
              if (c == q)
                q = 0;
              continue;
            }
            if (c == '\'' || c == '"')
            {
              q = c;
              continue;
            }
            if (c == '=')
            {
              op_pos = k;
              contains_op = (k > 0 && rest[k - 1] == '*');  // "*=" if true
              break;
            }
          }
        }

        if (op_pos == std::string::npos)
        {
          // EXISTS form: [@name] or [@*]
          std::string_view key = trim(rest);
          if (key.empty())
            throw std::runtime_error("Missing key in attribute predicate '[" + std::string(body) + "]'");
        }
        else
        {
          // EQUALS/CONTAINS: LHS must be a key (possibly "*"), RHS must be non-empty
          std::string_view key = trim(rest.substr(0, op_pos - (contains_op ? 1 : 0)));
          if (key.empty())
            throw std::runtime_error("Missing key in attribute predicate '[" + std::string(body) + "]'");

          std::string_view rhs = trim(rest.substr(op_pos + 1));
          if (rhs.empty())
            throw std::runtime_error("Missing value in attribute predicate '[" + std::string(body) + "]'");
        }
      }
      else
      {
        // If there is an (unquoted) '=' anywhere in body but no leading '@',
        // it's an attribute predicate missing the key (e.g. [=foo], [*=bar]).
        bool has_unquoted_eq = false;
        char q = 0;
        for (size_t k = 0; k < body.size(); ++k)
        {
          char c = body[k];
          if (q)
          {
            if (c == q)
              q = 0;
            continue;
          }
          if (c == '\'' || c == '"')
          {
            q = c;
            continue;
          }
          if (c == '=')
          {
            has_unquoted_eq = true;
            break;
          }
        }
        if (has_unquoted_eq)
        {
          throw std::runtime_error("Missing key in attribute predicate '[" + std::string(body) + "]'");
        }
        // Otherwise: digits-only index like [2] is allowed — no error here.
      }
    }

    i = j + 1;
  }
}

static std::vector<tinyxml2::XMLElement*> eval_one_step_or_root_retry(const tinyxml2::XMLElement* cur,
                                                                      std::string_view token)
{
  validate_attr_predicates_have_keys_and_values(token);

  auto sel = parse_selector(std::string(token));
  if (sel.steps.size() != 1)
    throw std::runtime_error("Invalid token in ${...} path: " + std::string(token));

  auto* cur_nc = const_cast<tinyxml2::XMLElement*>(cur);
  auto nexts = eval_step(cur_nc, sel.steps[0]);
  if (!nexts.empty())
    return nexts;

  // If we are at a top-level element under the document root AND the child search failed,
  // retry the same step against the document root element. This enables ../Object/... to
  // select sibling <Object> elements at the root.
  const tinyxml2::XMLElement* parent_root = nullptr;
  if (is_top_level_under_doc_root(cur, &parent_root) && parent_root)
  {
    auto* parent_nc = const_cast<tinyxml2::XMLElement*>(parent_root);
    auto retry = eval_step(parent_nc, sel.steps[0]);
    if (!retry.empty())
      return retry;
  }

  return {};
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
      if (expr.find("${") != std::string::npos)
        throw std::runtime_error("Unresolved ${...} after expansion");
      return evalBareMath(expr.c_str());
    }
  }

  throw std::runtime_error("Recursive reference detected or expansion too deep (> " + std::to_string(max_depth) + ").");
}

double eval_with_refs(const char* expr, const tinyxml2::XMLElement* scope, int max_depth = MAX_RECURSION_DEPTH)
{
  ExprEvalContext ctx = make_context(scope);
  return eval_with_refs(expr, ctx, max_depth);
}

static std::optional<std::string> resolve_ref_raw(std::string_view ref, const ExprEvalContext& ctx)
{
  ref = trim_sv(ref);
  if (ref.empty())
    return std::nullopt;

  const tinyxml2::XMLElement* cur = nullptr;

  // explicit anchor
  if (ref.size() >= 2 && ref.substr(0, 2) == "//")
  {
    ref.remove_prefix(2);
    cur = ctx.doc ? ctx.doc->RootElement() : nullptr;  // // document root
  }
  else if (!ref.empty() && ref.front() == '/')
  {
    ref.remove_prefix(1);
    cur = ctx.object_root;  // / enclosing Object root
  }
  else
  {
    cur = ctx.scope;  // default: current element scope
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

    if (tok == ".")  // ./ children scope is the default; "." is a no-op step
      continue;

    if (tok == "..")  // ../ parent scope
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

        // optional single-step descend before reading attribute
        if (!left.empty() && !(left.size() == 1 && left[0] == '.'))
        {
          auto nexts = eval_one_step_or_root_retry(cur, left);
          if (nexts.empty())
            return std::nullopt;
          cur = nexts.front();
        }

        if (attr.empty())
          throw std::runtime_error("Empty .attr on last segment");
        if (const char* aval = cur->Attribute(std::string(attr).c_str()))
        {
          return std::string(aval);  // raw text (may contain ${...} and math)
        }
        return std::nullopt;
      }
      // if last and not .attr → require .attr (explicit rule)
      throw std::runtime_error("Terminal element requires `.attr` (e.g., `.../Tag.attr`).");
    }

    // normal element step (./children scope by default) with top-level retry
    {
      auto nexts = eval_one_step_or_root_retry(cur, tok);
      if (nexts.empty())
        return std::nullopt;
      cur = nexts.front();
    }
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

  // explicit anchor
  if (ref.size() >= 2 && ref.substr(0, 2) == "//")
  {
    ref.remove_prefix(2);
    cur = ctx.doc ? ctx.doc->RootElement() : nullptr;  // // document root
  }
  else if (!ref.empty() && ref.front() == '/')
  {
    ref.remove_prefix(1);
    cur = ctx.object_root;  // / enclosing Object root
  }
  else
  {
    cur = ctx.scope;  // default: current element scope
  }
  if (!cur)
    return std::nullopt;

  // walk slash-separated tokens
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

        // Optional: descend one step into 'left' if provided (with top-level retry)
        if (!left.empty() && left != std::string_view("."))
        {
          auto nexts = eval_one_step_or_root_retry(cur, left);
          if (nexts.empty())
            return std::nullopt;
          cur = nexts.front();
        }

        if (attr.empty())
          throw std::runtime_error("Empty .attr on last segment");
        const char* aval = cur->Attribute(std::string(attr).c_str());
        if (!aval)
          return std::nullopt;
        return eval_with_refs(aval, ctx);
      }
    }

    // No '@' allowed — explicit .attr only
    if (tok.front() == '@')
      throw std::runtime_error("`@attr` is not supported. Use final `.attr` (e.g., `.../Tag.attr`).");

    if (tok == ".")  // ./ children scope
      continue;

    if (tok == "..")  // ../ parent scope
    {
      cur = parent_element(cur);
      if (!cur)
        return std::nullopt;
      continue;
    }

    // Element or Element[i], default child scope with top-level retry
    {
      auto nexts = eval_one_step_or_root_retry(cur, tok);
      if (nexts.empty())
        return std::nullopt;
      cur = nexts.front();
    }

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
    mu::Parser& parser = get_math_parser();  // reuse per-thread instance
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
  if (std::strstr(expr, "${") == nullptr)
    return evalBareMath(expr);
  return eval_with_refs(expr, scope);
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
    return fallback;
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
  if (std::strstr(raw, "${") == nullptr)
    return std::string(raw);

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
      auto rawv = resolve_ref_raw(ref, ctx);
      if (!rawv.has_value())
        throw std::runtime_error("Cannot resolve reference: " + ref);

      s.replace(l, r - l + 1, *rawv);
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
