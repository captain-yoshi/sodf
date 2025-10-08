#include <sodf/xml/patch_operations.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <sodf/xml/scene_model.h>

namespace sodf {
namespace xml {

// -------- OWNERSHIP: clone component once before any mutation --------
void ensure_owned(SceneComponent& sc)
{
  if (sc.owned_doc)
    return;
  sc.owned_doc = std::make_unique<tinyxml2::XMLDocument>();
  tinyxml2::XMLNode* cloned = sc.xml->DeepClone(sc.owned_doc.get());
  sc.owned_doc->InsertFirstChild(cloned);
  sc.xml = cloned->ToElement();
}

static tinyxml2::XMLElement* component_root(SceneComponent& sc)
{
  ensure_owned(sc);
  return const_cast<tinyxml2::XMLElement*>(sc.xml);
}

// -------------------------- Selector mini-language ----------------------------
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
  std::string tag;              // e.g., "Origin"
  std::optional<int> index;     // [0]-based
  std::vector<AttrPred> preds;  // @id=foo, @*=bar, @name*=gl*b
};

struct Selector
{
  std::vector<Step> steps;
};

static std::string trim_copy(std::string s)
{
  auto issp = [](unsigned char c) { return std::isspace(c); };
  while (!s.empty() && issp(s.front()))
    s.erase(s.begin());
  while (!s.empty() && issp(s.back()))
    s.pop_back();
  return s;
}

static bool match_contains_pattern(const char* hay, const std::string& pat)
{
  if (!hay)
    return false;
  if (pat.empty())
    return true;

  // Split by '*', enforce ordered containment; treat leading/trailing '*' as unanchored.
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

static bool step_matches(const tinyxml2::XMLElement* e, const Step& st)
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
    else  // CONTAINS
    {
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

static Selector parse_selector(std::string path)
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

static std::vector<tinyxml2::XMLElement*> children_by_tag(tinyxml2::XMLElement* parent, const std::string& tag)
{
  std::vector<tinyxml2::XMLElement*> out;
  for (auto* c = parent->FirstChildElement(tag.c_str()); c; c = c->NextSiblingElement(tag.c_str()))
    out.push_back(c);
  return out;
}

static std::vector<tinyxml2::XMLElement*> eval_step(tinyxml2::XMLElement* base, const Step& st)
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

static std::optional<SceneComponentType> first_step_comp_type(const Selector& sel)
{
  if (sel.steps.empty())
    return std::nullopt;
  return sceneComponentTypeFromString(sel.steps[0].tag);
}

static std::vector<std::reference_wrapper<SceneComponent>> select_target_components(SceneObject& obj,
                                                                                    const Selector& sel)
{
  std::vector<std::reference_wrapper<SceneComponent>> out;

  // optional fast-filter by @id
  std::optional<std::string> want_id;
  for (const auto& p : sel.steps[0].preds)
    if (p.op == AttrPred::Op::EQUALS && p.name == "id")
      want_id = p.value;

  for (auto& sc : obj.components)
  {
    tinyxml2::XMLElement* root = component_root(sc);
    if (!root)
      continue;
    if (std::string(root->Name()) != sel.steps[0].tag)
      continue;

    if (want_id)
    {
      const char* rid = root->Attribute("id");
      if (!rid || *want_id != rid)
        continue;
    }
    out.push_back(sc);
  }
  return out;
}

static std::vector<tinyxml2::XMLElement*>
select_elements_in_components(SceneObject& obj, const Selector& sel,
                              std::vector<std::reference_wrapper<SceneComponent>>& comps)
{
  std::vector<tinyxml2::XMLElement*> hits;
  for (SceneComponent& sc : comps)
  {
    tinyxml2::XMLElement* cur = component_root(sc);
    if (!step_matches(cur, sel.steps[0]))
      continue;

    std::vector<tinyxml2::XMLElement*> frontier{ cur };
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
    hits.insert(hits.end(), frontier.begin(), frontier.end());
  }
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

enum class InsertAnchor
{
  APPEND,
  PREPEND,
  BEFORE,
  AFTER
};

static InsertAnchor parse_anchor(const tinyxml2::XMLElement* op)
{
  const char* a = op->Attribute("anchor");
  if (!a)
    return InsertAnchor::APPEND;
  std::string s = a;
  if (s == "before")
    return InsertAnchor::BEFORE;
  if (s == "after")
    return InsertAnchor::AFTER;
  if (s == "prepend")
    return InsertAnchor::PREPEND;
  return InsertAnchor::APPEND;
}

struct Hit
{
  SceneComponent* sc;
  tinyxml2::XMLElement* elem;
};

static std::vector<Hit> select_hits_in_components(SceneObject& obj, const Selector& sel,
                                                  std::vector<std::reference_wrapper<SceneComponent>>& comps)
{
  std::vector<Hit> hits;
  for (SceneComponent& sc : comps)
  {
    tinyxml2::XMLElement* cur = component_root(sc);
    if (!step_matches(cur, sel.steps[0]))
      continue;

    std::vector<tinyxml2::XMLElement*> frontier{ cur };
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
    for (auto* e : frontier)
      hits.push_back(Hit{ &sc, e });
  }
  return hits;
}

// ------------------------------- Operations ----------------------------------

// UPDATE block=... : replace matched node with the sole child of <Update>
static void op_update_block(SceneObject& obj, const tinyxml2::XMLElement* op)
{
  const char* path = op->Attribute("block");
  if (!path)
    throw std::runtime_error("<Update> missing 'block' attribute at line " + std::to_string(op->GetLineNum()));

  const tinyxml2::XMLElement* repl = op->FirstChildElement();
  if (!repl || repl->NextSiblingElement())
    throw std::runtime_error("<Update block> must contain exactly one element payload at line " +
                             std::to_string(op->GetLineNum()));

  Selector sel = parse_selector(path);
  auto comps = select_target_components(obj, sel);
  auto hits = select_hits_in_components(obj, sel, comps);

  for (auto& h : hits)
  {
    tinyxml2::XMLElement* t = h.elem;
    if (!t)
      continue;

    tinyxml2::XMLDocument* owner = t->GetDocument();
    tinyxml2::XMLNode* newnode = repl->DeepClone(owner);

    if (tinyxml2::XMLElement* parent_el = t->Parent() ? t->Parent()->ToElement() : nullptr)
    {
      parent_el->InsertAfterChild(t, newnode);
      parent_el->DeleteChild(t);
      continue;
    }

    // target is a component root (document child)
    owner->InsertFirstChild(newnode);
    if (auto* p = t->Parent())
      p->DeleteChild(t);
    if (h.sc)
      h.sc->xml = newnode->ToElement();  // retarget component root
  }
}

// UPDATE tag=... name="NewName"
static void op_update_tag(SceneObject& obj, const tinyxml2::XMLElement* op)
{
  const char* path = op->Attribute("tag");
  const char* name = op->Attribute("name");
  if (!path || !name)
    throw std::runtime_error("<Update> tag=... requires 'name' at line " + std::to_string(op->GetLineNum()));

  Selector sel = parse_selector(path);
  auto comps = select_target_components(obj, sel);
  auto targets = select_elements_in_components(obj, sel, comps);
  for (auto* t : targets)
    t->SetName(name);
}

// UPDATE attr="path.attr" value="..."
static void op_update_attr(SceneObject& obj, const tinyxml2::XMLElement* op)
{
  const char* spec = op->Attribute("attr");
  const char* val = op->Attribute("value");
  if (!spec || !val)
    throw std::runtime_error("<Update> attr=... requires 'value' at line " + std::to_string(op->GetLineNum()));

  std::string path, attr;
  if (!split_attr_path(spec, path, attr))
    throw std::runtime_error(std::string("<Update> invalid attr spec '") + spec + "'");

  Selector sel = parse_selector(path);
  auto comps = select_target_components(obj, sel);
  auto targets = select_elements_in_components(obj, sel, comps);
  for (auto* t : targets)
    t->SetAttribute(attr.c_str(), val);
}

// Add/Upsert attr on matched elements
static void op_add_attr(SceneObject& obj, const tinyxml2::XMLElement* op, bool upsert)
{
  const char* path = op->Attribute("attr");
  const char* name = op->Attribute("name");
  const char* val = op->Attribute("value");
  if (!path || !name || !val)
    throw std::runtime_error(std::string("<") + (upsert ? "Upsert" : "Add") +
                             "> attr=... requires name/value at line " + std::to_string(op->GetLineNum()));

  Selector sel = parse_selector(path);
  auto comps = select_target_components(obj, sel);
  auto targets = select_elements_in_components(obj, sel, comps);
  for (auto* t : targets)
  {
    if (!upsert && t->Attribute(name) != nullptr)
      throw std::runtime_error("<Add attr> duplicates attribute '" + std::string(name) + "'.");
    t->SetAttribute(name, val);
  }
}

static void insert_before(tinyxml2::XMLNode* parent, tinyxml2::XMLNode* ref, tinyxml2::XMLNode* nn)
{
  if (!parent)
    return;
  if (auto* prev = ref ? ref->PreviousSibling() : nullptr)
    parent->InsertAfterChild(prev, nn);
  else
    parent->InsertFirstChild(nn);
}

static void insert_after(tinyxml2::XMLNode* parent, tinyxml2::XMLNode* ref, tinyxml2::XMLNode* nn)
{
  if (!parent)
    return;
  parent->InsertAfterChild(ref, nn);
}

// Add/Upsert block children into matched container (with optional anchor={append,prepend,before,after})
static void op_add_block(SceneObject& obj, const tinyxml2::XMLElement* op, bool upsert_is_noop)
{
  (void)upsert_is_noop;  // semantics currently same as Add for blocks (no duplicate-detect at container level)

  const char* path = op->Attribute("block");
  if (!path)
    throw std::runtime_error(std::string("<") + (upsert_is_noop ? "Upsert" : "Add") + "> missing 'block' at line " +
                             std::to_string(op->GetLineNum()));

  Selector sel = parse_selector(path);
  auto comps = select_target_components(obj, sel);
  auto targets = select_elements_in_components(obj, sel, comps);
  InsertAnchor anchor = parse_anchor(op);

  for (auto* t : targets)
  {
    tinyxml2::XMLDocument* owner = t->GetDocument();
    for (auto* payload = op->FirstChildElement(); payload; payload = payload->NextSiblingElement())
    {
      tinyxml2::XMLNode* node = payload->DeepClone(owner);
      switch (anchor)
      {
        case InsertAnchor::APPEND:
          t->InsertEndChild(node);
          break;
        case InsertAnchor::PREPEND:
          t->InsertFirstChild(node);
          break;
        case InsertAnchor::BEFORE:
          insert_before(t->Parent(), t, node);
          break;
        case InsertAnchor::AFTER:
          insert_after(t->Parent(), t, node);
          break;
      }
    }
  }
}

// Remove matching nodes. If selector root matches a component root, remove whole component.
// Strict mode: throw if nothing was removed.
static void op_remove_block(SceneObject& obj, const tinyxml2::XMLElement* op)
{
  const char* path = op->Attribute("block");
  if (!path)
    throw std::runtime_error("<Remove> missing 'block' at line " + std::to_string(op->GetLineNum()));

  Selector sel = parse_selector(path);
  if (sel.steps.empty())
    return;

  auto opt_type = first_step_comp_type(sel);
  if (!opt_type)
    throw std::runtime_error("Remove block must start with a known component type (e.g., FSM, Link, Origin).");

  bool removed_any = false;

  // Single-step: remove matching top-level components
  if (sel.steps.size() == 1)
  {
    std::vector<size_t> idx;
    for (size_t i = 0; i < obj.components.size(); ++i)
    {
      SceneComponent& sc = obj.components[i];
      tinyxml2::XMLElement* root = component_root(sc);
      if (root && step_matches(root, sel.steps[0]))
        idx.push_back(i);
    }
    if (!idx.empty())
    {
      removed_any = true;
      for (size_t k = 0; k < idx.size(); ++k)
        obj.components.erase(obj.components.begin() + (idx[idx.size() - 1 - k]));
    }
    else
    {
      throw std::runtime_error(std::string("<Remove> matched no elements for '") + path + "' at line " +
                               std::to_string(op->GetLineNum()));
    }
    return;
  }

  // Multi-step: compute precise hits
  auto comps = select_target_components(obj, sel);
  auto hits = select_hits_in_components(obj, sel, comps);

  // Remove any component whose root is directly matched
  std::vector<size_t> erase_idx;
  for (size_t i = 0; i < obj.components.size(); ++i)
  {
    SceneComponent& sc = obj.components[i];
    tinyxml2::XMLElement* root = component_root(sc);
    bool root_hit = false;
    for (const auto& h : hits)
      if (h.sc == &sc && h.elem == root)
      {
        root_hit = true;
        break;
      }
    if (root_hit)
      erase_idx.push_back(i);
  }
  if (!erase_idx.empty())
  {
    removed_any = true;
    for (size_t k = 0; k < erase_idx.size(); ++k)
      obj.components.erase(obj.components.begin() + (erase_idx.size() - 1 - k));
  }

  // Delete non-root matches inside still-present components
  for (const auto& h : hits)
  {
    bool still_present = std::any_of(obj.components.begin(), obj.components.end(),
                                     [&](const SceneComponent& sc) { return &sc == h.sc; });
    if (!still_present)
      continue;
    if (auto* p = h.elem->Parent())
    {
      p->DeleteChild(h.elem);
      removed_any = true;
    }
  }

  if (!removed_any)
    throw std::runtime_error(std::string("<Remove> matched no elements for '") + path + "' at line " +
                             std::to_string(op->GetLineNum()));
}

// ------------------------------ Public API -----------------------------------

bool is_patch_tag(std::string_view tag)
{
  return tag == "Add" || tag == "Update" || tag == "Upsert" || tag == "Remove";
}

void apply_overlay(SceneObject& obj, const tinyxml2::XMLElement* overlay_xml)
{
  for (const auto* child = overlay_xml->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    const std::string tag = child->Name();

    // 1) Direct component (acts like upsert)
    if (auto t = sceneComponentTypeFromString(tag))
    {
      SceneComponentType comp_type = *t;
      std::string comp_id = child->Attribute("id") ? child->Attribute("id") : "";

      auto match = [&](const SceneComponent& sc) { return sc.type == comp_type && sc.id == comp_id; };
      obj.components.erase(std::remove_if(obj.components.begin(), obj.components.end(), match), obj.components.end());
      obj.components.push_back({ comp_type, comp_id, child });
      continue;
    }

    // 2) Patch blocks inside <Overlay> use the path-style API
    if (is_patch_tag(tag))
    {
      apply_patch_block(obj, child, tag);
      continue;
    }

    // 3) Unexpected
    throw std::runtime_error("Unexpected tag <" + tag + "> inside <Overlay>.");
  }
}

void apply_patch_block(SceneObject& obj, const tinyxml2::XMLElement* block, std::string_view op_name)
{
  const bool has_block = block->Attribute("block");
  const bool has_tag = block->Attribute("tag");
  const bool has_attr = block->Attribute("attr");

  if (!(has_block || has_tag || has_attr))
  {
    throw std::runtime_error(std::string("<") + std::string(op_name) +
                             "> must use the path-style API: one of block=..., tag=..., or attr=... (line " +
                             std::to_string(block->GetLineNum()) + ").");
  }

  const bool is_add = (op_name == "Add");
  const bool is_update = (op_name == "Update");
  const bool is_upsert = (op_name == "Upsert");
  const bool is_remove = (op_name == "Remove");

  if (is_remove)
  {
    if (!has_block)
      throw std::runtime_error("<Remove> only supports block=... (line " + std::to_string(block->GetLineNum()) + ").");
    op_remove_block(obj, block);
    return;
  }

  if (is_update)
  {
    if (has_block)
    {
      op_update_block(obj, block);
      return;
    }
    if (has_tag)
    {
      op_update_tag(obj, block);
      return;
    }
    if (has_attr)
    {
      op_update_attr(obj, block);
      return;
    }
  }

  if (is_add || is_upsert)
  {
    if (has_attr)
    {
      op_add_attr(obj, block, /*upsert=*/is_upsert);
      return;
    }
    if (has_block)
    {
      op_add_block(obj, block, /*upsert_is_noop=*/is_upsert);
      return;
    }
  }

  throw std::runtime_error(std::string("<") + std::string(op_name) +
                           "> has an unsupported combination of attributes (line " +
                           std::to_string(block->GetLineNum()) + ").");
}

// Upsert a direct component tag like <Link>, <Joint>, <FSM>, ...
void upsert_direct_component(SceneObject& obj, const tinyxml2::XMLElement* child)
{
  const std::string tag = child->Name();
  auto opt_type = sceneComponentTypeFromString(tag);
  if (!opt_type)
    throw std::runtime_error("Unknown ComponentType string: " + tag);
  SceneComponentType comp_type = *opt_type;

  std::string comp_id = child->Attribute("id") ? child->Attribute("id") : "";
  auto match_pred = [&](const SceneComponent& sc) { return sc.type == comp_type && sc.id == comp_id; };

  obj.components.erase(std::remove_if(obj.components.begin(), obj.components.end(), match_pred), obj.components.end());
  obj.components.push_back({ comp_type, comp_id, child });
}

}  // namespace xml
}  // namespace sodf
