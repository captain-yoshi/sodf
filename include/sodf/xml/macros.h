#ifndef SODF_XML_MACROS_H_
#define SODF_XML_MACROS_H_

#include <string>
#include <functional>
#include <tinyxml2.h>
#include <exception>
#include "sodf/xml/expression_parser.h"

#include <sodf/database/database.h>

namespace sodf {
namespace xml {

// forward declaration
void parseActionMapComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                             database::EntityID eid);

void parseButtonComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                          database::EntityID eid);

void parseContainerComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                             database::EntityID eid);

void parseInsertionComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                             database::EntityID eid);

void parseFSMComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                       database::EntityID eid);

void parseJointComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                         database::EntityID eid);

void parseLinkComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                        database::EntityID eid);

void parseOriginComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                          database::EntityID eid);

void parseParallelGraspComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                                 database::EntityID eid);

void parseProductComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                           database::EntityID eid);

void parseShapeComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                         database::EntityID eid);

void parseShapeRefComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                            database::EntityID eid);

void parseStackedShapeComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                                database::EntityID eid);

void parseStackedShapeRefComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                                   database::EntityID eid);

void parseDomainShapeComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                               database::EntityID eid);

void parseDomainShapeRefComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                                  database::EntityID eid);

void parseTouchscreenComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                               database::EntityID eid);

void parseTransformComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                             database::EntityID eid);

void parseVirtualButtonComponent(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database& db,
                                 database::EntityID eid);

/// List of component tags allowed directly under <Object>
#define SODF_XML_COMPONENT_PARSERS(X)                                                                                  \
  X(Origin, parseOriginComponent)                                                                                      \
  X(Link, parseLinkComponent)                                                                                          \
  X(Joint, parseJointComponent)                                                                                        \
  X(Insertion, parseInsertionComponent)                                                                                \
  X(Product, parseProductComponent)                                                                                    \
  X(Touchscreen, parseTouchscreenComponent)                                                                            \
  X(FSM, parseFSMComponent)                                                                                            \
  X(Button, parseButtonComponent)                                                                                      \
  X(VirtualButton, parseVirtualButtonComponent)                                                                        \
  X(Shape, parseShapeComponent)                                                                                        \
  X(ShapeRef, parseShapeRefComponent)                                                                                  \
  X(StackedShape, parseStackedShapeComponent)                                                                          \
  X(StackedShapeRef, parseStackedShapeRefComponent)                                                                    \
  X(DomainShape, parseDomainShapeComponent)                                                                            \
  X(DomainShapeRef, parseDomainShapeRefComponent)                                                                      \
  X(ParallelGrasp, parseParallelGraspComponent)                                                                        \
  X(Container, parseContainerComponent)                                                                                \
  /* Special: Control tags, not a true components. The handler does nothing. */                                        \
  X(ForLoop,                                                                                                           \
    [](const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database&, database::EntityID) {})      \
  X(Overlay,                                                                                                           \
    [](const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database&, database::EntityID) {})

/// Only valid as nested subcomponents, never directly under <Object>
#define SODF_XML_SUBCOMPONENT_PARSERS(X)                                                                               \
  X(Transform, parseTransformComponent)                                                                                \
  X(ActionMap, parseActionMapComponent)

enum class SceneComponentType
{
#define X(name, func) name,
  SODF_XML_COMPONENT_PARSERS(X)
#undef X
      COUNT
};

enum class SceneSubComponentType
{
#define X(name, func) name,
  SODF_XML_SUBCOMPONENT_PARSERS(X)
#undef X
      COUNT
};

constexpr SceneComponentType ForLoopType = SceneComponentType::ForLoop;
constexpr size_t ForLoopIndex = static_cast<size_t>(ForLoopType);

inline std::optional<SceneComponentType> sceneComponentTypeFromString(const std::string& name)
{
#define X(tag, func)                                                                                                   \
  if (name == #tag)                                                                                                    \
    return SceneComponentType::tag;
  SODF_XML_COMPONENT_PARSERS(X)
#undef X
  return std::nullopt;
}

inline const char* sceneComponentTypeToString(SceneComponentType type)
{
  switch (type)
  {
#define X(name, func)                                                                                                  \
  case SceneComponentType::name:                                                                                       \
    return #name;
    SODF_XML_COMPONENT_PARSERS(X)
#undef X
    default:
      return "Unknown";
  }
}

inline std::optional<SceneSubComponentType> sceneSubComponentTypeFromString(const std::string& name)
{
#define X(tag, func)                                                                                                   \
  if (name == #tag)                                                                                                    \
    return SceneSubComponentType::tag;
  SODF_XML_SUBCOMPONENT_PARSERS(X)
#undef X
  return std::nullopt;
}

using ParseFunc = std::function<void(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx, database::Database&,
                                     database::EntityID)>;

static const std::vector<ParseFunc> parseFuncs = {
#define X(name, func) func,
  SODF_XML_COMPONENT_PARSERS(X)
#undef X
};

using SubParseFunc = std::function<void(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx,
                                        database::Database&, database::EntityID)>;

static const std::vector<SubParseFunc> subParseFuncs = {
#define X(name, func) func,
  SODF_XML_SUBCOMPONENT_PARSERS(X)
#undef X
};

}  // namespace xml
}  // namespace sodf

#endif  // SODF_XML_MACROS_H_
