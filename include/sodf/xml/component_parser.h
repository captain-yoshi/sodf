#ifndef SODF_XML_COMPONENT_PARSER_H_
#define SODF_XML_COMPONENT_PARSER_H_

#include <tinyxml2.h>

#include <sodf/ecs/database.h>

namespace sodf {
namespace xml {

void parseDerivedFromParallelShapes(const tinyxml2::XMLElement* obj_elem, ecs::Database& db, ecs::EntityID eid);

void parseActionMapComponent(const tinyxml2::XMLElement* obj_elem, ecs::Database& db, ecs::EntityID eid);
void parseButtonComponent(const tinyxml2::XMLElement* obj_elem, ecs::Database& db, ecs::EntityID eid);
void parseContainerComponent(const tinyxml2::XMLElement* obj_elem, ecs::Database& db, ecs::EntityID eid);
void parseInsertionComponent(const tinyxml2::XMLElement* obj_elem, ecs::Database& db, ecs::EntityID eid);
void parseFluidDomainShapeComponent(const tinyxml2::XMLElement* obj_elem, ecs::Database& db, ecs::EntityID eid);
void parseFSMComponent(const tinyxml2::XMLElement* obj_elem, ecs::Database& db, ecs::EntityID eid);
void parseJointComponent(const tinyxml2::XMLElement* obj_elem, ecs::Database& db, ecs::EntityID eid);
void parseLinkComponent(const tinyxml2::XMLElement* obj_elem, ecs::Database& db, ecs::EntityID eid);
void parseOriginComponent(const tinyxml2::XMLElement* obj_elem, ecs::Database& db, ecs::EntityID eid);
void parseParallelGraspComponent(const tinyxml2::XMLElement* obj_elem, ecs::Database& db, ecs::EntityID eid);
void parseProductComponent(const tinyxml2::XMLElement* obj_elem, ecs::Database& db, ecs::EntityID eid);
void parseShapeComponent(const tinyxml2::XMLElement* obj_elem, ecs::Database& db, ecs::EntityID eid);
void parseStackedShapeComponent(const tinyxml2::XMLElement* elem, ecs::Database& db, ecs::EntityID eid);
void parseTouchscreenComponent(const tinyxml2::XMLElement* obj_elem, ecs::Database& db, ecs::EntityID eid);
void parseTransformComponent(const tinyxml2::XMLElement* obj_elem, ecs::Database& db, ecs::EntityID eid);
void parseVirtualButtonComponent(const tinyxml2::XMLElement* obj_elem, ecs::Database& db, ecs::EntityID eid);

}  // namespace xml
}  // namespace sodf

#endif  // SODF_XML_COMPONENT_PARSER_H_
