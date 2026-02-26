#ifndef SODF_XML_COMPONENT_PARSER_H_
#define SODF_XML_COMPONENT_PARSER_H_

#include <tinyxml2.h>

#include <sodf/database/database.h>
#include <sodf/xml/expression_parser.h>

namespace sodf {
namespace xml {

void parseDerivedFromParallelShapes(const tinyxml2::XMLElement* elem, const XMLParseContext& ctx,
                                    database::Database& db, database::EntityID eid);

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

}  // namespace xml
}  // namespace sodf

#endif  // SODF_XML_COMPONENT_PARSER_H_
