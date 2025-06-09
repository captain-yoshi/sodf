#ifndef SODF_XML_PARSER_H_
#define SODF_XML_PARSER_H_

#include <set>
#include <unordered_map>
#include <memory>
#include <tinyxml2.h>

#include <sodf/ecs.h>
#include <sodf/components/shape.h>
#include <sodf/components/transform.h>
#include <sodf/components/finite_state_machine.h>

namespace sodf {

/**
 * @brief Main class for loading and parsing XML scene/entity/component files.
 */
class XMLParser
{
public:
  XMLParser();
  ~XMLParser();

  // Loads entities from a file (resolves includes relative to file location)
  bool loadEntitiesFromFile(const std::string& filename, ginseng::database& db);

  // Loads entities from text (optionally resolve includes relative to base_dir)
  bool loadEntitiesFromText(const std::string& text, ginseng::database& db, const std::string& base_dir = "");

private:
  bool loadEntities(tinyxml2::XMLDocument* doc, const std::string& base_dir, ginseng::database& db);

  /// Pointer to the loaded XML document (lifetime is managed).
  std::unique_ptr<tinyxml2::XMLDocument> doc;
};

// -----------------------------------------------------------------------------
// Math/Transform Parsing Utilities
// -----------------------------------------------------------------------------

/// Parse an Isometry3D from an XML element.
Eigen::Isometry3d parseIsometry3D(const tinyxml2::XMLElement* element);

/// Parse a position (3D vector) from an XML element.
void parsePosition(const tinyxml2::XMLElement* element, Eigen::Vector3d& pos);

/// Parse RPY orientation from an XML element.
void parseOrientationRPY(const tinyxml2::XMLElement* element, Eigen::Quaterniond& q);

/// Parse quaternion orientation from an XML element.
void parseQuaternion(const tinyxml2::XMLElement* element, Eigen::Quaterniond& q);

/// Parse a transform frame from an XML <Transform> element.
geometry::TransformNode parseTransformNode(const tinyxml2::XMLElement* transform_elem);

/// Parse a unit vector from an XML element, with optional tolerance.
Eigen::Vector3d parseUnitVector(const tinyxml2::XMLElement* element, double epsilon = 1e-8);

// -----------------------------------------------------------------------------
// FSM/Transition Parsing Utilities
// -----------------------------------------------------------------------------

/// Build FSM transition table from XML.
std::vector<std::vector<int>> buildTransitionTableFromXML(const tinyxml2::XMLElement* transitions_elem,
                                                          const components::FSMLabels& state_labels,
                                                          const components::FSMLabels& action_labels,
                                                          int invalid_value = -1);

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

geometry::Shape parseShape(const tinyxml2::XMLElement* elem);

// -----------------------------------------------------------------------------
// ECS Component Parsing Functions
// -----------------------------------------------------------------------------

void parseActionMapComponent(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseButtonComponent(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseContainerComponent(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseFitConstraintComponent(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseFluidDomainShapeComponent(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseFSMComponent(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseJointComponent(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseLinkComponent(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseOriginComponent(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseParallelGraspComponent(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseProductComponent(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseShapeComponent(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseTouchscreenComponent(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseTransformComponent(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);

}  // namespace sodf

#endif  // SODF_XML_PARSER_H_
