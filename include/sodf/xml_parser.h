#ifndef SODF_XML_PARSER_H_
#define SODF_XML_PARSER_H_

#include <tinyxml2.h>

#include <sodf/ecs.h>
#include <sodf/components/transform.h>
#include <sodf/components/finite_state_machine.h>

namespace sodf {

// XMLParser class
class XMLParser
{
public:
  XMLParser();
  ~XMLParser();
  bool loadXML(const std::string& filename);

  // Functions to parse Containers and Transforms
  void parseComponents(ginseng::database& db);
  /* void parseTransforms(components::Transforms& transforms); */

private:
  std::unique_ptr<tinyxml2::XMLDocument> doc;
  // Helper functions
  // void cloneContainer(components::Container& target, const std::string& sourceId,
  //                     const components::ContainerCollection& containers);
  // void handleContainerOperation(const std::string& id, const std::string& operation,
  //                               tinyxml2::XMLElement* containerElement, components::ContainerCollection& containers);
  // void updateContainer(components::Container& target, tinyxml2::XMLElement* containerElement);
  // geometry::BaseVolumePtr parseShape(tinyxml2::XMLElement* shapeElement);
};

void parsePosition(const tinyxml2::XMLElement* element, Eigen::Vector3d& pos);
void parseOrientationRPY(const tinyxml2::XMLElement* element, Eigen::Quaterniond& q);
void parseQuaternion(const tinyxml2::XMLElement* element, Eigen::Quaterniond& q);
void parseTransform(const tinyxml2::XMLElement* transform_elem, components::TransformFrame& frame);
void parseUnitVector(const tinyxml2::XMLElement* element, Eigen::Vector3d& vec, double epsilon = 1e-4);
std::vector<std::vector<int>> buildTransitionTableFromXML(const tinyxml2::XMLElement* transitions_elem,
                                                          const components::FSMLabels& state_labels,
                                                          const components::FSMLabels& action_labels,
                                                          int invalid_value = -1);

/// components
void parseObjectElement(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseOriginElement(const tinyxml2::XMLElement* obj_elem, components::TransformComponent& transform,
                        ginseng::database& db, EntityID eid);
void parseTransformElement(const tinyxml2::XMLElement* obj_elem, components::TransformComponent&& transform,
                           ginseng::database& db, EntityID eid);
void parseLinkElement(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseJointElement(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseFitConstraintElement(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseTouchscreenElement(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseFSMElement(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);
void parseActionMapElement(const tinyxml2::XMLElement* obj_elem, ginseng::database& db, EntityID eid);

}  // namespace sodf

#endif  // SODF_XML_PARSER_H_
