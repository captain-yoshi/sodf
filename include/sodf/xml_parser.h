#ifndef SODF_XML_PARSER_H_
#define SODF_XML_PARSER_H_

#include <tinyxml2.h>

#include <sodf/components/container.h>
#include <sodf/components/constraint.h>

namespace sodf {

// XMLParser class
class XMLParser
{
public:
  XMLParser(const std::string& filename);
  bool loadXML();

  // Functions to parse Containers and Transforms
  void parseContainers(components::ContainerCollection& containers);
  /* void parseTransforms(components::Transforms& transforms); */

private:
  tinyxml2::XMLDocument doc;
  std::string filename;

  // Helper functions
  void cloneContainer(components::Container& target, const std::string& sourceId,
                      const components::ContainerCollection& containers);
  void handleContainerOperation(const std::string& id, const std::string& operation,
                                tinyxml2::XMLElement* containerElement, components::ContainerCollection& containers);
  void updateContainer(components::Container& target, tinyxml2::XMLElement* containerElement);
  geometry::BaseVolumePtr parseShape(tinyxml2::XMLElement* shapeElement);
};

}  // namespace sodf

#endif  // SODF_XML_PARSER_H_
