#include <iostream>

#include <sodf/xml_parser.h>
#include <sodf/geometry/volume.h>

using namespace tinyxml2;
using namespace sodf::components;
using namespace sodf::geometry;

namespace sodf {

// Constructor to initialize filename
XMLParser::XMLParser(const std::string& filename) : filename(filename)
{
}

// Load XML file
bool XMLParser::loadXML()
{
  XMLError result = doc.LoadFile(filename.c_str());
  if (result != XML_SUCCESS)
  {
    std::cerr << "Error loading XML file: " << filename << std::endl;
    return false;
  }
  return true;
}

// Clone a container by copying from an existing one
void XMLParser::cloneContainer(Container& target, const std::string& sourceId,
                               const ContainerCollection& containerCollection)
{
  for (const auto& containerPair : containerCollection.container_map)
  {
    if (containerPair.first == sourceId)
    {
      target = containerPair.second;
      target.volume = containerPair.second.volume;  // Retain original volume
      return;
    }
  }
  std::cerr << "Error: Clone source '" << sourceId << "' not found." << std::endl;
}

// Parse a Shape element and return a BaseVolumePtr object
BaseVolumePtr XMLParser::parseShape(XMLElement* shapeElement)
{
  std::string shapeName = shapeElement->Attribute("name");

  if (shapeName == "SphericalCap")
  {
    double cap_radius, height;
    shapeElement->QueryDoubleAttribute("cap_radius", &cap_radius);
    shapeElement->QueryDoubleAttribute("height", &height);
    return std::make_shared<SphericalCapVolume>(cap_radius, height);
  }
  else if (shapeName == "TruncatedCone")
  {
    double base_radius, top_radius, height;
    shapeElement->QueryDoubleAttribute("base_radius", &base_radius);
    shapeElement->QueryDoubleAttribute("top_radius", &top_radius);
    shapeElement->QueryDoubleAttribute("height", &height);
    return std::make_shared<TruncatedConeVolume>(base_radius, top_radius, height);
  }
  else if (shapeName == "Cylinder")
  {
    double radius, height;
    shapeElement->QueryDoubleAttribute("radius", &radius);
    shapeElement->QueryDoubleAttribute("height", &height);
    return std::make_shared<CylinderVolume>(radius, height);
  }

  return nullptr;
}

// Handle container operations: remove, overwrite, or update
void XMLParser::handleContainerOperation(const std::string& id, const std::string& operation,
                                         XMLElement* containerElement, ContainerCollection& containerCollection)
{
  if (operation == "remove")
  {
    auto it = std::remove_if(containerCollection.container_map.begin(), containerCollection.container_map.end(),
                             [&id](const std::pair<std::string, Container>& pair) { return pair.first == id; });
    if (it != containerCollection.container_map.end())
    {
      containerCollection.container_map.erase(it, containerCollection.container_map.end());
      std::cout << "Container '" << id << "' removed successfully.\n";
    }
    else
    {
      std::cerr << "Error: Container '" << id << "' not found for removal.\n";
    }
  }
  else if (operation == "overwrite")
  {
    Container container;
    containerElement->QueryDoubleAttribute("volume", &container.volume);

    for (XMLElement* shapeElement = containerElement->FirstChildElement("Shape"); shapeElement != nullptr;
         shapeElement = shapeElement->NextSiblingElement("Shape"))
    {
      container.shape.push_back(parseShape(shapeElement));
    }

    containerCollection.container_map.push_back(std::make_pair(id, container));
  }
  else if (operation == "update")
  {
    for (auto& containerPair : containerCollection.container_map)
    {
      if (containerPair.first == id)
      {
        updateContainer(containerPair.second, containerElement);
        return;
      }
    }
    std::cerr << "Error: Cannot update. Container '" << id << "' not found.\n";
  }
  else
  {
    std::cerr << "Warning: Unknown operation '" << operation << "' for container '" << id << "'.\n";
  }
}

// Update container attributes or add/overwrite shapes
void XMLParser::updateContainer(Container& target, XMLElement* containerElement)
{
  containerElement->QueryDoubleAttribute("volume", &target.volume);

  for (XMLElement* shapeElement = containerElement->FirstChildElement("Shape"); shapeElement != nullptr;
       shapeElement = shapeElement->NextSiblingElement("Shape"))
  {
    BaseVolumePtr newShape = parseShape(shapeElement);
    bool shapeExists = false;

    // Check if shape already exists, update if found
    for (auto& shape : target.shape)
    {
      if (shape->getHeight(1.0) == newShape->getHeight(1.0))
      {                    // Compare using height as identifier
        shape = newShape;  // Overwrite existing shape
        shapeExists = true;
        break;
      }
    }

    // Add new shape if not found
    if (!shapeExists)
    {
      target.shape.push_back(newShape);
    }
  }
}

// Parse <Containers> section
void XMLParser::parseContainers(ContainerCollection& containerCollection)
{
  XMLElement* root = doc.RootElement();
  XMLElement* object = root->FirstChildElement("Object");
  XMLElement* containersElement = object->FirstChildElement("Containers");

  for (XMLElement* containerElement = containersElement->FirstChildElement("Container"); containerElement != nullptr;
       containerElement = containerElement->NextSiblingElement("Container"))
  {
    std::string id = containerElement->Attribute("id");

    // Check for operation attribute
    const char* operationAttr = containerElement->Attribute("operation");
    if (operationAttr)
    {
      std::string operation = operationAttr;
      handleContainerOperation(id, operation, containerElement, containerCollection);
      continue;  // Skip further parsing if operation is handled
    }

    // Handle cloned container
    Container container;
    const char* clone = containerElement->Attribute("clone");
    if (clone)
    {
      cloneContainer(container, clone, containerCollection);
    }
    else
    {
      containerElement->QueryDoubleAttribute("volume", &container.volume);

      // Parse all <Shape> elements inside the container
      for (XMLElement* shapeElement = containerElement->FirstChildElement("Shape"); shapeElement != nullptr;
           shapeElement = shapeElement->NextSiblingElement("Shape"))
      {
        container.shape.push_back(parseShape(shapeElement));
      }
    }

    containerCollection.container_map.push_back(std::make_pair(id, container));
  }
}

// // Parse <Transforms> section
// void XMLParser::parseTransforms(Transforms& transforms)
// {
//   XMLElement* root = doc.RootElement();
//   XMLElement* object = root->FirstChildElement("Object");
//   XMLElement* transformsElement = object->FirstChildElement("Transforms");

//   for (XMLElement* transformElement = transformsElement->FirstChildElement("Transform"); transformElement != nullptr;
//        transformElement = transformElement->NextSiblingElement("Transform"))
//   {
//     Transform transform;
//     transform.name = transformElement->Attribute("name");
//     transform.parent = transformElement->Attribute("parent");

//     // Parse Frame
//     XMLElement* frameElement = transformElement->FirstChildElement("Frame");
//     if (frameElement)
//     {
//       frameElement->QueryDoubleAttribute("x", &transform.transform.translation().x());
//       frameElement->QueryDoubleAttribute("y", &transform.transform.translation().y());
//       frameElement->QueryDoubleAttribute("z", &transform.transform.translation().z());
//       frameElement->QueryDoubleAttribute("qx", &transform.transform.rotation().x());
//       frameElement->QueryDoubleAttribute("qy", &transform.transform.rotation().y());
//       frameElement->QueryDoubleAttribute("qz", &transform.transform.rotation().z());
//       frameElement->QueryDoubleAttribute("qw", &transform.transform.rotation().w());
//     }
//     transforms.transforms.push_back(transform.transform);
//   }
// }

}  // namespace sodf
