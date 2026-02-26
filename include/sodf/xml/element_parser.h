#ifndef SODF_XML_ELEMENT_PARSER_H_
#define SODF_XML_ELEMENT_PARSER_H_

#include <tinyxml2.h>

#include <sodf/geometry/shape.h>
#include <sodf/geometry/transform.h>
#include <sodf/components/shape.h>
#include <sodf/components/button.h>
#include <sodf/components/joint.h>
#include <sodf/components/grasp.h>

#include <sodf/xml/expression_parser.h>
namespace sodf {
namespace xml {

void parsePosition(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx, Eigen::Vector3d& pos);

void parseOrientationRPY(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx, Eigen::Quaterniond& q);

void parseQuaternion(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx, Eigen::Quaterniond& q);

Eigen::Vector3d parseUnitVector(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx,
                                double epsilon = 1e-09);

Eigen::Isometry3d parseIsometry3D(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

// ==========================
// Shape parsing
// ==========================

geometry::Shape parseRectangleShape(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

geometry::Shape parseCircleShape(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

geometry::Shape parseTriangleShape(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

geometry::Shape parsePolygonShape(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

geometry::Shape parseBoxShape(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

geometry::Shape parseTriangularPrismShape(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

geometry::Shape parseCylinderShape(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

geometry::Shape parseSphereShape(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

geometry::Shape parseConeShape(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

geometry::Shape parseSphericalSegmentShape(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

geometry::Shape parsePlaneShape(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

geometry::Shape parseMeshShape(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

geometry::Shape parseLineShape(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

geometry::Shape parseShape(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

// ==========================
// Transform & stacked shapes
// ==========================

geometry::TransformNode parseTransformNode(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

geometry::StackedShape parseStackedShape(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

// ==========================
// Components
// ==========================

components::Button parseButton(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

components::VirtualButton parseVirtualButton(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

components::Joint parseSingleDofJoint(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

components::Joint parseMultiDofJoint(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

components::Joint parseJoint(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

components::ParallelGrasp parseParallelGrasp(const tinyxml2::XMLElement* xml_elem, const XMLParseContext& ctx);

}  // namespace xml
}  // namespace sodf

#endif  // SODF_XML_ELEMENT_PARSER_H_
