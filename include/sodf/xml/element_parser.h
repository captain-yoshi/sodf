#ifndef SODF_XML_ELEMENT_PARSER_H_
#define SODF_XML_ELEMENT_PARSER_H_

#include <tinyxml2.h>

#include <sodf/geometry/shape.h>
#include <sodf/geometry/transform.h>
#include <sodf/components/shape.h>
#include <sodf/components/button.h>
#include <sodf/components/joint.h>
#include <sodf/components/grasp.h>

namespace sodf {
namespace xml {

void parsePosition(const tinyxml2::XMLElement* element, Eigen::Vector3d& pos);
void parseOrientationRPY(const tinyxml2::XMLElement* element, Eigen::Quaterniond& q);
void parseQuaternion(const tinyxml2::XMLElement* element, Eigen::Quaterniond& q);
Eigen::Vector3d parseUnitVector(const tinyxml2::XMLElement* element, double epsilon = 1e-09);
Eigen::Isometry3d parseIsometry3D(const tinyxml2::XMLElement* transform_elem);

geometry::Shape parseRectangleShape(const tinyxml2::XMLElement* elem);
geometry::Shape parseCircleShape(const tinyxml2::XMLElement* elem);
geometry::Shape parseTriangleShape(const tinyxml2::XMLElement* elem);
geometry::Shape parsePolygonShape(const tinyxml2::XMLElement* elem);
geometry::Shape parseBoxShape(const tinyxml2::XMLElement* elem);
geometry::Shape parseTriangularPrismShape(const tinyxml2::XMLElement* elem);
geometry::Shape parseCylinderShape(const tinyxml2::XMLElement* elem);
geometry::Shape parseSphereShape(const tinyxml2::XMLElement* elem);
geometry::Shape parseConeShape(const tinyxml2::XMLElement* elem);
geometry::Shape parseSphericalSegmentShape(const tinyxml2::XMLElement* elem);
geometry::Shape parsePlaneShape(const tinyxml2::XMLElement* elem);
geometry::Shape parseMeshShape(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem);
geometry::Shape parseLineShape(const tinyxml2::XMLElement* elem);
geometry::Shape parseShape(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* elem);

geometry::TransformNode parseTransformNode(const tinyxml2::XMLElement* transform_elem);
geometry::StackedShape parseStackedShape(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* stacked_elem);
components::Button parseButton(const tinyxml2::XMLElement* btn_elem);
components::VirtualButton parseVirtualButton(const tinyxml2::XMLElement* vb_elem);
components::Joint parseSingleDofJoint(const tinyxml2::XMLElement* joint_elem);
components::Joint parseMultiDofJoint(const tinyxml2::XMLElement* joint_elem);
components::Joint parseJoint(const tinyxml2::XMLElement* joint_elem);
components::ParallelGrasp parseParallelGrasp(const tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* grasp_elem);

}  // namespace xml
}  // namespace sodf

#endif  // SODF_XML_ELEMENT_PARSER_H_
