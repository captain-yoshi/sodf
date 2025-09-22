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

geometry::Shape parseShape(const tinyxml2::XMLElement* elem);
geometry::TransformNode parseTransformNode(const tinyxml2::XMLElement* transform_elem);
components::StackedShape parseStackedShape(const tinyxml2::XMLElement* stacked_elem);
components::Button parseButton(const tinyxml2::XMLElement* btn_elem);
components::VirtualButton parseVirtualButton(const tinyxml2::XMLElement* vb_elem);
components::Joint parseSingleDofJoint(const tinyxml2::XMLElement* joint_elem);
components::Joint parseMultiDofJoint(const tinyxml2::XMLElement* joint_elem);
components::Joint parseJoint(const tinyxml2::XMLElement* joint_elem);
components::ParallelGrasp parseParallelGrasp(const tinyxml2::XMLElement* grasp_elem);

}  // namespace xml
}  // namespace sodf

#endif  // SODF_XML_ELEMENT_PARSER_H_
