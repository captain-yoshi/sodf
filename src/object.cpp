#include <sodf/object.h>

#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>

namespace sodf {

void splitObjectElement(const std::string& id, std::string_view& object, std::string_view& element,
                        const std::string& delimiter)
{
  object = std::string_view(id).substr(0, id.find(delimiter));
  element = (id.size() == object.size()) ? "" : std::string_view(id).substr(object.size() + delimiter.size(), -1);
}

Object::Object(const geometry::Transform& tf, const geometry::Mesh& mesh) : tf_(tf), mesh_(mesh)
{
  // HOTFIX to circumvent bug in kdl
  joints_ = std::make_shared<KDL::JntArray>(1);

  element_tree_ = KDL::Tree("_root");
  element_tree_.addSegment(KDL::Segment("root", KDL::Joint(KDL::Joint::RotX), KDL::Frame()), "_root");
}

bool Object::addElement(const std::string& id, Element::pointer&& element)
{
  if (!element)
    return false;

  auto node = elements_.find(id);
  if (node != elements_.end())
    return false;

  // populate tree
  if (!element->addFramesToTree(element_tree_))
    return false;

  // add to map
  elements_.emplace(id, std::move(element));

  return true;
}

bool Object::removeElement(const std::string& id)
{
  auto node = elements_.extract(id);
  if (node.empty())
    return false;

  auto cls = std::move(node.mapped());
  return true;
}

void Object::init()
{
  fk_solver_.reset(new KDL::TreeFkSolverPos_recursive(element_tree_));
}

KDL::Frame Object::displayInRoot(const std::string& from) const
{
  KDL::Frame frame;

  // const auto& it = element_tree_.getSegment(from);
  // if (it == element_tree_.getSegments().end())
  //   throw std::runtime_error("forward kinematic solver internal error:" + from);

  // return it->second.segment.getFrameToTip();

  auto rc = fk_solver_->JntToCart(*joints_, frame, from);

  if (rc < 0)
    throw std::runtime_error("forward kinematic solver internal error:" + from);

  return frame;
}

Eigen::Isometry3d Object::displayInRootEigen(const std::string& from) const
{
  Eigen::Isometry3d e;

  tf::transformKDLToEigen(displayInRoot(from), e);
  return e;
}

geometry_msgs::Pose Object::displayInRootPoseMsg(const std::string& from) const
{
  geometry_msgs::Pose pose;

  tf::poseKDLToMsg(displayInRoot(from), pose);

  return pose;
}

const KDL::Tree& Object::elementTree() const
{
  return element_tree_;
}

const geometry::Mesh& Object::mesh() const
{
  return mesh_;
}

const geometry::Transform& Object::tf() const
{
  return tf_;
}

}  // namespace sodf
