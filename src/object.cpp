#include <sodf/object.h>

#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>

#include <sodf/elements/mesh.h>
namespace sodf {

void splitObjectElement(const std::string& id, std::string_view& object, std::string_view& element,
                        const std::string& delimiter)
{
  object = std::string_view(id).substr(0, id.find(delimiter));
  element = (id.size() == object.size()) ? "" : std::string_view(id).substr(object.size() + delimiter.size(), -1);
}

void splitObjectElement(const std::string& id, std::string& object, std::string& element, const std::string& delimiter)
{
  object = id.substr(0, id.find(delimiter));
  element = (id.size() == object.size()) ? "" : id.substr(object.size() + delimiter.size(), -1);
}

namespace {

void addToElementTree(KDL::Tree& tree, const geometry::Transform& tf, const KDL::Joint& joint)
{
  tree.addSegment(KDL::Segment(tf.frameId(), joint, tf.frame()), tf.refFrameId());
}

}  // namespace

Object::Object(const ObjectID& id, const geometry::Transform& tf) : id_(id), tf_(tf)

{
  // HOTFIX to circumvent bug in kdl
  joints_ = std::make_shared<KDL::JntArray>(1);

  element_tree_ = std::make_shared<KDL::Tree>("_root");
  element_tree_->addSegment(KDL::Segment("root", KDL::Joint(KDL::Joint::RotX), KDL::Frame()), "_root");
}

const ObjectID& Object::id() const
{
  return id_;
}
ObjectPtr Object::parent()
{
  return parent_;
}

void Object::addParent(ObjectPtr parent)
{
  parent_ = parent;
}

void Object::addChildren(ObjectPtr child)
{
  if (!child)
    throw std::runtime_error("add children invalid pointer");

  childrens_.insert(child);
}

const std::set<ObjectPtr>& Object::children() const
{
  return childrens_;
}

bool Object::addElement(const ElementID& id, Element::pointer&& element)
{
  if (!element)
    return false;

  auto node = elements_.find(id);
  if (node != elements_.end())
    return false;

  // Add element frame+joint as a segment to the tree
  const auto element_tf = element->getTransform();
  const auto element_joint = element->getJoint();

  if (element_tf)
  {
    if (element_joint)
    {
      addToElementTree(element_tree_, *element_tf, *element_joint);
      joints_->resize(joints_->rows() + 1);

      // map element id to joint index
      joint_index_map_.emplace(element_tf->frameId(), joints_->rows() - 1);
    }
    else
      // Fixed joint
      addToElementTree(element_tree_, *element_tf, KDL::Joint(KDL::Joint::None));
  }

  // add meshes
  if (element->isMesh())
    meshes_.push_back(id);

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
  fk_solver_.reset(new KDL::TreeFkSolverPos_recursive(*element_tree_));
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

bool Object::updateJointPosition(const std::string& element_id, double position)
{
  auto node = joint_index_map_.find(element_id);
  if (node != joint_index_map_.end())
    return false;

  if (node->second >= joints_->rows())
    return false;

  joints_->data[node->second] = position;

  return true;
}

const KDL::Tree& Object::elementTree() const
{
  return element_tree_;
}

const std::vector<ElementID>& Object::meshes() const
{
  return meshes_;
}

const geometry::Transform& Object::tf() const
{
  return tf_;
}

}  // namespace sodf
