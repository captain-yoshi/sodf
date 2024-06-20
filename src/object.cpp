#include <sodf/object.h>

#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>

#include <sodf/elements/mesh.h>
#include <sodf/elements/joint.h>

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

void addToElementTree(KDL::Tree& tree, const std::string& segment_name, const KDL::Joint& joint,
                      const KDL::Frame& tip_frame, const std::string& ref_segment_name)
{
  if (!tree.addSegment(KDL::Segment(segment_name, joint, tip_frame), ref_segment_name))
    throw std::runtime_error("Reference segment '" + ref_segment_name + "' does not exists in the element tree.");
}

}  // namespace

Object::Object(const ObjectID& id, const geometry::Transform& tf) : id_(id), tf_(tf)

{
  element_tree_ = std::make_shared<KDL::Tree>("_root");

  // HOTFIX to circumvent bug in kdl, when there are no joints
  // Joint MUST be different then Joint::None
  joints_ = std::make_shared<KDL::JntArray>(1);
  addToElementTree(*element_tree_, "root", KDL::Joint(KDL::Joint::RotX), KDL::Frame(), "_root");
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
    if (!element_joint || element_joint->getType() == element_joint->None)
    {
      // Fixed joint
      addToElementTree(*element_tree_, element_tf->frameId(), KDL::Joint(KDL::Joint::None), element_tf->frame(),
                       element_tf->refFrameId());
    }
    else
    {
      // Make joint located at the tip instead of the root frame.
      std::string tip_name = "_" + id;
      auto node = elements_.find(tip_name);
      if (node != elements_.end())
        return false;

      addToElementTree(*element_tree_, tip_name, KDL::Joint(KDL::Joint::None), element_tf->frame(),
                       element_tf->refFrameId());

      elements_.emplace(tip_name, std::move(nullptr));

      addToElementTree(*element_tree_, id, *element_joint, KDL::Frame(), tip_name);
      joints_->resize(joints_->data.size() + 1);

      // map element id to joint index
      joint_index_map_.emplace(element_tf->frameId(), joints_->data.size() - 1);
    }
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
  if (node == joint_index_map_.end())
    return false;

  if (node->second >= joints_->data.size())
    return false;

  joints_->data[node->second] = position;

  return true;
}

bool Object::updateJointPosition(const std::string& element_id, const std::string& position)
{
  auto joint_node = joint_index_map_.find(element_id);
  if (joint_node == joint_index_map_.end())
    return false;

  auto joint = getElement<elements::PassiveJoint>(element_id);
  if (!joint)
    return false;

  const auto& j_map = joint->jointPositionMap();

  auto pair = j_map.find(position);
  if (pair == j_map.end())
    return false;

  return updateJointPosition(element_id, pair->second);
}

const std::vector<ElementID>& Object::meshes() const
{
  return meshes_;
}

geometry_msgs::InertiaStamped Object::inertia() const
{
  geometry_msgs::InertiaStamped inertia;
  inertia.header.frame_id = id();
  inertia.inertia.m = 0.0;

  KDL::Vector com;

  for (const auto& mesh_name : meshes())
  {
    auto element = getElement<elements::Mesh>(mesh_name);
    if (!element)
      throw std::runtime_error("Cannot find Mesh element " + mesh_name + " within the object " + id());

    const auto& mesh_inertia = element->inertia();

    auto tf_root_mesh = displayInRoot(mesh_name);
    KDL::Frame tf_mesh_com;
    tf_mesh_com.p = KDL::Vector(mesh_inertia.com.x, mesh_inertia.com.y, mesh_inertia.com.z);

    auto tf_root_com = tf_root_mesh * tf_mesh_com;

    // compute total mass
    inertia.inertia.m += mesh_inertia.m;

    // compute CoM numerator mi*di + ...
    com += (mesh_inertia.m * KDL::Vector(tf_root_com.p.x(), tf_root_com.p.y(), tf_root_com.p.z()));
  }

  if (inertia.inertia.m == 0.0)
    throw std::runtime_error("Object " + id() + " mesh/es has no mass, cannot compute inertia");

  // CoM num / total mass
  com = com / inertia.inertia.m;

  inertia.inertia.com.x = com.x();
  inertia.inertia.com.y = com.y();
  inertia.inertia.com.z = com.z();

  return inertia;
}

const geometry::Transform& Object::tf() const
{
  return tf_;
}

}  // namespace sodf
