#ifndef SODF_COMPONENTS_JOINT_H_
#define SODF_COMPONENTS_JOINT_H_

#include <Eigen/Geometry>
#include <sodf/components/data_type.h>

namespace sodf {
namespace components {

enum class JointType
{
  FIXED,      // No motion
  REVOLUTE,   // 1 DOF rotation
  PRISMATIC,  // 1 DOF translation
  SPHERICAL,  // 3 DOF rotation
  PLANAR,     // 2 DOF translation + 1 DOF rotation
  FLOATING    // 6 DOF (translation + rotation)
};

enum class JointActuation
{
  FIXED,     // when JointType = FIXED
  ACTUATED,  // move by commanded actuator (motor, piston, hydraulic, pneumatic, etc.)
  PASSIVE,   // compliant joint moved by external forces e.g. gravity or restoring force (spring + damper)
  VIRTUAL,   // logical/virtual joints (e.g., liquid level).
};

struct JointLimit
{
  Eigen::VectorXd min_position;      // [rad] or [m], per DOF
  Eigen::VectorXd max_position;      //
  Eigen::VectorXd min_velocity;      // [rad/s] or [m/s]
  Eigen::VectorXd max_velocity;      //
  Eigen::VectorXd min_acceleration;  // [rad/s^2] or [m/s^2]
  Eigen::VectorXd max_acceleration;  //
  Eigen::VectorXd min_jerk;          // [rad/s^3] or [m/s^3]
  Eigen::VectorXd max_jerk;          //
  Eigen::VectorXd min_effort;        // [Nm] or [N], per DOF
  Eigen::VectorXd max_effort;        //

  void resize(int dof)
  {
    min_position.setZero(dof);
    max_position.setZero(dof);
    min_velocity.setZero(dof);
    max_velocity.setZero(dof);
    min_acceleration.setZero(dof);
    max_acceleration.setZero(dof);
    min_jerk.setZero(dof);
    max_jerk.setZero(dof);
    min_effort.setZero(dof);
    max_effort.setZero(dof);
  }
};

struct JointDynamics
{
  Eigen::VectorXd stiffness;                         // [Nm/rad] or [N/m], per DOF
  Eigen::VectorXd damping;                           // [Nm*s/rad] or [Ns/m], per DOF
  std::vector<std::optional<double>> rest_position;  // [rad] or [m], per DOF (nullopt = 0)
  Eigen::VectorXd inertia;                           // [kg*m^2] or [kg], per DOF

  void resize(int dof)
  {
    stiffness.setZero(dof);
    damping.setZero(dof);
    inertia.setZero(dof);
    rest_position.resize(dof, std::nullopt);
  }

  // NOTE:
  // For PASSIVE joints (e.g., with elastic or compliant behavior), stiffness and damping
  // model the restoring force/torque: F = -stiffness*(q - rest_position) - damping*q_dot.
  // If rest_position is unset (nullopt), zero is assumed as the equilibrium position.
  // For ACTUATED joints, these fields may be left at zero unless additional compliance
  // is desired in the controller or simulation.
};

struct Joint
{
  JointType type = JointType::FIXED;
  JointActuation actuation = JointActuation::FIXED;

  // Each column is a 3D axis vector for one DOF (size 3 x dof)
  // [trans_x, trans_y, trans_z, rot_x, rot_y, rot_z]
  Eigen::Matrix<double, 3, Eigen::Dynamic> axes;

  // Per-DOF states and commands
  Eigen::VectorXd position;  // [rad] or [m], per DOF
  Eigen::VectorXd velocity;  // [rad/s] or [m/s], per DOF
  Eigen::VectorXd effort;    // [Nm] or [N], per DOF

  JointLimit limit;
  JointDynamics dynamics;

  // Get DOF count for this joint type
  int dof() const
  {
    switch (type)
    {
      case JointType::REVOLUTE:
        return 1;
      case JointType::PRISMATIC:
        return 1;
      case JointType::PLANAR:
        return 3;
      case JointType::SPHERICAL:
        return 3;
      case JointType::FLOATING:
        return 6;
      default:
        return 0;
    }
  }

  // Resize all vectors/matrices for the right DOF count
  void resize(int dof)
  {
    axes.setZero(3, dof);
    position.setZero(dof);
    velocity.setZero(dof);
    effort.setZero(dof);
    limit.resize(dof);
    dynamics.resize(dof);
  }

  // Convenience: initialize to match current joint type
  void initialize_for_type()
  {
    resize(dof());
  }
};

struct JointComponent
{
  ElementMap<std::string, Joint> elements;
};

inline JointType jointTypeFromString(const std::string& s)
{
  if (s == "Fixed")
    return JointType::FIXED;
  if (s == "Revolute")
    return JointType::REVOLUTE;
  if (s == "Prismatic")
    return JointType::PRISMATIC;
  if (s == "Spherical")
    return JointType::SPHERICAL;
  if (s == "Planar")
    return JointType::PLANAR;
  if (s == "Floating")
    return JointType::FLOATING;
  throw std::runtime_error("Unknown JointType: " + s);
}

inline JointActuation jointActuationFromString(const std::string& s)
{
  if (s == "Fixed")
    return JointActuation::FIXED;
  if (s == "Actuated")
    return JointActuation::ACTUATED;
  if (s == "Passive")
    return JointActuation::PASSIVE;
  if (s == "Virtual")
    return JointActuation::VIRTUAL;
  throw std::runtime_error("Unknown JointActuation: " + s);
}

inline std::string jointTypeToString(JointType type)
{
  switch (type)
  {
    case JointType::FIXED:
      return "Fixed";
    case JointType::REVOLUTE:
      return "Revolute";
    case JointType::PRISMATIC:
      return "Prismatic";
    case JointType::SPHERICAL:
      return "Spherical";
    case JointType::PLANAR:
      return "Planar";
    case JointType::FLOATING:
      return "Floating";
    default:
      return "Unknown";
  }
}

inline std::string jointActuationToString(JointActuation actuation)
{
  switch (actuation)
  {
    case JointActuation::FIXED:
      return "Fixed";
    case JointActuation::ACTUATED:
      return "Actuated";
    case JointActuation::PASSIVE:
      return "Passive";
    case JointActuation::VIRTUAL:
      return "Virtual";
    default:
      return "Unkwown";
  }
}

inline std::ostream& operator<<(std::ostream& os, const Joint& joint)
{
  const int dof = static_cast<int>(joint.position.size());

  os << "Joint{ type=" << jointTypeToString(joint.type) << ", actuation=" << jointActuationToString(joint.actuation)
     << ", dof=" << dof << " }\n";

  // Helper for Eigen vectors
  auto printVec = [&](const char* label, const Eigen::VectorXd& v, const char* indent = "  ") {
    if (v.size() == 0)
      return;
    os << indent << label << " = [";
    for (int i = 0; i < v.size(); ++i)
    {
      if (i > 0)
        os << ", ";
      os << v[i];
    }
    os << "]\n";
  };

  // Axes
  if (joint.axes.cols() > 0)
  {
    os << "  axes:\n";
    for (int i = 0; i < joint.axes.cols(); ++i)
    {
      os << "    axis[" << i << "] = [" << joint.axes(0, i) << ", " << joint.axes(1, i) << ", " << joint.axes(2, i)
         << "]\n";
    }
  }

  // State
  printVec("position", joint.position);
  printVec("velocity", joint.velocity);
  printVec("effort", joint.effort);

  // Limits
  os << "  limits:\n";
  printVec("min_position", joint.limit.min_position, "    ");
  printVec("max_position", joint.limit.max_position, "    ");
  printVec("min_velocity", joint.limit.min_velocity, "    ");
  printVec("max_velocity", joint.limit.max_velocity, "    ");
  printVec("min_acceleration", joint.limit.min_acceleration, "    ");
  printVec("max_acceleration", joint.limit.max_acceleration, "    ");
  printVec("min_jerk", joint.limit.min_jerk, "    ");
  printVec("max_jerk", joint.limit.max_jerk, "    ");
  printVec("min_effort", joint.limit.min_effort, "    ");
  printVec("max_effort", joint.limit.max_effort, "    ");

  // Dynamics
  os << "  dynamics:\n";
  printVec("stiffness", joint.dynamics.stiffness, "    ");
  printVec("damping", joint.dynamics.damping, "    ");
  printVec("inertia", joint.dynamics.inertia, "    ");

  if (!joint.dynamics.rest_position.empty())
  {
    os << "    rest_position = [";
    for (std::size_t i = 0; i < joint.dynamics.rest_position.size(); ++i)
    {
      if (i > 0)
        os << ", ";
      if (joint.dynamics.rest_position[i])
        os << *joint.dynamics.rest_position[i];
      else
        os << "null";
    }
    os << "]\n";
  }

  return os;
}

}  // namespace components
}  // namespace sodf

#endif
