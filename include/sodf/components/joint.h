#ifndef SODF_COMPONENTS_JOINT_H_
#define SODF_COMPONENTS_JOINT_H_

#include <Eigen/Geometry>
#include <sodf/ecs.h>

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
  FlatMap<std::string, Joint> map;
};

inline JointType jointTypeFromString(const std::string& s)
{
  if (s == "FIXED")
    return JointType::FIXED;
  if (s == "REVOLUTE")
    return JointType::REVOLUTE;
  if (s == "PRISMATIC")
    return JointType::PRISMATIC;
  if (s == "SPHERICAL")
    return JointType::SPHERICAL;
  if (s == "PLANAR")
    return JointType::PLANAR;
  if (s == "FLOATING")
    return JointType::FLOATING;
  throw std::runtime_error("Unknown JointType: " + s);
}

inline JointActuation jointActuationFromString(const std::string& s)
{
  if (s == "FIXED")
    return JointActuation::FIXED;
  if (s == "ACTUATED")
    return JointActuation::ACTUATED;
  if (s == "PASSIVE")
    return JointActuation::PASSIVE;
  if (s == "VIRTUAL")
    return JointActuation::VIRTUAL;
  throw std::runtime_error("Unknown JointActuation: " + s);
}

inline std::string jointTypeToString(JointType type)
{
  switch (type)
  {
    case JointType::FIXED:
      return "FIXED";
    case JointType::REVOLUTE:
      return "REVOLUTE";
    case JointType::PRISMATIC:
      return "PRISMATIC";
    case JointType::SPHERICAL:
      return "SPHERICAL";
    case JointType::PLANAR:
      return "PLANAR";
    case JointType::FLOATING:
      return "FLOATING";
    default:
      return "UNKNOWN";
  }
}

inline std::string jointActuationToString(JointActuation actuation)
{
  switch (actuation)
  {
    case JointActuation::FIXED:
      return "FIXED";
    case JointActuation::ACTUATED:
      return "ACTUATED";
    case JointActuation::PASSIVE:
      return "PASSIVE";
    case JointActuation::VIRTUAL:
      return "VIRTUAL";
    default:
      return "UNKNOWN";
  }
}

}  // namespace components
}  // namespace sodf

#endif
