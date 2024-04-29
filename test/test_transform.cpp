#include <sodf/geometry/transform.h>
#include <eigen_conversions/eigen_msg.h>

#include <eigen_conversions/eigen_kdl.h>

#include <gtest/gtest.h>

using namespace sodf::geometry;

constexpr double POSITION_EPSILON = 1e-06;
constexpr double ROTATION_EPSILON = 1e-06;

TEST(Transform, Constructor)
{
  KDL::Frame kdl_frame;
  kdl_frame.p.x(-0.3);
  kdl_frame.p.y(1.5);
  kdl_frame.p.z(-6);
  kdl_frame.M.RPY(M_PI_4, M_PI_2, M_PI / 3.0);

  double quat_x, quat_y, quat_z, quat_w;
  kdl_frame.M.GetQuaternion(quat_x, quat_y, quat_z, quat_w);

  Eigen::Isometry3d eig_frame;
  tf::transformKDLToEigen(kdl_frame, eig_frame);

  // Constructor KDL::Frame
  {
    Transform tf(kdl_frame, "");

    EXPECT_NEAR(kdl_frame.p.x(), tf.frame().p.x(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.y(), tf.frame().p.y(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.z(), tf.frame().p.z(), POSITION_EPSILON);

    double qx, qy, qz, qw;
    tf.frame().M.GetQuaternion(qx, qy, qz, qw);

    EXPECT_NEAR(quat_x, qx, ROTATION_EPSILON);
    EXPECT_NEAR(quat_y, qy, ROTATION_EPSILON);
    EXPECT_NEAR(quat_z, qz, ROTATION_EPSILON);
    EXPECT_NEAR(quat_w, qw, ROTATION_EPSILON);
  }

  // Constructor KDL::Vector, KDL::Rotation
  {
    Transform tf(kdl_frame.p, kdl_frame.M, "");

    EXPECT_NEAR(kdl_frame.p.x(), tf.frame().p.x(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.y(), tf.frame().p.y(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.z(), tf.frame().p.z(), POSITION_EPSILON);

    double qx, qy, qz, qw;
    tf.frame().M.GetQuaternion(qx, qy, qz, qw);

    EXPECT_NEAR(quat_x, qx, ROTATION_EPSILON);
    EXPECT_NEAR(quat_y, qy, ROTATION_EPSILON);
    EXPECT_NEAR(quat_z, qz, ROTATION_EPSILON);
    EXPECT_NEAR(quat_w, qw, ROTATION_EPSILON);
  }

  // Constructor Eigen::Isometry3d
  {
    Transform tf(eig_frame, "");

    EXPECT_NEAR(kdl_frame.p.x(), tf.frame().p.x(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.y(), tf.frame().p.y(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.z(), tf.frame().p.z(), POSITION_EPSILON);

    double qx, qy, qz, qw;
    tf.frame().M.GetQuaternion(qx, qy, qz, qw);

    EXPECT_NEAR(quat_x, qx, ROTATION_EPSILON);
    EXPECT_NEAR(quat_y, qy, ROTATION_EPSILON);
    EXPECT_NEAR(quat_z, qz, ROTATION_EPSILON);
    EXPECT_NEAR(quat_w, qw, ROTATION_EPSILON);
  }

  // Constructor Eigen::Vector3d, Eigen::Quaterniond
  {
    Eigen::Vector3d vector = eig_frame.translation();
    Eigen::Quaterniond quaternion(eig_frame.linear());

    Transform tf(vector, quaternion, "");

    EXPECT_NEAR(kdl_frame.p.x(), tf.frame().p.x(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.y(), tf.frame().p.y(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.z(), tf.frame().p.z(), POSITION_EPSILON);

    double qx, qy, qz, qw;
    tf.frame().M.GetQuaternion(qx, qy, qz, qw);

    EXPECT_NEAR(quat_x, qx, ROTATION_EPSILON);
    EXPECT_NEAR(quat_y, qy, ROTATION_EPSILON);
    EXPECT_NEAR(quat_z, qz, ROTATION_EPSILON);
    EXPECT_NEAR(quat_w, qw, ROTATION_EPSILON);
  }

  // Constructor double px .. double qw
  {
    Transform tf(kdl_frame.p.x(), kdl_frame.p.y(), kdl_frame.p.z(), quat_x, quat_y, quat_z, quat_w, "");

    EXPECT_NEAR(kdl_frame.p.x(), tf.frame().p.x(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.y(), tf.frame().p.y(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.z(), tf.frame().p.z(), POSITION_EPSILON);

    double qx, qy, qz, qw;
    tf.frame().M.GetQuaternion(qx, qy, qz, qw);

    EXPECT_NEAR(quat_x, qx, ROTATION_EPSILON);
    EXPECT_NEAR(quat_y, qy, ROTATION_EPSILON);
    EXPECT_NEAR(quat_z, qz, ROTATION_EPSILON);
    EXPECT_NEAR(quat_w, qw, ROTATION_EPSILON);
  }

  // Constructor geometry_msgs::Transform
  {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.transform.translation.x = kdl_frame.p.x();
    tf_msg.transform.translation.y = kdl_frame.p.y();
    tf_msg.transform.translation.z = kdl_frame.p.z();
    tf_msg.transform.rotation.x = quat_x;
    tf_msg.transform.rotation.y = quat_y;
    tf_msg.transform.rotation.z = quat_z;
    tf_msg.transform.rotation.w = quat_w;

    Transform tf(tf_msg, "");

    EXPECT_NEAR(kdl_frame.p.x(), tf.frame().p.x(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.y(), tf.frame().p.y(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.z(), tf.frame().p.z(), POSITION_EPSILON);

    double qx, qy, qz, qw;
    tf.frame().M.GetQuaternion(qx, qy, qz, qw);

    EXPECT_NEAR(quat_x, qx, ROTATION_EPSILON);
    EXPECT_NEAR(quat_y, qy, ROTATION_EPSILON);
    EXPECT_NEAR(quat_z, qz, ROTATION_EPSILON);
    EXPECT_NEAR(quat_w, qw, ROTATION_EPSILON);
  }

  // Constructor geometry_msgs::Vector3, geometry_msgs::Quaternion
  {
    geometry_msgs::Transform tf_msg;
    tf_msg.translation.x = kdl_frame.p.x();
    tf_msg.translation.y = kdl_frame.p.y();
    tf_msg.translation.z = kdl_frame.p.z();
    tf_msg.rotation.x = quat_x;
    tf_msg.rotation.y = quat_y;
    tf_msg.rotation.z = quat_z;
    tf_msg.rotation.w = quat_w;

    Transform tf(tf_msg.translation, tf_msg.rotation, "");

    EXPECT_NEAR(kdl_frame.p.x(), tf.frame().p.x(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.y(), tf.frame().p.y(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.z(), tf.frame().p.z(), POSITION_EPSILON);

    double qx, qy, qz, qw;
    tf.frame().M.GetQuaternion(qx, qy, qz, qw);

    EXPECT_NEAR(quat_x, qx, ROTATION_EPSILON);
    EXPECT_NEAR(quat_y, qy, ROTATION_EPSILON);
    EXPECT_NEAR(quat_z, qz, ROTATION_EPSILON);
    EXPECT_NEAR(quat_w, qw, ROTATION_EPSILON);
  }

  // Constructor geometry_msgs::Pose
  {
    geometry_msgs::Pose tf_msg;
    tf_msg.position.x = kdl_frame.p.x();
    tf_msg.position.y = kdl_frame.p.y();
    tf_msg.position.z = kdl_frame.p.z();
    tf_msg.orientation.x = quat_x;
    tf_msg.orientation.y = quat_y;
    tf_msg.orientation.z = quat_z;
    tf_msg.orientation.w = quat_w;

    Transform tf(tf_msg, "");

    EXPECT_NEAR(kdl_frame.p.x(), tf.frame().p.x(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.y(), tf.frame().p.y(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.z(), tf.frame().p.z(), POSITION_EPSILON);

    double qx, qy, qz, qw;
    tf.frame().M.GetQuaternion(qx, qy, qz, qw);

    EXPECT_NEAR(quat_x, qx, ROTATION_EPSILON);
    EXPECT_NEAR(quat_y, qy, ROTATION_EPSILON);
    EXPECT_NEAR(quat_z, qz, ROTATION_EPSILON);
    EXPECT_NEAR(quat_w, qw, ROTATION_EPSILON);
  }

  // Constructor geometry_msgs::Point, geometry_msgs::Quaternion
  {
    geometry_msgs::Pose tf_msg;
    tf_msg.position.x = kdl_frame.p.x();
    tf_msg.position.y = kdl_frame.p.y();
    tf_msg.position.z = kdl_frame.p.z();
    tf_msg.orientation.x = quat_x;
    tf_msg.orientation.y = quat_y;
    tf_msg.orientation.z = quat_z;
    tf_msg.orientation.w = quat_w;

    Transform tf(tf_msg.position, tf_msg.orientation, "");

    EXPECT_NEAR(kdl_frame.p.x(), tf.frame().p.x(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.y(), tf.frame().p.y(), POSITION_EPSILON);
    EXPECT_NEAR(kdl_frame.p.z(), tf.frame().p.z(), POSITION_EPSILON);

    double qx, qy, qz, qw;
    tf.frame().M.GetQuaternion(qx, qy, qz, qw);

    EXPECT_NEAR(quat_x, qx, ROTATION_EPSILON);
    EXPECT_NEAR(quat_y, qy, ROTATION_EPSILON);
    EXPECT_NEAR(quat_z, qz, ROTATION_EPSILON);
    EXPECT_NEAR(quat_w, qw, ROTATION_EPSILON);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
