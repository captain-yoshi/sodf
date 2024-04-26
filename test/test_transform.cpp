#include <sodf/geometry/transform.h>
#include <eigen_conversions/eigen_msg.h>

#include <gtest/gtest.h>

using namespace sodf::geometry;

constexpr double POSITION_EPSILON = 1e-06;
constexpr double ROTATION_EPSILON = 1e-06;

TEST(Transform, Constructor)
{
  Eigen::Vector3d position(-0.3, 1.5, -6);
  Eigen::Quaterniond quat = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());

  Eigen::Isometry3d eig_tf(quat);
  eig_tf.pretranslate(position);

  // Constructor Eigen::Isometry3d
  {
    Transform tf(eig_tf);

    EXPECT_NEAR(-0.3, tf.tf().translation().x(), POSITION_EPSILON);
    EXPECT_NEAR(1.5, tf.tf().translation().y(), POSITION_EPSILON);
    EXPECT_NEAR(-6, tf.tf().translation().z(), POSITION_EPSILON);
    Eigen::Quaterniond q(tf.tf().linear());

    EXPECT_NEAR(0.7071068, q.x(), ROTATION_EPSILON);
    EXPECT_NEAR(0.0, q.y(), ROTATION_EPSILON);
    EXPECT_NEAR(0.7071068, q.z(), ROTATION_EPSILON);
    EXPECT_NEAR(0.0, q.w(), ROTATION_EPSILON);
  }

  // Constructor Eigen::Vector3d, Eigen::Quaterniond
  {
    Transform tf(position, quat);

    EXPECT_NEAR(-0.3, tf.tf().translation().x(), POSITION_EPSILON);
    EXPECT_NEAR(1.5, tf.tf().translation().y(), POSITION_EPSILON);
    EXPECT_NEAR(-6, tf.tf().translation().z(), POSITION_EPSILON);
    Eigen::Quaterniond q(tf.tf().linear());

    EXPECT_NEAR(0.7071068, q.x(), ROTATION_EPSILON);
    EXPECT_NEAR(0.0, q.y(), ROTATION_EPSILON);
    EXPECT_NEAR(0.7071068, q.z(), ROTATION_EPSILON);
    EXPECT_NEAR(0.0, q.w(), ROTATION_EPSILON);
  }

  // Constructor double px .. double qw
  {
    Transform tf(-0.3, 1.5, -6, quat.x(), quat.y(), quat.z(), quat.w());

    EXPECT_NEAR(-0.3, tf.tf().translation().x(), POSITION_EPSILON);
    EXPECT_NEAR(1.5, tf.tf().translation().y(), POSITION_EPSILON);
    EXPECT_NEAR(-6, tf.tf().translation().z(), POSITION_EPSILON);
    Eigen::Quaterniond q(tf.tf().linear());

    EXPECT_NEAR(0.7071068, q.x(), ROTATION_EPSILON);
    EXPECT_NEAR(0.0, q.y(), ROTATION_EPSILON);
    EXPECT_NEAR(0.7071068, q.z(), ROTATION_EPSILON);
    EXPECT_NEAR(0.0, q.w(), ROTATION_EPSILON);
  }

  // Constructor geometry_msgs::Transform
  {
    geometry_msgs::Transform tf_msg;
    tf_msg.translation.x = position.x();
    tf_msg.translation.y = position.y();
    tf_msg.translation.z = position.z();
    tf_msg.rotation.x = quat.x();
    tf_msg.rotation.y = quat.y();
    tf_msg.rotation.z = quat.z();
    tf_msg.rotation.w = quat.w();

    Transform tf(tf_msg);

    EXPECT_NEAR(-0.3, tf.tf().translation().x(), POSITION_EPSILON);
    EXPECT_NEAR(1.5, tf.tf().translation().y(), POSITION_EPSILON);
    EXPECT_NEAR(-6, tf.tf().translation().z(), POSITION_EPSILON);
    Eigen::Quaterniond q(tf.tf().linear());

    EXPECT_NEAR(0.7071068, q.x(), ROTATION_EPSILON);
    EXPECT_NEAR(0.0, q.y(), ROTATION_EPSILON);
    EXPECT_NEAR(0.7071068, q.z(), ROTATION_EPSILON);
    EXPECT_NEAR(0.0, q.w(), ROTATION_EPSILON);
  }

  // Constructor geometry_msgs::Vector3, geometry_msgs::Quaternion
  {
    geometry_msgs::Transform tf_msg;
    tf_msg.translation.x = position.x();
    tf_msg.translation.y = position.y();
    tf_msg.translation.z = position.z();
    tf_msg.rotation.x = quat.x();
    tf_msg.rotation.y = quat.y();
    tf_msg.rotation.z = quat.z();
    tf_msg.rotation.w = quat.w();

    Transform tf(tf_msg.translation, tf_msg.rotation);

    EXPECT_NEAR(-0.3, tf.tf().translation().x(), POSITION_EPSILON);
    EXPECT_NEAR(1.5, tf.tf().translation().y(), POSITION_EPSILON);
    EXPECT_NEAR(-6, tf.tf().translation().z(), POSITION_EPSILON);
    Eigen::Quaterniond q(tf.tf().linear());

    EXPECT_NEAR(0.7071068, q.x(), ROTATION_EPSILON);
    EXPECT_NEAR(0.0, q.y(), ROTATION_EPSILON);
    EXPECT_NEAR(0.7071068, q.z(), ROTATION_EPSILON);
    EXPECT_NEAR(0.0, q.w(), ROTATION_EPSILON);
  }

  // Constructor geometry_msgs::Pose
  {
    geometry_msgs::Pose tf_msg;
    tf_msg.position.x = position.x();
    tf_msg.position.y = position.y();
    tf_msg.position.z = position.z();
    tf_msg.orientation.x = quat.x();
    tf_msg.orientation.y = quat.y();
    tf_msg.orientation.z = quat.z();
    tf_msg.orientation.w = quat.w();

    Transform tf(tf_msg);

    EXPECT_NEAR(-0.3, tf.tf().translation().x(), POSITION_EPSILON);
    EXPECT_NEAR(1.5, tf.tf().translation().y(), POSITION_EPSILON);
    EXPECT_NEAR(-6, tf.tf().translation().z(), POSITION_EPSILON);
    Eigen::Quaterniond q(tf.tf().linear());

    EXPECT_NEAR(0.7071068, q.x(), ROTATION_EPSILON);
    EXPECT_NEAR(0.0, q.y(), ROTATION_EPSILON);
    EXPECT_NEAR(0.7071068, q.z(), ROTATION_EPSILON);
    EXPECT_NEAR(0.0, q.w(), ROTATION_EPSILON);
  }

  // Constructor geometry_msgs::Point, geometry_msgs::Quaternion
  {
    geometry_msgs::Pose tf_msg;
    tf_msg.position.x = position.x();
    tf_msg.position.y = position.y();
    tf_msg.position.z = position.z();
    tf_msg.orientation.x = quat.x();
    tf_msg.orientation.y = quat.y();
    tf_msg.orientation.z = quat.z();
    tf_msg.orientation.w = quat.w();

    Transform tf(tf_msg.position, tf_msg.orientation);

    EXPECT_NEAR(-0.3, tf.tf().translation().x(), POSITION_EPSILON);
    EXPECT_NEAR(1.5, tf.tf().translation().y(), POSITION_EPSILON);
    EXPECT_NEAR(-6, tf.tf().translation().z(), POSITION_EPSILON);
    Eigen::Quaterniond q(tf.tf().linear());

    EXPECT_NEAR(0.7071068, q.x(), ROTATION_EPSILON);
    EXPECT_NEAR(0.0, q.y(), ROTATION_EPSILON);
    EXPECT_NEAR(0.7071068, q.z(), ROTATION_EPSILON);
    EXPECT_NEAR(0.0, q.w(), ROTATION_EPSILON);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
