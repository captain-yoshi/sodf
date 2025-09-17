#pragma once

#include <Eigen/Geometry>

namespace sodf {
namespace geometry {

/* @brief Align and center vector CD with vector AB
 *
 * @param wTa transform: point A in frame W
 * @param wTb transform: point B in frame W
 * @param xTc transform: point C in frame X
 * @param xTd transform: point D in frame X
 * @param epsilon minimally acceptable threshold between ||AB|| and ||CD||
 *
 * @return The transform {W}T{X} such that segment CD (from {X}) is aligned to segment AB (in {W})
 *         and centered on it.
 *
 *   {W}
 *
 *         B
 *        /
 *       /
 *      /
 *     /
 *    A
 *          C--------D  {X}
 */
Eigen::Isometry3d alignCenterFrames(const Eigen::Isometry3d& wTa, const Eigen::Isometry3d& wTb,
                                    const Eigen::Isometry3d& xTc, const Eigen::Isometry3d& xTd, double epsilon);

}  // namespace geometry
}  // namespace sodf
