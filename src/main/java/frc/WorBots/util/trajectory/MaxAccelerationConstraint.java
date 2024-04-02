// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;

public class MaxAccelerationConstraint implements TrajectoryConstraint {
  private final double maxAcceleration;
  private final double maxDeceleration;

  public MaxAccelerationConstraint(double maxAcceleration, double maxDeceleration) {
    this.maxAcceleration = maxAcceleration;
    this.maxDeceleration = maxDeceleration;
  }

  @Override
  public double getMaxVelocityMetersPerSecond(
      Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond) {
    return Double.MAX_VALUE;
  }

  @Override
  public MinMax getMinMaxAccelerationMetersPerSecondSq(
      Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond) {
    // TODO Auto-generated method stub
    // Only limit on acceleration, not deceleration
    // if (velocityMetersPerSecond > 0.0) {

    // }
    return new MinMax(-maxDeceleration, maxAcceleration);
  }
}
