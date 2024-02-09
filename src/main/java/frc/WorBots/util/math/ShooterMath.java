// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.WorBots.FieldConstants;

public class ShooterMath {
  /** The maximum distance the robot can shoot, in meters */
  private static final double MAX_RANGE = 3.5;

  /** The maximum robot angle the robot can reliably shoot from, in radians */
  private static final double MAX_ANGLE = Units.degreesToRadians(70);

  /** Distance -> RPM */
  private static final InterpolatingTable RPM_LOOKUP = new InterpolatingTable(new double[][] {});

  /** Distance -> pivot angle */
  private static final InterpolatingTable ANGLE_LOOKUP = new InterpolatingTable(new double[][] {});

  /** Difference confidence levels for a shot */
  public enum ShotConfidence {
    LOW,
    MEDIUM,
    HIGH,
  }

  /** Calculated outputs for a shot */
  public static record ShotData(double rpm, double angle, ShotConfidence confidence) {}

  /**
   * Calculates outputs for a shot based on robot position
   *
   * @param robot The robot pose
   * @return The output data for the shot
   */
  public static ShotData calculateShotData(Pose2d robot) {
    final double distance = getGoalDistance(robot);
    final double rpm = RPM_LOOKUP.get(distance);
    final double angle = ANGLE_LOOKUP.get(distance);
    final ShotConfidence confidence = getConfidence(robot);
    return new ShotData(rpm, angle, confidence);
  }

  /**
   * Calculates confidence for a shot based on robot position
   *
   * @param robot The robot pose
   * @return The calculated confidence
   */
  public static ShotConfidence getConfidence(Pose2d robot) {
    if (!inRange(robot)) {
      return ShotConfidence.LOW;
    }

    return ShotConfidence.HIGH;
  }

  /**
   * Get the location of the goal on the field
   *
   * @return The translation of the goal
   */
  public static Translation2d getGoal() {
    return FieldConstants.Speaker.position;
  }

  /**
   * Get the distance between the robot and the goal
   *
   * @param robot The robot pose
   * @return The euclidean distance
   */
  public static double getGoalDistance(Pose2d robot) {
    return robot.getTranslation().getDistance(getGoal());
  }

  /**
   * Determine whether the robot is in range to make a shot
   *
   * @param robot The robot pose
   * @return Whether the robot is in range
   */
  public static boolean inRange(Pose2d robot) {
    return getGoalDistance(robot) <= MAX_RANGE;
  }
}
