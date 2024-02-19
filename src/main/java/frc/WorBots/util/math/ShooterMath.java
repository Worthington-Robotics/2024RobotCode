// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.WorBots.FieldConstants;
import java.util.Optional;

public class ShooterMath {
  /** The maximum distance the robot can reliably shoot, in meters */
  private static final double MAX_RELIABLE_RANGE = 4.3;

  /** The maximum distance the robot can shoot, in meters */
  private static final double MAX_RANGE = 7.0;

  /**
   * The maximum angle from the goal to the robot that the robot can reliably shoot from, in radians
   */
  private static final double MAX_RELIABLE_ANGLE = Units.degreesToRadians(50);

  /** The maximum angle from the goal to the robot that the robot can shoot from, in radians */
  private static final double MAX_ANGLE = Units.degreesToRadians(75);

  public static final double MAX_SHOOTER_RPM = 5600;

  public static final double CLOSEST_RANGE = 1.02;

  public static final double RPM_FALLOFF_COEFFICIENT = 0.46;

  /** Distance -> RPM */
  private static final InterpolatingTable RPM_LOOKUP =
      new InterpolatingTable(new double[][] {{0.0, 0.0}});

  /** Distance -> pivot angle */
  private static final InterpolatingTable ANGLE_LOOKUP =
      new InterpolatingTable(
          new double[][] {
            {1.096, 0.498},
            {1.406, 0.52},
            {2.197, 0.8102},
            {2.379, 0.814},
            {4.305, 1.0105},
            {5.295, 1.0601}
          });

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
    final double distance = getGoalDistance(robot);
    final Rotation2d angle = getGoalToRobotAngle(robot);
    if (distance > MAX_RANGE || Math.abs(angle.getRadians()) > MAX_ANGLE) {
      return ShotConfidence.LOW;
    }

    if (distance > MAX_RELIABLE_RANGE || Math.abs(angle.getRadians()) > MAX_RELIABLE_ANGLE) {
      return ShotConfidence.MEDIUM;
    }

    return ShotConfidence.HIGH;
  }

  /**
   * Get the location of the goal on the field
   *
   * @return The translation of the goal
   */
  public static Translation2d getGoal() {
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();
    Translation2d out = FieldConstants.Speaker.position;
    if (currentAlliance.isPresent() && currentAlliance.get() == Alliance.Red) {
      out = out.plus(new Translation2d(FieldConstants.fieldLength, 0.0));
    }
    return out;
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
   * Get the angle to make the robot face the goal
   *
   * @param robot The robot pose
   * @return The yaw angle for the robot
   */
  public static Rotation2d getGoalTheta(Pose2d robot) {
    final var alliance = DriverStation.getAlliance();
    double translatedX =
        (alliance.isPresent() && alliance.get() == Alliance.Red
            ? robot.getX() - FieldConstants.fieldLength
            : robot.getX());
    final double angle = Math.atan2(robot.getY() - getGoal().getY(), translatedX);
    return new Rotation2d(angle);
  }

  public static Rotation2d getGoalToRobotAngle(Pose2d robot) {
    final Translation2d goal = getGoal();
    double angle = Math.atan2(robot.getX() - goal.getX(), robot.getY() - goal.getY()) + Math.PI / 2;
    final var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      angle -= Math.PI;
    }
    return new Rotation2d(angle);
  }

  public static double calculateShooterRPM(Pose2d robot) {
    double distance = getGoalDistance(robot);
    // Clamp the distance to prevent bad values
    distance = MathUtil.clamp(distance, CLOSEST_RANGE, MAX_RELIABLE_RANGE);

    double scalar =
        1.0 - GeneralMath.getScalarPosition(distance, CLOSEST_RANGE, MAX_RELIABLE_RANGE);
    // 1.0 - x so that it is inverted
    scalar = 1.0 - scalar;

    final double rpmAmount = MAX_SHOOTER_RPM * RPM_FALLOFF_COEFFICIENT;
    final double base = MAX_SHOOTER_RPM - rpmAmount;
    return base + rpmAmount * scalar;
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
