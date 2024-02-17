// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.math;

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
  private static final double MAX_RELIABLE_RANGE = 5.0;

  /** The maximum distance the robot can shoot, in meters */
  private static final double MAX_RANGE = 7.0;

  /**
   * The maximum angle from the goal to the robot that the robot can reliably shoot from, in radians
   */
  private static final double MAX_RELIABLE_ANGLE = Units.degreesToRadians(50);

  /** The maximum angle from the goal to the robot that the robot can shoot from, in radians */
  private static final double MAX_ANGLE = Units.degreesToRadians(75);

  /** Distance -> RPM */
  private static final InterpolatingTable RPM_LOOKUP =
      new InterpolatingTable(new double[][] {{0.0, 0.0}});

  /** Distance -> pivot angle */
  private static final InterpolatingTable ANGLE_LOOKUP =
      new InterpolatingTable(
          new double[][] {
            {1.121, 0.5459},
            {2.228, 0.88425},
            {3.973, 0.9756},
            {4.305, 1.0205},
            {6.348, 1.0823}
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
