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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.WorBots.Constants;
import frc.WorBots.FieldConstants;
import frc.WorBots.util.debug.TunableDouble;
import java.util.Optional;

public class ShooterMath {
  // Confidence calculation constants

  /** The maximum distance the robot can reliably shoot, in meters */
  private static final double MAX_RELIABLE_RANGE = 4.5;

  /** The maximum distance the robot can shoot, in meters */
  private static final double MAX_RANGE = 7.0;

  /**
   * The maximum angle from the goal to the robot that the robot can reliably shoot from, in radians
   */
  private static final double MAX_RELIABLE_ANGLE = Units.degreesToRadians(50);

  /** The maximum angle from the goal to the robot that the robot can shoot from, in radians */
  private static final double MAX_ANGLE = Units.degreesToRadians(75);

  /** The maximum speed the robot can reliably move at while shooting, in meters per second */
  private static final double MAX_RELIABLE_ROBOT_VELOCITY = 1.0;

  /** The maximum speed the robot can move at while shooting, in meters per second */
  private static final double MAX_ROBOT_VELOCITY = 3.0;

  // Constants for RPM calculation

  /** The maxmimum RPM we can shoot at */
  public static final double MAX_SHOOTER_RPM = 5600;

  /**
   * The closest range we can shoot at. The point where the RPM falloff results in the lowest RPM
   */
  private static final double CLOSEST_RANGE = 1.02;

  /** The range past which the calculated RPM will be MAX_SHOOTER_RPM */
  private static final double MAX_RPM_FALLOFF_RANGE = 4.3;

  /**
   * How much of the RPM range is controlled by linear distance falloff. From 0-1. The minimum RPM
   * will be (1 - RPM_FALLOFF_COEFFICIENT) * MAX_SHOOTER_RPM, which will be when the robot is
   * CLOSEST_RANGE away from the goal
   */
  private static final double RPM_FALLOFF_COEFFICIENT = 0.46;

  // Side shot constants

  /** The amount to move the pivot down for side shots */
  private static final TunableDouble SIDE_SHOT_PIVOT_COEFFICIENT =
      new TunableDouble("Tuning", "Shooting", "Side Pivot Coeff", 0.045);

  /** The amount to adjust the robot angle for side shots */
  private static final TunableDouble SIDE_SHOT_ROBOT_ANGLE_COEFFICIENT =
      new TunableDouble("Tuning", "Shooting", "Side Robot Coeff", 0.032);

  // Momentum compensation constants

  /**
   * The amount to adjust the robot pose we are calculating with based on robot velocity. Will be
   * multiplied by the robot period to find the period of time over which to apply the robot
   * velocity to the pose to get the expected pose
   */
  private static final double PREDICTION_FACTOR = 0.0;

  private static final double PREDICTION_DISTANCE_FACTOR = 2.0;

  /** The amount to adjust the robot angle based on the robot velocity */
  private static final double ROBOT_ANGLE_MOMENTUM_COMP_COEFFICIENT = 0.00;

  /** The amount to increase robot angle momentum compensation by depending on the range */
  private static final double ROBOT_ANGLE_MOMENTUM_COMP_RANGE_AMOUNT = 0.000;

  /** Distance -> pivot angle */
  private static final InterpolatingTable PIVOT_ANGLE_LOOKUP =
      new InterpolatingTable(
          new double[][] {
            {1.096, 0.498},
            {1.406, 0.52},
            {2.197, 0.8072},
            {2.379, 0.816},
            {4.305, 1.0005},
            {5.295, 1.0601}
          });

  /** Difference confidence levels for a shot */
  public enum ShotConfidence {
    LOW,
    MEDIUM,
    HIGH,
  }

  /** Calculated outputs for a shot */
  public static record ShotData(
      double rpm, double pivotAngle, Rotation2d robotAngle, ShotConfidence confidence) {}

  /**
   * Calculates outputs for a shot based on robot position
   *
   * @param robot The robot pose
   * @param robotSpeeds The field-relative speeds of the robot
   * @return The output data for the shot
   */
  public static ShotData calculateShotData(Pose2d robot, ChassisSpeeds robotSpeeds) {
    final double distance = getGoalDistance(robot);
    final Rotation2d goalToRobotAngle = getGoalToRobotAngle(robot);
    final Pose2d predicted = predictNextPose(robot, robotSpeeds);
    final double predictedDistance = getGoalDistance(predicted);
    final Rotation2d predictedGoalToRobotAngle = getGoalToRobotAngle(predicted);

    final double rpm = calculateShooterRPM(distance);
    final double pivotAngle = calculatePivotAngle(predictedDistance, predictedGoalToRobotAngle);
    final Rotation2d robotAngle = calculateRobotAngle(predicted, robotSpeeds);
    final ShotConfidence confidence = calculateConfidence(distance, goalToRobotAngle, robotSpeeds);

    return new ShotData(rpm, pivotAngle, robotAngle, confidence);
  }

  /**
   * Calculates the desired pivot angle from robot information
   *
   * @param distance The distance to the goal
   * @param goalToRobotAngle The goal-to-robot angle
   * @return The desired fused pivot angle
   */
  public static double calculatePivotAngle(double distance, Rotation2d goalToRobotAngle) {
    double pivotAngle = PIVOT_ANGLE_LOOKUP.get(distance);
    // Move the pivot slightly down for side shots
    pivotAngle += Math.abs(goalToRobotAngle.getRadians()) * SIDE_SHOT_PIVOT_COEFFICIENT.get();
    return pivotAngle;
  }

  /**
   * Calculates confidence for a shot based on robot position
   *
   * @param robot The robot pose without prediction
   * @param robotSpeeds The field-relative speeds of the robot
   * @return The calculated confidence
   */
  public static ShotConfidence calculateConfidence(Pose2d robot, ChassisSpeeds robotSpeeds) {
    final double distance = getGoalDistance(robot);
    final Rotation2d angle = getGoalToRobotAngle(robot);
    return calculateConfidence(distance, angle, robotSpeeds);
  }

  /**
   * Calculates confidence for a shot based on robot position
   *
   * @param distance The distance to the goal
   * @param goalToRobotAngle The goal-to-robot angle
   * @param robotSpeeds The field-relative speeds of the robot
   * @return The calculated confidence
   */
  public static ShotConfidence calculateConfidence(
      double distance, Rotation2d goalToRobotAngle, ChassisSpeeds robotSpeeds) {
    if (distance > MAX_RANGE || Math.abs(goalToRobotAngle.getRadians()) > MAX_ANGLE) {
      return ShotConfidence.LOW;
    }

    if (distance > MAX_RELIABLE_RANGE
        || Math.abs(goalToRobotAngle.getRadians()) > MAX_RELIABLE_ANGLE) {
      return ShotConfidence.MEDIUM;
    }

    // Speed checks
    final double robotVelocity = GeomUtil.getChassisSpeedsMagnitude(robotSpeeds);
    if (robotVelocity > MAX_ROBOT_VELOCITY) {
      return ShotConfidence.LOW;
    }

    if (robotVelocity > MAX_RELIABLE_ROBOT_VELOCITY) {
      return ShotConfidence.MEDIUM;
    }

    return ShotConfidence.HIGH;
  }

  /**
   * Calculate the desired robot angle
   *
   * @param robot The robot pose without prediction
   * @param robotSpeeds The field-relative speeds of the robot
   * @return The desired yaw for the robot
   */
  public static Rotation2d calculateRobotAngle(Pose2d robot, ChassisSpeeds robotSpeeds) {
    final Pose2d predicted = predictNextPose(robot, robotSpeeds);
    final Rotation2d goalToRobotAngle = getGoalToRobotAngle(predicted);
    final double distance = getGoalDistance(predicted);
    return calculateRobotAngle(predicted, goalToRobotAngle, robotSpeeds, distance);
  }

  /**
   * Calculate the desired robot angle
   *
   * @param robot The robot pose with prediction
   * @param goalToRobotAngle The goal-to-robot angle
   * @param robotSpeeds The field-relative speeds of the robot
   * @param distance The distance to the goal
   * @return The desired yaw for the robot
   */
  public static Rotation2d calculateRobotAngle(
      Pose2d robot, Rotation2d goalToRobotAngle, ChassisSpeeds robotSpeeds, double distance) {
    Rotation2d robotAngle = getGoalTheta(robot);
    // Rotate the robot slightly away from the goal wall for side shots
    robotAngle = robotAngle.minus(goalToRobotAngle.times(SIDE_SHOT_ROBOT_ANGLE_COEFFICIENT.get()));
    // Apply momentum compensation
    robotAngle =
        robotAngle.plus(
            Rotation2d.fromRadians(
                calculateRobotAngleMomentumCompensation(robotAngle, robotSpeeds, distance)));
    return robotAngle;
  }

  /**
   * Calculates momentum compensation for the robot angle based on robot speed
   *
   * @param robotAngle The angle for the robot that has it facing the goal
   * @param robotSpeeds The field-relative speeds of the robot
   * @param distance The distance to the goal
   * @return The amount, in radians, to add to the desired robot angle
   */
  public static double calculateRobotAngleMomentumCompensation(
      Rotation2d robotAngle, ChassisSpeeds robotSpeeds, double distance) {
    // Calculate the velocity vector that is perpendicular to the goal
    final Rotation2d robotAngleRotated = robotAngle.rotateBy(Rotation2d.fromDegrees(90));
    // Calculate the dot product to find the robot velocity perpendicular to the
    // goal
    final double dotProduct =
        (robotSpeeds.vxMetersPerSecond * robotAngleRotated.getCos())
            + (robotSpeeds.vyMetersPerSecond * robotAngleRotated.getSin());

    // We increase the compensation by the distance so that the effect is more
    // pronounced the farther away we are
    final double rangeCompensation = dotProduct * distance * ROBOT_ANGLE_MOMENTUM_COMP_RANGE_AMOUNT;
    return dotProduct * ROBOT_ANGLE_MOMENTUM_COMP_COEFFICIENT + rangeCompensation;
  }

  /**
   * Calculate the desired shooter RPM based on the distance to the goal
   *
   * @param robot The robot pose
   * @return The desired RPM for the shooter
   */
  public static double calculateShooterRPM(Pose2d robot) {
    double distance = getGoalDistance(robot);
    return calculateShooterRPM(distance);
  }

  /**
   * Calculate the desired shooter RPM based on the distance to the goal
   *
   * @param distance The distance in meters to the goal
   * @return The desired RPM for the shooter
   */
  public static double calculateShooterRPM(double distance) {
    // Clamp the distance to prevent bad values
    distance = MathUtil.clamp(distance, CLOSEST_RANGE, MAX_RPM_FALLOFF_RANGE);

    double scalar =
        1.0 - GeneralMath.getScalarPosition(distance, CLOSEST_RANGE, MAX_RPM_FALLOFF_RANGE);
    // 1.0 - x so that it is inverted
    scalar = 1.0 - scalar;

    // The linear falloff part of the RPM
    final double rpmAmount = MAX_SHOOTER_RPM * RPM_FALLOFF_COEFFICIENT;
    // The minimum RPM
    final double base = MAX_SHOOTER_RPM - rpmAmount;
    return base + rpmAmount * scalar;
  }

  /**
   * Applies ChassisSpeeds to the robot pose to predict what it will be in the future based on
   * constants
   *
   * @param robot The robot pose
   * @param robotSpeeds The field-relative speeds of the robot
   * @return The modified pose
   */
  public static Pose2d predictNextPose(Pose2d robot, ChassisSpeeds robotSpeeds) {
    final double period =
        Constants.ROBOT_PERIOD * PREDICTION_FACTOR
            + Constants.ROBOT_PERIOD * PREDICTION_DISTANCE_FACTOR * getGoalDistance(robot);
    return GeomUtil.applyChassisSpeeds(robot, robotSpeeds, period);
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

  /**
   * Gets the angle from the goal to the robot. When viewing from behind the alliance wall, this
   * will be zero when the robot is straight ahead, -pi/2 when the robot is all the way to your
   * left, and pi/2 when the robot is all the way to the right. Any angles outside of this range are
   * clamped.
   *
   * @param robot The robot pose
   * @return The angle
   */
  public static Rotation2d getGoalToRobotAngle(Pose2d robot) {
    final Translation2d goal = getGoal();
    double angle = Math.atan2(robot.getX() - goal.getX(), robot.getY() - goal.getY()) + Math.PI / 2;
    final var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      angle -= Math.PI;
      // angle *= -1;
      // Convert pi-3pi/2 range to negative max
      if (angle < -Math.PI) {
        angle = Math.PI;
      }
    } else {
      // Convert pi-3pi/2 range to negative max
      if (angle > Math.PI) {
        angle = -Math.PI;
      }
    }
    // Clamp the angle between -pi/2 and pi/2. We don't want weird wrapping behavior
    // when the angle is beyond the alliance wall
    angle = GeneralMath.clampMagnitude(angle, Math.PI / 2);
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
