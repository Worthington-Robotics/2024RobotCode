// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.WorBots.FieldConstants;
import frc.WorBots.util.GeomUtil;

/** Math used to calculate information about automatic shots */
public class ShotCalculator {
  // The maximum distance the robot can shoot, in meters
  private static final double MAX_RANGE = 2.0;

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

  /**
   * Calculates the robot angle for an automatic shot based on robot pose and speed
   *
   * @param robot The robot pose
   * @param speed The robot speed
   * @return The angle for the robot
   */
  public static Rotation2d getShotAngle(Pose2d robot, ChassisSpeeds speed) {
    double rawDistToGoal = getGoalDistance(robot);

    Rotation2d speakerToRobotAngle = robot.getTranslation().minus(getGoal()).getAngle();
    final Translation2d linearFieldVelocity =
        new Translation2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
    Translation2d tangentialVelocity =
        linearFieldVelocity.rotateBy(speakerToRobotAngle.unaryMinus());
    // Positive when velocity is away from speaker
    double radialComponent = tangentialVelocity.getX();
    // Positive when traveling CCW about speaker
    double tangentialComponent = tangentialVelocity.getY();

    // Ig this is the estimated time of the note in the air
    // later on this will be a function of the distance
    final double shotTime = getShotTime(rawDistToGoal);

    // Add robot velocity to raw shot speed
    double shotSpeed = rawDistToGoal / shotTime + radialComponent;
    if (shotSpeed <= 0.0) {
      shotSpeed = 0.0;
    }
    // Rotate back into field frame then add take opposite
    Rotation2d goalHeading =
        robot
            .times(-1)
            .transformBy(GeomUtil.translationToTransform(getGoal()))
            .getTranslation()
            .getAngle();
    // Aim opposite of tangentialComponent (negative lead when tangentialComponent
    // is positive)
    goalHeading = goalHeading.plus(new Rotation2d(shotSpeed, tangentialComponent));
    double effectiveDist = shotTime * Math.hypot(tangentialComponent, shotSpeed);

    return goalHeading;
  }

  /**
   * Calculate the amount of time that a shot will take based on distance
   *
   * @param distance Distance to the goal
   * @return The time in seconds that the shot will take
   */
  public static final double getShotTime(double distance) {
    return 1.05;
  }
}
