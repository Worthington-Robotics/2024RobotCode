// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.WorBots.Constants;

/**
 * Geometry utilities for working with translations, rotations, transforms, and poses. Partial
 * credit to team 6328.
 */
public class GeomUtil {
  /**
   * Creates a pure translating transform
   *
   * @param translation The translation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d translationToTransform(Translation2d translation) {
    return new Transform2d(translation, new Rotation2d());
  }

  /**
   * Creates a pure translating transform
   *
   * @param x The x componenet of the translation
   * @param y The y componenet of the translation
   * @return The resulting transform
   */
  public static Transform2d translationToTransform(double x, double y) {
    return new Transform2d(new Translation2d(x, y), new Rotation2d());
  }

  /**
   * Creates a pure rotating transform
   *
   * @param rotation The rotation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d rotationToTransform(Rotation2d rotation) {
    return new Transform2d(new Translation2d(), rotation);
  }

  /**
   * Converts a Pose2d to a Transform2d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform2d poseToTransform(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic
   * chain
   *
   * @param transform The transform that will represent the pose
   * @return The resulting pose
   */
  public static Pose2d transformToPose(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }

  /**
   * Creates a pure translated pose
   *
   * @param translation The translation to create the pose with
   * @return The resulting pose
   */
  public static Pose2d translationToPose(Translation2d translation) {
    return new Pose2d(translation, new Rotation2d());
  }

  /**
   * Creates a pure rotated pose
   *
   * @param rotation The rotation to create the pose with
   * @return The resulting pose
   */
  public static Pose2d rotationToPose(Rotation2d rotation) {
    return new Pose2d(new Translation2d(), rotation);
  }

  /**
   * Multiplies a twist by a scaling factor
   *
   * @param twist The twist to multiply
   * @param factor The scaling factor for the twist components
   * @return The new twist
   */
  public static Twist2d multiplyTwist(Twist2d twist, double factor) {
    return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
  }

  /**
   * Converts a Pose3d to a Transform3d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform3d pose3dToTransform3d(Pose3d pose) {
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * Converts a Transform3d to a Pose3d to be used as a position or as the start of a kinematic
   * chain
   *
   * @param transform The transform that will represent the pose
   * @return The resulting pose
   */
  public static Pose3d transform3dToPose3d(Transform3d transform) {
    return new Pose3d(transform.getTranslation(), transform.getRotation());
  }

  /**
   * Converts a Translation3d to a Translation2d by extracting two dimensions (X and Y). chain
   *
   * @param transform The original translation
   * @return The resulting translation
   */
  public static Translation2d translation3dTo2dXY(Translation3d translation) {
    return new Translation2d(translation.getX(), translation.getY());
  }

  /**
   * Converts a Translation3d to a Translation2d by extracting two dimensions (X and Z). chain
   *
   * @param transform The original translation
   * @return The resulting translation
   */
  public static Translation2d translation3dTo2dXZ(Translation3d translation) {
    return new Translation2d(translation.getX(), translation.getZ());
  }

  /**
   * Checks if a Transform2d is within a bounding box of two Transform2d corners.
   *
   * @param transform The original translation
   * @param bounds The two corners of the bounding box to check for collision with
   * @return The resulting translation
   */
  public static boolean translation2dInBoundingBox(
      Translation2d translation, Translation2d[] bounds) {
    if (translation.getX() < bounds[0].getX()) {
      return false;
    }
    if (translation.getX() > bounds[1].getX()) {
      return false;
    }
    if (translation.getY() < bounds[0].getY()) {
      return false;
    }
    if (translation.getY() > bounds[1].getY()) {
      return false;
    }
    return true;
  }

  /**
   * Gets the magnitude of a ChassisSpeeds
   *
   * @param speeds The speeds to get the magnitude of
   * @return The magnitude of the speeds vector. Does not factor in rotational velocity.
   */
  public static double getChassisSpeedsMagnitude(ChassisSpeeds speeds) {
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  /**
   * Applies a ChassisSpeeds to a Pose2d to get what that pose will be after a period of time
   *
   * @param pose The pose to modify
   * @param speeds The speeds to apply to the pose
   * @param period The period of time to apply over, in seconds
   * @return The modified pose
   */
  public static Pose2d applyChassisSpeeds(Pose2d pose, ChassisSpeeds speeds, double period) {
    return new Pose2d(
        pose.getX() + speeds.vxMetersPerSecond * period,
        pose.getY() + speeds.vyMetersPerSecond * period,
        pose.getRotation().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * period)));
  }

  /**
   * Checks if pose1's position is near pose2's position
   *
   * @param pose1 The first pose
   * @param pose2 The second pose
   * @param threshold The distance threshold
   * @return Whether the distance is <= the threshold
   */
  public static boolean isPose2dNear(Pose2d pose1, Pose2d pose2, double threshold) {
    return isTranslation2dNear(pose1.getTranslation(), pose2.getTranslation(), threshold);
  }

  /**
   * Checks if pose1's position is near pose2's position
   *
   * @param pose1 The first pose
   * @param pose2 The second pose
   * @param threshold The distance threshold
   * @return Whether the distance is <= the threshold
   */
  public static boolean isTranslation2dNear(
      Translation2d pose1, Translation2d pose2, double threshold) {
    return pose1.getDistance(pose2) <= threshold;
  }

  /**
   * Discretizes a ChassisSpeeds with an additional correction for drift
   *
   * @param speeds The desired speeds
   * @param driftRate The measured drift rate of the swerve drive, in radians per second
   * @return The discretized and corrected speeds
   */
  public static ChassisSpeeds driftCorrectChassisSpeeds(ChassisSpeeds speeds, double driftRate) {
    final double correctedOmega = speeds.omegaRadiansPerSecond + driftRate;
    final Pose2d futurePose =
        new Pose2d(
            speeds.vxMetersPerSecond * Constants.ROBOT_PERIOD,
            speeds.vyMetersPerSecond * Constants.ROBOT_PERIOD,
            new Rotation2d(correctedOmega * Constants.ROBOT_PERIOD));

    final Twist2d twistForPose = new Pose2d().log(futurePose);

    final ChassisSpeeds corrected =
        new ChassisSpeeds(
            twistForPose.dx / Constants.ROBOT_PERIOD,
            twistForPose.dy / Constants.ROBOT_PERIOD,
            twistForPose.dtheta / Constants.ROBOT_PERIOD);
    return corrected;
  }
}
