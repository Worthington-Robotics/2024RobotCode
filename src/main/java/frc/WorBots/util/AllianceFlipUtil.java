package frc.WorBots.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.WorBots.FieldConstants;
import frc.WorBots.util.trajectory.RotationSequence;

/**
 * Utility functions for flipping from the blue to red alliance. By default, all
 * translations and
 * poses in {@link FieldConstants} are stored with the origin at the rightmost
 * point on the blue
 * alliance wall.
 */
public class AllianceFlipUtil {
  /**
   * Flips a translation to the correct side of the field based on the current
   * alliance color.
   * 
   * @param translation The translation to modify
   * @return The modified translation
   */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return applyAgnostic(translation);
    } else {
      return translation;
    }
  }

  /**
   * Flips a translation to the other side of the field not based on the current
   * alliance color.
   * 
   * @param translation The translation to modify
   * @return The modified translation
   */
  public static Translation2d applyAgnostic(Translation2d translation) {
    return new Translation2d(FieldConstants.fieldLength - translation.getX(), translation.getY());
  }

  /**
   * Flips an x coordinate to the correct side of the field based on the current
   * alliance color.
   */
  public static double apply(double xCoordinate) {
    if (shouldFlip()) {
      return FieldConstants.fieldLength - xCoordinate;
    } else {
      return xCoordinate;
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else {
      return rotation;
    }
  }

  /**
   * Flips a pose to the correct side of the field based on the current alliance
   * color.
   */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(
          FieldConstants.fieldLength - pose.getX(),
          pose.getY(),
          new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    } else {
      return pose;
    }
  }

  /**
   * Flips a trajectory state to the correct side of the field based on the
   * current alliance color.
   */
  public static Trajectory.State apply(Trajectory.State state) {
    if (shouldFlip()) {
      return new Trajectory.State(
          state.timeSeconds,
          state.velocityMetersPerSecond,
          state.accelerationMetersPerSecondSq,
          new Pose2d(
              FieldConstants.fieldLength - state.poseMeters.getX(),
              state.poseMeters.getY(),
              new Rotation2d(
                  -state.poseMeters.getRotation().getCos(),
                  state.poseMeters.getRotation().getSin())),
          -state.curvatureRadPerMeter);
    } else {
      return state;
    }
  }

  /** Flips a rotation sequence state based on the current alliance color. */
  public static RotationSequence.State apply(RotationSequence.State state) {
    if (shouldFlip()) {
      return new RotationSequence.State(
          new Rotation2d(-state.position.getCos(), state.position.getSin()),
          -state.velocityRadiansPerSec);
    } else {
      return state;
    }
  }

  private static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
  }
}
