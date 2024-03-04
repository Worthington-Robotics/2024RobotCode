// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.math;

import edu.wpi.first.math.geometry.Rotation2d;

/** General math utilities */
public class GeneralMath {
  /**
   * Clamps a directional velocity output to create a smooth limiting function at both ends of a
   * boundary. The input velocity will be slowly clamped as the position gets too close to either
   * end of the boundary, but only if it is moving toward that boundary.
   *
   * @param velocity The velocity to limit
   * @param position The current position of the system
   * @param maxVelocity The maximum velocity. Output is guaranteed to be <= to this.
   * @param maxPosition The maximum position of the boundary. The boundary goes from 0-maxPosition
   * @param limitDistance The distance from both ends of the boundary to start the soft limit at
   * @return The clamped velocity
   */
  public static double softLimitVelocity(
      double velocity,
      double position,
      double maxVelocity,
      double maxPosition,
      double limitDistance) {
    return softLimitVelocity(
        velocity, position, maxVelocity, maxPosition, limitDistance, limitDistance);
  }

  /**
   * Clamps a directional velocity output to create a smooth limiting function at both ends of a
   * boundary. The input velocity will be slowly clamped as the position gets too close to either
   * end of the boundary, but only if it is moving toward that boundary.
   *
   * @param velocity The velocity to limit
   * @param position The current position of the system
   * @param maxVelocity The maximum velocity. Output is guaranteed to be <= to this.
   * @param maxPosition The maximum position of the boundary. The boundary goes from 0-maxPosition
   * @param backwardLimitDistance The distance from the left end of the boundary to start the soft
   *     limit at
   * @param forwardLimitDistance The distance from both the right end of the boundary to start the
   *     soft limit at
   * @return The clamped velocity
   */
  public static double softLimitVelocity(
      double velocity,
      double position,
      double maxVelocity,
      double maxPosition,
      double backwardLimitDistance,
      double forwardLimitDistance) {
    velocity = clampMagnitude(velocity, maxVelocity);
    // Hard limits
    if (position <= 0) {
      if (velocity < 0) {
        return 0;
      } else {
        return velocity;
      }
    }
    if (position >= maxPosition) {
      if (velocity > 0) {
        return 0;
      } else {
        return velocity;
      }
    }

    /*
     * Determine which boundary to check based on the velocity direction.
     * Then we get the extent to which the value is in the boundary
     * and multiply that by the max
     */
    if (Math.signum(velocity) == -1) {
      // Left boundary check
      if (position < backwardLimitDistance) {
        final double scalar = getScalarPosition(position, 0, backwardLimitDistance);
        final double clamp = clampMagnitude(velocity, maxVelocity * scalar);
        return clamp;
      }
    } else {
      // Right boundary check
      final double rightBound = maxPosition - forwardLimitDistance;
      if (position > rightBound) {
        // Get the reciprocal because we want the slope to go the other way
        final double scalar = 1 - getScalarPosition(position, rightBound, maxPosition);
        final double clamp = clampMagnitude(velocity, maxVelocity * scalar);
        return clamp;
      }
    }

    return velocity;
  }

  /**
   * Gets the scalar position of a value within a range. For example, the position of 4 within 1 and
   * 7 would be 0.5
   *
   * @param value The value within the range
   * @param min The minimum value of the range
   * @param max The maximum value of the range
   * @return The position of the value within the range. If the input is outside of the range, will
   *     return extraneous results
   */
  public static double getScalarPosition(double value, double min, double max) {
    return (value - min) / (max - min);
  }

  /**
   * Clamps the magnitude of a value to a maximum only, but in both negative and positive
   * directions. The sign of the input will be retained.
   *
   * @param value The value to clamp
   * @param max The maximum magnitude
   * @return The clamped value
   */
  public static double clampMagnitude(double value, double max) {
    if (Math.abs(value) > max) {
      value = Math.copySign(max, value);
    }
    return value;
  }

  /**
   * Checks if an error value is within a margin
   *
   * @param measurement The measured value
   * @param goal The setpoint margin
   * @param margin The absolute margin to check
   * @return Whether the error is in the margin
   */
  public static boolean checkError(double measurement, double goal, double margin) {
    return checkError(goal - measurement, margin);
  }

  /**
   * Checks if an error value is within a margin
   *
   * @param error The error value
   * @param margin The absolute margin to check
   * @return Whether the error is in the margin
   */
  public static boolean checkError(double error, double margin) {
    return Math.abs(error) <= margin;
  }

  /**
   * Curves a value by raising it to a power, but retaining the original sign
   *
   * @param value The value to curve
   * @param magnitude The amount to curve the value by. Must be greater than 0. Values greater than
   *     1 will create a curve stronger than linear, and values less than one will create a curve
   *     that is weaker than linear
   * @return The curved value
   */
  public static double curve(double value, double magnitude) {
    return Math.copySign(Math.abs(Math.pow(value, magnitude)), value);
  }

  /**
   * Snaps a rotation to intervals
   *
   * @param rotation The input rotation. Should not be negative and should be wrapped.
   * @param interval The interval to snap to. Should not be negative and should be wrapped.
   * @param snapDirection Which direction to snap in
   * @return The snapped rotation
   */
  public static Rotation2d snapRotation(
      Rotation2d rotation, Rotation2d interval, SnapDirection snapDirection) {
    double val = rotation.getRadians() / interval.getRadians();
    switch (snapDirection) {
      case Positive:
        val = Math.ceil(val);
        break;
      case Negative:
        val = Math.floor(val);
        break;
      case Balanced:
        val = Math.round(val);
        break;
    }
    val *= interval.getRadians();
    return Rotation2d.fromRadians(val);
  }

  /** Different directions for snapping */
  public enum SnapDirection {
    Positive,
    Negative,
    Balanced,
  }

  /**
   * Calculates the moment of inertia of a flywheel
   *
   * @param mass The mass of the flywheel in kilograms
   * @param radius The radius of the flywheel in meters
   * @return The moment of inertia in jkgm^2
   */
  public static double calculateFlywheelMOI(double mass, double radius) {
    return 0.5 * mass * Math.pow(radius, 2);
  }
}
