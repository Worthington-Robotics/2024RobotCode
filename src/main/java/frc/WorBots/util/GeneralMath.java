// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util;

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
    // Hard limits
    if (position < 0 && Math.signum(velocity) == -1) {
      return 0;
    }
    if (position > maxPosition && Math.signum(velocity) == 1) {
      return 0;
    }

    velocity = clampMagnitude(velocity, maxVelocity);

    /*
     * Determine which boundary to check based on the velocity direction.
     * Then we get the extent to which the value is in the boundary
     * and multiply that by the max
     */
    if (Math.signum(velocity) == -1) {
      // Left boundary check
      if (position < limitDistance) {
        final double scalar = getScalarPosition(position, 0, limitDistance);
        final double clamp = clampMagnitude(velocity, maxVelocity * scalar);
        return clamp;
      }
    } else {
      // Right boundary check
      final double rightBound = maxPosition - limitDistance;
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
}
