// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.drive;

public interface GyroIO {
  /** The inputs of the gyro */
  public static class GyroIOInputs {
    public boolean connected = false;
    public double rollPositionRad = 0.0;
    public double pitchPositionRad = 0.0;
    public double yawPositionRad = 0.0;
    public double rollVelocityRadPerSec = 0.0;
    public double pitchVelocityRadPerSec = 0.0;
    public double yawVelocityRadPerSec = 0.0;
  }

  /**
   * Updates the current inputs of the selected IO implementation.
   *
   * @param inputs The inputs to be modified.
   */
  public default void updateInputs(GyroIOInputs inputs) {}

  /**
   * Set the rotational velocity (in rads/sec) that the drive is expected to be moving at so that
   * the simulated gyro can work
   */
  public default void setExpectedPitchVelocity(double vPitch) {}

  /**
   * Set the rotational velocity (in rads/sec) that the drive is expected to be moving at so that
   * the simulated gyro can work
   */
  public default void setExpectedRollVelocity(double vRoll) {}

  /**
   * Set the rotational velocity (in rads/sec) that the drive is expected to be moving at so that
   * the simulated gyro can work
   */
  public default void setExpectedYawVelocity(double vYaw) {}
}