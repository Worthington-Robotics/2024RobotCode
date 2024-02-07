// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.shooter;

public interface ShooterIO {
  /** The shooter inputs that need to be updated every cycle. */
  public static class ShooterIOInputs {
    boolean isConnected = false;
    public double velocityRPMTop = 0;
    public double velocityRPMBottom = 0;

    public double feederWheelPositionRads = 0.0;
    public double feederWheelVelocityRadPerSec = 0.0;
    public double feederWheelCurrentAmps = 0.0;
    public double timeOfFlightDistanceMeters = 0.0;
  }

  /**
   * Sets the top flywheel's voltage.
   *
   * @param volts The voltage to be set.
   */
  public default void setTopFlywheelVolts(double volts) {}

  /**
   * Sets the bottom flywheel's voltage.
   *
   * @param volts The voltage to be set.
   */
  public default void setBottomFlywheelVolts(double volts) {}

  /**
   * Sets the feeder wheels voltage.
   *
   * @param volts The voltage to be set.
   */
  public default void setFeederWheelVoltage(double volts) {}

  /**
   * The current inputs that need to be updated.
   *
   * @param inputs The inputs to be modified.
   */
  public default void updateInputs(ShooterIOInputs inputs) {}
}
