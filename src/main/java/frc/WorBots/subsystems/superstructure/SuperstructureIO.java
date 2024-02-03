// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.superstructure;

public interface SuperstructureIO {
  /** The superstructure inputs. */
  public static class SuperstructureIOInputs {
    public double elevatorPositionMeters = 0.0;
    public double elevatorVelocityMetersPerSec = 0.0;
    public double elevatorTemp = 0.0;
    public double elevatorVoltage = 0.0;
    public boolean elevatorConnected = true;

    public double pivotPositionAbsRad = 0.0;
    public double pivotPositionRelRad = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotVoltageApplied = 0.0;
    public double pivotTemp = 0.0;
    public boolean pivotConnected = true;
  }

  /**
   * Sets the elevator motor's voltage.
   *
   * @param volts The volts to be set.
   */
  public default void setElevatorVoltage(double volts) {}

  /**
   * Sets the pivot motor's voltage.
   *
   * @param volts The volts to be set.
   */
  public default void setPivotVoltage(double volts) {}

  /**
   * Updates the current inputs of the superstructure subsystem.
   *
   * @param inputs The inputs to be updated.
   */
  public default void updateInputs(SuperstructureIOInputs inputs) {}
}
