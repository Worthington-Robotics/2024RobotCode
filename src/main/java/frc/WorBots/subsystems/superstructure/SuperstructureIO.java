// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.superstructure;

import frc.WorBots.util.HardwareUtils.TalonInputsPositional;

public interface SuperstructureIO {
  /** The superstructure inputs. */
  public static class SuperstructureIOInputs {
    public TalonInputsPositional elevator =
        new TalonInputsPositional("Superstructure", "Elevator Motor");
    public double elevatorPositionMeters = 0.0;
    public double elevatorVelocityMetersPerSec = 0.0;
    public double elevatorPercentageRaised = 0.0;
    public double elevatorCurrentAmps = 0.0;

    public TalonInputsPositional pivot = new TalonInputsPositional("Superstructure", "Pivot Motor");
    public double pivotPositionAbsRad = 0.0;
    public double pivotPositionRelRad = 0.0;
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

  /** Resets the elevator zero to the current position */
  public default void resetElevator() {}
}
