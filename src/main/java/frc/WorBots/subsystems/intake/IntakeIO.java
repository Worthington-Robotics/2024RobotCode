// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.intake;

import frc.WorBots.util.HardwareUtils.TalonInputsPositional;

public interface IntakeIO {
  /** The inputs to the intake. */
  public static class IntakeIOInputs {
    TalonInputsPositional motor = new TalonInputsPositional("Intake", "Motor");
    double timeOfFlightDistanceMeters = 0.0;
    boolean isConnected = false;
  }

  /**
   * Updates the current inputs of the intake.
   *
   * @param inputs The inpus to be updated.
   */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * Sets the voltage of the intake motor.
   *
   * @param volts The voltage.
   */
  public default void setIntakeVoltage(double volts) {}
}
