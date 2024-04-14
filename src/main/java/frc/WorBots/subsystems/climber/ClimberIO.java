// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.climber;

import frc.WorBots.util.HardwareUtils.TalonInputsPositional;

public interface ClimberIO {
  /** The climber inputs. */
  public static class ClimberIOInputs {
    public TalonInputsPositional climber =
        new TalonInputsPositional("Superstructure", "Climber Motor");
  }

  /**
   * Sets the climber motor's voltage.
   *
   * @param volts The volts to be set.
   */
  public default void setClimberVoltage(double volts) {}

  /**
   * Updates the current inputs of the climber subsystem.
   *
   * @param inputs The inputs to be updated.
   */
  public default void updateInputs(ClimberIOInputs inputs) {}
}
