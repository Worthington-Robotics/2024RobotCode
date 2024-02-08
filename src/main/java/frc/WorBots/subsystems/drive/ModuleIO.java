// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.drive;

import frc.WorBots.util.HardwareUtils.TalonInputsPositional;

public interface ModuleIO {
  /** The module inputs that need to be updated once per cycle */
  public static class ModuleIOInputs {
    public TalonInputsPositional drive;

    public TalonInputsPositional turn;
    public double turnAbsolutePositionRad = 0.0;

    public boolean isConnected = false;

    public ModuleIOInputs(int index) {
      drive = new TalonInputsPositional("Drive", "Module " + index + " Drive Motor");
      turn = new TalonInputsPositional("Drive", "Module " + index + " Turn Motor");
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public default void setTurnVoltage(double volts) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}
}
