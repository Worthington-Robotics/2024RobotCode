// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots;

import edu.wpi.first.wpilibj.RobotBase;

/** Miscellaneous global constants for the robot and code */
public class Constants {
  /**
   * Gets whether the robot is running in a simulation or is real
   *
   * @return True if the robot is simulated, false otherwise
   */
  public static final boolean getSim() {
    return RobotBase.isSimulation();
  }

  /** The CAN bus name used for swerve devices and pigeon */
  public static final String SWERVE_CAN_BUS = "Swerve";
}
