// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/** Miscellaneous global constants for the robot and code */
public class Constants {
  /** Whether the robot is in competition mode */
  public static final boolean IS_COMP = DriverStation.isFMSAttached() && false;

  /** Whether to enable debugging routines in the auto selector */
  public static final boolean ENABLE_DEBUG_ROUTINES = true;

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

  /** The robot update period in seconds */
  public static final double ROBOT_PERIOD = 0.02;

  /** The width of the robot in meters */
  public static final double ROBOT_WIDTH = Units.inchesToMeters(26);

  /** The length of the robot in meters */
  public static final double ROBOT_LENGTH = Units.inchesToMeters(26);
}
