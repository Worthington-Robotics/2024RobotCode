// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.control;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.WorBots.Constants;

/** Filter for swerve setpoints to avoid aggressive flipping */
public class SwerveSetpointFilter {
  /** The last demanded direction of the swerve module */
  private double lastDirection = 0.0;

  private final TrapezoidProfile profile = new TrapezoidProfile(new Constraints(1000.0, 1000.0));
  private final LinearFilter filter = LinearFilter.singlePoleIIR(0.02, Constants.ROBOT_PERIOD);

  public SwerveSetpointFilter() {}

  /** Calculate the filtered output of one module setpoint */
  public SwerveModuleState calculate(SwerveModuleState state) {
    // final double originalSign = Math.signum(state.speedMetersPerSecond);
    // state.speedMetersPerSecond = filter.calculate(state.speedMetersPerSecond);
    // if (Math.signum(state.speedMetersPerSecond) != originalSign) {
    //   state.angle = state.angle.plus(Rotation2d.fromDegrees(180));
    // }
    return state;
  }
}
