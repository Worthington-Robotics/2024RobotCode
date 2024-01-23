// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.drive.ModuleIO.ModuleIOInputs;
import frc.WorBots.util.StatusPage;
import frc.WorBots.util.TunablePIDController;
import frc.WorBots.util.TunablePIDController.TunablePIDGains;

public class Module {
  private final int index;
  private final ModuleIO io;
  private final ModuleIOInputs inputs = new ModuleIOInputs();

  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
  private static TunablePIDGains driveFeedbackGains =
      new TunablePIDGains("Drive/Gains", "SModule Drive Feedback");
  private TunablePIDController driveFeedback = new TunablePIDController(driveFeedbackGains);
  private static TunablePIDGains turnFeedbackGains =
      new TunablePIDGains("Drive/Gains", "SModule Turn Feedback");
  private TunablePIDController turnFeedback = new TunablePIDController(turnFeedbackGains);

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    if (Constants.getSim()) {
      driveFeedforward = new SimpleMotorFeedforward(0.116970, 0.133240);
      driveFeedbackGains.setGains(0.9, 0.0, 0.0);
      turnFeedbackGains.setGains(23.0, 0.0, 0.0);
    } else {
      driveFeedforward = new SimpleMotorFeedforward(0.18868, 0.12825);
      driveFeedbackGains.setGains(0.08, 0.0, 0.0);
      turnFeedbackGains.setGains(4.0, 0.0, 0.0);
    }
    turnFeedback.pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void periodic() {
    driveFeedback.update();
    turnFeedback.update();
    io.updateInputs(inputs);
    StatusPage.reportStatus(StatusPage.SMODULE_PREFIX + index, inputs.isConnected);
  }

  public SwerveModuleState runState(SwerveModuleState state) {
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    io.setTurnVoltage(
        turnFeedback.pid.calculate(getAngle().getRadians(), optimizedState.angle.getRadians()));

    optimizedState.speedMetersPerSecond *= Math.cos(turnFeedback.pid.getPositionError());

    double velocityRadPerSec = optimizedState.speedMetersPerSecond / Units.inchesToMeters(2.0);
    io.setDriveVoltage(
        driveFeedforward.calculate(velocityRadPerSec)
            + driveFeedback.pid.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));

    return optimizedState;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);
  }

  public double getPositionMeters() {
    return inputs.drivePositionRad * Units.inchesToMeters(2.0);
  }

  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * Units.inchesToMeters(2.0);
  }

  public Rotation2d getAngle() {
    return new Rotation2d(inputs.turnAbsolutePositionRad);
  }
}
