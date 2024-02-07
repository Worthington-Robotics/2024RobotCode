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

  /**
   * This module represents one swerve module, which includes a turn motor, and drive motor.
   *
   * @param io The module IO to be ran with.
   * @param index The index of the module from 0-3.
   */
  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    if (Constants.getSim()) { // Sim constants
      driveFeedforward = new SimpleMotorFeedforward(0.116970, 0.133240);
      driveFeedbackGains.setGains(0.08, 0.0, 0.0);
      turnFeedbackGains.setGains(12.0, 0.0, 0.0);
    } else { // Real constants
      driveFeedforward = new SimpleMotorFeedforward(0.18868, 0.12825);
      driveFeedbackGains.setGains(0.08, 0.0, 0.0);
      turnFeedbackGains.setGains(9.0, 0.01, 0.0);
    }
    turnFeedback.pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void periodic() {
    driveFeedback.update();
    turnFeedback.update();
    io.updateInputs(inputs);
    StatusPage.reportStatus(StatusPage.SMODULE_PREFIX + index, inputs.isConnected);
  }

  /**
   * Calculates and sets the current motors to the provided state. It is recommended to first call
   * optimizeState() on the setpoint in order to have optimal control
   *
   * @param state The desired state.
   */
  public void runState(SwerveModuleState state) {
    io.setTurnVoltage(
        turnFeedback.pid.calculate(getAngle().getRadians(), state.angle.getRadians()));

    double velocityRadPerSec = state.speedMetersPerSecond / Units.inchesToMeters(2.0);
    io.setDriveVoltage(
        driveFeedforward.calculate(velocityRadPerSec)
            + driveFeedback.pid.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));
  }

  /**
   * Calculates the optimized version of a setpoint state
   *
   * @param state The setpoint state to optimize
   * @return The optimized state
   */
  public SwerveModuleState optimizeState(SwerveModuleState state) {
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Stray module correction
    optimizedState.speedMetersPerSecond *= Math.cos(turnFeedback.pid.getPositionError());

    return optimizedState;
  }

  /**
   * Gets the current state of the module.
   *
   * @return The state.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Sets both motors to 0 volts */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);
  }

  /**
   * Gets the current position of the drive motor.
   *
   * @return The position in meters.
   */
  public double getPositionMeters() {
    return inputs.drivePositionRad * Units.inchesToMeters(2.0);
  }

  /**
   * Gets the current velocity of the module.
   *
   * @return The velocity in meters per second.
   */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * Units.inchesToMeters(2.0);
  }

  /**
   * Gets the current angle of the module.
   *
   * @return The angle as a rotation.
   */
  public Rotation2d getAngle() {
    return new Rotation2d(inputs.turnAbsolutePositionRad);
  }
}
