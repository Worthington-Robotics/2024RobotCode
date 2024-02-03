// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.superstructure;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.superstructure.SuperstructureIO.SuperstructureIOInputs;
import frc.WorBots.util.*;
import java.util.function.Supplier;

/** The elevator and pivot of the robot, responsible for shooting and climbing. */
public class Superstructure extends SubsystemBase {
  private SuperstructureIO io;
  private SuperstructureIOInputs inputs = new SuperstructureIOInputs();
  private SuperstructureState state = SuperstructureState.POSE;
  private SuperstructureVisualizer visualizer;
  private SuperstructurePose.Preset setpoint = SuperstructurePose.Preset.HOME;
  private double pivotAbsAngleRad = 0.0;
  private Supplier<Double> shootingAngleRad = () -> 0.0;
  private static final double firstCarriageRangeMeters[] = {0.0, 0.0};
  private static final double secondCarriageRangeMeters[] = {0.0, 0.0};
  private double firstCarriagePositionMeters;
  private double secondCarriagePositionMeters;

  private ProfiledPIDController pivotController;
  private ProfiledPIDController elevatorController;
  private ElevatorFeedforward elevatorFeedForward;
  private ArmFeedforward pivotFeedForward;

  /** The states that the superstructure can be in. */
  public enum SuperstructureState {
    POSE,
    SHOOTING
  }

  /**
   * Constructs an instance of the superstructure.
   *
   * @param io The IO interface to be used.
   */
  public Superstructure(SuperstructureIO io) {
    this.io = io;
    io.updateInputs(inputs);
    pivotAbsAngleRad = inputs.pivotPositionAbsRad;
    if (!Constants.getSim()) { // Real
      pivotController = new ProfiledPIDController(50.0, 0, 0, new Constraints(1.0, 1.0));
      elevatorController = new ProfiledPIDController(8.0, 0, 0, new Constraints(1.0, 1.0));
      elevatorFeedForward = new ElevatorFeedforward(0.0, 0.0, 0.0);
      pivotFeedForward = new ArmFeedforward(0.0, 0.0, 0.0);
    } else { // Sim
      pivotController = new ProfiledPIDController(100, 0, 0, new Constraints(5.0, 5.0));
      elevatorController = new ProfiledPIDController(50.0, 0, 0, new Constraints(1.0, 1.0));
      elevatorFeedForward = new ElevatorFeedforward(0.0, 0.0, 0.0);
      pivotFeedForward = new ArmFeedforward(1.0, 1.0, 0.0);
    }
    visualizer = new SuperstructureVisualizer("Superstructure");
    StatusPage.reportStatus(StatusPage.SUPERSTRUCTURE_SUBSYSTEM, true);
  }

  /** The function that runs once per cycle. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().setSuperstructureInputs(inputs);
    Logger.getInstance().setSuperstructureMode(state.name());
    StatusPage.reportStatus(StatusPage.PIVOT_CONNECTED, inputs.pivotConnected);
    StatusPage.reportStatus(StatusPage.ELEVATOR_CONNECTED, inputs.elevatorConnected);

    firstCarriagePositionMeters =
        ((firstCarriageRangeMeters[0] - firstCarriageRangeMeters[1])
                * inputs.elevatorPercentageRaised)
            + firstCarriageRangeMeters[0];
    secondCarriagePositionMeters =
        ((secondCarriageRangeMeters[0] - secondCarriageRangeMeters[1])
                * inputs.elevatorPercentageRaised)
            + secondCarriageRangeMeters[0];

    switch (state) {
      case POSE:
        Logger.getInstance().setSuperstructureElevatorPosSetpoint(setpoint.getVecPose().get(0, 0));
        Logger.getInstance().setSuperstructurePivotPosSetpoint(setpoint.getVecPose().get(1, 0));
        double elevatorVoltage =
            elevatorController.calculate(
                    inputs.elevatorPositionMeters, setpoint.getVecPose().get(0, 0))
                + elevatorFeedForward.calculate(inputs.elevatorVelocityMetersPerSec);
        double pivotVoltage =
            pivotController.calculate(
                    inputs.pivotPositionRelRad + pivotAbsAngleRad, setpoint.getVecPose().get(1, 0))
                + pivotFeedForward.calculate(
                    inputs.pivotPositionRelRad + pivotAbsAngleRad, inputs.pivotVelocityRadPerSec);
        Logger.getInstance().setSuperstructurePivotVoltageSetpoint(pivotVoltage);
        Logger.getInstance().setSuperstructureElevatorVoltageSetpoint(elevatorVoltage);
        io.setElevatorVoltage(elevatorVoltage);
        io.setPivotVoltage(pivotVoltage);
        break;
      case SHOOTING:
        Logger.getInstance().setSuperstructureElevatorPosSetpoint(0.0);
        Logger.getInstance().setSuperstructurePivotPosSetpoint(shootingAngleRad.get());
        double elevatorVoltageShooting =
            elevatorController.calculate(inputs.elevatorPositionMeters, 0.0)
                + elevatorFeedForward.calculate(inputs.elevatorVelocityMetersPerSec);
        double pivotVoltageShooting =
            pivotController.calculate(
                    inputs.pivotPositionRelRad + pivotAbsAngleRad, shootingAngleRad.get())
                + pivotFeedForward.calculate(
                    inputs.pivotPositionRelRad + pivotAbsAngleRad, inputs.pivotVelocityRadPerSec);
        Logger.getInstance().setSuperstructurePivotVoltageSetpoint(pivotVoltageShooting);
        Logger.getInstance().setSuperstructureElevatorVoltageSetpoint(elevatorVoltageShooting);
        io.setElevatorVoltage(elevatorVoltageShooting);
        io.setPivotVoltage(pivotVoltageShooting);
        break;
    }
    if (DriverStation.isDisabled()) {
      io.setElevatorVoltage(0.0);
      io.setPivotVoltage(0.0);
    }
    visualizer.update(
        VecBuilder.fill(
            inputs.elevatorPositionMeters, inputs.pivotPositionRelRad + pivotAbsAngleRad));
  }

  /**
   * Sets the desired shooting angle.
   *
   * @param angle The desired angle in radians.
   */
  public void setShootingAngleRad(double angle) {
    shootingAngleRad = () -> angle;
  }

  /**
   * Sets the desired shooting angle.
   *
   * @param angle The desired angle in radians, as a supplier.
   */
  public void setShootingAngleRad(Supplier<Double> supplier) {
    shootingAngleRad = supplier;
  }

  /**
   * Returns whether or not the superstructure is in position.
   *
   * @return True if in position, false if not.
   */
  public boolean isAtSetpoint() {
    return elevatorController.atSetpoint() && pivotController.atSetpoint();
  }

  /**
   * Sets the desired pose of the subsystem.
   *
   * @param pose The desired pose.
   * @return The command.
   */
  public Command setPose(SuperstructurePose.Preset pose) {
    return this.runOnce(
            () -> {
              this.setpoint = pose;
            })
        .alongWith(Commands.waitUntil(this::isAtSetpoint));
  }

  /**
   * Sets the desired state of the superstructure subsystem.
   *
   * @param state The state to be set.
   * @return The command.
   */
  public Command setMode(SuperstructureState state) {
    return this.runOnce(
        () -> {
          this.state = state;
        });
  }

  /**
   * Sets the desired mode of the superstructure subsystem, not a command.
   *
   * @param state The state to be set.
   */
  public void setModeVoid(SuperstructureState state) {
    this.state = state;
  }

  /**
   * Gets the first and 2nd carriage positions based on where the percent the elevator is extended.
   *
   * @return The two values for each of the carriages.
   */
  public double[] getElevatorPositions() {
    return new double[] {firstCarriagePositionMeters, secondCarriagePositionMeters};
  }
}
