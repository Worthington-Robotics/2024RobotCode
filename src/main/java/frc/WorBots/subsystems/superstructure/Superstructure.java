// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.superstructure;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.superstructure.SuperstructureIO.SuperstructureIOInputs;
import frc.WorBots.util.*;
import frc.WorBots.util.TunablePIDController.TunablePIDGains;
import frc.WorBots.util.TunablePIDController.TunableProfiledPIDController;
import frc.WorBots.util.TunablePIDController.TunableTrapezoidConstraints;
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
  private Supplier<Double> climbingVolts = () -> 0.0;
  private static final double firstCarriageRangeMeters[] = {0.0, Units.inchesToMeters(8.875)};
  private static final double secondCarriageRangeMeters[] = {0.0, Units.inchesToMeters(11.0)};
  private double firstCarriagePositionMeters;
  private double secondCarriagePositionMeters;

  private TunableProfiledPIDController pivotController =
      new TunableProfiledPIDController(
          new TunablePIDGains(tableName, "Pivot Gains"),
          new TunableTrapezoidConstraints(tableName, "Pivot Constraints"));
  private TunableProfiledPIDController elevatorController =
      new TunableProfiledPIDController(
          new TunablePIDGains(tableName, "Elevator Gains"),
          new TunableTrapezoidConstraints(tableName, "Elevator Constraints"));
  private ElevatorFeedforward elevatorFeedForward;
  private ArmFeedforward pivotFeedForward;

  // Constants
  private static final String tableName = "Superstructure";
  private static final double limitDistance = 0.25;

  /** The states that the superstructure can be in. */
  public enum SuperstructureState {
    POSE,
    SHOOTING,
    CLIMBING
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
      pivotController.setGains(50.0, 0, 0);
      pivotController.setConstraints(1.0, 1.0);
      elevatorController.setGains(185, 0.095, 0);
      elevatorController.setConstraints(2.0, 1.2);
      elevatorFeedForward = new ElevatorFeedforward(0.2, 0.0, 0.0);
      pivotFeedForward = new ArmFeedforward(0.0, 0.0, 0.0);
    } else { // Sim
      pivotController.setGains(50.0, 0, 0);
      pivotController.setConstraints(1.0, 1.0);
      elevatorController.setGains(185, 0.095, 0);
      elevatorController.setConstraints(2.0, 1.2);
      elevatorFeedForward = new ElevatorFeedforward(0.2, 0.0, 0.0);
      pivotFeedForward = new ArmFeedforward(0.0, 0.0, 0.0);
    }
    visualizer = new SuperstructureVisualizer("Superstructure");
    StatusPage.reportStatus(StatusPage.SUPERSTRUCTURE_SUBSYSTEM, true);
  }

  /** The function that runs once per cycle. */
  public void periodic() {
    io.updateInputs(inputs);

    // Log info
    Logger.getInstance().setSuperstructureInputs(inputs);
    Logger.getInstance().setSuperstructureMode(state.name());
    StatusPage.reportStatus(StatusPage.PIVOT_CONNECTED, inputs.pivotConnected);
    StatusPage.reportStatus(StatusPage.ELEVATOR_CONNECTED, inputs.elevator.isConnected);

    // Update tunables
    pivotController.update();
    elevatorController.update();

    firstCarriagePositionMeters =
        ((firstCarriageRangeMeters[1] - firstCarriageRangeMeters[0])
                * inputs.elevatorPercentageRaised)
            + firstCarriageRangeMeters[0];
    secondCarriagePositionMeters =
        (((secondCarriageRangeMeters[1] - secondCarriageRangeMeters[0])
                    * inputs.elevatorPercentageRaised)
                + secondCarriageRangeMeters[0])
            + firstCarriagePositionMeters
            + Units.inchesToMeters(1.0);

    if (DriverStation.isDisabled()) {
      setElevatorVoltage(0.0);
      setPivotVoltage(0.0);
    } else {
      switch (state) {
        case POSE:
          runPose(setpoint.getElevator(), setpoint.getPivot());
          break;
        case SHOOTING:
          runPose(0.0, shootingAngleRad.get());
          break;
        case CLIMBING:
          final double volts = climbingVolts.get();
          setElevatorVoltage(volts);
          setPivotVoltage(0.0);
          break;
      }
    }

    inputs.elevator.publish();
    inputs.pivot.publish();

    visualizer.update(
        VecBuilder.fill(
            inputs.pivotPositionRelRad + pivotAbsAngleRad,
            firstCarriagePositionMeters,
            secondCarriagePositionMeters,
            inputs.elevatorPositionMeters));
  }

  /**
   * Calculates elevator voltage from setpoint
   *
   * @param setpoint The elevator setpoint
   * @return The voltage for the elevator
   */
  private double calculateElevator(double setpoint) {
    final double elevatorVoltage =
        elevatorController.pid.calculate(inputs.elevatorPositionMeters, setpoint)
            + elevatorFeedForward.calculate(inputs.elevatorVelocityMetersPerSec);

    return elevatorVoltage;
  }

  /**
   * Calculates pivot voltage from setpoint
   *
   * @param setpoint The pivot setpoint
   * @return The voltage for the pivot
   */
  private double calculatePivot(double setpoint) {
    final double pivotVoltage =
        pivotController.pid.calculate(inputs.pivotPositionRelRad + pivotAbsAngleRad, setpoint)
            + pivotFeedForward.calculate(
                inputs.pivotPositionRelRad + pivotAbsAngleRad, inputs.pivotVelocityRadPerSec);

    return pivotVoltage;
  }

  /**
   * Runs a pose on the superstructure with full logging
   *
   * @param elevatorPose The desired elevator position
   * @param pivotPose The desired pivot angle
   */
  private void runPose(double elevatorPose, double pivotPose) {
    Logger.getInstance().setSuperstructureElevatorPosSetpoint(elevatorPose);
    Logger.getInstance().setSuperstructurePivotPosSetpoint(pivotPose);
    final double elevatorVoltage = calculateElevator(elevatorPose);
    final double pivotVoltage = calculatePivot(pivotPose);
    setElevatorVoltage(elevatorVoltage);
    setPivotVoltage(pivotVoltage);
  }

  /**
   * Sets the elevator voltage in the IO and also logs it
   *
   * @param volts The elevator voltage
   */
  private void setElevatorVoltage(double volts) {
    SmartDashboard.putNumber("Superstructure/Raw Elevator Setpoint", volts);

    // Do soft limiting
    volts =
        GeneralMath.softLimitVelocity(
            volts, inputs.elevatorPercentageRaised, 12.0, 1.0, limitDistance);

    // Do hard limiting based on limit switches
    if (inputs.bottomLimitReached && volts < 0.0) {
      volts = 0.0;
    }
    if (inputs.topLimitReached && volts > 0.0) {
      volts = 0.0;
    }

    setElevatorVoltageRaw(volts);
  }

  /**
   * Sets and logs the elevator voltage while bypassing software limits
   *
   * @param volts The elevator voltage
   */
  private void setElevatorVoltageRaw(double volts) {
    io.setElevatorVoltage(volts);
    Logger.getInstance().setSuperstructureElevatorVoltageSetpoint(volts);
  }

  /**
   * Sets the pivot voltage in the IO and also logs it
   *
   * @param volts The pivot voltage
   */
  private void setPivotVoltage(double volts) {
    // Pivot disabled for now
    // io.setPivotVoltage(volts);
    Logger.getInstance().setSuperstructurePivotVoltageSetpoint(volts);
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
   * Sets the desired voltage for manual climbing.
   *
   * @param volts The desired voltage.
   */
  public void setClimbingVolts(double volts) {
    climbingVolts = () -> volts;
  }

  /**
   * Sets the desired voltage for manual climbing.
   *
   * @param volts The desired voltage.
   */
  public void setClimbingVolts(Supplier<Double> supplier) {
    climbingVolts = supplier;
  }

  /**
   * Returns whether or not the superstructure is in position.
   *
   * @return True if in position, false if not.
   */
  public boolean isAtSetpoint() {
    return elevatorController.pid.atSetpoint() && pivotController.pid.atSetpoint();
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

  /**
   * Gets the height of the shooter in meters
   *
   * @return The shooter height
   */
  public double getShooterHeightMeters() {
    return inputs.elevatorPositionMeters + 0.6;
  }

  /**
   * Returns a command that will automatically find the elevator zero by moving it into the bottom
   * limit switch
   *
   * @return The command that will auto zero
   */
  public Command autoZero() {
    return this.runEnd(
            () -> {
              setElevatorVoltageRaw(-4.0);
            },
            () -> setElevatorVoltageRaw(0.0))
        .onlyWhile(() -> !inputs.bottomLimitReached)
        .andThen(this.runOnce(() -> io.resetElevator()));
  }
}
