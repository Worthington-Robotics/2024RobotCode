// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.commands.UtilCommands;
import frc.WorBots.subsystems.superstructure.SuperstructureIO.SuperstructureIOInputs;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.Preset;
import frc.WorBots.util.debug.Logger;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.util.debug.TunablePIDController.TunablePIDGains;
import frc.WorBots.util.debug.TunablePIDController.TunableProfiledPIDController;
import frc.WorBots.util.debug.TunablePIDController.TunableTrapezoidConstraints;
import frc.WorBots.util.math.GeneralMath;
import java.util.function.Supplier;

/** The elevator and pivot of the robot, responsible for shooting and climbing. */
public class Superstructure extends SubsystemBase {
  private SuperstructureIO io;
  private SuperstructureIOInputs inputs = new SuperstructureIOInputs();

  /** The state of the superstructure for different control modes */
  private SuperstructureState state = SuperstructureState.DISABLED;

  /** The current setpoint pose of the superstructure */
  private SuperstructurePose.Preset setpoint = SuperstructurePose.Preset.HOME;

  /** The initial absolute position of the pivot */
  private double initZeroPoseRad = ABSOLUTE_ZERO_OFFSET;

  /**
   * Whether we have obtained a zero position reading from the absolute encoder. We only need to do
   * this once.
   */
  private boolean hasGottenZeroPosition = false;

  /** A supplier for the shooting angle in shooting mode */
  private Supplier<Double> shootingAngleRad = () -> 0.0;

  /** A supplier for the elevator volts in manual mode */
  private Supplier<Double> manualElevatorVolts = () -> 0.0;

  /** A supplier for the pivot volts in manual mode */
  private Supplier<Double> manualPivotVolts = () -> 0.0;

  private final TunableProfiledPIDController pivotController =
      new TunableProfiledPIDController(
          new TunablePIDGains(tableName, "Pivot Gains"),
          new TunableTrapezoidConstraints(tableName, "Pivot Constraints"));
  private final TunableProfiledPIDController elevatorController =
      new TunableProfiledPIDController(
          new TunablePIDGains(tableName, "Elevator Gains"),
          new TunableTrapezoidConstraints(tableName, "Elevator Constraints"));
  private ElevatorFeedforward elevatorFeedForward;
  private ArmFeedforward pivotFeedForward;

  // Constants
  private static final String tableName = "Superstructure";

  /** Limit distance for the elevator */
  private static final double ELEVATOR_LIMIT_DISTANCE = 0.28;

  /** Limit distance for the pivot in the forward direction */
  private static final double PIVOT_BACKWARD_LIMIT_DISTANCE = 0.90;

  /** Limit distance for the pivot in the backward direction */
  private static final double PIVOT_FORWARD_LIMIT_DISTANCE = 1.05;

  /** The offset for the pivot abs encoder, in radians */
  private static final double ABSOLUTE_ZERO_OFFSET = 0.2827;

  /** The minimum value for the absolute encoder */
  private static final double ABSOLUTE_MIN_VALUE = 0.265;

  /** The maximum value for the absolute encoder at startup */
  private static final double ABSOLUTE_MAX_VALUE = 0.9;

  /** The max angle the pivot can go to, in radians */
  public static final double PIVOT_MAX_ANGLE = 2.71;

  /** The max distance the elevator can go to, in meters */
  public static final double ELEVATOR_MAX_METERS = 0.2674420965;

  /** The offset from the zero needed for the pivot to be horizontal */
  public static final double PIVOT_HORIZONTAL_OFFSET = 0.36368;

  /** The error threshold for the elevator, in meters */
  private static final double ELEVATOR_THRESHOLD = 0.027;

  /** The error threshold for the pivot, in radians */
  private static final double PIVOT_THRESHOLD = Units.degreesToRadians(0.75);

  /** Multiplier for the pivot PID output when we are near the amp pose */
  private static final double PIVOT_OSCILLATION_MULTIPLIER = 0.05;

  private static final double PIVOT_OSCILLATION_RANGE = 0.05;

  /** The states that the superstructure can be in. */
  public enum SuperstructureState {
    POSE,
    SHOOTING,
    MANUAL,
    DISABLED,
  }

  /**
   * Constructs an instance of the superstructure.
   *
   * @param io The IO interface to be used.
   */
  public Superstructure(SuperstructureIO io) {
    this.io = io;
    if (RobotBase.isReal()) { // Real
      pivotController.setGains(9.6, 0.12, 0);
      pivotController.setConstraints(11, 18);
      pivotFeedForward = new ArmFeedforward(0.045, 0.26, 0.01);

      elevatorController.setGains(130, 0.2, 0);
      elevatorController.setConstraints(2.0, 1.65);
      elevatorFeedForward = new ElevatorFeedforward(0.0, 0.0, 0.0);
    } else { // Sim
      pivotController.setGains(50, 0.25, 0);
      pivotController.setConstraints(12, 8);
      pivotFeedForward = new ArmFeedforward(0.1, 0.3, 0.0);

      elevatorController.setGains(160, 0.00, 0);
      elevatorController.setConstraints(1.0, 1.65);
      elevatorFeedForward = new ElevatorFeedforward(0.1, 0.0, 0.0);
    }
    pivotController.pid.setTolerance(PIVOT_THRESHOLD);
    elevatorController.pid.setTolerance(ELEVATOR_THRESHOLD);
    StatusPage.reportStatus(StatusPage.SUPERSTRUCTURE_SUBSYSTEM, true);
  }

  /** The function that runs once per cycle. */
  public void periodic() {
    io.updateInputs(inputs);

    // Get our absolute zero position if we haven't already
    if (!hasGottenZeroPosition) {
      // If we have an invalid value from the absolute encoder, fallback to the stow
      // position offset
      if (inputs.pivotPositionAbsRad >= ABSOLUTE_MIN_VALUE
          && inputs.pivotPositionAbsRad < ABSOLUTE_MAX_VALUE) {
        initZeroPoseRad = inputs.pivotPositionAbsRad;
        hasGottenZeroPosition = true;
      }
    }

    // Log info
    Logger.getInstance().setSuperstructureInputs(inputs);
    Logger.getInstance().setSuperstructureMode(state.name());
    Logger.getInstance().setSuperstructurePivotFusedRad(getPivotPoseRads());
    Logger.getInstance().setSuperstructureAtSetpoint(isAtSetpoint());
    Logger.getInstance().setSuperstructureInHandoff(inHandoff());
    StatusPage.reportStatus(StatusPage.PIVOT_CONNECTED, inputs.pivot.isConnected);
    StatusPage.reportStatus(StatusPage.ELEVATOR_CONNECTED, inputs.elevator.isConnected);

    // Update tunables
    pivotController.update();
    elevatorController.update();

    if (DriverStation.isDisabled()) {
      setElevatorVoltageRaw(0.0);
      setPivotVoltageRaw(0.0);
    } else {
      switch (state) {
        case DISABLED:
          setElevatorVoltageRaw(0.0);
          setPivotVoltageRaw(0.0);
          break;
        case POSE:
          runPose(setpoint.getElevator(), setpoint.getPivot());
          break;
        case SHOOTING:
          // Shoot with the elevator at the bottom and the pivot where it needs to be
          runPose(0.0, shootingAngleRad.get());
          break;
        case MANUAL:
          final double volts = manualElevatorVolts.get();
          setElevatorVoltage(volts);
          double pivotVolts = manualPivotVolts.get();
          // pivotVolts += calculatePivotFeedforward();
          pivotVolts += 0.12;
          setPivotVoltage(pivotVolts);
          break;
      }
    }

    inputs.elevator.publish();
    inputs.pivot.publish();
  }

  /**
   * Calculates elevator voltage from setpoint
   *
   * @param setpoint The elevator setpoint
   * @return The voltage for the elevator
   */
  private double calculateElevator(double setpoint) {
    // Clamp the setpoint
    setpoint = MathUtil.clamp(setpoint, 0.0, ELEVATOR_MAX_METERS);
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
    // Clamp the setpoint
    setpoint = MathUtil.clamp(setpoint, 0.0, PIVOT_MAX_ANGLE);
    double pivotVoltage =
        pivotController.pid.calculate(getPivotPoseRads(), setpoint) + calculatePivotFeedforward();

    // Anti-oscillation for the amp pose by reducing output
    if (atAmpPose()) {
      pivotVoltage *= PIVOT_OSCILLATION_MULTIPLIER;
    }

    return pivotVoltage;
  }

  /**
   * Calculate the feedforward value for the pivot
   *
   * @return The feedforward value to be added to the control output
   */
  private double calculatePivotFeedforward() {
    final double adjusted = getPivotPoseRads() - PIVOT_HORIZONTAL_OFFSET;
    final double out = pivotFeedForward.calculate(adjusted, inputs.pivot.velocityRadsPerSec);
    return out;
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
    io.setPivotVoltage(pivotVoltage);
  }

  /**
   * Sets the elevator voltage in the IO and also logs it
   *
   * @param volts The elevator voltage
   */
  private void setElevatorVoltage(double volts) {
    Logger.getInstance().setSuperstructureElevatorVoltageSetpoint(volts);

    // Do soft limiting
    volts =
        GeneralMath.softLimitVelocity(
            volts, inputs.elevatorPercentageRaised, 12.0, 1.0, ELEVATOR_LIMIT_DISTANCE);

    // Do hard limiting based on limit switches
    if (inputs.bottomLimitReached && volts < 0.0) {
      volts = 0.0;
    }
    if (inputs.topLimitReached && volts > 0.0) {
      volts = 0.0;
    }

    volts = MathUtil.clamp(volts, -12, 12);

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
    volts =
        GeneralMath.softLimitVelocity(
            volts,
            getPivotPoseRads(),
            6.5,
            PIVOT_MAX_ANGLE,
            PIVOT_BACKWARD_LIMIT_DISTANCE,
            PIVOT_FORWARD_LIMIT_DISTANCE);

    setPivotVoltageRaw(volts);
  }

  /**
   * Sets and logs the pivot voltage while bypassing software limits
   *
   * @param volts The pivot voltage
   */
  private void setPivotVoltageRaw(double volts) {
    io.setPivotVoltage(volts);
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
   * Sets the desired voltage for manual elevator
   *
   * @param volts The desired voltage
   */
  public void setManualElevatorVolts(double volts) {
    manualElevatorVolts = () -> volts;
  }

  /**
   * Sets the desired voltage for manual elevator
   *
   * @param volts The desired voltage
   */
  public void setManualElevatorVolts(Supplier<Double> supplier) {
    manualElevatorVolts = supplier;
  }

  /**
   * Sets the desired voltage for manual pivoting
   *
   * @param volts The desired voltage
   */
  public void setManualPivotVolts(double volts) {
    manualPivotVolts = () -> volts;
  }

  /**
   * Sets the desired voltage for manual pivoting
   *
   * @param volts The desired voltage
   */
  public void setManualPivotVolts(Supplier<Double> supplier) {
    manualPivotVolts = supplier;
  }

  /**
   * Returns whether or not the superstructure is in position
   *
   * @return True if in position, false if not
   */
  public boolean isAtSetpoint() {
    return isElevatorAtSetpoint() && isPivotAtSetpoint();
  }

  /**
   * Checks if the elevator is at it's setpoint
   *
   * @return True if the elevator is at the setpoint
   */
  public boolean isElevatorAtSetpoint() {
    double setpoint = 0.0;
    /*
     * We check the setpoint manually instead of using the PID because the PID might
     * not have its setpoint yet
     */
    if (isShooting()) {
      setpoint = 0.0;
    } else if (isInPoseMode()) {
      setpoint = this.setpoint.getElevator();
    }
    return GeneralMath.checkError(inputs.elevatorPositionMeters, setpoint, ELEVATOR_THRESHOLD);
  }

  /**
   * Checks if the pivot is at it's setpoint
   *
   * @return True if the pivot is at the setpoint
   */
  public boolean isPivotAtSetpoint() {
    double setpoint = 0.0;
    /*
     * We check the setpoint manually instead of using the PID because the PID might
     * not have its setpoint yet
     */
    if (isShooting()) {
      setpoint = shootingAngleRad.get();
    } else if (isInPoseMode()) {
      setpoint = this.setpoint.getPivot();
      // Amp will never reach setpoint due to anti-oscillation
      if (this.setpoint.equals(Preset.AMP)) {
        setpoint -= PIVOT_OSCILLATION_RANGE;
      }
    }
    return GeneralMath.checkError(getPivotPoseRads(), setpoint, PIVOT_THRESHOLD);
  }

  /**
   * Sets the desired pose of the subsystem and waits for it to get there. Also sets the
   * superstructure to pose mode.
   *
   * @param pose The desired pose.
   * @return The command, exits when superstructure is at the desired pose.
   */
  public Command goToPose(SuperstructurePose.Preset pose) {
    return UtilCommands.optimalSequence(
            this.runOnce(
                () -> {
                  this.setPose(pose);
                }),
            Commands.waitUntil(() -> isAtSetpoint()))
        .finallyDo(
            () -> {
              this.setModeVoid(SuperstructureState.POSE);
            });
  }

  /**
   * Sets the pose of the superstructure to a pose and also sets it to pose mode
   *
   * @param pose The pose to set
   */
  public void setPose(SuperstructurePose.Preset pose) {
    this.setModeVoid(SuperstructureState.POSE);
    this.setpoint = pose;
  }

  /**
   * Sets the desired state of the superstructure subsystem.
   *
   * @param state The state to be set.
   * @return The command, instantly exits.
   */
  public Command setMode(SuperstructureState state) {
    return this.runOnce(
        () -> {
          setModeVoid(state);
        });
  }

  /**
   * Sets the desired mode of the superstructure subsystem, not a command.
   *
   * @param state The state to be set.
   */
  public void setModeVoid(SuperstructureState state) {
    if (!state.equals(this.state)) {
      // Zero out manual volts when entering manual mode
      if (state.equals(SuperstructureState.MANUAL)) {
        setManualElevatorVolts(0.0);
        setManualPivotVolts(0.0);
      }

      // Reset controller integral and derivative term
      pivotController.pid.reset(getPivotPoseRads());
    }
    this.state = state;
  }

  /**
   * Checks if the superstructure is following a certain pose
   *
   * @param pose The pose to check
   * @return If the superstructure is in pose mode and following the pose
   */
  public boolean isInPose(SuperstructurePose.Preset pose) {
    return this.state == SuperstructureState.POSE && this.setpoint.equals(pose);
  }

  /**
   * Gets the fused angle of the pivot
   *
   * @return The fused angle with absolute and relative readings
   */
  public double getPivotPoseRads() {
    return inputs.pivotPositionRelRad + initZeroPoseRad - ABSOLUTE_ZERO_OFFSET;
  }

  /**
   * Gets the current pose preset of the superstructure
   *
   * @return The current pose
   */
  public Preset getCurrentPose() {
    return this.setpoint;
  }

  /**
   * Gets the percentage that the elevator is raised
   *
   * @return The percentage, from 0 to 1
   */
  public double getElevatorPercentageRaised() {
    return inputs.elevatorPercentageRaised;
  }

  /**
   * Checks if the superstructure is currently near a pose position. This does not check if it is in
   * pose mode or even has that pose set, but only if it's current position is within the given
   * tolerances of the pose
   *
   * @param pose The pose to check
   * @param elevatorTolerance The tolerance in meters for the elevator
   * @param pivotTolerance The tolerance in radians for the pivot
   * @return Whether the current pose is within both tolerances
   */
  public boolean isNearPose(
      SuperstructurePose.Preset pose, double elevatorTolerance, double pivotTolerance) {
    return GeneralMath.checkError(
            pose.getElevator(), inputs.elevatorPositionMeters, elevatorTolerance)
        && GeneralMath.checkError(pose.getPivot(), getPivotPoseRads(), pivotTolerance);
  }

  /**
   * Checks if the superstructure is in shooting mode
   *
   * @return True if the superstructure is in shooting mode
   */
  public boolean isShooting() {
    return this.state == SuperstructureState.SHOOTING;
  }

  /**
   * Checks if the superstructure is in pose mode
   *
   * @return True if the superstructure is in pose mode
   */
  public boolean isInPoseMode() {
    return this.state == SuperstructureState.POSE;
  }

  /**
   * Checks if the superstructure is close enough to the handoff pose to be able to handoff
   *
   * @return Whether we are near handoff
   */
  public boolean inHandoff() {
    return this.isNearPose(Preset.HANDOFF, 0.02, Units.degreesToRadians(2.8));
  }

  /**
   * Checks if the superstructure is close enough to the stow pose or below
   *
   * @return Whether we are near stow
   */
  public boolean inStow() {
    final boolean isBelow =
        inputs.elevatorPositionMeters < 0.01 && getPivotPoseRads() <= Preset.STOW.getPivot();
    return isBelow || this.isNearPose(Preset.STOW, 0.015, Units.degreesToRadians(1.9));
  }

  private boolean atAmpPose() {
    return isInPose(Preset.AMP)
        && isNearPose(Preset.AMP, ELEVATOR_THRESHOLD, PIVOT_OSCILLATION_RANGE);
  }
}
