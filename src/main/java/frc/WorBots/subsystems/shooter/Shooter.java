// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.shooter;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.util.debug.TunablePIDController;
import frc.WorBots.util.debug.TunablePIDController.TunablePIDGains;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase { // 532 rpm/v
  private ShooterIO io;
  private ShooterIOInputs inputs = new ShooterIOInputs();
  private boolean hasGamePiece = false;
  private double topFlywheelRPM = 0.0;
  private double bottomFlywheelRPM = 0.0;
  private double feederWheelVolts = 0.0;

  // Logging classes
  private static final String shooterTableName = "Shooter";
  private NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private NetworkTable shooterTable = instance.getTable(shooterTableName);
  private DoublePublisher topFlywheelSpeedPub =
      shooterTable.getDoubleTopic("Top Flywheel RPM").publish();
  private DoublePublisher bottomFlywheelSpeedPub =
      shooterTable.getDoubleTopic("Bottom Flywheel RPM").publish();
  private DoublePublisher topFlywheelSetpointPub =
      shooterTable.getDoubleTopic("Top Flywheel Setpoint").publish();
  private DoublePublisher bottomFlywheelSetpointPub =
      shooterTable.getDoubleTopic("Bottom Flywheel Setpoint").publish();
  private DoublePublisher timeOfFlightDistancePub =
      shooterTable.getDoubleTopic("Time of Flight Distance").publish();
  private BooleanPublisher hasGamePiecePub =
      shooterTable.getBooleanTopic("Has Game Piece").publish();

  // Constants
  /** Distance threshold for the ToF */
  private static final double DISTANCE_THRESHOLD = 0.075;

  /**
   * Threshold for backwards wheel speed where the PID control will allow the motors to coast down
   * instead of violently braking. In volts.
   */
  private static final double COAST_DOWN_THRESHOLD = -0.3;

  /** The default idle speed for the flywheels */
  private static final double IDLE_SPEED = 500.0;

  /** The voltage to run the feeder wheels at when feeding */
  private static final double FEED_VOLTS = -2.0;

  private static final String tableName = "Shooter";

  private TunablePIDController topFlywheelController =
      new TunablePIDController(new TunablePIDGains(tableName, "Top Flywheel Gains"));
  private TunablePIDController bottomFlywheelController =
      new TunablePIDController(new TunablePIDGains(tableName, "Bottom Flywheel Gains"));
  private TunablePIDController feederWheelController =
      new TunablePIDController(new TunablePIDGains(tableName, "Feeder Wheel Gains"));
  private SimpleMotorFeedforward topFlywheelFeedForward;
  private SimpleMotorFeedforward bottomFlywheelFeedforward;

  /**
   * The shooter subsystem, responsible for shooting into the speaker.
   *
   * @param io The IO inputs.
   */
  public Shooter(ShooterIO io) {
    this.io = io;

    if (!Constants.getSim()) { // Real
      topFlywheelController.setGains(0.005, 0.0, 0);
      bottomFlywheelController.setGains(0.005, 0.0, 0);
      topFlywheelFeedForward = new SimpleMotorFeedforward(0.0, 0.002);
      bottomFlywheelFeedforward = new SimpleMotorFeedforward(0.0, 0.002);
      feederWheelController.setGains(0.0, 0.0, 0.0);
      topFlywheelController.pid.setTolerance(120);
      bottomFlywheelController.pid.setTolerance(120);
    } else { // Sim
      topFlywheelController.setGains(0.00050, 0, 0);
      bottomFlywheelController.setGains(0.00050, 0, 0);
      topFlywheelFeedForward = new SimpleMotorFeedforward(0, 0.002);
      bottomFlywheelFeedforward = new SimpleMotorFeedforward(0.0, 0.002);
      feederWheelController.setGains(1.0, 0.0, 0.0);
      topFlywheelController.pid.setTolerance(120);
      bottomFlywheelController.pid.setTolerance(120);
    }

    StatusPage.reportStatus(StatusPage.SHOOTER_SUBSYSTEM, true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Update logging
    topFlywheelSpeedPub.set(inputs.velocityRPMTop);
    bottomFlywheelSpeedPub.set(inputs.velocityRPMBottom);
    topFlywheelSetpointPub.set(topFlywheelRPM);
    bottomFlywheelSetpointPub.set(bottomFlywheelRPM);
    timeOfFlightDistancePub.set(inputs.timeOfFlightDistanceMeters);
    hasGamePiecePub.set(hasGamePiece);

    // Update tunables
    topFlywheelController.update();
    bottomFlywheelController.update();
    feederWheelController.update();

    // Check if we have gamepiece
    hasGamePiece = inputs.timeOfFlightDistanceMeters < DISTANCE_THRESHOLD;

    if (DriverStation.isDisabled()) { // Set voltages to 0 if we are disabled.
      io.setTopFlywheelVolts(0.0);
      io.setBottomFlywheelVolts(0.0);
      io.setFeederWheelVoltage(0.0);
      // Clear setpoints
      topFlywheelRPM = 0.0;
      bottomFlywheelRPM = 0.0;
      feederWheelVolts = 0.0;
    } else {
      // Calculate the desired voltages based on the setpoints
      if (topFlywheelRPM > 0.0) {
        double setpoint =
            topFlywheelController.pid.calculate(inputs.velocityRPMTop, topFlywheelRPM)
                + topFlywheelFeedForward.calculate(topFlywheelRPM);
        if (setpoint < COAST_DOWN_THRESHOLD) {
          setpoint = 0.0;
        }
        io.setTopFlywheelVolts(setpoint);
      } else {
        io.setTopFlywheelVolts(0);
      }
      if (bottomFlywheelRPM > 0.0) {
        double setpoint =
            bottomFlywheelController.pid.calculate(inputs.velocityRPMBottom, bottomFlywheelRPM)
                + bottomFlywheelFeedforward.calculate(bottomFlywheelRPM);
        if (setpoint < COAST_DOWN_THRESHOLD) {
          setpoint = 0.0;
        }
        io.setBottomFlywheelVolts(setpoint);
      } else {
        io.setBottomFlywheelVolts(0.0);
      }

      // If we want to move the feeder wheel.
      io.setFeederWheelVoltage(feederWheelVolts);
    }

    inputs.feederWheel.publish();
    inputs.top.publish();
    inputs.bottom.publish();

    StatusPage.reportStatus(StatusPage.SHOOTER_CONNECTED, inputs.isConnected);
  }

  /**
   * A function that spins up the flywheels and returns when at the desired speed
   *
   * @param topRPM The desired RPM for both flywheels
   * @return The command, exits when flywheels are up to speed
   */
  public Command spinToSpeed(double rpm) {
    return spinToSpeed(rpm, rpm);
  }

  public Command spinToSpeed(Supplier<Double> supplier) {
    return spinToSpeed(supplier.get());
  }

  /**
   * A function that spins up the flywheels and returns when at the desired speed
   *
   * @param topRPM The desired top RPM
   * @param bottomRPM The desired bottom RPM
   * @return The command, exits when flywheels are up to speed
   */
  public Command spinToSpeed(double topRPM, double bottomRPM) {
    return this.setSpeed(bottomRPM).alongWith(Commands.waitUntil(this::isAtSetpoint));
  }

  public void setSpeedVoid(double rpm) {
    topFlywheelRPM = rpm;
    bottomFlywheelRPM = rpm;
  }

  public Command setSpeed(double rpm) {
    return this.runOnce(
        () -> {
          setSpeedVoid(rpm);
        });
  }

  public Command setSpeedContinuous(double rpm) {
    return this.run(
        () -> {
          setSpeedVoid(rpm);
        });
  }

  /**
   * Sets the raw volts of the feeder wheels. Negative voltages feed forward.
   *
   * @param volts The volts to set
   */
  public void setRawFeederVolts(double volts) {
    feederWheelVolts = volts;
  }

  /**
   * Returns a command that sets the raw volts of the feeder wheels. Does not require the subsystem.
   *
   * @param volts The volts to set
   * @return The command to run
   */
  public Command setRawFeederVoltsCommand(double volts) {
    return Commands.runOnce(
        () -> {
          feederWheelVolts = volts;
        });
  }

  /**
   * Returns a command that feeds the game piece into the shooter and automatically stops the feeder
   * when the command is interrupted
   *
   * @return The command to run
   */
  public Command feed() {
    return Commands.startEnd(() -> feederWheelVolts = FEED_VOLTS, () -> feederWheelVolts = 0.0);
  }

  /**
   * Returns a command that idles the flywheels
   *
   * @return The command to run
   */
  public Command idleCommand() {
    return this.setSpeedContinuous(IDLE_SPEED);
  }

  /**
   * Sets the flywheel desired RPM's to 0.
   *
   * @return The command.
   */
  public Command stopFlywheels() {
    return this.runOnce(
        () -> {
          topFlywheelRPM = 0;
          bottomFlywheelRPM = 0;
        });
  }

  /**
   * Gets wether or not the flwheels are at their desired setpoint.
   *
   * @return true if is at setpoint, false otherwise.
   */
  public boolean isAtSetpoint() {
    return topFlywheelController.pid.atSetpoint() && bottomFlywheelController.pid.atSetpoint();
  }

  /**
   * Reads the time of flight to detect if the robot has a game piece in the shooter.
   *
   * @return True if we have a gamepiece, otherwise false.
   */
  public boolean hasGamePiece() {
    return hasGamePiece;
  }
}
