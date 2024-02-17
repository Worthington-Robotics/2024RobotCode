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

public class Shooter extends SubsystemBase { // 532 rpm/v
  private ShooterIO io;
  private ShooterIOInputs inputs = new ShooterIOInputs();
  private boolean hasGamePiece = false;
  private boolean shouldIncrement = false;
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
  private BooleanPublisher shouldIncrementPub =
      shooterTable.getBooleanTopic("Should Increment").publish();
  private BooleanPublisher hasGamePiecePub =
      shooterTable.getBooleanTopic("Has Game Piece").publish();

  // Constants
  private static final double increasePositionRads = 2 * Math.PI;
  private static final double distanceThreshold = 0.065;
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
      topFlywheelController.setGains(0.006, 0.0, 0);
      bottomFlywheelController.setGains(0.006, 0.0, 0);
      topFlywheelFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
      bottomFlywheelFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
      feederWheelController.setGains(0.0, 0.0, 0.0);
      topFlywheelController.pid.setTolerance(80);
      bottomFlywheelController.pid.setTolerance(80);
    } else { // Sim
      topFlywheelController.setGains(1, 0, 0);
      bottomFlywheelController.setGains(1, 0, 0);
      topFlywheelFeedForward = new SimpleMotorFeedforward(0, 0);
      bottomFlywheelFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
      feederWheelController.setGains(1.0, 0.0, 0.0);
      topFlywheelController.pid.setTolerance(50);
      bottomFlywheelController.pid.setTolerance(50);
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
    shouldIncrementPub.set(shouldIncrement);
    hasGamePiecePub.set(hasGamePiece);

    // Update tunables
    topFlywheelController.update();
    bottomFlywheelController.update();
    feederWheelController.update();

    // Check if we have gamepiece
    hasGamePiece = inputs.timeOfFlightDistanceMeters < distanceThreshold;

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
        io.setTopFlywheelVolts(
            topFlywheelController.pid.calculate(inputs.velocityRPMTop, topFlywheelRPM)
                + topFlywheelFeedForward.calculate(topFlywheelRPM));
      } else {
        io.setTopFlywheelVolts(0);
      }
      if (bottomFlywheelRPM > 0.0) {
        io.setBottomFlywheelVolts(
            bottomFlywheelController.pid.calculate(inputs.velocityRPMBottom, bottomFlywheelRPM)
                + bottomFlywheelFeedforward.calculate(bottomFlywheelRPM));
      } else {
        io.setBottomFlywheelVolts(0.0);
      }

      // If we want to move the feeder wheel.
      io.setFeederWheelVoltage(feederWheelVolts);
      if (shouldIncrement) {
        io.setFeederWheelVoltage(
            feederWheelController.pid.calculate(
                inputs.feederWheel.positionRads,
                inputs.feederWheel.positionRads + increasePositionRads));
      } else if (shouldIncrement
          && feederWheelController.pid
              .atSetpoint()) { // if we want to move it, and its at the setpoint.
        shouldIncrement = false;
      }
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

  /**
   * A function that spins up the flywheels and returns when at the desired speed
   *
   * @param topRPM The desired top RPM
   * @param bottomRPM The desired bottom RPM
   * @return The command, exits when flywheels are up to speed
   */
  public Command spinToSpeed(double topRPM, double bottomRPM) {
    return this.runOnce(
            () -> {
              topFlywheelRPM = topRPM;
              bottomFlywheelRPM = bottomRPM;
            })
        .alongWith(Commands.waitUntil(this::isAtSetpoint));
  }

  public void spinToSpeedVoid(double rpm) {
    topFlywheelRPM = rpm;
    bottomFlywheelRPM = rpm;
  }

  /**
   * Spins both sets of flywheels to a speed, and then increments the game piece when it is ready
   *
   * @param rpm The RPM to set the flywheels to
   * @return The command
   */
  public Command shootCommand(double rpm) {
    return this.runOnce(() -> setRawFeederVolts(0.0))
        .andThen(this.spinToSpeed(rpm, rpm))
        .andThen(this.run(() -> setRawFeederVolts(1.0)))
        .andThen(Commands.run(() -> {}))
        .finallyDo(
            () -> {
              setRawFlywheelSpeed(0);
              setRawFeederVolts(0);
            });
  }

  public void setRawFlywheelSpeed(double rpm) {
    topFlywheelRPM = rpm;
    bottomFlywheelRPM = rpm;
  }

  public void setRawFeederVolts(double volts) {
    feederWheelVolts = volts;
  }

  public Command setRawFeederVoltsCommand(double volts) {
    return this.runOnce(
        () -> {
          feederWheelVolts = volts;
        });
  }

  /**
   * This function increments the game piece forwards to move it into the flywheels.
   *
   * @return The command.
   */
  public Command incrementGamePiece() {
    return this.runOnce(
            () -> {
              shouldIncrement = true;
            })
        .alongWith(Commands.waitUntil(this::hasGamePiece));
  }

  public Command runFeederWheel(double volts) {
    return this.runEnd(
        () -> {
          feederWheelVolts = volts;
        },
        () -> {
          feederWheelVolts = 0;
        });
  }

  /**
   * Sets the flwheel desired RPM's to 0.
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
