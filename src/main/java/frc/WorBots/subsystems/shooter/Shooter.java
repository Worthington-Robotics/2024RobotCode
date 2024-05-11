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
import frc.WorBots.util.debug.TunableDouble;
import frc.WorBots.util.debug.TunablePIDController;
import frc.WorBots.util.debug.TunablePIDController.TunablePIDGains;
import frc.WorBots.util.math.GeneralMath;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
  private ShooterIO io;
  private ShooterIOInputs inputs = new ShooterIOInputs();
  private boolean hasGamePiece = false;
  private double topFlywheelRPM = 0.0;
  private double bottomFlywheelRPM = 0.0;
  private double feederWheelVolts = 0.0;
  private boolean idlingDisabled = false;

  // Logging classes
  private final NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable(TABLE_NAME);
  private final DoublePublisher topFlywheelSpeedPub =
      shooterTable.getDoubleTopic("Top Flywheel RPM").publish();
  private final DoublePublisher bottomFlywheelSpeedPub =
      shooterTable.getDoubleTopic("Bottom Flywheel RPM").publish();
  private final DoublePublisher topFlywheelSetpointPub =
      shooterTable.getDoubleTopic("Top Flywheel Setpoint").publish();
  private final DoublePublisher bottomFlywheelSetpointPub =
      shooterTable.getDoubleTopic("Bottom Flywheel Setpoint").publish();
  private final DoublePublisher feederWheelSetpointPub =
      shooterTable.getDoubleTopic("Feeder Setpoint Volts").publish();
  private final DoublePublisher timeOfFlightDistancePub =
      shooterTable.getDoubleTopic("Time of Flight Distance").publish();
  private final BooleanPublisher hasGamePiecePub =
      shooterTable.getBooleanTopic("Has Game Piece").publish();
  private final DoublePublisher noteDistancePub =
      shooterTable.getDoubleTopic("Note Distance").publish();

  // Constants

  /** The position where we want the note to be in meters from the ToF */
  private static final TunableDouble NOTE_POSITION =
      new TunableDouble("Shooter", "Tuning", "Note Position", 0.050);

  /** Distance threshold for the note position to say that it is correctly positioned */
  private static final TunableDouble NOTE_DISTANCE_THRESHOLD =
      new TunableDouble("Shooter", "Tuning", "Note Distance Threshold", 0.007);

  /**
   * Threshold for backwards wheel speed where the PID control will allow the motors to coast down
   * instead of violently braking, in volts
   */
  private static final double COAST_DOWN_THRESHOLD = -0.3;

  /** The default idle speed for the flywheels */
  private static final double IDLE_SPEED = 500.0;

  /** The voltage to run the feeder wheels at when feeding */
  private static final double FEED_VOLTS = 2.5;

  /** Error threshold for the shooter wheels, in RPM */
  private static final double RPM_THRESHOLD = 100.0;

  private static final String TABLE_NAME = "Shooter";

  private final TunablePIDController topFlywheelController =
      new TunablePIDController(new TunablePIDGains(TABLE_NAME, "Top Flywheel Gains"));
  private final TunablePIDController bottomFlywheelController =
      new TunablePIDController(new TunablePIDGains(TABLE_NAME, "Bottom Flywheel Gains"));
  private final TunablePIDController feederWheelController =
      new TunablePIDController(new TunablePIDGains(TABLE_NAME, "Feeder Wheel Gains"));
  private final SimpleMotorFeedforward topFlywheelFeedForward;
  private final SimpleMotorFeedforward bottomFlywheelFeedforward;

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
      topFlywheelController.pid.setTolerance(RPM_THRESHOLD);
      bottomFlywheelController.pid.setTolerance(RPM_THRESHOLD);
    } else { // Sim
      topFlywheelController.setGains(0.00050, 0, 0);
      bottomFlywheelController.setGains(0.00050, 0, 0);
      topFlywheelFeedForward = new SimpleMotorFeedforward(0, 0.002);
      bottomFlywheelFeedforward = new SimpleMotorFeedforward(0.0, 0.002);
      feederWheelController.setGains(1.0, 0.0, 0.0);
      topFlywheelController.pid.setTolerance(RPM_THRESHOLD);
      bottomFlywheelController.pid.setTolerance(RPM_THRESHOLD);
    }

    StatusPage.reportStatus(StatusPage.SHOOTER_SUBSYSTEM, true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Check if we have gamepiece
    hasGamePiece = inputs.timeOfFlightDistanceMeters <= 0.085;

    // Update logging
    topFlywheelSpeedPub.set(inputs.velocityRPMTop);
    bottomFlywheelSpeedPub.set(inputs.velocityRPMBottom);
    topFlywheelSetpointPub.set(topFlywheelRPM);
    bottomFlywheelSetpointPub.set(bottomFlywheelRPM);
    timeOfFlightDistancePub.set(inputs.timeOfFlightDistanceMeters);
    hasGamePiecePub.set(hasGamePiece);
    noteDistancePub.set(getNotePositionDistance());

    // Update tunables
    topFlywheelController.update();
    bottomFlywheelController.update();
    feederWheelController.update();

    if (DriverStation.isDisabled()) { // Set voltages to 0 if we are disabled.
      io.setTopFlywheelVolts(0.0);
      io.setBottomFlywheelVolts(0.0);
      io.setFeederWheelVoltage(0.0);
      feederWheelSetpointPub.set(0.0);
      // Clear setpoints
      topFlywheelRPM = 0.0;
      bottomFlywheelRPM = 0.0;
      feederWheelVolts = 0.0;
    } else {
      // Calculate the desired voltages based on the setpoints
      if (topFlywheelRPM != 0.0) {
        double setpoint =
            topFlywheelController.pid.calculate(inputs.velocityRPMTop, topFlywheelRPM)
                + topFlywheelFeedForward.calculate(topFlywheelRPM);
        if (setpoint < COAST_DOWN_THRESHOLD && topFlywheelRPM > 0) {
          setpoint = 0.0;
        }
        io.setTopFlywheelVolts(setpoint);
      } else {
        io.setTopFlywheelVolts(0);
      }
      if (bottomFlywheelRPM != 0.0) {
        double setpoint =
            bottomFlywheelController.pid.calculate(inputs.velocityRPMBottom, bottomFlywheelRPM)
                + bottomFlywheelFeedforward.calculate(bottomFlywheelRPM);
        if (setpoint < COAST_DOWN_THRESHOLD && bottomFlywheelRPM > 0) {
          setpoint = 0.0;
        }
        io.setBottomFlywheelVolts(setpoint);
      } else {
        io.setBottomFlywheelVolts(0.0);
      }

      // If we want to move the feeder wheel.
      io.setFeederWheelVoltage(feederWheelVolts);
      feederWheelSetpointPub.set(feederWheelVolts);
    }

    // Publish data
    inputs.feederWheel.publish();
    inputs.top.publish();
    inputs.bottom.publish();

    StatusPage.reportStatus(StatusPage.SHOOTER_CONNECTED, inputs.isConnected);
  }

  public double getToFDistanceMeters() {
    return inputs.timeOfFlightDistanceMeters;
  }

  /**
   * A command that spins up the flywheels and returns when at the desired speed
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

  /** Sets the speed of both flywheels */
  public void setSpeedVoid(double rpm) {
    topFlywheelRPM = rpm;
    bottomFlywheelRPM = rpm;
  }

  /** Sets the speed of both flywheels */
  public void setSpeedVoid(double topRPM, double bottomRPM) {
    topFlywheelRPM = topRPM;
    bottomFlywheelRPM = bottomRPM;
  }

  /** Returns a command that sets the speed of both flywheels once */
  public Command setSpeed(double rpm) {
    return this.runOnce(
        () -> {
          setSpeedVoid(rpm);
        });
  }

  /** Returns a command that sets the speed of both flywheels continuously */
  public Command setSpeedContinuous(double rpm) {
    return this.run(
        () -> {
          setSpeedVoid(rpm);
        });
  }

  /** Returns a command that sets the speed of both flywheels continuously */
  public Command setSpeedContinuous(double topRPM, double bottomRPM) {
    return this.run(
        () -> {
          setSpeedVoid(topRPM, bottomRPM);
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
    return this.run(
        () -> {
          this.idle();
        });
  }

  /** Idles the flywheels */
  public void idle() {
    if (this.idlingDisabled) {
      this.setSpeedVoid(0);
    } else {
      this.setSpeedVoid(IDLE_SPEED);
    }
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
    /*
     * We check the setpoint manually instead of using the PID because the PID might
     * not have its setpoint yet
     */
    return GeneralMath.checkError(inputs.velocityRPMTop, topFlywheelRPM, RPM_THRESHOLD)
        && GeneralMath.checkError(inputs.velocityRPMBottom, bottomFlywheelRPM, RPM_THRESHOLD);
  }

  /**
   * Reads the time of flight to detect if the robot has a game piece in the shooter.
   *
   * @return True if we have a gamepiece, otherwise false.
   */
  public boolean hasGamePiece() {
    return hasGamePiece;
  }

  /** Returns whether a game piece is in the shooter and in the right position */
  public boolean isGamePieceInPosition() {
    return Math.abs(getNotePositionDistance()) < NOTE_DISTANCE_THRESHOLD.get();
  }

  public double getNotePositionDistance() {
    return inputs.timeOfFlightDistanceMeters - NOTE_POSITION.get();
  }

  /** Enable / disable idling */
  public void setIdlingDisabled(boolean disable) {
    this.idlingDisabled = disable;
  }
}
