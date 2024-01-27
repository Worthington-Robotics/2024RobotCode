// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.shooter;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.WorBots.util.StatusPage;

public class Shooter extends SubsystemBase { // 532 rpm/v
  private ShooterIO io;
  private ShooterIOInputs inputs = new ShooterIOInputs();
  private boolean hasGamePiece = false;
  private boolean shouldIncrement = false;
  private double topFlywheelRPM = 0.0;
  private double bottomFlywheelRPM = 0.0;

  // Constants
  private static final double increasePositionRads = 2 * Math.PI;
  private static final double distanceThreshold = 0.25;

  private PIDController topFlywheelController;
  private PIDController bottomFlywheelController;
  private PIDController feederWheelController;
  private SimpleMotorFeedforward topFlywheelFeedForward;
  private SimpleMotorFeedforward bottomFlywheelFeedforward;

  public Shooter(ShooterIO io) {
    this.io = io;

    if (!Constants.getSim()) {
      topFlywheelController = new PIDController(1, 0, 0);
      bottomFlywheelController = new PIDController(1, 0, 0);
      topFlywheelFeedForward = new SimpleMotorFeedforward(0, 0);
      bottomFlywheelFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
      feederWheelController = new PIDController(1.0, 0.0, 0.0);
    } else {
      topFlywheelController = new PIDController(1, 0, 0);
      bottomFlywheelController = new PIDController(1, 0, 0);
      topFlywheelFeedForward = new SimpleMotorFeedforward(0, 0);
      bottomFlywheelFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
      feederWheelController = new PIDController(1.0, 0.0, 0.0);
    }

    StatusPage.reportStatus(StatusPage.SHOOTER_SUBSYSTEM, true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Check if we have gamepiece
    if (inputs.timeOfFlightDistanceMeters < distanceThreshold) {
      hasGamePiece = true;
    } else {
      hasGamePiece = false;
    }
    // Calculate the desired voltages based on the setpoints
    io.setTopFlywheelVolts(
        topFlywheelController.calculate(inputs.velocityRPMTop, topFlywheelRPM)
            + topFlywheelFeedForward.calculate(topFlywheelRPM));
    io.setBottomFlywheelVolts(
        bottomFlywheelController.calculate(inputs.velocityRPMBottom, bottomFlywheelRPM)
            + bottomFlywheelFeedforward.calculate(bottomFlywheelRPM));
    // If we want to move the feeder wheel.
    if (shouldIncrement) {
      io.setFeederWheelVoltage(
          feederWheelController.calculate(
              inputs.feederWheelPositionRads,
              inputs.feederWheelPositionRads + increasePositionRads));
    } else if (shouldIncrement
        && feederWheelController.atSetpoint()) { // if we want to move it, and its at the setpoint.
      io.setFeederWheelVoltage(0.0);
      shouldIncrement = false;
    }

    if (DriverStation.isDisabled()) { // Set voltages to 0 if we are disabled.
      io.setTopFlywheelVolts(0.0);
      io.setBottomFlywheelVolts(0.0);
      io.setFeederWheelVoltage(0.0);
    }
    StatusPage.reportStatus(StatusPage.SHOOTER_CONNECTED, inputs.isConnected);
  }

  /**
   * A function that spins up the flywheels and returns when at the desired speed;
   *
   * @param topRPM This is the desired top RPM;
   * @param bottomRPM This is desired bottom RPM;
   * @return The command, exits when flywheels are up to speed;
   */
  public Command spinToSpeed(double topRPM, double bottomRPM) {
    return this.runOnce(
            () -> {
              topFlywheelRPM = topRPM;
              bottomFlywheelRPM = bottomRPM;
            })
        .alongWith(Commands.waitUntil(this::isAtSetpoint));
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
    return topFlywheelController.atSetpoint() && bottomFlywheelController.atSetpoint();
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
