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
  private static final double distanceThreshold = 0.25;
  private double topFlywheelRPM = 0.0;
  private double bottomFlywheelRPM = 0.0;

  private PIDController topFlywheelController;
  private PIDController bottomFlywheelController;
  private SimpleMotorFeedforward topFlywheelFeedForward;
  private SimpleMotorFeedforward bottomFlywheelFeedforward;

  public Shooter(ShooterIO io) {
    this.io = io;

    if (!Constants.getSim()) {
      topFlywheelController = new PIDController(1, 0, 0);
      bottomFlywheelController = new PIDController(1, 0, 0);
      topFlywheelFeedForward = new SimpleMotorFeedforward(0, 0);
      bottomFlywheelFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
    } else {
      topFlywheelController = new PIDController(1, 0, 0);
      bottomFlywheelController = new PIDController(1, 0, 0);
      topFlywheelFeedForward = new SimpleMotorFeedforward(0, 0);
      bottomFlywheelFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
    }

    StatusPage.reportStatus(StatusPage.SHOOTER_SUBSYSTEM, true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    if (inputs.timeOfFlightDistanceMeters < distanceThreshold) {
      hasGamePiece = true;
    } else {
      hasGamePiece = false;
    }
    io.setTopFlywheelVolts(
        topFlywheelController.calculate(inputs.velocityRPMTop, topFlywheelRPM)
            + topFlywheelFeedForward.calculate(topFlywheelRPM));
    io.setBottomFlywheelVolts(
        bottomFlywheelController.calculate(inputs.velocityRPMBottom, bottomFlywheelRPM)
            + bottomFlywheelFeedforward.calculate(bottomFlywheelRPM));
    if (DriverStation.isEnabled()) {
      io.setTopFlywheelVolts(0.0);
      io.setBottomFlywheelVolts(0.0);
      io.setFeederWheelVoltage(0.0);
    }
    StatusPage.reportStatus(StatusPage.SHOOTER_CONNECTED, inputs.isConnected);
  }

  public Command spinToSpeed(double topRPM, double bottomRPM) {
    return this.runOnce(() -> {
      topFlywheelRPM = topRPM;
      bottomFlywheelRPM = bottomRPM;
    }).alongWith(Commands.waitUntil(this::isAtSetpoint));
  }

  public Command stopFlywheels() {
    return this.runOnce(() -> {
      topFlywheelRPM = 0;
      bottomFlywheelRPM = 0;
    });
  }

  public boolean isAtSetpoint() {
    return topFlywheelController.atSetpoint() && bottomFlywheelController.atSetpoint();
  }

  public boolean hasGamePiece() {
    return hasGamePiece;
  }
}
