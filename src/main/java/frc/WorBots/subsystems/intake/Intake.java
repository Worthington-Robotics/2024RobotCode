// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.intake;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.WorBots.util.debug.StatusPage;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputs inputs = new IntakeIOInputs();
  private double setpointVolts = 0.0;
  private boolean hasGamepiece = false;

  // Publishers
  private NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private NetworkTable intakeTable = instance.getTable("Intake");
  private DoublePublisher setpointPub = intakeTable.getDoubleTopic("Setpoint Volts").publish();
  private BooleanPublisher hasGamePiecePub =
      intakeTable.getBooleanTopic("Has Game Piece").publish();
  private DoublePublisher timeOfFlightDistancePub =
      intakeTable.getDoubleTopic("Time of Flight Distance").publish();

  // Constants
  public static final double distanceThreshold = 0.22;
  private static final double intakeVolts = 4.0;
  private static final double constantForce = 0.0;
  private static final double maxTemperature = 80.0;

  /**
   * The intake subsystem, responsible for intaking game pieces from the ground and passing them to
   * the shooter.
   *
   * @param io
   */
  public Intake(IntakeIO io) {
    this.io = io;
    StatusPage.reportStatus(StatusPage.INTAKE_SUBSYSTEM, true);
  }

  public void periodic() {
    io.updateInputs(inputs);

    if (inputs.timeOfFlightDistanceMeters > distanceThreshold) {
      hasGamepiece = false;
    } else {
      hasGamepiece = true;
    }

    if (inputs.motor.temperatureCelsius > maxTemperature || DriverStation.isDisabled()) {
      setpointVolts = 0.0;
    }

    StatusPage.reportStatus(
        StatusPage.INTAKE_CONNECTED,
        inputs.isConnected && inputs.motor.temperatureCelsius <= maxTemperature);

    inputs.motor.publish();

    io.setIntakeVoltage(setpointVolts);

    setpointPub.set(setpointVolts);
    hasGamePiecePub.set(hasGamepiece);
    timeOfFlightDistancePub.set(inputs.timeOfFlightDistanceMeters);
  }

  /**
   * Set the intake voltage
   *
   * @param volts The voltage for the intake
   */
  public void setVolts(double volts) {
    setpointVolts = volts;
  }

  /**
   * A command that runs the intake until a game piece is detected.
   *
   * @return The command.
   */
  public Command intake() {
    return this.runOnce(
            () -> {
              if (hasGamepiece == true) {
                setpointVolts = constantForce;
              } else {
                setpointVolts = intakeVolts;
              }
            })
        .andThen(Commands.waitUntil(this::hasGamePiece))
        .finallyDo(
            () -> {
              setpointVolts = constantForce;
            });
  }

  public Command intakeRaw() {
    return this.run(
            () -> {
              setpointVolts = intakeVolts;
            })
        .finallyDo(
            () -> {
              setpointVolts = 0.0;
            });
  }

  public Command spitRaw() {
    return this.run(
            () -> {
              setpointVolts = -intakeVolts;
            })
        .finallyDo(
            () -> {
              setpointVolts = 0.0;
            });
  }

  /**
   * Gets wether or not a game piece is currently held.
   *
   * @return The command.
   */
  public boolean hasGamePiece() {
    return hasGamepiece;
  }
}
