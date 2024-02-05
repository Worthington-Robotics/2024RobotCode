// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.intake;

import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.WorBots.util.StatusPage;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputs inputs = new IntakeIOInputs();
  private double setpointVolts = 0.0;
  private boolean hasGamepiece = false;
  public static final double distanceThreshold = 0.25;

  /**
   * The intake subsystem, responsible for intaking game pieces from the ground
   * and passing them to
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
    hasGamepiece = false;

    if (inputs.temperatureCelsius > 80) {
      setpointVolts = 0.0;
    }

    StatusPage.reportStatus(StatusPage.INTAKE_CONNECTED, inputs.isConnected);

    io.setIntakeVoltage(setpointVolts);
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
            setpointVolts = 0.5;
          } else {
            setpointVolts = 8.0;
          }
        })
        .andThen(Commands.waitUntil(this::hasGamePiece))
        .finallyDo(
            () -> {
              setpointVolts = 0.5;
            });
  }

  public Command intakeRaw() {
    return this.runOnce(() -> {
      setpointVolts = 5.0;
    }).finallyDo(() -> {
      setpointVolts = 0.0;
    });
  }

  public Command outtakeRaw() {
    return this.runOnce(() -> {
      setpointVolts = -5.0;
    }).finallyDo(() -> {
      setpointVolts = 0.0;
    });
  }

  /**
   * Hands off the game piece to the shooter.
   *
   * @return The command.
   */
  public Command handoff() {
    return this.runOnce(
        () -> {
          setpointVolts = 8.0;
        })
        .finallyDo(
            () -> {
              setpointVolts = 0.5;
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
