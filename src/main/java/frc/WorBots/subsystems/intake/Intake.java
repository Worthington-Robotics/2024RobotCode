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
import frc.WorBots.util.debug.TunableDouble;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  /** Setpoint voltage for the intake */
  private double setpointVolts = 0.0;

  /** Whether a game piece is in the intake */
  private boolean hasGamepiece = false;

  // Publishers
  private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private final NetworkTable intakeTable = instance.getTable("Intake");
  private final DoublePublisher setpointPub =
      intakeTable.getDoubleTopic("Setpoint Volts").publish();
  private final BooleanPublisher hasGamePiecePub =
      intakeTable.getBooleanTopic("Has Game Piece").publish();
  private final DoublePublisher timeOfFlightDistancePub =
      intakeTable.getDoubleTopic("Time of Flight Distance").publish();

  // Constants
  /** Distance threshold for the time of flight to say it has a game piece */
  public static final double DISTANCE_THRESHOLD = 0.255;

  /** The position where we want the note to be in meters from the ToF */
  private static final TunableDouble NOTE_POSITION =
      new TunableDouble("Intake", "Tuning", "Note Position", 0.240);

  /** Voltage for intaking */
  private static final double INTAKE_VOLTS = 4.25;

  /** Max temperature for the intake motor */
  private static final double MAX_TEMP = 80.0;

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

    hasGamepiece = inputs.timeOfFlightDistanceMeters <= DISTANCE_THRESHOLD;

    if (inputs.motor.temperatureCelsius > MAX_TEMP || DriverStation.isDisabled()) {
      setpointVolts = 0.0;
    }

    StatusPage.reportStatus(
        StatusPage.INTAKE_CONNECTED,
        inputs.isConnected && inputs.motor.temperatureCelsius <= MAX_TEMP);

    io.setIntakeVoltage(setpointVolts);

    // Publish data
    inputs.motor.publish();
    setpointPub.set(setpointVolts);
    hasGamePiecePub.set(hasGamepiece);
    timeOfFlightDistancePub.set(inputs.timeOfFlightDistanceMeters);
  }

  public double getSetpointVolts() {
    return setpointVolts;
  }

  public double getToFDistanceMeters() {
    return inputs.timeOfFlightDistanceMeters;
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
   * Returns a command that spits the game piece
   *
   * @return The command
   */
  public Command spitRaw() {
    return this.run(
            () -> {
              setpointVolts = -INTAKE_VOLTS;
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

  public double getNotePositionDistance() {
    return inputs.timeOfFlightDistanceMeters - NOTE_POSITION.get();
  }
}
