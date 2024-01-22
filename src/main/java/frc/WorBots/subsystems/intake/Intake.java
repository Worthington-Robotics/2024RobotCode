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

    if (inputs.temperatureCelsius > 75) {
      setpointVolts = 0.0;
    }

    StatusPage.reportStatus(StatusPage.INTAKE_CONNECTED, inputs.isConnected);

    io.setIntakeVoltage(setpointVolts);
  }

  public Command intake() {
    return this.runOnce(() -> {
      if (hasGamepiece == true) {
        setpointVolts = 1.0;
      } else {
        setpointVolts = 8.0;
      }
    }).andThen(Commands.waitUntil(this::hasGamePiece)).finallyDo(() -> {
      setpointVolts = 1.0;
    });
  }

  public boolean hasGamePiece() {
    return hasGamepiece;
  }
}