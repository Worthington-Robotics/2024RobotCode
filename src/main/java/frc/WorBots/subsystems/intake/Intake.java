package frc.WorBots.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.WorBots.util.StatusPage;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputs inputs = new IntakeIOInputs();
  private boolean hasGamePiece = true;
  private double setpointVolts = 0.0;
  private double velocityThreshold;
  private double ampsThreshold;

  public Intake(IntakeIO io) {
    this.io = io;

    if (Constants.getSim()) {
      velocityThreshold = 1.0;
      ampsThreshold = 8.0;
    } else {
      velocityThreshold = 1.0;
      ampsThreshold = 2.0;
    }

    StatusPage.reportStatus(StatusPage.INTAKE_SUBSYSTEM, true);
  }

  public void periodic() {
    io.updateInputs(inputs);
    if (inputs.velocityRadsPerSec > velocityThreshold && inputs.currentDrawAmps > ampsThreshold) {
      hasGamePiece = true;
    } else {
      hasGamePiece = false;
    }
    // hasGamePiece = true;

    if (inputs.temperatureCelsius > 75) {
      setpointVolts = 0.0;
    }

    SmartDashboard.putNumber("Intake/Setpoint", setpointVolts);
    SmartDashboard.putNumber("Intake/TemperatureC", inputs.temperatureCelsius);
    SmartDashboard.putNumber("Intake/VelocityRadPerSec", inputs.velocityRadsPerSec);
    SmartDashboard.putNumber("Intake/CurrentDrawAmps", inputs.currentDrawAmps);
    SmartDashboard.putNumber("Intake/AppliedPowerVolts", inputs.appliedPowerVolts);
    StatusPage.reportStatus(StatusPage.INTAKE_CONNECTED, inputs.isConnected);

    io.setIntakeVoltage(setpointVolts);
  }
}