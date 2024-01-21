package frc.WorBots.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.WorBots.util.StatusPage;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputs inputs = new IntakeIOInputs();
  private double setpointVolts = 0.0;
  private boolean hasGamepiece = false;
  private static final double velocityThresholdRadsPerSec = 1.0;
  private static final double currentThresholdAmps = 100.0;

  public Intake(IntakeIO io) {
    this.io = io;
    StatusPage.reportStatus(StatusPage.INTAKE_SUBSYSTEM, true);
  }

  public void periodic() {
    io.updateInputs(inputs);

    hasGamepiece = (inputs.currentDrawAmps > currentThresholdAmps
        && inputs.velocityRadsPerSec < velocityThresholdRadsPerSec ? true : false);

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