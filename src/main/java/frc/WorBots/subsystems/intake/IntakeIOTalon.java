package frc.WorBots.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;

public class IntakeIOTalon implements IntakeIO {
  private TalonFX intakeMotor;

  public IntakeIOTalon() {
    intakeMotor = new TalonFX(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedPowerVolts = intakeMotor.getMotorVoltage().getValue();
    inputs.currentDrawAmps = intakeMotor.getStatorCurrent().getValue();
    inputs.temperatureCelsius = intakeMotor.getDeviceTemp().getValue();
    inputs.velocityRadsPerSec = Units.rotationsToRadians(intakeMotor.getVelocity().getValue());
    inputs.isConnected = inputs.temperatureCelsius != 0.0 ? true : false;
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }
}
