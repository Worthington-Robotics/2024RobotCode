package frc.WorBots.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.util.Units;

public class IntakeIOTalon implements IntakeIO {
  private TalonFX intakeMotor;
  private TimeOfFlight timeOfFlight;

  public IntakeIOTalon() {
    intakeMotor = new TalonFX(0);
    timeOfFlight= new TimeOfFlight(0);
    timeOfFlight.setRangingMode(RangingMode.Short, 24);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedPowerVolts = intakeMotor.getMotorVoltage().getValue();
    inputs.currentDrawAmps = intakeMotor.getStatorCurrent().getValue();
    inputs.temperatureCelsius = intakeMotor.getDeviceTemp().getValue();
    inputs.velocityRadsPerSec = Units.rotationsToRadians(intakeMotor.getVelocity().getValue());
    inputs.timeOfFlightDistanceMeters = timeOfFlight.getRange()/1000;
    inputs.isConnected = inputs.temperatureCelsius != 0.0 ? true : false;
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }
}
