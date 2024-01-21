package frc.WorBots.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  FlywheelSim topFlywheelSim;
  FlywheelSim bottomFlywheelSim;

  public ShooterIOSim() {
    topFlywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.5);
    bottomFlywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.5);
  }

  @Override
  public void setBottomFlywheelVolts(double volts) {
    bottomFlywheelSim.setInputVoltage(volts);
  }

  @Override
  public void setTopFlywheelVolts(double volts) {
    topFlywheelSim.setInputVoltage(volts);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.isConnected = true;
    inputs.velocityRPMBottom = bottomFlywheelSim.getAngularVelocityRPM();
    inputs.velocityRPMTop = topFlywheelSim.getAngularVelocityRPM();
  }
}
