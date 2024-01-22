package frc.WorBots.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  FlywheelSim topFlywheelSim;
  FlywheelSim bottomFlywheelSim;
  FlywheelSim feederWheel;

  public ShooterIOSim() {
    topFlywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.5);
    bottomFlywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.5);
    feederWheel = new FlywheelSim(DCMotor.getKrakenX60(1), 1.0, 0.5);
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
  public void setFeederWheelVoltage(double volts) {
    feederWheel.setInputVoltage(volts);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    topFlywheelSim.update(0.02);
    bottomFlywheelSim.update(0.02);
    feederWheel.update(0.02);
    inputs.isConnected = true;
    inputs.velocityRPMBottom = bottomFlywheelSim.getAngularVelocityRPM();
    inputs.velocityRPMTop = topFlywheelSim.getAngularVelocityRPM();
  }
}
