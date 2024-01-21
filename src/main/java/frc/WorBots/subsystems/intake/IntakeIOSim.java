package frc.WorBots.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
  FlywheelSim sim;

  public IntakeIOSim() {
    sim = new FlywheelSim(DCMotor.getKrakenX60(1), 1, 0.05);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    sim.update(0.02);
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.currentDrawAmps = sim.getCurrentDrawAmps();
    inputs.isConnected = true;
  }
}