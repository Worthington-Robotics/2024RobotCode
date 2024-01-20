package frc.WorBots.subsystems.superstructure;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SuperstructureIOSim implements SuperstructureIO {
  private ElevatorSim elevator;
  private SingleJointedArmSim pivot;

  public SuperstructureIOSim() {
    elevator = new ElevatorSim(DCMotor.getKrakenX60(1), 2.0, 1.0, 0.02, 0.0, 1.0, true, 0.5);
    pivot = new SingleJointedArmSim(DCMotor.getKrakenX60(1), 1.0, 1.0, 0.02, 0, 3.14, true, 0);
  }

  public void setElevatorVoltage(double volts) {
    elevator.setInputVoltage(volts);
  }

  public void setPivotVoltage(double volts) {
    pivot.setInputVoltage(volts);
  }

  public void updateInputs(SuperstructureIOInputs inputs) {
    elevator.update(0.02);
    pivot.update(0.02);
    
    inputs.elevatorPositionMeters = elevator.getPositionMeters();
    inputs.elevatorVelocityMetersPerSec = elevator.getVelocityMetersPerSecond();
    inputs.elevatorVoltage = elevator.getOutput(0);
    
    inputs.pivotPositionAbsRad = 0.0;
    inputs.pivotPositionRelRad = pivot.getAngleRads();
    inputs.elevatorVelocityMetersPerSec = pivot.getVelocityRadPerSec();
  }
}
