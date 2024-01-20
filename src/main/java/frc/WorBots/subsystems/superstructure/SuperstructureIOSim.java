package frc.WorBots.subsystems.superstructure;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SuperstructureIOSim implements SuperstructureIO {
  private ElevatorSim elevator = new ElevatorSim(DCMotor.getKrakenX60(1), 0, 0, 0, 0, 0, true, 0);
  private SingleJointedArmSim pivot = new SingleJointedArmSim(DCMotor.getKrakenX60(1), 0, 0, 0, 0, 0, true, 0);

  @Override
  public void setElevatorVoltage(double volts) {
    elevator.setInputVoltage(volts);
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivot.setInputVoltage(volts);
  }

  @Override
  public void updateInputs(SuperstructureIOInputs inputs) {
    elevator.update(0.02);
    pivot.update(0.02);
    
    inputs.elevatorPositionMeters = elevator.getPositionMeters();
    inputs.elevatorVelocityMetersPerSec = elevator.getVelocityMetersPerSecond();
    
    inputs.pivotPositionAbsRad = 0.0;
    inputs.pivotPositionRelRad = pivot.getAngleRads();
    inputs.elevatorVelocityMetersPerSec = pivot.getVelocityRadPerSec();
  }
}
