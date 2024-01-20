package frc.WorBots.subsystems.superstructure;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class SuperstructureIOTalon implements SuperstructureIO {
  private final TalonFX elevator;
  private final boolean isElevatorInverted = false;

  private final TalonFX pivot;
  private final boolean isPivotInverted = false;
  private final DutyCycleEncoder pivotAbsEncoder;
  private final Encoder pivotRelEncoder;

  private static final double elevatorGearing = 1.0; // in meters per revolution

  public SuperstructureIOTalon() {
    elevator = new TalonFX(0);
    pivot = new TalonFX(0);
    pivotAbsEncoder = new DutyCycleEncoder(0);
    pivotRelEncoder = new Encoder(0, 0);
  }

  public void setElevatorVoltage(double volts) {
    elevator.setVoltage(volts);
  }

  public void setPivotVoltage(double volts) {
    pivot.setVoltage(volts);
  }

  public void updateInputs(SuperstructureIOInputs inputs) {
    inputs.elevatorPositionMeters = (elevator.getPosition().getValue() / elevatorGearing)
        * (isElevatorInverted ? 1.0 : -1.0);
    inputs.elevatorVelocityMetersPerSec = (elevator.getVelocity().getValue() / elevatorGearing)
        * (isElevatorInverted ? 1.0 : -1.0);
    inputs.elevatorVoltage = elevator.getMotorVoltage().getValue();
    inputs.elevatorTemp = elevator.getDeviceTemp().getValue();

    inputs.pivotPositionAbsRad = pivotAbsEncoder.get() * (isPivotInverted ? 1.0 : -1.0);
    inputs.pivotPositionRelRad = pivotRelEncoder.getDistance() * (isPivotInverted ? 1.0 : -1.0);
    inputs.pivotVelocityRadPerSec = pivotRelEncoder.getRate() * (isPivotInverted ? 1.0 : -1.0);
    inputs.pivotTemp = pivot.getDeviceTemp().getValue();
    inputs.pivotVoltageApplied = pivot.getMotorVoltage().getValue();
  }
}
