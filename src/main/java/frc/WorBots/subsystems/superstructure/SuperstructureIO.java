package frc.WorBots.subsystems.superstructure;

public interface SuperstructureIO {
  public static class SuperstructureIOInputs {
    double elevatorPositionMeters = 0.0;
    double elevatorVelocityMetersPerSec = 0.0;
    double elevatorTemp = 0.0;
    double elevatorVoltage = 0.0;

    double pivotPositionAbsRad = 0.0;
    double pivotPositionRelRad = 0.0;
    double pivotVelocityRadPerSec = 0.0;
  }

  public default void setElevatorVoltage(double volts) {}

  public default void setPivotVoltage(double volts) {}

  public default void updateInputs(SuperstructureIOInputs inputs) {}
}
