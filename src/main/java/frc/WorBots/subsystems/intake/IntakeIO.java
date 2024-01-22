package frc.WorBots.subsystems.intake;

public interface IntakeIO {
  public static class IntakeIOInputs {
    double temperatureCelsius = 0.0;
    double velocityRadsPerSec = 0.0;
    double appliedPowerVolts = 0.0;
    double currentDrawAmps = 0.0;
    double timeOfFlightDistanceMeters = 0.0;
    boolean isConnected = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {
  }

  public default void setIntakeVoltage(double volts) {

  }
}
