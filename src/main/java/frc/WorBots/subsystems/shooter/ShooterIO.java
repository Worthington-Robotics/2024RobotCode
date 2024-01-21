package frc.WorBots.subsystems.shooter;

public interface ShooterIO {
  public static class ShooterIOInputs {
    boolean isConnected = false;
    public double velocityRPMTop = 0;
    public double velocityRPMBottom = 0;
  }

  public default void setTopFlywheelVolts(double volts) {
  }

  public default void setBottomFlywheelVolts(double volts) {
  }

  public default void updateInputs(ShooterIOInputs inputs) {
  }
}
