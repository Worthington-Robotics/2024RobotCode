package frc.WorBots.subsystems.shooter;

public interface ShooterIO {
    public static class ShooterIOInputs {
      boolean isConnected = false;
      double velocityRPM = 0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }
}
