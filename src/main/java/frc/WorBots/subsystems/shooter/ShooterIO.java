package frc.WorBots.subsystems.shooter;

public interface ShooterIO {
    public static class ShooterIOInputs {
        boolean isConnected = false;
    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }
}
