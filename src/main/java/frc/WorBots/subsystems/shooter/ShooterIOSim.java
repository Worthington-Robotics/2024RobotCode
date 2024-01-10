package frc.WorBots.subsystems.shooter;

public class ShooterIOSim implements ShooterIO {
    public ShooterIOSim() {
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.isConnected = true;
    }
}
