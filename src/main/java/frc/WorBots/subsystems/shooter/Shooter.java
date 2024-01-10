package frc.WorBots.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.WorBots.util.StatusPage;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOInputs inputs = new ShooterIOInputs();

    public Shooter(ShooterIO io) {
        this.io = io;

        StatusPage.reportStatus(StatusPage.SHOOTER_SUBSYSTEM, true);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        StatusPage.reportStatus(StatusPage.SHOOTER_CONNECTED, inputs.isConnected);
    }
}
