package frc.WorBots.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOTalon implements ShooterIO {
  private TalonFX topFlywheel;
  private TalonFX bottomFlywheel;

  private TalonFX queueFlywheel;

  public ShooterIOTalon() {
    topFlywheel = new TalonFX(0);
    bottomFlywheel = new TalonFX(0);
    queueFlywheel = new TalonFX(0);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocityRPMBottom = bottomFlywheel.getVelocity().getValue() * 60;
    inputs.velocityRPMTop = topFlywheel.getVelocity().getValue() * 60;
  }
}
