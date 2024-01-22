package frc.WorBots.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.util.Units;

public class ShooterIOTalon implements ShooterIO {
  private TalonFX topFlywheel;
  private TalonFX bottomFlywheel;
  private TalonFX feederWheel;
  private TimeOfFlight timeOfFlight;

  public ShooterIOTalon() {
    topFlywheel = new TalonFX(0);
    bottomFlywheel = new TalonFX(0);
    feederWheel = new TalonFX(0);
    timeOfFlight = new TimeOfFlight(0);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocityRPMBottom = bottomFlywheel.getVelocity().getValue() * 60;
    inputs.velocityRPMTop = topFlywheel.getVelocity().getValue() * 60;

    inputs.feederWheelPositionRads = Units.rotationsToRadians(feederWheel.getPosition().getValue());
    inputs.feederWheelVelocityRadPerSec = Units.rotationsToRadians(feederWheel.getVelocity().getValue());
    inputs.feederWheelCurrentAmps = feederWheel.getStatorCurrent().getValue();
    inputs.timeOfFlightDistanceMeters = timeOfFlight.getRange() / 1000.0;
  }
}
