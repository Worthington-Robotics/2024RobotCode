package frc.WorBots.subsystems.drive;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;

  public GyroIOPigeon2() {
    pigeon = new Pigeon2(0);
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getYaw().setUpdateFrequency(100);
    pigeon.optimizeBusUtilization();
    pigeon.reset();
  }

  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = pigeon.getTemperature().getValue() != 0.0;
    inputs.rollPositionRad = Units.degreesToRadians(pigeon.getRoll().getValue());
    inputs.pitchPositionRad = Units.degreesToRadians(pigeon.getPitch().getValue());
    inputs.yawPositionRad = Units.degreesToRadians(pigeon.getYaw().getValue());
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityXWorld().getValue());
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityYDevice().getValue());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityZDevice().getValue());
  }
}