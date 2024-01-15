package frc.WorBots.subsystems.drive;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.math.util.Units;

public class GyroIOPigeon implements GyroIO {
  private final PigeonIMU pigeon;
  private final double[] yprDegrees = new double[3];
  private final double[] xyzDps = new double[3];

  public GyroIOPigeon() {
    pigeon = new PigeonIMU(0);

    pigeon.clearStickyFaults();
    pigeon.setYaw(0.0);
    pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 20);
  }

  public void updateInputs(GyroIOInputs inputs) {
    pigeon.getYawPitchRoll(yprDegrees);
    pigeon.getRawGyro(xyzDps);
    inputs.connected = pigeon.getTemp() != 0.0;
    inputs.rollPositionRad = Units.degreesToRadians(yprDegrees[1]);
    inputs.pitchPositionRad = Units.degreesToRadians(-yprDegrees[2]);
    inputs.yawPositionRad = Units.degreesToRadians(yprDegrees[0]);
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(xyzDps[1]);
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(-xyzDps[0]);
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(xyzDps[2]);
  }
}