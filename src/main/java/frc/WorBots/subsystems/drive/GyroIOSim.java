package frc.WorBots.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class GyroIOSim implements GyroIO {
  private SimAxis pitchAxis = new SimAxis();
  private SimAxis rollAxis = new SimAxis();
  private SimAxis yawAxis = new SimAxis();

  public GyroIOSim() {
  }

  public void updateInputs(GyroIOInputs inputs) {
    pitchAxis.update();
    rollAxis.update();
    yawAxis.update();

    inputs.pitchPositionRad = pitchAxis.angle.getRadians();
    inputs.pitchVelocityRadPerSec = pitchAxis.velocity;

    inputs.rollPositionRad = rollAxis.angle.getRadians();
    inputs.rollVelocityRadPerSec = rollAxis.velocity;

    inputs.yawPositionRad = yawAxis.angle.getRadians();
    inputs.yawVelocityRadPerSec = yawAxis.velocity;

    inputs.connected = true;
  }

  public void setExpectedPitchVelocity(double vPitch) {
    pitchAxis.velocity = vPitch;
  }

  public void setExpectedRollVelocity(double vRoll) {
    rollAxis.velocity = vRoll;
  }

  public void setExpectedYawVelocity(double vYaw) {
    yawAxis.velocity = vYaw;
  }

  /**
   * Simulation for one axis of the gyroscope.
   * It basically just applies a rotational velocity to a stored rotation
   * every update, with compensation for time difference
   */
  private static class SimAxis {
    private Rotation2d angle = new Rotation2d();
    // Current velocity in radians per second
    private double velocity = 0.0;
    // Timestamp when update() was last called
    private double lastTimestamp = 0.0;

    public void update() {
      final double timestamp = Timer.getFPGATimestamp();
      final double diff = timestamp - lastTimestamp;
      // Scale the velocity in radians per second to match the time period
      final double motion = velocity * diff;

      angle = angle.plus(Rotation2d.fromRadians(motion));

      lastTimestamp = timestamp;
    }
  }
}
