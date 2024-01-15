package frc.WorBots.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ModuleIOKraken implements ModuleIO {
  private TalonFX driveMotor;
  private TalonFX turnMotor;
  private CANcoder absoluteEncoder;
  private Rotation2d encoderOffset;

  public ModuleIOKraken(int index) {
    switch (index) {
      case 0:
        driveMotor = new TalonFX(0);
        turnMotor = new TalonFX(0);
        absoluteEncoder = new CANcoder(0);
        encoderOffset = new Rotation2d(Units.degreesToRadians(0));
        break;
      case 1:
        driveMotor = new TalonFX(0);
        turnMotor = new TalonFX(0);
        absoluteEncoder = new CANcoder(0);
        encoderOffset = new Rotation2d(Units.degreesToRadians(0));
        break;
      case 2:
        driveMotor = new TalonFX(0);
        turnMotor = new TalonFX(0);
        absoluteEncoder = new CANcoder(0);
        encoderOffset = new Rotation2d(Units.degreesToRadians(0));
        break;
      case 3:
        driveMotor = new TalonFX(0);
        turnMotor = new TalonFX(0);
        absoluteEncoder = new CANcoder(0);
        encoderOffset = new Rotation2d(Units.degreesToRadians(0));
        break;
      default:
        throw new RuntimeException("Invalid swerve module index");
    }
    driveMotor.setInverted(true);
    turnMotor.setInverted(true);
  }

  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = Units
        .rotationsToRadians(driveMotor.getPosition().getValue() * ((14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0)));
    inputs.driveVelocityRadPerSec = Units
        .rotationsToRadians(driveMotor.getVelocity().getValue() * ((14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0)));
    inputs.driveAppliedVolts = driveMotor.get() * driveMotor.getSupplyVoltage().getValue();
    inputs.driveCurrentAmps = new double[] { driveMotor.getStatorCurrent().getValue() };
    inputs.driveTempCelcius = new double[] { driveMotor.getDeviceTemp().getValue() };

    inputs.turnAbsolutePositionRad = MathUtil
        .angleModulus(Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition().getValue())
            - encoderOffset.getRadians());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(absoluteEncoder.getVelocity().getValue());
    inputs.turnPositionRad = Units.rotationsToRadians(absoluteEncoder.getPosition().getValue());
    inputs.turnAppliedVolts = turnMotor.get() * turnMotor.getSupplyVoltage().getValue();
    inputs.turnCurrentAmps = new double[] { turnMotor.getStatorCurrent().getValue() };
    inputs.turnTempCelcius = new double[] { turnMotor.getDeviceTemp().getValue() };

    inputs.isConnected = true;
    if (!turnMotor.isAlive()) {
      inputs.isConnected = false;
    } else if (!driveMotor.isAlive()) {
      inputs.isConnected = false;
    }
  }

  public void setDriveVoltage(double volts) {
    MathUtil.clamp(volts, -12, 12);
    driveMotor.setVoltage(volts);
  }

  public void setTurnVoltage(double volts) {
    MathUtil.clamp(volts, -12, 12);
    turnMotor.setVoltage(-volts);
  }

  public void setDriveBrakeMode(boolean enable) {
  }

  public void setTurnBrakeMode(boolean enable) {
  }
}
