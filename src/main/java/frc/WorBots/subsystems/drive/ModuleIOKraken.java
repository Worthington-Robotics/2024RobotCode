// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ModuleIOKraken implements ModuleIO {
  private static final String CAN_BUS = "Swerve";
  private static final double DRIVE_ROTATIONS_TO_RADIANS =
      (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

  private TalonFX driveMotor;
  private TalonFX turnMotor;
  private CANcoder absoluteEncoder;
  private Rotation2d encoderOffset;

  public ModuleIOKraken(int index) {
    switch (index) {
      case 0:
        driveMotor = new TalonFX(0, CAN_BUS);
        turnMotor = new TalonFX(1, CAN_BUS);
        absoluteEncoder = new CANcoder(2, CAN_BUS);
        encoderOffset = new Rotation2d(Units.degreesToRadians(0.0));
        break;
      case 1:
        driveMotor = new TalonFX(3, CAN_BUS);
        turnMotor = new TalonFX(4, CAN_BUS);
        absoluteEncoder = new CANcoder(5, CAN_BUS);
        encoderOffset = new Rotation2d(Units.degreesToRadians(0.0));
        break;
      case 2:
        driveMotor = new TalonFX(6, CAN_BUS);
        turnMotor = new TalonFX(7, CAN_BUS);
        absoluteEncoder = new CANcoder(8, CAN_BUS);
        encoderOffset = new Rotation2d(Units.degreesToRadians(0.0));
        break;
      case 3:
        driveMotor = new TalonFX(9, CAN_BUS);
        turnMotor = new TalonFX(10, CAN_BUS);
        absoluteEncoder = new CANcoder(11, CAN_BUS);
        encoderOffset = new Rotation2d(Units.degreesToRadians(0.0));
        break;
      default:
        throw new RuntimeException("Invalid swerve module index");
    }
    // Configure devices
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnMotor.getConfigurator().apply(config);

    driveMotor.setInverted(true);
    turnMotor.setInverted(true);
  }

  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveMotor.getPosition().getValue() * DRIVE_ROTATIONS_TO_RADIANS);
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveMotor.getVelocity().getValue() * DRIVE_ROTATIONS_TO_RADIANS);
    inputs.driveAppliedVolts = driveMotor.get() * driveMotor.getSupplyVoltage().getValue();
    inputs.driveCurrentAmps = new double[] {driveMotor.getStatorCurrent().getValue()};
    inputs.driveTempCelcius = new double[] {driveMotor.getDeviceTemp().getValue()};

    inputs.turnAbsolutePositionRad =
        MathUtil.angleModulus(
            Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition().getValue())
                - encoderOffset.getRadians());
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(absoluteEncoder.getVelocity().getValue());
    inputs.turnPositionRad = Units.rotationsToRadians(absoluteEncoder.getPosition().getValue());
    inputs.turnAppliedVolts = turnMotor.get() * turnMotor.getSupplyVoltage().getValue();
    inputs.turnCurrentAmps = new double[] {turnMotor.getStatorCurrent().getValue()};
    inputs.turnTempCelcius = new double[] {turnMotor.getDeviceTemp().getValue()};

    inputs.isConnected = turnMotor.isAlive() && driveMotor.isAlive();
  }

  public void setDriveVoltage(double volts) {
    volts = MathUtil.clamp(volts, -12, 12);
    driveMotor.setVoltage(volts);
  }

  public void setTurnVoltage(double volts) {
    volts = MathUtil.clamp(volts, -12, 12);
    turnMotor.setVoltage(volts);
  }

  public void setDriveBrakeMode(boolean enable) {}

  public void setTurnBrakeMode(boolean enable) {}
}
