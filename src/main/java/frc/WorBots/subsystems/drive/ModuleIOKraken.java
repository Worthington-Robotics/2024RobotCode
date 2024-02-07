// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.WorBots.Constants;

public class ModuleIOKraken implements ModuleIO {
  private static final double DRIVE_ROTATIONS_TO_RADIANS =
      (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

  private TalonFX driveMotor;
  private TalonFX turnMotor;
  private CANcoder absoluteEncoder;
  private Rotation2d encoderOffset;

  public ModuleIOKraken(int index) {
    switch (index) {
      case 0: // Front Left
        driveMotor = new TalonFX(1, Constants.SWERVE_CAN_BUS);
        turnMotor = new TalonFX(2, Constants.SWERVE_CAN_BUS);
        absoluteEncoder = new CANcoder(3, Constants.SWERVE_CAN_BUS);
        encoderOffset = new Rotation2d(Units.degreesToRadians(0.0));
        break;
      case 1: // Front Right
        driveMotor = new TalonFX(4, Constants.SWERVE_CAN_BUS);
        turnMotor = new TalonFX(5, Constants.SWERVE_CAN_BUS);
        absoluteEncoder = new CANcoder(6, Constants.SWERVE_CAN_BUS);
        encoderOffset = new Rotation2d(Units.degreesToRadians(0.0));
        break;
      case 2: // Back Left
        driveMotor = new TalonFX(7, Constants.SWERVE_CAN_BUS);
        turnMotor = new TalonFX(8, Constants.SWERVE_CAN_BUS);
        absoluteEncoder = new CANcoder(9, Constants.SWERVE_CAN_BUS);
        encoderOffset = new Rotation2d(Units.degreesToRadians(0.0));
        break;
      case 3: // Back Right
        driveMotor = new TalonFX(10, Constants.SWERVE_CAN_BUS);
        turnMotor = new TalonFX(11, Constants.SWERVE_CAN_BUS);
        absoluteEncoder = new CANcoder(12, Constants.SWERVE_CAN_BUS);
        encoderOffset = new Rotation2d(Units.degreesToRadians(0.0));
        break;
      default:
        throw new RuntimeException("Invalid swerve module index");
    }
    // Configure devices
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
    turnMotor.setNeutralMode(NeutralModeValue.Brake);

    driveMotor.setInverted(false);
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
    volts = MathUtil.clamp(volts, -11.5, 11.5);
    driveMotor.setVoltage(volts);
  }

  public void setTurnVoltage(double volts) {
    volts = MathUtil.clamp(volts, -11, 11);
    turnMotor.setVoltage(volts);
  }

  public void setDriveBrakeMode(boolean enable) {}

  public void setTurnBrakeMode(boolean enable) {}
}
