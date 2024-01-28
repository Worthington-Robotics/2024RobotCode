// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

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
        driveMotor = new TalonFX(8);
        turnMotor = new TalonFX(7);
        absoluteEncoder = new CANcoder(2);
        encoderOffset = new Rotation2d(Units.degreesToRadians(76.1));
        break;
      case 1:
        driveMotor = new TalonFX(6);
        turnMotor = new TalonFX(5);
        absoluteEncoder = new CANcoder(1);
        encoderOffset = new Rotation2d(Units.degreesToRadians(292.236));
        break;
      case 2:
        driveMotor = new TalonFX(12);
        turnMotor = new TalonFX(11);
        absoluteEncoder = new CANcoder(4);
        encoderOffset = new Rotation2d(Units.degreesToRadians(219.7));
        break;
      case 3:
        driveMotor = new TalonFX(10);
        turnMotor = new TalonFX(9);
        absoluteEncoder = new CANcoder(3);
        encoderOffset = new Rotation2d(Units.degreesToRadians(33.8));
        break;
      default:
        throw new RuntimeException("Invalid swerve module index");
    }
    driveMotor.setInverted(true);
    turnMotor.setInverted(true);
  }

  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(
            driveMotor.getPosition().getValue() * ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)));
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(
            driveMotor.getVelocity().getValue() * ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)));
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

  public void setDriveBrakeMode(boolean enable) {}

  public void setTurnBrakeMode(boolean enable) {}
}
