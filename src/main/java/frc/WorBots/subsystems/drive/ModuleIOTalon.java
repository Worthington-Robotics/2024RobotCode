// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.WorBots.Constants;

public class ModuleIOTalon implements ModuleIO {
  private static final double DRIVE_ROTATIONS_TO_RADIANS =
      (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANcoder absoluteEncoder;
  private final Rotation2d encoderOffset;

  private final StatusSignal<Double> drivePosSignal;
  private final StatusSignal<Double> driveVelSignal;
  private final StatusSignal<Double> driveVoltsSignal;
  private final StatusSignal<Double> driveCurrentSignal;
  private final StatusSignal<Double> driveTempSignal;

  private final StatusSignal<Double> turnAbsPosSignal;
  private final StatusSignal<Double> turnPosSignal;
  private final StatusSignal<Double> turnVelSignal;
  private final StatusSignal<Double> turnVoltsSignal;
  private final StatusSignal<Double> turnCurrentSignal;
  private final StatusSignal<Double> turnTempSignal;

  public ModuleIOTalon(int index) {
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

    // Signals
    drivePosSignal = driveMotor.getPosition();
    driveVelSignal = driveMotor.getVelocity();
    driveVoltsSignal = driveMotor.getSupplyVoltage();
    driveCurrentSignal = driveMotor.getSupplyCurrent();
    driveTempSignal = driveMotor.getDeviceTemp();

    turnAbsPosSignal = absoluteEncoder.getAbsolutePosition();
    turnPosSignal = turnMotor.getPosition();
    turnVelSignal = turnMotor.getVelocity();
    turnVoltsSignal = turnMotor.getSupplyVoltage();
    turnCurrentSignal = turnMotor.getSupplyCurrent();
    turnTempSignal = turnMotor.getDeviceTemp();

    StatusSignal.setUpdateFrequencyForAll(
        100,
        drivePosSignal,
        driveVelSignal,
        driveVoltsSignal,
        driveCurrentSignal,
        driveTempSignal,
        turnAbsPosSignal,
        turnPosSignal,
        turnVelSignal,
        turnVoltsSignal,
        turnCurrentSignal,
        turnTempSignal);

    driveMotor.optimizeBusUtilization();
    turnMotor.optimizeBusUtilization();
    absoluteEncoder.optimizeBusUtilization();
  }

  public void updateInputs(ModuleIOInputs inputs) {
    StatusSignal.refreshAll(
        drivePosSignal,
        driveVelSignal,
        driveVoltsSignal,
        driveCurrentSignal,
        driveTempSignal,
        turnAbsPosSignal,
        turnPosSignal,
        turnVelSignal,
        turnVoltsSignal,
        turnCurrentSignal,
        turnTempSignal);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosSignal.getValue() * DRIVE_ROTATIONS_TO_RADIANS);
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelSignal.getValue() * DRIVE_ROTATIONS_TO_RADIANS);
    inputs.driveAppliedVolts = driveMotor.get() * driveVoltsSignal.getValue();
    inputs.driveCurrentAmps = new double[] {driveCurrentSignal.getValue()};
    inputs.driveTempCelcius = new double[] {driveTempSignal.getValue()};

    inputs.turnAbsolutePositionRad =
        MathUtil.angleModulus(
            Units.rotationsToRadians(turnAbsPosSignal.getValue()) - encoderOffset.getRadians());
    inputs.turnPositionRad = Units.rotationsToRadians(turnPosSignal.getValue());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelSignal.getValue());
    inputs.turnAppliedVolts = turnMotor.get() * turnVoltsSignal.getValue();
    inputs.turnCurrentAmps = new double[] {turnCurrentSignal.getValue()};
    inputs.turnTempCelcius = new double[] {turnTempSignal.getValue()};

    inputs.isConnected = turnMotor.isAlive() && driveMotor.isAlive();
  }

  public void setDriveVoltage(double volts) {
    volts = MathUtil.clamp(volts, -9, 9);
    driveMotor.setVoltage(volts);
  }

  public void setTurnVoltage(double volts) {
    volts = MathUtil.clamp(volts, -6, 6);
    turnMotor.setVoltage(volts);
  }

  public void setDriveBrakeMode(boolean enable) {}

  public void setTurnBrakeMode(boolean enable) {}
}
