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
import frc.WorBots.util.HardwareUtils;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;

public class ModuleIOTalon implements ModuleIO {
  private static final double DRIVE_ROTATIONS_TO_RADIANS =
      (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANcoder absoluteEncoder;
  private final Rotation2d encoderOffset;

  private final TalonSignalsPositional driveSignals;
  private final TalonSignalsPositional turnSignals;
  private final StatusSignal<Double> turnAbsPosSignal;

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
    driveSignals = new TalonSignalsPositional(driveMotor);
    turnSignals = new TalonSignalsPositional(turnMotor);
    turnAbsPosSignal = absoluteEncoder.getAbsolutePosition();

    StatusSignal.setUpdateFrequencyForAll(120, turnAbsPosSignal);

    driveMotor.optimizeBusUtilization();
    turnMotor.optimizeBusUtilization();
    absoluteEncoder.optimizeBusUtilization();
  }

  public void updateInputs(ModuleIOInputs inputs) {
    driveSignals.update(inputs.drive, driveMotor);
    turnSignals.update(inputs.turn, turnMotor);
    StatusSignal.refreshAll(turnAbsPosSignal);

    inputs.drive.positionRads *= DRIVE_ROTATIONS_TO_RADIANS;
    inputs.drive.velocityRadsPerSec *= DRIVE_ROTATIONS_TO_RADIANS;

    inputs.turnAbsolutePositionRad =
        MathUtil.angleModulus(
            Units.rotationsToRadians(turnAbsPosSignal.getValue()) - encoderOffset.getRadians());

    inputs.isConnected = inputs.turn.isConnected && inputs.drive.isConnected;
  }

  public void setDriveVoltage(double volts) {
    final double limit =
        (driveSignals.getSupplyVoltage() < HardwareUtils.idealBatteryVoltage) ? 6.8 : 9;
    driveSignals.setTalonVoltage(driveMotor, volts, limit);
  }

  public void setTurnVoltage(double volts) {
    turnSignals.setTalonVoltage(turnMotor, volts, 6);
  }

  public void setDriveBrakeMode(boolean enable) {}

  public void setTurnBrakeMode(boolean enable) {}
}
