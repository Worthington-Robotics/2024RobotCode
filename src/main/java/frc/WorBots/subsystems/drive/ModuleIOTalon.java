// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.WorBots.Constants;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;
import frc.WorBots.util.debug.TunablePIDController;
import frc.WorBots.util.debug.TunablePIDController.TunablePIDGains;

public class ModuleIOTalon implements ModuleIO {
  private static final double DRIVE_MULTIPLIER = 0.9579;
  // L3 gear ratio
  // (https://www.swervedrivespecialties.com/products/mk4i-swerve-module)
  private static final double DRIVE_GEAR_RATIO =
      (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0) * DRIVE_MULTIPLIER;

  private ModuleIOInputs inputs;

  private final SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(0.18868, 0.12825);
  private static final TunablePIDGains driveFeedbackGains =
      new TunablePIDGains("Drive/Gains", "SModule Drive Feedback");
  private final TunablePIDController driveFeedback = new TunablePIDController(driveFeedbackGains);
  private static final TunablePIDGains turnFeedbackGains =
      new TunablePIDGains("Drive/Gains", "SModule Turn Feedback");
  private final TunablePIDController turnFeedback = new TunablePIDController(turnFeedbackGains);

  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANcoder absoluteEncoder;

  private final Rotation2d encoderOffset;
  private final double wheelRadius;

  private final TalonSignalsPositional driveSignals;
  private final TalonSignalsPositional turnSignals;
  private final StatusSignal<Double> turnAbsPosSignal;

  public ModuleIOTalon(int index) {
    driveFeedbackGains.setGains(0.015, 0.001, 0.0);
    turnFeedbackGains.setGains(6.05, 0.019, 0.0);
    turnFeedback.pid.enableContinuousInput(-Math.PI, Math.PI);

    inputs = new ModuleIOInputs(index);

    switch (index) {
      case 0: // Front Left
        driveMotor = new TalonFX(1, Constants.SWERVE_CAN_BUS);
        turnMotor = new TalonFX(2, Constants.SWERVE_CAN_BUS);
        absoluteEncoder = new CANcoder(3, Constants.SWERVE_CAN_BUS);
        encoderOffset = new Rotation2d(2.783 + Units.degreesToRadians(180));
        wheelRadius = Units.inchesToMeters(2.0);
        break;
      case 1: // Front Right
        driveMotor = new TalonFX(4, Constants.SWERVE_CAN_BUS);
        turnMotor = new TalonFX(5, Constants.SWERVE_CAN_BUS);
        absoluteEncoder = new CANcoder(6, Constants.SWERVE_CAN_BUS);
        encoderOffset = new Rotation2d(0.996 + Units.degreesToRadians(0));
        wheelRadius = Units.inchesToMeters(2.0);
        break;
      case 2: // Back Left
        driveMotor = new TalonFX(7, Constants.SWERVE_CAN_BUS);
        turnMotor = new TalonFX(8, Constants.SWERVE_CAN_BUS);
        absoluteEncoder = new CANcoder(9, Constants.SWERVE_CAN_BUS);
        encoderOffset = new Rotation2d(2.692 + Units.degreesToRadians(180));
        wheelRadius = Units.inchesToMeters(2.0);
        break;
      case 3: // Back Right
        driveMotor = new TalonFX(10, Constants.SWERVE_CAN_BUS);
        turnMotor = new TalonFX(11, Constants.SWERVE_CAN_BUS);
        absoluteEncoder = new CANcoder(12, Constants.SWERVE_CAN_BUS);
        encoderOffset = new Rotation2d(0.756 + Units.degreesToRadians(0));
        wheelRadius = Units.inchesToMeters(2.0);
        break;
      default:
        throw new RuntimeException("Invalid swerve module index");
    }
    // Configure devices

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = 25;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveMotor.getConfigurator().apply(driveConfig);

    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    turnMotor.getConfigurator().refresh(turnConfig);
    turnConfig.CurrentLimits.SupplyCurrentLimit = 40;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnMotor.getConfigurator().apply(turnConfig);

    driveMotor.setNeutralMode(NeutralModeValue.Brake);
    turnMotor.setNeutralMode(NeutralModeValue.Brake);

    driveMotor.setInverted(false);
    turnMotor.setInverted(true);

    driveMotor.setPosition(0.0);
    turnMotor.setPosition(0.0);

    // Signals
    driveSignals = new TalonSignalsPositional(driveMotor);
    turnSignals = new TalonSignalsPositional(turnMotor);
    turnAbsPosSignal = absoluteEncoder.getAbsolutePosition();

    StatusSignal.setUpdateFrequencyForAll(100, turnAbsPosSignal);

    driveMotor.optimizeBusUtilization();
    turnMotor.optimizeBusUtilization();
    absoluteEncoder.optimizeBusUtilization();
  }

  public void updateInputs() {
    driveFeedback.update();
    turnFeedback.update();

    driveSignals.update(inputs.drive, driveMotor);
    turnSignals.update(inputs.turn, turnMotor);
    StatusSignal.refreshAll(turnAbsPosSignal);

    inputs.drive.positionRads *= DRIVE_GEAR_RATIO;
    inputs.drive.velocityRadsPerSec *= DRIVE_GEAR_RATIO;

    inputs.driveDistanceMeters = inputs.drive.positionRads * wheelRadius;
    inputs.driveVelocityMetersPerSec = inputs.drive.velocityRadsPerSec * wheelRadius;

    inputs.turnAbsolutePositionRad =
        MathUtil.angleModulus(
            Units.rotationsToRadians(turnAbsPosSignal.getValue()) - encoderOffset.getRadians());

    inputs.turnPositionErrorRad = turnFeedback.pid.getPositionError();

    inputs.isConnected = inputs.turn.isConnected && inputs.drive.isConnected;
  }

  public ModuleIOInputs getInputs() {
    return this.inputs;
  }

  public void setDriveSpeed(double speedMetersPerSecond) {
    final double velocityRadPerSec = speedMetersPerSecond / wheelRadius;
    final double driveVolts =
        driveFeedforward.calculate(velocityRadPerSec)
            + driveFeedback.pid.calculate(inputs.drive.velocityRadsPerSec, velocityRadPerSec);
    setDriveVoltage(driveVolts);
  }

  public void setAngle(double angleRadians) {
    setTurnVoltage(turnFeedback.pid.calculate(inputs.turnAbsolutePositionRad, angleRadians));
  }

  public void setDriveVoltage(double volts) {
    driveSignals.setVoltage(driveMotor, volts, 11.0);
  }

  public void setTurnVoltage(double volts) {
    turnSignals.setVoltage(turnMotor, volts, 11.0);
  }
}
