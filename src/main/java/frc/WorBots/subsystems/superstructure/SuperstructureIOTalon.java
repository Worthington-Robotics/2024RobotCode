// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.superstructure;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class SuperstructureIOTalon implements SuperstructureIO {
  private final TalonFX elevator;
  private final TalonFX elevatorFollower;
  private final boolean isElevatorInverted = false;

  private final TalonFX pivot;
  private final boolean isPivotInverted = false;
  private final DutyCycleEncoder pivotAbsEncoder;
  private final Encoder pivotRelEncoder;

  private static final double elevatorGearing = 1.0; // in meters per revolution

  public SuperstructureIOTalon() {
    elevator = new TalonFX(0);
    elevatorFollower = new TalonFX(0);
    pivot = new TalonFX(0);
    pivotAbsEncoder = new DutyCycleEncoder(0);
    pivotRelEncoder = new Encoder(0, 0);
    elevator.setPosition(0.0);
    elevator.setInverted(isElevatorInverted);
    elevatorFollower.setInverted(!isElevatorInverted);
  }

  public void setElevatorVoltage(double volts) {
    volts = MathUtil.clamp(volts, -10.5, 10.5);
    elevator.setVoltage(volts);
    elevatorFollower.setVoltage(volts);
  }

  public void setPivotVoltage(double volts) {
    volts = MathUtil.clamp(volts, -10.5, 10.5);
    pivot.setVoltage(volts);
  }

  public void updateInputs(SuperstructureIOInputs inputs) {
    inputs.elevatorPositionMeters =
        (elevator.getPosition().getValue() / elevatorGearing) * (isElevatorInverted ? 1.0 : -1.0);
    inputs.elevatorVelocityMetersPerSec =
        (elevator.getVelocity().getValue() / elevatorGearing) * (isElevatorInverted ? 1.0 : -1.0);
    inputs.elevatorVoltage = elevator.getMotorVoltage().getValue();
    inputs.elevatorTemp = elevator.getDeviceTemp().getValue();
    inputs.elevatorConnected = elevator.isAlive();

    final double pivotSign = (isPivotInverted ? 1.0 : -1.0);
    inputs.pivotPositionAbsRad = pivotAbsEncoder.get() * pivotSign;
    inputs.pivotPositionRelRad = pivotRelEncoder.getDistance() * pivotSign;
    inputs.pivotVelocityRadPerSec = pivotRelEncoder.getRate() * pivotSign;
    inputs.pivotTemp = pivot.getDeviceTemp().getValue();
    inputs.pivotVoltageApplied = pivot.getMotorVoltage().getValue();
    inputs.pivotConnected = pivot.isAlive() && pivotAbsEncoder.isConnected();
  }
}
