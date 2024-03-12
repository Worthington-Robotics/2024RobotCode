// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.superstructure;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;

public class SuperstructureIOTalon implements SuperstructureIO {
  private final TalonFX elevator;
  private final boolean isPivotInverted = true;

  private final TalonFX pivot;
  private final DutyCycleEncoder pivotAbsEncoder;
  private final Encoder pivotRelEncoder;

  private final TalonSignalsPositional elevatorSignals;
  private final TalonSignalsPositional pivotSignals;

  // Constants
  private static final double MAX_ELEVATION_ROTATIONS = (158.1 * 0.97) / 3.0;
  private static final double ELEVATOR_GEARING =
      591.156 / 3.0; // In meter per revolution of the encoder

  public SuperstructureIOTalon() {
    elevator = new TalonFX(3);
    pivot = new TalonFX(10);
    pivotAbsEncoder = new DutyCycleEncoder(9);
    pivotRelEncoder = new Encoder(8, 7);
    pivotRelEncoder.setDistancePerPulse((2 * Math.PI) / 2048);

    resetElevator();
    elevator.setInverted(false);
    elevator.setNeutralMode(NeutralModeValue.Brake);
    pivot.setNeutralMode(NeutralModeValue.Brake);
    pivot.setInverted(true);

    elevatorSignals = new TalonSignalsPositional(elevator);

    elevator.optimizeBusUtilization();

    pivotSignals = new TalonSignalsPositional(pivot);
    pivot.optimizeBusUtilization();
  }

  public void setElevatorVoltage(double volts) {
    elevatorSignals.setTalonVoltage(elevator, volts, 6.0);
  }

  public void setPivotVoltage(double volts) {
    pivotSignals.setTalonVoltage(pivot, volts, 11);
  }

  public void updateInputs(SuperstructureIOInputs inputs) {
    elevatorSignals.update(inputs.elevator, elevator);
    pivotSignals.update(inputs.pivot, pivot);

    inputs.elevatorPositionMeters = elevator.getPosition().getValue() / ELEVATOR_GEARING;
    inputs.elevatorVelocityMetersPerSec =
        inputs.elevator.velocityRadsPerSec / ELEVATOR_GEARING / (2 * Math.PI);
    inputs.elevatorPercentageRaised =
        inputs.elevator.positionRads / MAX_ELEVATION_ROTATIONS / (2 * Math.PI);
    inputs.elevatorCurrentAmps = elevator.getStatorCurrent().getValue();

    final double pivotSign = (isPivotInverted ? 1.0 : -1.0);
    inputs.pivotPositionAbsRad = pivotAbsEncoder.get() * 2 * Math.PI * pivotSign;
    inputs.pivotPositionRelRad = (pivotRelEncoder.getDistance()) * pivotSign;
    inputs.pivot.velocityRadsPerSec = pivotRelEncoder.getRate() * pivotSign;
    inputs.pivot.temperatureCelsius = pivot.getDeviceTemp().getValue();
    inputs.pivot.supplyVoltage = pivot.getMotorVoltage().getValue();
    inputs.pivot.isConnected = pivot.isAlive() && pivotAbsEncoder.isConnected();
  }

  public void resetElevator() {
    elevator.setPosition(0.0);
  }
}
