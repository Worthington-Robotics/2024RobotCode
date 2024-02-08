// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.superstructure;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;

public class SuperstructureIOTalon implements SuperstructureIO {
  private final TalonFX elevator;
  private final TalonFX elevatorFollower;
  private final DigitalInput bottomLimitSwitch = new DigitalInput(7);
  private final DigitalInput topLimitSwitch = new DigitalInput(8);

  // private final TalonFX pivot;
  private final boolean isPivotInverted = false;
  // private final DutyCycleEncoder pivotAbsEncoder;
  // private final Encoder pivotRelEncoder;

  private final TalonSignalsPositional elevatorSignals;
  // private final TalonSignalsPositional pivotSignals;

  // Constants
  private static final double maxElevationRotations = 158.1;
  private static final double elevatorGearing = 591.156; // in meter per rotation of 1st carriage

  public SuperstructureIOTalon() {
    elevator = new TalonFX(2);
    elevatorFollower = new TalonFX(3);
    // pivot = new TalonFX(0);
    // pivotAbsEncoder = new DutyCycleEncoder(0);
    // pivotRelEncoder = new Encoder(0, 0);
    elevator.setPosition(0.0);
    elevator.setInverted(false);
    elevatorFollower.setInverted(false);
    elevatorFollower.setControl(new Follower(elevator.getDeviceID(), true));

    elevatorSignals = new TalonSignalsPositional(elevator);
    elevator.optimizeBusUtilization();
    // pivotSignals = new TalonSignalsPositional(pivot);
    // pivot.optimizeBusUtilization();
  }

  public void setElevatorVoltage(double volts) {
    elevatorSignals.setTalonVoltage(elevator, volts, 10);
  }

  public void setPivotVoltage(double volts) {
    // pivotSignals.setTalonVoltage(pivot, volts, 10.5);
  }

  public void updateInputs(SuperstructureIOInputs inputs) {
    elevatorSignals.update(inputs.elevator, elevator);
    // pivotSignals.update(inputs.pivot, pivot);

    inputs.elevatorPositionMeters = elevator.getPosition().getValue() / elevatorGearing;
    inputs.elevatorVelocityMetersPerSec =
        inputs.elevator.velocityRadsPerSec / elevatorGearing / (2 * Math.PI);
    inputs.elevatorPercentageRaised =
        inputs.elevator.positionRads / maxElevationRotations / (2 * Math.PI);

    // Enable when limit switches are added
    // inputs.bottomLimitReached = bottomLimitSwitch.get();
    // inputs.topLimitReached = topLimitSwitch.get();

    // final double pivotSign = (isPivotInverted ? 1.0 : -1.0);
    // inputs.pivotPositionAbsRad = pivotAbsEncoder.get() * pivotSign;
    // inputs.pivotPositionRelRad = pivotRelEncoder.getDistance() * pivotSign;
    // inputs.pivot.velocityRadsPerSec = pivotRelEncoder.getRate() * pivotSign;
    // inputs.pivot.isConnected &= pivotAbsEncoder.isConnected();
  }

  public void resetElevator() {
    elevator.setPosition(0.0);
  }
}
