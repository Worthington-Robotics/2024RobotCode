// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.superstructure;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SuperstructureIOSim implements SuperstructureIO {
  private ElevatorSim elevator;
  private SingleJointedArmSim pivot;

  public SuperstructureIOSim() {
    elevator =
        new ElevatorSim(
            DCMotor.getKrakenX60(2),
            81.0,
            Units.lbsToKilograms(12),
            Units.inchesToMeters(1.75/2),
            0,
            Units.inchesToMeters(33 - 5.74),
            true,
            Units.inchesToMeters(12));
    pivot =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), 100, 0.25, 0.07, -Math.PI / 6, Math.PI, true, 0);
  }

  public void setElevatorVoltage(double volts) {
    elevator.setInputVoltage(volts);
  }

  public void setPivotVoltage(double volts) {
    pivot.setInputVoltage(volts);
  }

  public void updateInputs(SuperstructureIOInputs inputs) {
    elevator.update(0.02);
    pivot.update(0.02);

    inputs.elevatorPositionMeters = elevator.getPositionMeters();
    inputs.elevatorVelocityMetersPerSec = elevator.getVelocityMetersPerSecond();

    inputs.pivotPositionAbsRad = 0.0;
    inputs.pivotPositionRelRad = pivot.getAngleRads();
    inputs.elevatorVelocityMetersPerSec = pivot.getVelocityRadPerSec();
  }
}
