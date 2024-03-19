// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.superstructure;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.WorBots.Constants;

public class SuperstructureIOSim implements SuperstructureIO {
  private ElevatorSim elevator;
  private SingleJointedArmSim pivot;

  public SuperstructureIOSim() {
    elevator =
        new ElevatorSim(
            DCMotor.getKrakenX60(2),
            81.0,
            Units.lbsToKilograms(12),
            Units.inchesToMeters(1.75 / 2),
            0,
            0.26,
            true,
            0.0);
    pivot =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            112.5,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(12), Units.lbsToKilograms(10)),
            Units.inchesToMeters(12),
            0.0,
            Superstructure.PIVOT_MAX_ANGLE,
            true,
            0);
  }

  public void setElevatorVoltage(double volts) {
    elevator.setInputVoltage(volts);
  }

  public void setPivotVoltage(double volts) {
    pivot.setInputVoltage(volts);
  }

  public void updateInputs(SuperstructureIOInputs inputs) {
    elevator.update(Constants.ROBOT_PERIOD);
    pivot.update(Constants.ROBOT_PERIOD);

    inputs.elevatorPositionMeters = elevator.getPositionMeters();
    inputs.elevatorVelocityMetersPerSec = elevator.getVelocityMetersPerSecond();

    inputs.pivotPositionAbsRad = 0.0;
    inputs.pivotPositionRelRad = pivot.getAngleRads();
    inputs.elevatorVelocityMetersPerSec = pivot.getVelocityRadPerSec();
    inputs.elevatorPercentageRaised =
        inputs.elevatorPositionMeters / Units.inchesToMeters(33 - 5.74);
    inputs.bottomLimitReached = elevator.hasHitLowerLimit();
    inputs.topLimitReached = elevator.hasHitUpperLimit();
  }

  public void resetElevator() {
    elevator.setState(0.0, elevator.getVelocityMetersPerSecond());
  }
}
