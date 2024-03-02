// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.WorBots.Constants;

public class IntakeIOSim implements IntakeIO {
  FlywheelSim sim;

  public IntakeIOSim() {
    sim = new FlywheelSim(DCMotor.getKrakenX60(1), 1, 0.05);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    sim.update(Constants.ROBOT_PERIOD);
    inputs.motor.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.motor.currentDrawAmps = sim.getCurrentDrawAmps();
    inputs.isConnected = true;
    inputs.motor.isConnected = true;
  }
}
