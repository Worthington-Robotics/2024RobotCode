// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.WorBots.Constants;

public class ShooterIOSim implements ShooterIO {
  FlywheelSim topFlywheelSim;
  FlywheelSim bottomFlywheelSim;
  FlywheelSim feederWheel;

  public ShooterIOSim() {
    topFlywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.5);
    bottomFlywheelSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.5);
    feederWheel = new FlywheelSim(DCMotor.getCIM(1), 100, 2);
  }

  @Override
  public void setBottomFlywheelVolts(double volts) {
    bottomFlywheelSim.setInputVoltage(volts);
  }

  @Override
  public void setTopFlywheelVolts(double volts) {
    topFlywheelSim.setInputVoltage(volts);
  }

  @Override
  public void setFeederWheelVoltage(double volts) {
    feederWheel.setInputVoltage(volts);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    topFlywheelSim.update(Constants.ROBOT_PERIOD);
    bottomFlywheelSim.update(Constants.ROBOT_PERIOD);
    feederWheel.update(Constants.ROBOT_PERIOD);
    inputs.feederWheel.isConnected = true;
    inputs.feederWheel.velocityRadsPerSec = feederWheel.getAngularVelocityRadPerSec();
    inputs.isConnected = true;
    inputs.velocityRPMBottom = bottomFlywheelSim.getAngularVelocityRPM();
    inputs.velocityRPMTop = topFlywheelSim.getAngularVelocityRPM();
    inputs.feederWheel.positionRads = 0.0;
    inputs.timeOfFlightDistanceMeters = (feederWheel.getAngularVelocityRPM() <= 0.01) ? 0.02 : 0.2;
  }
}
