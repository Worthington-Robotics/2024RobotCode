// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.WorBots.Constants;

public class ModuleIOSim implements ModuleIO {
  private FlywheelSim driveSim = new FlywheelSim(DCMotor.getKrakenX60(1), 6.75, 0.025);
  private FlywheelSim turnSim = new FlywheelSim(DCMotor.getKrakenX60(1), 150.0 / 7.0, 0.004);

  private double turnRelativePositionRad = 0.0;
  private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(Constants.ROBOT_PERIOD);
    turnSim.update(Constants.ROBOT_PERIOD);

    double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * Constants.ROBOT_PERIOD;
    turnRelativePositionRad += angleDiffRad;
    turnAbsolutePositionRad += angleDiffRad;
    while (turnAbsolutePositionRad < 0) {
      turnAbsolutePositionRad += 2.0 * Math.PI;
    }
    while (turnAbsolutePositionRad > 2.0 * Math.PI) {
      turnAbsolutePositionRad -= 2.0 * Math.PI;
    }

    inputs.drive.positionRads += (driveSim.getAngularVelocityRadPerSec() * Constants.ROBOT_PERIOD);
    inputs.drive.velocityRadsPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.drive.appliedPowerVolts = driveAppliedVolts;
    inputs.drive.currentDrawAmps = Math.abs(driveSim.getCurrentDrawAmps());
    inputs.drive.temperatureCelsius = 30.0;

    inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
    inputs.turn.positionRads = turnRelativePositionRad;
    inputs.turn.velocityRadsPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turn.appliedPowerVolts = turnAppliedVolts;
    inputs.turn.currentDrawAmps = Math.abs(turnSim.getCurrentDrawAmps());
    inputs.turn.temperatureCelsius = 30.0;
    inputs.isConnected = true;
  }

  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }
}
