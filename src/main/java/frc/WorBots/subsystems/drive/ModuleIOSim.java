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

public class ModuleIOSim implements ModuleIO {
  private FlywheelSim driveSim = new FlywheelSim(DCMotor.getKrakenX60(1), 6.75, 0.025);
  private FlywheelSim turnSim = new FlywheelSim(DCMotor.getKrakenX60(1), 150.0 / 7.0, 0.004);

  private double turnRelativePositionRad = 0.0;
  private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(0.02);
    turnSim.update(0.02);

    double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * 0.02;
    turnRelativePositionRad += angleDiffRad;
    turnAbsolutePositionRad += angleDiffRad;
    while (turnAbsolutePositionRad < 0) {
      turnAbsolutePositionRad += 2.0 * Math.PI;
    }
    while (turnAbsolutePositionRad > 2.0 * Math.PI) {
      turnAbsolutePositionRad -= 2.0 * Math.PI;
    }

    inputs.drivePositionRad += (driveSim.getAngularVelocityRadPerSec() * 0.02);
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};
    inputs.driveTempCelcius = new double[] {};

    inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
    inputs.turnPositionRad = turnRelativePositionRad;
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(turnSim.getCurrentDrawAmps())};
    inputs.turnTempCelcius = new double[] {};
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
