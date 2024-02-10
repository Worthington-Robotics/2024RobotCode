// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;
import java.util.function.Supplier;

public class SuperstructureManual extends Command {
  private Supplier<Double> elevatorValue;
  private Supplier<Double> pivotValue;
  private Superstructure superstructure;

  public SuperstructureManual(
      Superstructure superstructure, Supplier<Double> elevatorValue, Supplier<Double> pivotValue) {
    this.elevatorValue = elevatorValue;
    this.pivotValue = pivotValue;
    this.superstructure = superstructure;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.setModeVoid(SuperstructureState.MANUAL);
  }

  @Override
  public void execute() {
    double joystick = MathUtil.applyDeadband(elevatorValue.get(), 0.09);
    double volts = joystick * 9.0;
    superstructure.setClimbingVolts(volts);
    joystick = MathUtil.applyDeadband(pivotValue.get(), 0.09);
    volts = joystick * 6.0;
    superstructure.setManualPivotVolts(volts);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.setModeVoid(SuperstructureState.POSE);
    superstructure.setClimbingVolts(0.0);
    superstructure.setManualPivotVolts(0.0);
  }
}
