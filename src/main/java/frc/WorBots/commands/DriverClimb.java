// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
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

public class DriverClimb extends Command {
  private Supplier<Double> joystickValue;
  private Superstructure superstructure;

  public DriverClimb(Superstructure superstructure, Supplier<Double> joystickValue) {
    this.joystickValue = joystickValue;
    addRequirements(superstructure);
  }

  @Override
  public void initialize() {
    superstructure.setMode(SuperstructureState.CLIMBING);
  }

  @Override
  public void execute() {
    double joystick = MathUtil.applyDeadband(joystickValue.get(), 0.09);
    double volts = joystick * 10.0;
    superstructure.setClimbingVolts(volts);
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.setMode(SuperstructureState.POSE);
  }
}
