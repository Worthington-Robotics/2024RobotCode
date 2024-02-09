// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.superstructure.Superstructure;
import java.util.function.Supplier;

public class DriverShootingTest extends Command {
  private Supplier<Double> pivotValue;
  private Supplier<Double> shootValue;
  private Supplier<Boolean> feederButton;
  private final Superstructure superstructure;
  private final Shooter shooter;

  public DriverShootingTest(
      Superstructure superstructure,
      Shooter shooter,
      Supplier<Double> pivotValue,
      Supplier<Double> shootValue,
      Supplier<Boolean> feederButton) {
    this.pivotValue = pivotValue;
    this.shootValue = shootValue;
    this.feederButton = feederButton;
    this.superstructure = superstructure;
    this.shooter = shooter;
    addRequirements(superstructure, shooter);
  }

  @Override
  public void initialize() {
    // superstructure.setModeVoid(SuperstructureState.SHOOTING);
    superstructure.setManualPivotVolts(0.0);
  }

  @Override
  public void execute() {
    double pivot = MathUtil.applyDeadband(pivotValue.get(), 0.09);
    double pivotVolts = pivot * 1.0;
    shooter.setRawFeederVolts(pivotVolts);
    double shoot = MathUtil.applyDeadband(shootValue.get(), 0.09);
    double shootVolts = shoot * 12;
    shooter.setRawFlywheelSpeed(shootVolts);
    // if (feederButton.get()) {
    //   shooter.setRawFeederVolts(1.0);
    // } else {
    //   shooter.setRawFeederVolts(0.0);
    // }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.setManualPivotVolts(0.0);
    // superstructure.setModeVoid(SuperstructureState.POSE);
    shooter.setRawFlywheelSpeed(0.0);
    shooter.setRawFeederVolts(0.0);
  }
}
