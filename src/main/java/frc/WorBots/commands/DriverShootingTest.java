// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;
import java.util.function.Supplier;

public class DriverShootingTest extends Command {
  private Supplier<Double> elevatorValue;
  private Supplier<Double> pivotValue;
  private Supplier<Double> shootValue;
  private Supplier<Boolean> feederButton;
  private final Superstructure superstructure;
  private final Shooter shooter;

  public DriverShootingTest(
      Superstructure superstructure,
      Shooter shooter,
      Supplier<Double> elevatorValue,
      Supplier<Double> pivotValue,
      Supplier<Double> shootValue,
      Supplier<Boolean> feederButton) {
    this.elevatorValue = elevatorValue;
    this.pivotValue = pivotValue;
    this.shootValue = shootValue;
    this.feederButton = feederButton;
    this.superstructure = superstructure;
    this.shooter = shooter;
    addRequirements(superstructure, shooter);
  }

  @Override
  public void initialize() {
    superstructure.setModeVoid(SuperstructureState.MANUAL);
    superstructure.setManualPivotVolts(0.0);
  }

  @Override
  public void execute() {
    double elevator = MathUtil.applyDeadband(elevatorValue.get(), 0.09);
    double elevatorVolts = elevator * 9.0;
    superstructure.setClimbingVolts(elevatorVolts);
    double pivot = MathUtil.applyDeadband(pivotValue.get(), 0.09);
    double pivotVolts = pivot * 5.0;
    superstructure.setManualPivotVolts(pivotVolts);
    double shoot = MathUtil.applyDeadband(shootValue.get(), 0.09);
    double shootRPM = shoot * 3000;
    shooter.setRawFlywheelSpeed(shootRPM);
    SmartDashboard.putNumber("Manual Shoot RPM", shootRPM);
    if (feederButton.get()) {
      shooter.setRawFeederVolts(0.6);
    } else {
      shooter.setRawFeederVolts(0.0);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.setManualPivotVolts(0.0);
    superstructure.setModeVoid(SuperstructureState.STATIC);
    shooter.setRawFlywheelSpeed(0.0);
    shooter.setRawFeederVolts(0.0);
  }
}
