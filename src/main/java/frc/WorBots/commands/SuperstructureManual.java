// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.climber.Climber;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;
import java.util.function.Supplier;

/** Command for manual control of the elevator and pivot using joysticks */
public class SuperstructureManual extends Command {
  // Constants
  /** Joystick deadband */
  private static final double DEADBAND = 0.1;

  /** Voltage multiplier for the elevator */
  private static final double ELEVATOR_VOLTS = 8.0;

  /** Voltage multiplier for the pivot */
  private static final double PIVOT_VOLTS = 6.0;

  /** Voltage multiplier for the climber */
  private static final double CLIMBER_VOLTS = 5.5;

  private final Supplier<Double> elevatorValue;
  private final Supplier<Double> pivotValue;
  private final Supplier<Double> climberValue;
  private final Superstructure superstructure;
  private final Climber climber;

  public SuperstructureManual(
      Superstructure superstructure,
      Climber climber,
      Supplier<Double> elevatorValue,
      Supplier<Double> pivotValue,
      Supplier<Double> climberValue) {
    this.elevatorValue = elevatorValue;
    this.pivotValue = pivotValue;
    this.climberValue = climberValue;
    this.superstructure = superstructure;
    this.climber = climber;
    addRequirements(superstructure, climber);
  }

  @Override
  public void initialize() {
    superstructure.setModeVoid(SuperstructureState.MANUAL);
  }

  @Override
  public void execute() {
    double joystick = MathUtil.applyDeadband(elevatorValue.get(), DEADBAND);
    double volts = joystick * ELEVATOR_VOLTS;
    superstructure.setManualElevatorVolts(volts);
    joystick = MathUtil.applyDeadband(pivotValue.get(), DEADBAND);
    volts = joystick * PIVOT_VOLTS;
    superstructure.setManualPivotVolts(volts);
    joystick = MathUtil.applyDeadband(climberValue.get(), DEADBAND);
    volts = joystick * CLIMBER_VOLTS;
    climber.setManualClimberVolts(volts);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.setModeVoid(SuperstructureState.DISABLED);
    superstructure.setManualElevatorVolts(0.0);
    superstructure.setManualPivotVolts(0.0);
    climber.setManualClimberVolts(0.0);
  }
}
