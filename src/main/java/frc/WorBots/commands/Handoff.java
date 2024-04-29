// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.subsystems.intake.*;
import frc.WorBots.subsystems.shooter.*;
import frc.WorBots.subsystems.superstructure.*;

/**
 * This command does all of the smart intaking for the system. It will attempt to move the game
 * piece as far up as it can safely in either the intake or feeder
 */
public class Handoff extends Command {
  private final Superstructure superstructure;
  private final Intake intake;
  private final Shooter shooter;

  // Constants
  /** Max voltage to run the intake at */
  private static final double MAX_INTAKE_VOLTAGE = 6.45;

  /** The amount to scale the intake power based on ToF distance. Larger values reduce the speed */
  private static final double INTAKE_DISTANCE_SCALING = 0.56;

  /**
   * The amount to scale the feeder power based on ToF distance. Larger values increase the speed
   */
  private static final double FEEDER_DISTANCE_SCALING = 1.0;

  /** Amount to multiply the intake voltage by when we are in handoff */
  private static final double HANDOFF_INTAKE_MULTIPLIER = 1.0;

  /** Voltage for the feeder wheels */
  private static final double FEEDER_VOLTAGE = 0.50;

  public Handoff(Intake intake, Superstructure superstructure, Shooter shooter) {
    this.intake = intake;
    this.superstructure = superstructure;
    this.shooter = shooter;
    addRequirements(intake, shooter);
  }

  @Override
  public void execute() {
    if (superstructure.inHandoff()) {
      if (!shooter.hasGamePiece()) {
        intake.setVolts(MAX_INTAKE_VOLTAGE * HANDOFF_INTAKE_MULTIPLIER);
        final double volts =
            MathUtil.clamp(
                (shooter.getToFDistanceMeters() * FEEDER_DISTANCE_SCALING) * FEEDER_VOLTAGE,
                0,
                FEEDER_VOLTAGE);
        shooter.setRawFeederVolts(volts);
      } else {
        intake.setVolts(0.0);
        shooter.setRawFeederVolts(0.0);
      }
    } else {
      if (!intake.hasGamePiece() && !shooter.hasGamePiece()) {
        final double volts =
            MathUtil.clamp(
                (intake.getToFDistanceMeters() / INTAKE_DISTANCE_SCALING) * MAX_INTAKE_VOLTAGE,
                0,
                MAX_INTAKE_VOLTAGE);
        intake.setVolts(volts);
      } else {
        intake.setVolts(0.0);
      }
      shooter.setRawFeederVolts(0.0);
    }
  }

  @Override
  public boolean isFinished() {
    return shooter.hasGamePiece();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setRawFeederVolts(0.0);
    intake.setVolts(0.0);
  }
}
