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
import frc.WorBots.util.debug.TunableDouble;

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
  private static final double MAX_INTAKE_VOLTAGE = 8.2;

  /** The amount to scale the intake power based on ToF distance. Larger values reduce the speed */
  private static final double INTAKE_DISTANCE_SCALING = 0.56;

  /** Amount to multiply the intake voltage by when we are in handoff */
  private static final TunableDouble HANDOFF_INTAKE_MULTIPLIER =
      new TunableDouble("Tuning", "Handoff", "Handoff Intake Multiplier", 4.0);

  /** Static gain for the intake */
  private static final TunableDouble INTAKE_STATIC_GAIN =
      new TunableDouble("Tuning", "Handoff", "Intake Static Gain", 1.0);

  /** Static gain for the feeder */
  private static final TunableDouble FEEDER_STATIC_GAIN =
      new TunableDouble("Tuning", "Handoff", "Feeder Static Gain", 0.35);

  public Handoff(Intake intake, Superstructure superstructure, Shooter shooter) {
    this.intake = intake;
    this.superstructure = superstructure;
    this.shooter = shooter;
    addRequirements(intake, shooter);
  }

  @Override
  public void execute() {
    // Intake up to feeder
    if (superstructure.inHandoff()) {
      double intakeVolts =
          shooter.getNotePositionDistance() * HANDOFF_INTAKE_MULTIPLIER.get() * MAX_INTAKE_VOLTAGE;
      intakeVolts =
          MathUtil.clamp(
              intakeVolts + Math.signum(intakeVolts) * INTAKE_STATIC_GAIN.get(),
              -MAX_INTAKE_VOLTAGE,
              MAX_INTAKE_VOLTAGE);
      intake.setVolts(intakeVolts);
      shooter.setRawFeederVolts(Math.signum(intakeVolts) * FEEDER_STATIC_GAIN.get());
    } else {
      // Intake only up to intake
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
    return shooter.isGamePieceInPosition();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setRawFeederVolts(0.0);
    intake.setVolts(0.0);
  }
}
