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
 * This command waits for the driver to intake a game piece, then moves it into the shooter to prep
 * it for shooting.
 */
public class Handoff extends Command {
  private final Superstructure superstructure;
  private final Intake intake;
  private final Shooter shooter;

  // Constants
  /** Max voltage to run the intake at */
  private static final double MAX_INTAKE_VOLTAGE = 5.7;

  /** The amount to scale the intake power based on ToF distance. Larger values reduce the speed */
  private static final double INTAKE_DISTANCE_SCALING = 0.44;

  /** Amount to multiply the intake voltage by when we are in handoff */
  private static final double HANDOFF_INTAKE_MULTIPLIER = 1.2;

  /** Voltage for the feeder wheels */
  private static final double FEEDER_VOLTAGE = 0.5;

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
        shooter.setRawFeederVolts(FEEDER_VOLTAGE);
      } else {
        intake.setVolts(0.0);
        shooter.setRawFeederVolts(0.0);
      }
    } else {
      if (!intake.hasGamePiece()) {
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
