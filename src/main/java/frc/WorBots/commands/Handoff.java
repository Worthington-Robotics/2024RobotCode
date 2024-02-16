// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.subsystems.intake.*;
import frc.WorBots.subsystems.shooter.*;
import frc.WorBots.subsystems.superstructure.*;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.*;

/**
 * This command waits for the driver to intake a game piece, then moves it into the shooter to prep
 * it for shooting.
 */
public class Handoff extends Command {
  private final Superstructure superstructure;
  private final Intake intake;
  private final Shooter shooter;

  /** Whether the pose is in place and the handoff has begun */
  private boolean hasBegun = false;

  public Handoff(Intake intake, Superstructure superstructure, Shooter shooter) {
    this.intake = intake;
    this.superstructure = superstructure;
    this.shooter = shooter;
    addRequirements(intake, shooter);
  }

  @Override
  public void execute() {
    if (superstructure.getCurrentPose() == Preset.HANDOFF && superstructure.isAtSetpoint()) {
      if (!shooter.hasGamePiece()) {
        intake.setVolts(4.0);
        shooter.setRawFeederVolts(-0.5);
      } else {
        shooter.setRawFeederVolts(0);
        intake.setVolts(0.0);
      }
    } else {
      if (intake.hasGamePiece() || shooter.hasGamePiece()) {
        intake.setVolts(0.0);
      } else {
        intake.setVolts(4.0);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setRawFeederVolts(0.0);
    intake.setVolts(0.0);
  }
}
