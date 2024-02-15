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
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;
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
    addRequirements(intake, superstructure, shooter);
  }

  @Override
  public void initialize() {
    superstructure.setModeVoid(SuperstructureState.POSE);
    superstructure.setPose(Preset.HANDOFF).schedule();
  }

  @Override
  public void execute() {
    // This check is here so we don't schedule the handoff every execution
    if (!hasBegun) {
      if (intake.hasGamePiece() && superstructure.isAtSetpoint()) {
        intake.handoff().schedule();
        shooter.runFeederWheel(3.0);
        hasBegun = true;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return shooter.hasGamePiece();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.runFeederWheel(0.0);
    superstructure.setModeVoid(SuperstructureState.DISABLED);
  }
}
