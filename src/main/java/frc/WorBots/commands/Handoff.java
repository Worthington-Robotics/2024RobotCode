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
import frc.WorBots.subsystems.superstructure.Superstructure.*;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.*;

/**
 * This command waits for the driver to intake a game piece, then moves it into the shooter to prep
 * it for shooting.
 */
public class Handoff extends Command {
  private Superstructure superstructure;
  private Intake intake;
  private Shooter shooter;

  public Handoff(Intake intake, Superstructure superstructure, Shooter shooter) {
    this.intake = intake;
    this.superstructure = superstructure;
    this.shooter = shooter;
    addRequirements(intake, superstructure, shooter);
  }

  @Override
  public void initialize() {
    superstructure.setModeVoid(SuperstructureState.POSE);
    superstructure.setPose(Preset.HANDOFF);
  }

  @Override
  public void execute() {
    if (intake.hasGamePiece() && superstructure.isAtSetpoint()) {
      intake.handoff().execute();
      shooter.runFeederWheel(3.0);
    }
  }

  @Override
  public boolean isFinished() {
    return shooter.hasGamePiece();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.runFeederWheel(0.0);
    superstructure.setPose(Preset.HOME);
    superstructure.setModeVoid(SuperstructureState.POSE);
  }
}
