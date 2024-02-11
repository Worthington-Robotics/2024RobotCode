// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.intake.Intake;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.Preset;

public class SmartIntake extends Command {
  private final Intake intake;
  private final Shooter shooter;
  private final Superstructure superstructure;

  private boolean finished = false;

  private static final double intakeVolts = 4.0;
  private static final double feederVolts = 1.0;

  public SmartIntake(Intake intake, Shooter shooter, Superstructure superstructure) {
    this.intake = intake;
    this.shooter = shooter;
    this.superstructure = superstructure;
    // Don't add superstructure as a requirement
    addRequirements(intake, shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Check if we are in handoff pose
    final boolean inHandoff =
        superstructure.isNearPose(
            Preset.HANDOFF, Units.inchesToMeters(0.5), Units.degreesToRadians(10));
    // If we are ready for handoff, intake up to the shooter
    if (inHandoff) {
      intake.setVolts(intakeVolts);
      if (!shooter.hasGamePiece()) {
        shooter.setRawFeederVolts(feederVolts);
      } else {
        finished = true;
      }
    } else {
      // Otherwise, intake up to the intake
      if (!intake.hasGamePiece()) {
        intake.setVolts(intakeVolts);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

  @Override
  public void end(boolean interrupted) {
    intake.setVolts(0);
    shooter.setRawFeederVolts(0);
  }
}
