// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.intake.Intake;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.Preset;

/** A command routine that runs a quick test of all robot systems */
public class PitTest extends SequentialCommandGroup {
  private final Drive drive;
  private final Superstructure superstructure;
  private final Intake intake;
  private final Shooter shooter;

  public PitTest(Drive drive, Superstructure superstructure, Intake intake, Shooter shooter) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.intake = intake;
    this.shooter = shooter;
    addRequirements(drive, superstructure, intake, shooter);
    addCommands(
        testDrive(),
        UtilCommands.waitForDriverstationButton(),
        testSuperstructure(),
        UtilCommands.waitForDriverstationButton(),
        testIntake());
  }

  private Command testDrive() {
    return Commands.sequence(
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0))).withTimeout(2.0),
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 1.0, 0.0))).withTimeout(2.0));
  }

  private Command testSuperstructure() {
    return Commands.sequence(
        superstructure.setPose(Preset.HANDOFF),
        UtilCommands.waitForDriverstationButton(),
        superstructure.setPose(Preset.AMP),
        UtilCommands.waitForDriverstationButton(),
        superstructure.setPose(Preset.HANDOFF));
  }

  private Command testIntake() {
    return Commands.sequence(new Handoff(intake, superstructure, shooter));
  }
}
