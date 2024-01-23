// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.drive.Drive;

public class Orbit extends Command {
  private Drive drive;
  // Starting position used to calculate where to put the center of rotation
  private Translation2d startPosition;

  public Orbit(Drive drive) {
    addRequirements(drive);
    this.drive = drive;
  }

  @Override
  public void initialize() {
    this.startPosition = drive.getPose().getTranslation();
  }

  @Override
  public void execute() {
    final Translation2d currentPosition = drive.getPose().getTranslation();
    final Translation2d difference = startPosition.minus(currentPosition);
    drive.setCenterOfRotation(difference);
  }

  @Override
  public void end(boolean interrupted) {
    drive.setCenterOfRotation(new Translation2d());
  }
}
