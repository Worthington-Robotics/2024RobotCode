// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.Preset;

public class PoseCommands {
  /** Turns to the alliance amp and moves the superstructure to amp pose */
  public static Command amp(Drive drive, Superstructure superstructure) {
    return Commands.runOnce(
            () -> {
              Rotation2d angle = Rotation2d.fromDegrees(90);
              if (DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Blue) {
                angle = Rotation2d.fromDegrees(270);
              }
              drive.setSingleThetaSetpoint(angle);
            })
        .alongWith(superstructure.setPose(Preset.AMP))
        .andThen(Commands.waitSeconds(2.0))
        .andThen(
            () -> {
              drive.removeThetaSetpoint();
            })
        .handleInterrupt(
            () -> {
              drive.removeThetaSetpoint();
            });
  }
}
