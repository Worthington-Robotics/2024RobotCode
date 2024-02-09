// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.Preset;

public class PoseCommands {
  /** Turns to the alliance amp wall and moves the superstructure to amp pose */
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
        .andThen(Commands.waitSeconds(3.5))
        .andThen(
            () -> {
              drive.removeThetaSetpoint();
            })
        .handleInterrupt(
            () -> {
              drive.removeThetaSetpoint();
            });
  }

  /** Automatically climbs the chain when under it */
  public static Command autoClimb(Drive drive, Superstructure superstructure) {
    return Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-0.5, 0.0, 0.0)), drive)
        .withTimeout(0.6)
        .alongWith(superstructure.setPose(Preset.START_CLIMB))
        .andThen(Commands.waitSeconds(2.0))
        .andThen(
            Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.5, 0.0, 0.0)), drive)
                .withTimeout(1.2)
                .alongWith(superstructure.setPose(Preset.CLIMB)));
  }

  /** Automatically gets down from the chain and drives out */
  public static Command climbDown(Drive drive, Superstructure superstructure) {
    return superstructure
        .setPose(Preset.START_CLIMB)
        .andThen(Commands.waitSeconds(3))
        .andThen(
            new ParallelCommandGroup(
                superstructure.setPose(Preset.HOME),
                Commands.waitSeconds(1.25)
                    .andThen(
                        Commands.run(
                                () -> drive.runVelocity(new ChassisSpeeds(0.5, 0.0, 0.0)), drive)
                            .withTimeout(1.0))));
  }

  /** Return the drive base and superstructure back to their zero positions */
  public static Command fullZero(Drive drive, Superstructure superstructure) {
    return superstructure
        .setPose(Preset.HOME)
        .alongWith(Commands.runOnce(() -> drive.setSingleThetaSetpoint(new Rotation2d())))
        .andThen(Commands.waitSeconds(3.5).andThen(drive.removeThetaSetpointCommand()));
  }
}
