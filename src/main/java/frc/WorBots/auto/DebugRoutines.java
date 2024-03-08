// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.WorBots.commands.Handoff;
import frc.WorBots.commands.UtilCommands;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.intake.Intake;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.Preset;

public class DebugRoutines {
  // Subsystems
  private final Drive drive;
  private final Superstructure superstructure;
  private final Intake intake;
  private final Shooter shooter;

  public DebugRoutines(Drive drive, Superstructure superstructure, Intake intake, Shooter shooter) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.intake = intake;
    this.shooter = shooter;
  }

  /**
   * A very nice routine from 6328 that turns the robot in place for a bit to calculate the turn to
   * meters values (radii) of the wheels
   */
  public Command characterizeOdometry() {
    final double turnSpeed = Units.degreesToRadians(20);
    // The radius of the circle created by the drive modules
    final double driveRadius = Drive.WHEELBASE * Math.sqrt(2);

    return Commands.race(
            Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0, turnSpeed)), drive),
            // Start turning for half a second before starting to record data
            Commands.waitSeconds(0.5)
                .andThen(
                    () -> {
                      odometryStartingRotation = drive.getYaw();
                      odometryStartingPositions = drive.getModuleDistances();
                    })
                .andThen(Commands.waitSeconds(20.0))
                .andThen(
                    () -> {
                      final Rotation2d finalRotation = drive.getYaw();
                      final double[] finalPositions = drive.getModuleDistances();
                      final double arcLength =
                          finalRotation.minus(odometryStartingRotation).getRadians() * driveRadius;
                      for (int i = 0; i < finalPositions.length; i++) {
                        final double delta =
                            Math.abs(finalPositions[i] - odometryStartingPositions[i]);
                        final double value = arcLength / delta;
                        System.out.println(
                            "Calculated wheel radius for module "
                                + i
                                + ": "
                                + value
                                + "m, "
                                + Units.metersToInches(value)
                                + "in");
                      }
                    }))
        .andThen(() -> drive.runVelocity(new ChassisSpeeds()));
  }

  private static Rotation2d odometryStartingRotation = new Rotation2d();
  private static double[] odometryStartingPositions = new double[4];

  /** Runs a tests of all systems in the pit */
  public Command pitTest() {
    return UtilCommands.namedSequence(
        "Pit Test Progress",
        testDrive(),
        UtilCommands.waitForDriverstationButton(),
        testSuperstructure(),
        UtilCommands.waitForDriverstationButton(),
        testIntake(),
        UtilCommands.waitForDriverstationButton(),
        testShooter());
  }

  private Command testDrive() {
    return Commands.sequence(
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0))).withTimeout(2.0),
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, -1.0, 0.0))).withTimeout(2.0),
        Commands.runOnce(drive::stop));
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
    return Commands.sequence(
        new Handoff(intake, superstructure, shooter).withTimeout(2.5),
        UtilCommands.waitForDriverstationButton(),
        intake.spitRaw().alongWith(shooter.setRawFeederVoltsCommand(1.2).withTimeout(1.3)),
        superstructure.setPose(Preset.HOME),
        new Handoff(intake, superstructure, shooter).withTimeout(2.5));
  }

  private Command testShooter() {
    return Commands.sequence(
        shooter.setSpeedContinuous(2000).withTimeout(3.0), shooter.stopFlywheels());
  }
}
