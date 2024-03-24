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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.WorBots.commands.Handoff;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.intake.Intake;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.subsystems.lights.Lights.LightsMode;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.Preset;
import frc.WorBots.util.UtilCommands;

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
  public Command pitTest(boolean isFull) {
    return UtilCommands.namedSequence(
        "Pit Test Progress",
        Commands.runOnce(
            () -> {
              Lights.getInstance().setMode(LightsMode.PitTest);
              pitTestStep = 0;
              isFullPitTest = isFull;
            }),
        waitForNextStep("Drive; Multiple directions"),
        testDrive(),
        waitForNextStep("Superstructure; Handoff"),
        testSuperstructure(),
        waitForNextStep("Intake; Intake piece"),
        testIntake(),
        waitForNextStep("Shooter; Pose + Shoot"),
        testShooter());
  }

  private Command testDrive() {
    return UtilCommands.namedSequence(
        "Pit Test Drive Progress",
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(2.0, 0.0, 0.0)), drive)
            .withTimeout(1.0),
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-2.0, 0.0, 0.0)), drive)
            .withTimeout(1.0),
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 2.0, 0.0)), drive)
            .withTimeout(1.0),
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, -2.0, 0.0)), drive)
            .withTimeout(1.0),
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 2.0)), drive)
            .withTimeout(1.0),
        Commands.runOnce(drive::stop));
  }

  private Command testSuperstructure() {
    return UtilCommands.namedSequence(
        "Pit Test Superstructure Progress",
        superstructure.goToPose(Preset.HANDOFF),
        waitForNextStep("Go to amp"),
        superstructure.goToPose(Preset.AMP),
        waitForNextStep("Go to handoff"),
        superstructure.goToPose(Preset.HANDOFF));
  }

  private Command testIntake() {
    return UtilCommands.namedSequence(
        "Pit Test Intake Progress",
        new Handoff(intake, superstructure, shooter),
        waitForNextStep("Spit piece"),
        intake.spitRaw().alongWith(shooter.setRawFeederVoltsCommand(-1.2)).withTimeout(0.7),
        shooter.setRawFeederVoltsCommand(0.0),
        waitForNextStep("Go to stow + intake"),
        superstructure.goToPose(Preset.STOW).withTimeout(1.0),
        new Handoff(intake, superstructure, shooter)
            .raceWith(Commands.waitUntil(intake::hasGamePiece)),
        waitForNextStep("Intake to top"),
        superstructure.goToPose(Preset.HANDOFF),
        new Handoff(intake, superstructure, shooter));
  }

  private Command testShooter() {
    return UtilCommands.namedSequence(
        "Pit Test Shooter Progress",
        superstructure.goToPose(Preset.SUBWOOFER_SHOOT),
        waitForNextStep("Shoot"),
        shooter
            .setSpeedContinuous(900)
            .alongWith(
                Commands.waitSeconds(1.4)
                    .andThen(shooter.feed().until(() -> !shooter.hasGamePiece()))),
        shooter.stopFlywheels(),
        waitForNextStep("Back to stow"),
        superstructure.goToPose(Preset.STOW));
  }

  private Command waitForNextStep(String description) {
    return UtilCommands.optimalSequence(
        Commands.runOnce(
            () -> {
              Lights.getInstance().setPitTestFlashing(true);
              Lights.getInstance().setPitTestStep(pitTestStep);
              SmartDashboard.putString("DB/String 0", description);
            }),
        UtilCommands.waitForDriverstationButton().onlyIf(() -> !isFullPitTest),
        Commands.waitSeconds(0.3).onlyIf(() -> isFullPitTest),
        Commands.runOnce(
            () -> {
              Lights.getInstance().setPitTestFlashing(false);
              pitTestStep++;
              SmartDashboard.putString("DB/String 0", "Running test step...");
            }));
  }

  private static int pitTestStep = 0;
  private static boolean isFullPitTest = false;
}
