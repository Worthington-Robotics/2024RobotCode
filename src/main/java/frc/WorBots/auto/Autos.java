// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.WorBots.FieldConstants;
import frc.WorBots.auto.AutoSelector.AutoQuestionResponse;
import frc.WorBots.auto.AutoUtil.CommandWithPose;
import frc.WorBots.commands.DriveToPose;
import frc.WorBots.commands.UtilCommands;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.intake.Intake;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.util.RobotSimulator;
import frc.WorBots.util.math.AllianceFlipUtil;
import frc.WorBots.util.trajectory.Waypoint;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

/** Holder for the autos we want to run */
public class Autos {
  // Subsystems
  private final Drive drive;
  private final Shooter shooter;

  // Supplier for question responses, from the AutoSelector
  private final Supplier<List<AutoQuestionResponse>> responses;

  // Util
  private final AutoUtil util;

  public Autos(
      Drive drive,
      Superstructure superstructure,
      Intake intake,
      Shooter shooter,
      Supplier<List<AutoQuestionResponse>> responses) {
    this.drive = drive;
    this.shooter = shooter;

    this.responses = responses;

    util = new AutoUtil(drive, superstructure, intake, shooter);
  }

  public Command onePiece() {
    return selectFromStartingLocation(this::onePiece);
  }

  private Command onePiece(int startingLocation) {
    final Pose2d startingPose = util.startingLocations[startingLocation];
    final var autoShoot = util.moveAndShoot(startingPose, true, false, false, 0.1);
    return createSequence(util.reset(startingPose).command(), autoShoot.command());
  }

  public Command twoPiece() {
    return selectFromStartingLocation(this::twoPiece);
  }

  private Command twoPiece(int startingLocation) {
    final Pose2d startingPose = util.twoPieceStartingLocations[startingLocation];
    // We are already in the right spot for the center start, so give it a shorter
    // timeout
    final double shootTimeout = startingLocation == 1 ? 0.1 : 2.0;

    // A better spot to shoot from, near the starting position
    final var driveToBetterSpotPose1 =
        AllianceFlipUtil.addToFlipped(startingPose, Units.inchesToMeters(20));
    final var driveToBetterSpotPose2 =
        AllianceFlipUtil.addToFlipped(startingPose, Units.inchesToMeters(50));

    // Drive to the better spot before the first shot for the amp side, for a more
    // accurate shot
    final var driveToBetterSpot1 =
        startingLocation == 0
            ? new CommandWithPose(
                new DriveToPose(drive, driveToBetterSpotPose1), driveToBetterSpotPose1)
            : new CommandWithPose(Commands.none(), startingPose);
    // Shoot the loaded game piece
    final var autoShoot1 =
        util.moveAndShoot(driveToBetterSpot1.pose(), true, true, true, shootTimeout);

    // Intake the piece right behind us
    final var driveIntakeWing1 =
        util.driveAndIntakeWing(autoShoot1.pose(), false, false, startingLocation);
    // Drive to the better spot before the second shot for the wall side, so that we
    // don't run into the stage while targeting
    final var driveToBetterSpot2 =
        startingLocation == 2
            ? new CommandWithPose(
                new DriveToPose(drive, driveToBetterSpotPose2), driveToBetterSpotPose2)
            : new CommandWithPose(Commands.none(), driveIntakeWing1.pose());
    // Shoot the second piece
    final var autoShoot2 =
        util.moveAndShoot(driveToBetterSpot2.pose(), true, true, true, shootTimeout);
    return createSequence(
        util.reset(startingPose).command(),
        driveToBetterSpot1.command(),
        autoShoot1.command(),
        driveIntakeWing1.command(),
        driveToBetterSpot2.command(),
        autoShoot2.command(),
        util.driveOutToCenter(autoShoot2.pose()).command());
  }

  public Command threePieceClose() {
    return selectFromDirection(this::threePieceClose);
  }

  private Command threePieceClose(boolean isWallSide) {
    final Pose2d startingPose = util.startingLocations[1];
    // Shoot the piece we start with
    final var autoShoot1 = util.moveAndShoot(startingPose, true, false, false, 0.1);

    // Move back, intake, and shoot
    final var intake1 = util.driveAndIntakeWing(autoShoot1.pose(), false, false, 1);
    final var autoShoot2 = util.moveAndShoot(intake1.pose(), false, false, false, 0.5);

    // Intake the third piece, then move to the center and shoot it
    final int thirdPiecePosition = isWallSide ? 2 : 0;
    final var intake2 = util.driveAndIntakeWing(autoShoot2.pose(), true, false, thirdPiecePosition);
    final var autoShoot3 =
        util.moveAndShoot(util.wingGamePieceLocations[1], false, false, false, 2.0);
    return createSequence(
        util.reset(startingPose).command(),
        autoShoot1.command(),
        intake1.command(),
        autoShoot2.command(),
        intake2.command(),
        util.path(
                Waypoint.fromHolonomicPose(intake2.pose()),
                Waypoint.fromHolonomicPose(util.wingGamePieceLocations[1]))
            .command(),
        autoShoot3.command());
  }

  public Command fourPieceClose() {
    final Pose2d startingPose = util.startingLocations[1];
    // Shoot the piece we start with
    final var autoShoot1 = util.moveAndShoot(startingPose, true, false, false, 0.1);

    // Move back, intake, and shoot
    final var intake1 = util.driveAndIntakeWing(autoShoot1.pose(), false, false, 1);
    final var autoShoot2 = util.moveAndShoot(intake1.pose(), false, false, false, 0.5);

    // Intake the third piece, then move to the center and shoot it
    final var intake2 = util.driveAndIntakeWing(autoShoot2.pose(), true, false, 0);
    final var autoShoot3 =
        util.moveAndShoot(util.wingGamePieceLocations[1], false, false, false, 0.5);
    final var intake3 = util.driveAndIntakeWing(autoShoot3.pose(), true, false, 2);
    final var autoShoot4 =
        util.moveAndShoot(util.wingGamePieceLocations[1], false, false, false, 2.0);
    return createSequence(
        util.reset(startingPose).command(),
        autoShoot1.command(),
        intake1.command(),
        autoShoot2.command(),
        intake2.command(),
        util.path(
                Waypoint.fromHolonomicPose(intake2.pose()),
                Waypoint.fromHolonomicPose(util.wingGamePieceLocations[1]))
            .command(),
        autoShoot3.command(),
        intake3.command(),
        util.path(
                Waypoint.fromHolonomicPose(intake3.pose()),
                Waypoint.fromHolonomicPose(util.wingGamePieceLocations[1]))
            .command(),
        autoShoot4.command());
  }

  public Command fourFromMiddle() {
    final Pose2d startingPose = util.startingLocations[1];
    // Starting shot
    final var autoShoot1 = util.moveAndShoot(startingPose, true, false, false, 0.3);

    // Intake center piece, then shoot from far pose
    final var path1 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot1.pose()),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(util.betweenZeroAndOne, Units.inchesToMeters(-24))),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(util.betweenZeroAndOne, Units.inchesToMeters(48))),
            Waypoint.fromHolonomicPose(util.centerGamePieceLocations[0]),
            Waypoint.fromHolonomicPose(util.getAutoShootPose(util.farShootingPose)));
    final var autoShoot2 = util.moveAndShoot(path1.pose(), false, false, false, 1.2);

    // Intake center piece, then shoot from far pose
    final var path2 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot2.pose()),
            Waypoint.fromHolonomicPose(util.centerGamePieceLocations[1]),
            Waypoint.fromHolonomicPose(
                util.getWingLinePose(util.centerGamePieceLocations[1], new Rotation2d())),
            Waypoint.fromHolonomicPose(util.getAutoShootPose(util.farShootingPose)));
    final var autoShoot3 = util.moveAndShoot(path2.pose(), false, false, false, 1.2);

    // Intake center piece, then shoot from far pose
    final var path3 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot3.pose()),
            Waypoint.fromHolonomicPose(util.centerGamePieceLocations[2]),
            Waypoint.fromHolonomicPose(
                util.getWingLinePose(util.centerGamePieceLocations[1], new Rotation2d())),
            Waypoint.fromHolonomicPose(util.getAutoShootPose(util.farShootingPose)));
    final var autoShoot4 = util.moveAndShoot(path3.pose(), false, false, false, 1.2);

    return createSequence(
        util.reset(startingPose).command(),
        autoShoot1.command(),
        Commands.deadline(
            path1.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhenNear(util.centerGamePieceLocations[0], 1.0))
                .andThen(util.prepareShooting(path1.pose()))),
        autoShoot2.command(),
        Commands.deadline(
            path2.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhenNear(util.centerGamePieceLocations[1], 1.0))
                .andThen(util.prepareShooting(path2.pose()))),
        autoShoot3.command(),
        Commands.deadline(
            path3.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhenNear(util.centerGamePieceLocations[2], 1.0))
                .andThen(util.prepareShooting(path3.pose()))),
        autoShoot4.command());
  }

  public Command fourPieceLong() {
    final Pose2d startingPose = util.startingLocations[1];
    // Starting shot
    final var autoShoot1 = util.moveAndShoot(startingPose, true, false, false, 0.3);

    // Intake center piece, then shoot from far pose
    final var path1 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot1.pose()),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(util.betweenZeroAndOne, Units.inchesToMeters(-24))),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(util.betweenZeroAndOne, Units.inchesToMeters(48))),
            Waypoint.fromHolonomicPose(util.centerGamePieceLocations[0]),
            Waypoint.fromHolonomicPose(util.getAutoShootPose(util.farShootingPose)));
    final var autoShoot2 = util.moveAndShoot(path1.pose(), false, false, false, 1.2);

    // Intake center piece, then drive up to speaker and shoot
    final var path2 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot2.pose()),
            Waypoint.fromHolonomicPose(util.centerGamePieceLocations[1]),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(util.betweenZeroAndOne, Units.inchesToMeters(100))),
            Waypoint.fromHolonomicPose(util.betweenZeroAndOne),
            Waypoint.fromHolonomicPose(
                util.getAutoShootPose(
                    AllianceFlipUtil.addToFlipped(
                        util.betweenZeroAndOne, -Units.inchesToMeters(35)))));
    final var autoShoot3 = util.moveAndShoot(path2.pose(), false, false, false, 1.2);

    // Move back and intake wing piece, then shoot it
    final var path3 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot3.pose()),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(
                    util.getAutoShootPose(util.wingGamePieceLocations[0])
                        .plus(new Transform2d(0.0, Units.inchesToMeters(10), new Rotation2d())),
                    Units.inchesToMeters(15))));
    final var autoShoot4 = util.moveAndShoot(path3.pose(), false, false, false, 3.5);

    return createSequence(
        util.reset(startingPose).command(),
        autoShoot1.command(),
        Commands.deadline(
            path1.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhenNear(util.centerGamePieceLocations[0], 1.0))
                .andThen(util.prepareShooting(path1.pose()))),
        autoShoot2.command(),
        Commands.deadline(
            path2.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhenNear(util.centerGamePieceLocations[1], 1.0))
                .andThen(util.prepareShooting(path2.pose()))),
        autoShoot3.command(),
        // Parallel so that we don't stop intaking when we get to the piece
        Commands.parallel(
            path3.command(),
            util.prepareHandoff().andThen(util.intakeWhenNear(path3.pose(), 0.45))),
        autoShoot4.command());
  }

  public Command fivePieceLong() {
    final Pose2d startingPose = util.startingLocations[1];
    // Starting shot
    final var autoShoot1 = util.moveAndShoot(startingPose, true, false, false, 0.3);

    // Intake center piece, then shoot from far pose
    final var path1 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot1.pose()),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(util.betweenZeroAndOne, Units.inchesToMeters(-8))),
            Waypoint.fromHolonomicPose(util.betweenZeroAndOne),
            Waypoint.fromHolonomicPose(util.centerGamePieceLocations[0]),
            Waypoint.fromHolonomicPose(util.getAutoShootPose(util.farShootingPose)));
    final var autoShoot2 = util.moveAndShoot(path1.pose(), false, false, false, 1.2);

    // Intake center piece, then drive up to speaker and shoot
    final var path2 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot2.pose()),
            Waypoint.fromHolonomicPose(util.centerGamePieceLocations[1]),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(util.betweenZeroAndOne, Units.inchesToMeters(100))),
            Waypoint.fromHolonomicPose(util.betweenZeroAndOne),
            Waypoint.fromHolonomicPose(
                util.getAutoShootPose(
                    AllianceFlipUtil.addToFlipped(
                        util.betweenZeroAndOne, -Units.inchesToMeters(35)))));
    final var autoShoot3 = util.moveAndShoot(path2.pose(), false, false, false, 1.2);

    // Move back and intake wing piece, then shoot it
    final var path3 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot3.pose()),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(
                    util.getAutoShootPose(util.wingGamePieceLocations[0])
                        .plus(new Transform2d(0.0, Units.inchesToMeters(10), new Rotation2d())),
                    Units.inchesToMeters(15))));
    final var autoShoot4 = util.moveAndShoot(path3.pose(), false, false, false, 1.5);

    final var path4 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot4.pose()),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(util.betweenZeroAndOne, -Units.inchesToMeters(35))),
            // Waypoint.fromHolonomicPose(util.wingGamePieceLocations[1]),
            Waypoint.fromHolonomicPose(
                util.getAutoShootPose(
                    util.wingGamePieceLocations[1].plus(
                        new Transform2d(0.0, -Units.inchesToMeters(5), new Rotation2d())))));
    final var autoShoot5 = util.moveAndShoot(path4.pose(), false, false, false, 3.5);

    return createSequence(
        util.reset(startingPose).command(),
        autoShoot1.command(),
        Commands.deadline(
            path1.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhenNear(util.centerGamePieceLocations[0], 1.5))
                .andThen(util.prepareShooting(path1.pose()))),
        autoShoot2.command(),
        Commands.deadline(
            path2.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhenNear(util.centerGamePieceLocations[1], 1.5))
                .andThen(util.prepareShooting(path2.pose()))),
        autoShoot3.command(),
        // Parallel so that we don't stop intaking when we get to the piece
        Commands.parallel(
            path3.command(), util.prepareHandoff().andThen(util.intakeWhenNear(path3.pose(), 0.5))),
        autoShoot4.command(),
        Commands.parallel(
            path4.command(), util.prepareHandoff().andThen(util.intakeWhenNear(path4.pose(), 0.5))),
        autoShoot5.command());
  }

  public Command mobility() {
    return selectFromStartingLocation(this::mobility);
  }

  private Command mobility(int startingLocation) {
    Pose2d startingPose = util.startingLocations[startingLocation];
    return createSequence(
        util.reset(startingPose).command(),
        util.path(
                Waypoint.fromHolonomicPose(startingPose),
                Waypoint.fromHolonomicPose(
                    startingPose.plus(new Transform2d(0.9, 0, new Rotation2d()))))
            .command());
  }

  public Command plow() {
    final Pose2d startingPose =
        new Pose2d(
            util.twoPieceStartingLocations[0].getX(),
            FieldConstants.fieldWidth * (1.0 - 0.065),
            new Rotation2d());
    final Pose2d centerPose = util.centerGamePieceLocations[0];
    final var intake = new DriveToPose(drive, centerPose);
    double rv = Units.degreesToRadians(-30);
    if (AllianceFlipUtil.shouldFlip()) {
      rv *= -1;
    }
    final double rv2 = rv;
    final var plow =
        Commands.run(
                () ->
                    drive.runVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(0.0, -2.3, rv2, drive.getRotation())),
                drive)
            .raceWith(
                Commands.waitUntil(() -> drive.getPose().getY() < FieldConstants.midLineY * 1.37));
    return shooter
        .setSpeedContinuous(500)
        .raceWith(createSequence(util.reset(startingPose).command(), intake, plow));
  }

  /**
   * Create the named sequence of commands for the auto
   *
   * @param commands The commands to run in the auto
   * @return The full auto sequence command
   */
  private Command createSequence(Command... commands) {
    return UtilCommands.timer(
        "Auto Time",
        UtilCommands.optimalSequence(
            Commands.runOnce(() -> RobotSimulator.getInstance().loadGamePiece()),
            UtilCommands.namedSequence("Auto Progress", commands)));
  }

  private interface AutoWithStartingLocation {
    public Command run(int startingLocation);
  }

  private Command selectFromStartingLocation(AutoWithStartingLocation auto) {
    return Commands.select(
        Map.of(
            AutoQuestionResponse.AMP_SIDE,
            auto.run(0),
            AutoQuestionResponse.CENTER,
            auto.run(1),
            AutoQuestionResponse.WALL_SIDE,
            auto.run(2)),
        () -> responses.get().get(0));
  }

  private interface AutoWithDirection {
    public Command run(boolean isWallSide);
  }

  private Command selectFromDirection(AutoWithDirection auto) {
    return Commands.select(
        Map.of(
            AutoQuestionResponse.AMP_SIDE,
            auto.run(false),
            AutoQuestionResponse.WALL_SIDE,
            auto.run(true)),
        () -> responses.get().get(0));
  }
}
