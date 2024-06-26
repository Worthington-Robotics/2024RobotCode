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
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.intake.Intake;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.util.RobotSimulator;
import frc.WorBots.util.UtilCommands;
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

  public Command testAuto() {
    final Pose2d startingPose = util.startingLocations[1];

    // Intake center piece, then drive up to speaker and shoot
    final var path1 =
        util.path(
            Waypoint.fromHolonomicPose(startingPose),
            Waypoint.fromHolonomicPose(util.ampSideCenterpoint),
            Waypoint.fromHolonomicPose(util.getWingLinePose(startingPose, new Rotation2d())));

    return createSequence(util.reset(startingPose).command(), path1.command());
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
        AllianceFlipUtil.addToFlipped(startingPose, Units.inchesToMeters(10));
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
        util.moveAndShoot(driveToBetterSpot1.pose(), true, false, true, shootTimeout);

    // Turn to the correct direction before moving
    final var turn1 = util.turnTo(autoShoot1.pose(), AllianceFlipUtil.apply(new Rotation2d()));

    // Intake the piece right behind us
    final var intake1 = util.driveAndIntakeWing(turn1.pose(), false, false, startingLocation);

    // Drive to the better spot before the second shot for the wall side, so that we
    // don't run into the stage while targeting
    final var driveToBetterSpot2 =
        startingLocation == 2
            ? util.driveTo(driveToBetterSpotPose2)
            : new CommandWithPose(Commands.none(), intake1.pose());

    // Shoot the second piece
    final var autoShoot2 =
        util.moveAndShoot(driveToBetterSpot2.pose(), true, false, true, shootTimeout);
    return createSequence(
        util.reset(startingPose).command(),
        driveToBetterSpot1.command(),
        autoShoot1.command(),
        turn1.command(),
        intake1.command(),
        driveToBetterSpot2.command(),
        autoShoot2.command(),
        util.driveOutToCenter(autoShoot2.pose(), startingLocation == 2).command());
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
    final var autoShoot2 = util.moveAndShoot(intake1.pose(), false, false, true, 1.0);

    // Intake the third piece, then move to the center and shoot it
    final int thirdPiecePosition = isWallSide ? 2 : 0;
    final var intake2 = util.driveAndIntakeWing(autoShoot2.pose(), true, false, thirdPiecePosition);
    final var autoShoot3 =
        util.moveAndShoot(util.wingGamePieceLocations[1], false, false, true, 3.0);
    return createSequence(
        util.reset(startingPose).command(),
        autoShoot1.command(),
        intake1.command(),
        autoShoot2.command(),
        intake2.command(),
        Commands.deadline(
            util.path(
                    Waypoint.fromHolonomicPose(intake2.pose()),
                    Waypoint.fromHolonomicPose(util.wingGamePieceLocations[1]))
                .command(),
            util.fullHandoff()),
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
        util.moveAndShoot(util.wingGamePieceLocations[1], false, false, true, 1.0);
    final var intake3 = util.driveAndIntakeWing(autoShoot3.pose(), true, false, 2);
    final var autoShoot4 =
        util.moveAndShoot(util.wingGamePieceLocations[1], false, false, true, 2.0);
    return createSequence(
        util.reset(startingPose).command(),
        autoShoot1.command(),
        intake1.command(),
        autoShoot2.command(),
        intake2.command().withTimeout(4.0),
        Commands.deadline(
            util.path(
                    Waypoint.fromHolonomicPose(intake2.pose()),
                    Waypoint.fromHolonomicPose(util.wingGamePieceLocations[1]))
                .command(),
            util.fullHandoff()),
        autoShoot3.command(),
        intake3.command().withTimeout(4.0),
        Commands.deadline(
            util.path(
                    Waypoint.fromHolonomicPose(intake3.pose()),
                    Waypoint.fromHolonomicPose(util.wingGamePieceLocations[1]))
                .command()
                .withTimeout(4.5),
            util.fullHandoff()),
        autoShoot4.command());
  }

  public Command fourPieceCloseAlt() {
    final Pose2d startingPose = util.startingLocations[1];
    // Shoot the piece we start with
    final var autoShoot1 = util.moveAndShoot(startingPose, true, false, false, 0.1);

    // Move back, intake, and shoot
    final var intake1 = util.driveAndIntakeWing(autoShoot1.pose(), false, false, 1);
    final var autoShoot2 = util.moveAndShoot(intake1.pose(), false, false, false, 0.5);

    // Intake the third piece, then move to the center and shoot it
    final var intake2 = util.driveAndIntakeWing(autoShoot2.pose(), true, false, 2);
    final var autoShoot3 =
        util.moveAndShoot(util.wingGamePieceLocations[1], false, false, true, 1.0);
    final var intake3 = util.driveAndIntakeWing(autoShoot3.pose(), true, false, 0);
    final var autoShoot4 =
        util.moveAndShoot(util.wingGamePieceLocations[1], false, false, true, 2.0);
    return createSequence(
        util.reset(startingPose).command(),
        autoShoot1.command(),
        intake1.command(),
        autoShoot2.command(),
        intake2.command().withTimeout(4.0),
        Commands.deadline(
            util.path(
                    Waypoint.fromHolonomicPose(intake2.pose()),
                    Waypoint.fromHolonomicPose(util.wingGamePieceLocations[1]))
                .command(),
            util.fullHandoff()),
        autoShoot3.command(),
        intake3.command().withTimeout(4.0),
        Commands.deadline(
            util.path(
                    Waypoint.fromHolonomicPose(intake3.pose()),
                    Waypoint.fromHolonomicPose(util.wingGamePieceLocations[1]))
                .command()
                .withTimeout(4.5),
            util.fullHandoff()),
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
                .andThen(util.intakeWhileNear(util.centerGamePieceLocations[0], 1.0))
                .andThen(util.prepareShooting(path1.pose()))),
        autoShoot2.command(),
        Commands.deadline(
            path2.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhileNear(util.centerGamePieceLocations[1], 1.0))
                .andThen(util.prepareShooting(path2.pose()))),
        autoShoot3.command(),
        Commands.deadline(
            path3.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhileNear(util.centerGamePieceLocations[2], 1.0))
                .andThen(util.prepareShooting(path3.pose()))),
        autoShoot4.command());
  }

  public Command fourPieceLong() {
    final Pose2d startingPose = util.startingLocations[1];
    // Starting shot
    final var autoShoot1 = util.moveAndShoot(startingPose, false, false, false, 0.3);

    // Intake center piece, then shoot from far pose
    final var path1 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot1.pose()),
            Waypoint.fromHolonomicPose(util.betweenZeroAndOne),
            Waypoint.fromHolonomicPose(util.centerGamePieceLocations[0]),
            Waypoint.fromHolonomicPose(util.getAutoShootPose(util.farShootingPose)));
    final var autoShoot2 = util.moveAndShoot(path1.pose(), true, false, false, 2.5);

    // Intake center piece, then drive up to speaker and shoot
    final var path2 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot2.pose()),
            Waypoint.fromHolonomicPose(util.centerGamePieceLocations[1]),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(util.betweenZeroAndOne, Units.inchesToMeters(120))),
            Waypoint.fromHolonomicPose(util.betweenZeroAndOne),
            Waypoint.fromHolonomicPose(
                util.getAutoShootPose(
                    AllianceFlipUtil.addToFlipped(
                        util.betweenZeroAndOne, -Units.inchesToMeters(35)))));
    final var autoShoot3 = util.moveAndShoot(path2.pose(), true, false, false, 2.5);

    // Move back and intake wing piece, then shoot it
    final var path3 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot3.pose()),
            Waypoint.fromHolonomicPose(
                util.getAutoShootPose(
                    util.posePlus(
                        util.wingGamePieceLocations[0],
                        util.transform(Units.inchesToMeters(18), Units.inchesToMeters(13))))));
    final var autoShoot4 = util.moveAndShoot(path3.pose(), true, false, true, 2.0);

    return createSequence(
        util.reset(startingPose).command(),
        autoShoot1.command(),
        Commands.deadline(
            path1.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhileNear(util.centerGamePieceLocations[0], 1.8))
                .andThen(util.prepareShooting(path1.pose()))),
        // Slight time delay to ensure full stop
        Commands.waitSeconds(0.1),
        autoShoot2.command(),
        Commands.deadline(
            path2.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhileNear(util.centerGamePieceLocations[1], 1.8))
                .andThen(util.prepareShooting(path2.pose()))),
        autoShoot3.command(),
        // Parallel so that we don't stop intaking when we get to the piece
        Commands.parallel(path3.command(), util.fullHandoff()),
        autoShoot4.command());
  }

  public Command threePieceLongWallSide() {
    final Pose2d startingPose = util.twoPieceStartingLocations[2];
    // Move for starting shot
    final var move1 =
        util.driveToNoTheta(
            util.posePlus(
                startingPose, util.transform(Units.inchesToMeters(32), Units.inchesToMeters(10))));
    // Starting shot
    final var autoShoot1 = util.moveAndShoot(move1.pose(), true, false, true, 2.8);

    final Pose2d inFrontOfStagePiece =
        AllianceFlipUtil.addToFlipped(util.wingGamePieceLocations[2], Units.inchesToMeters(-60));

    // Intake center piece, then drive up to speaker and shoot
    final var path1 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot1.pose()),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(util.wallSideCenterpoint, -0.60)),
            Waypoint.fromHolonomicPose(
                util.posePlus(
                    util.posePlus(
                        util.centerGamePieceLocations[3],
                        util.transform(Units.inchesToMeters(8), Units.inchesToMeters(5.0))),
                    util.transformRotate(
                        AllianceFlipUtil.negRotation(Rotation2d.fromDegrees(25))))),
            Waypoint.fromHolonomicPose(
                util.posePlus(util.wallSideCenterpoint, util.transform(1.3, -0.20))),
            Waypoint.fromHolonomicPose(
                util.getAutoShootPose(
                    util.posePlus(
                        inFrontOfStagePiece,
                        util.transform(Units.inchesToMeters(40.0), Units.inchesToMeters(-45.0))))));
    final var autoShoot2 = util.moveAndShoot(path1.pose(), false, false, true, 2.5);

    // Intake stage piece, then shoot from close
    final var path2 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot2.pose()),
            Waypoint.fromHolonomicPose(
                util.posePlus(
                    inFrontOfStagePiece,
                    util.transform(Units.inchesToMeters(20.0), Units.inchesToMeters(-24.0)))),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(
                    util.wingGamePieceLocations[2], Units.inchesToMeters(12))),
            Waypoint.fromHolonomicPose(
                util.getAutoShootPose(
                    util.posePlus(inFrontOfStagePiece, util.transformY(Units.inchesToMeters(8))))));
    final var autoShoot3 = util.moveAndShoot(path2.pose(), true, false, true, 2.5);

    final var path3 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot3.pose()),
            Waypoint.fromHolonomicPose(util.wallSideCenterpoint),
            Waypoint.fromHolonomicPose(util.centerGamePieceLocations[4]));

    return createSequence(
        util.reset(startingPose).command(),
        move1.command().withTimeout(2.8),
        autoShoot1.command(),
        Commands.deadline(
            path1.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhileNear(util.centerGamePieceLocations[3], 2.2))
                .andThen(util.prepareShooting(path1.pose()))),
        // Slight time delay to ensure full stop
        Commands.waitSeconds(0.25),
        autoShoot2.command(),
        Commands.deadline(
            path2.command().withTimeout(4.3),
            util.prepareHandoff()
                .andThen(util.intakeWhenNear(util.wingGamePieceLocations[2], 1.4))
                .andThen(util.prepareShooting(path2.pose()))),
        autoShoot3.command(),
        Commands.parallel(path3.command(), util.fullHandoff()));
  }

  public Command fourPieceLongWallSide() {
    final Pose2d startingPose = util.twoPieceStartingLocations[2];
    // Starting shot
    final var autoShoot1 = util.moveAndShoot(startingPose, true, false, true, 2.8);

    final Pose2d inFrontOfStagePiece =
        AllianceFlipUtil.addToFlipped(util.wingGamePieceLocations[2], Units.inchesToMeters(-60));

    final Pose2d shootingPose =
        util.posePlus(
            inFrontOfStagePiece,
            util.transform(Units.inchesToMeters(40.0), Units.inchesToMeters(-45.0)));

    // Intake center piece, then drive up to speaker and shoot
    final var path1 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot1.pose()),
            Waypoint.fromHolonomicPose(util.getWingLinePose(autoShoot1.pose(), new Rotation2d())),
            Waypoint.fromHolonomicPose(
                util.posePlus(
                    util.posePlus(
                        util.centerGamePieceLocations[3],
                        util.transformY(Units.inchesToMeters(5.0))),
                    util.transformRotate(
                        AllianceFlipUtil.flipRotation(
                            AllianceFlipUtil.apply(Rotation2d.fromDegrees(75)))))),
            Waypoint.fromHolonomicPose(
                util.posePlus(util.wallSideCenterpoint, util.transform(1.3, 0.1))),
            Waypoint.fromHolonomicPose(util.getAutoShootPose(shootingPose)));
    final var autoShoot2 = util.moveAndShoot(path1.pose(), false, false, true, 2.5);

    // Intake stage piece, then shoot from close
    final var path2 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot2.pose()),
            Waypoint.fromHolonomicPose(
                util.posePlus(
                    inFrontOfStagePiece,
                    util.transform(Units.inchesToMeters(20.0), Units.inchesToMeters(-24.0)))),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(
                    util.wingGamePieceLocations[2], Units.inchesToMeters(12))),
            Waypoint.fromHolonomicPose(
                util.getAutoShootPose(
                    util.posePlus(inFrontOfStagePiece, util.transformY(Units.inchesToMeters(8))))));
    final var autoShoot3 = util.moveAndShoot(path2.pose(), true, false, true, 2.5);

    final var path3 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot3.pose()),
            Waypoint.fromHolonomicPose(util.wallSideCenterpoint),
            Waypoint.fromHolonomicPose(util.centerGamePieceLocations[4]),
            Waypoint.fromHolonomicPose(util.getAutoShootPose(shootingPose)));

    return createSequence(
        util.reset(startingPose).command(),
        autoShoot1.command(),
        Commands.deadline(
            path1.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhileNear(util.centerGamePieceLocations[3], 1.8))
                .andThen(util.prepareShooting(path1.pose()))),
        autoShoot2.command(),
        Commands.deadline(
            path2.command().withTimeout(4.3),
            util.prepareHandoff()
                .andThen(util.intakeWhenNear(util.wingGamePieceLocations[2], 1.4))
                .andThen(util.prepareShooting(path2.pose()))),
        autoShoot3.command(),
        Commands.deadline(
            path3.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhenNear(util.centerGamePieceLocations[4], 1.4))
                .andThen(util.prepareShooting(path3.pose()))));
  }

  public Command theAdibSpecial() {
    final Pose2d startingPose =
        util.posePlus(util.twoPieceStartingLocations[2], util.transform(0.0, 0.13));
    // Move for starting shot
    // final var move1 =
    // util.driveTo(
    // util.posePlus(
    // startingPose, util.transform(Units.inchesToMeters(32),
    // Units.inchesToMeters(10))));
    // Starting shot
    final var autoShoot1 = util.moveAndShoot(startingPose, true, false, true, 2.8);

    final Pose2d inFrontOfStagePiece =
        AllianceFlipUtil.addToFlipped(util.wingGamePieceLocations[2], Units.inchesToMeters(-60));

    final Pose2d shootingPose =
        util.posePlus(
            inFrontOfStagePiece,
            util.transform(Units.inchesToMeters(40.0), Units.inchesToMeters(-45.0)));

    // Intake center piece, then drive up to speaker and shoot
    final var path1 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot1.pose()),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(util.wallSideCenterpoint, -1.5)),
            Waypoint.fromHolonomicPose(
                util.posePlus(
                    util.posePlus(
                        util.centerGamePieceLocations[4],
                        util.transform(Units.inchesToMeters(15), Units.inchesToMeters(10.0))),
                    util.transformRotate(AllianceFlipUtil.negRotation(Rotation2d.fromDegrees(0))))),
            Waypoint.fromHolonomicPose(
                util.posePlus(util.wallSideCenterpoint, util.transform(1.3, -0.20))),
            Waypoint.fromHolonomicPose(util.getAutoShootPose(shootingPose)));
    final var autoShoot2 = util.moveAndShoot(path1.pose(), false, false, true, 2.5);

    final var path2 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot2.pose()),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(util.wallSideCenterpoint, 0.8)),
            Waypoint.fromHolonomicPose(
                util.posePlus(
                    util.posePlus(
                        util.centerGamePieceLocations[3],
                        util.transform(Units.inchesToMeters(15), Units.inchesToMeters(5.0))),
                    util.transformRotate(
                        AllianceFlipUtil.negRotation(Rotation2d.fromDegrees(55))))),
            Waypoint.fromHolonomicPose(
                util.posePlus(util.wallSideCenterpoint, util.transform(1.3, -0.20))),
            Waypoint.fromHolonomicPose(util.getAutoShootPose(shootingPose)));
    final var autoShoot3 = util.moveAndShoot(path2.pose(), true, false, true, 2.5);

    final var path3 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot3.pose()),
            Waypoint.fromHolonomicPose(util.wallSideCenterpoint),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(util.wallSideCenterpoint, 3.5)),
            Waypoint.fromHolonomicPose(
                util.posePlus(
                    util.centerGamePieceLocations[2],
                    util.transformRotate(
                        AllianceFlipUtil.negRotation(Rotation2d.fromDegrees(90.0))))));

    final var path4 =
        util.path(
            Waypoint.fromHolonomicPose(path3.pose()),
            Waypoint.fromHolonomicPose(util.centerGamePieceLocations[4]));

    return createSequence(
        util.reset(startingPose).command(),
        // move1.command(),
        autoShoot1.command(),
        Commands.deadline(
            path1.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhileNear(util.centerGamePieceLocations[4], 2.2))
                .andThen(util.prepareShooting(path1.pose()))),
        // Slight time delay to ensure full stop
        Commands.waitSeconds(0.25),
        autoShoot2.command(),
        Commands.deadline(
            path2.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhenNear(util.centerGamePieceLocations[3], 2.2))
                .andThen(util.prepareShooting(path2.pose()))),
        autoShoot3.command(),
        Commands.parallel(path3.command(), util.fullHandoff()),
        path4.command());
  }

  public Command fivePieceLong() {
    final Pose2d startingPose = util.startingLocations[1];
    // Starting shot
    final var autoShoot1 = util.moveAndShoot(startingPose, true, false, false, 0.3);

    // Intake center piece, then shoot from far pose
    final var path1 =
        util.path(
            Waypoint.fromHolonomicPose(autoShoot1.pose()),
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
                .andThen(util.intakeWhileNear(util.centerGamePieceLocations[0], 1.5))
                .andThen(util.prepareShooting(path1.pose()))),
        autoShoot2.command(),
        Commands.deadline(
            path2.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhileNear(util.centerGamePieceLocations[1], 1.5))
                .andThen(util.prepareShooting(path2.pose()))),
        autoShoot3.command(),
        // Parallel so that we don't stop intaking when we get to the piece
        Commands.parallel(
            path3.command(),
            util.prepareHandoff().andThen(util.intakeWhileNear(path3.pose(), 0.5))),
        autoShoot4.command(),
        Commands.parallel(
            path4.command(),
            util.prepareHandoff().andThen(util.intakeWhileNear(path4.pose(), 0.5))),
        autoShoot5.command());
  }

  public Command ampLine() {
    final Pose2d startingPose = util.twoPieceStartingLocations[0];

    // Drive to the better spot before the first shot for a more accurate shot
    final var drive1 =
        util.driveTo(AllianceFlipUtil.addToFlipped(startingPose, Units.inchesToMeters(10)));

    // Shoot the loaded game piece
    final var shoot1 = util.moveAndShoot(drive1.pose(), true, false, true, 2.5);

    // Turn to the correct direction before moving
    final var turn1 = util.turnTo(shoot1.pose(), AllianceFlipUtil.apply(new Rotation2d()));

    // Intake the piece right behind us
    final var intake1 = util.driveAndIntakeWing(turn1.pose(), false, false, 0);

    // Shoot the second piece
    final var shoot2 = util.moveAndShoot(intake1.pose(), true, false, true, 2.5);

    // Drive and intake the center piece, then move back
    final var path1 =
        util.path(
            Waypoint.fromHolonomicPose(shoot2.pose()),
            Waypoint.fromHolonomicPose(util.centerGamePieceLocations[0]),
            Waypoint.fromHolonomicPose(util.ampSideCenterpoint),
            Waypoint.fromHolonomicPose(util.getAutoShootPose(util.wingGamePieceLocations[0])));

    // Make the final shot
    final var shoot3 = util.moveAndShoot(path1.pose(), false, false, true, 2.5);

    return createSequence(
        util.reset(startingPose).command(),
        drive1.command(),
        shoot1.command(),
        turn1.command(),
        intake1.command(),
        shoot2.command(),
        Commands.parallel(
            path1.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhileNear(util.wingGamePieceLocations[0], 1.0))),
        shoot3.command());
  }

  public Command ampLineFour() {
    final Pose2d startingPose = util.twoPieceStartingLocations[0];

    // Drive to the better spot before the first shot for a more accurate shot
    final var drive1 =
        util.driveTo(AllianceFlipUtil.addToFlipped(startingPose, Units.inchesToMeters(10)));

    // Shoot the loaded game piece
    final var shoot1 = util.moveAndShoot(drive1.pose(), true, false, true, 2.5);

    // Turn to the correct direction before moving
    final var turn1 = util.turnTo(shoot1.pose(), AllianceFlipUtil.apply(new Rotation2d()));

    // Intake the piece right behind us
    final var intake1 = util.driveAndIntakeWing(turn1.pose(), false, false, 0);

    // Shoot the second piece
    final var shoot2 = util.moveAndShoot(intake1.pose(), true, false, true, 2.5);

    final var farShootPose =
        util.posePlus(util.wingGamePieceLocations[0], util.transform(Units.inchesToMeters(50), 0));

    // Drive and intake the center piece, then move back
    final var path1 =
        util.path(
            Waypoint.fromHolonomicPose(shoot2.pose()),
            Waypoint.fromHolonomicPose(
                util.rotatePose(
                    AllianceFlipUtil.addToFlipped(util.centerGamePieceLocations[1], 0.3),
                    AllianceFlipUtil.negRotation(Rotation2d.fromDegrees(-20)))),
            Waypoint.fromHolonomicPose(util.getAutoShootPose(farShootPose)));

    // Make the final shot
    final var shoot3 = util.moveAndShoot(path1.pose(), false, false, true, 2.5);

    // Drive and intake the center piece, then move back
    final var path2 =
        util.path(
            Waypoint.fromHolonomicPose(shoot3.pose()),
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(util.centerGamePieceLocations[0], 0.3)),
            Waypoint.fromHolonomicPose(util.getWingLinePose(startingPose, new Rotation2d())),
            Waypoint.fromHolonomicPose(util.getAutoShootPose(farShootPose)));

    // Make the final shot
    final var shoot4 = util.moveAndShoot(path2.pose(), false, false, true, 2.5);

    return createSequence(
        util.reset(startingPose).command(),
        drive1.command(),
        shoot1.command(),
        turn1.command(),
        intake1.command(),
        shoot2.command(),
        Commands.parallel(
            path1.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhileNear(util.wingGamePieceLocations[1], 1.5))),
        shoot3.command(),
        Commands.parallel(
            path2.command(),
            util.prepareHandoff()
                .andThen(util.intakeWhileNear(util.wingGamePieceLocations[0], 1.5))),
        shoot4.command());
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
