// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.constraint.*;
import edu.wpi.first.math.util.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.*;
import frc.WorBots.AutoSelector.*;
import frc.WorBots.subsystems.drive.*;
import frc.WorBots.subsystems.intake.*;
import frc.WorBots.subsystems.shooter.*;
import frc.WorBots.subsystems.superstructure.*;
import frc.WorBots.subsystems.superstructure.Superstructure.*;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.*;
import frc.WorBots.util.math.AllianceFlipUtil;
import frc.WorBots.util.math.ShooterMath;
import frc.WorBots.util.trajectory.*;
import java.util.*;
import java.util.function.*;

public class AutoCommands extends Command {
  // Subsystems
  private final Drive drive;
  private final Superstructure superstructure;
  private final Intake intake;
  private final Shooter shooter;
  private Supplier<List<AutoQuestionResponse>> responses;

  // Constants
  private final Pose2d[] startingLocations;
  private final Pose2d[] twoPieceStartingLocations;
  private final Pose2d[] wingGamePieceLocations;
  private final Pose2d[] centerGamePieceLocations;
  private final Pose2d[] shootingPositions;
  private final Pose2d betweenZeroAndOne;
  private final Pose2d betweenOneandTwo;
  private final Pose2d farShootingPose;

  // Other
  public static record CommandWithPose(Command command, Pose2d pose) {}

  private Supplier<Double> pivotAngle;

  /**
   * This command houses all of the auto commands that are selected by the auto chooser.
   *
   * @param drive The drive subsystem.
   * @param superstructure The superstrucute subsystem.
   * @param intake The intake subsystem.
   * @param shooter The shooter subsystem.
   * @param responses The list of auto responses from the AutoSelector.
   */
  public AutoCommands(
      Drive drive,
      Superstructure superstructure,
      Intake intake,
      Shooter shooter,
      Supplier<List<AutoQuestionResponse>> responses) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.intake = intake;
    this.shooter = shooter;
    this.responses = responses;
    startingLocations =
        new Pose2d[] {
          AllianceFlipUtil.apply(
              new Pose2d(
                  new Translation2d(0.66, 6.62), new Rotation2d(Units.degreesToRadians(60)))),
          AllianceFlipUtil.apply(
              new Pose2d(
                  FieldConstants.StartingZone.endX - Units.inchesToMeters(22),
                  FieldConstants.Speaker.speakerY,
                  new Rotation2d())),
          AllianceFlipUtil.apply(
              new Pose2d(
                  new Translation2d(0.69, 4.51), new Rotation2d(Units.degreesToRadians(-60)))),
          AllianceFlipUtil.apply(new Pose2d(1.44, 6.36, new Rotation2d()))
        };
    wingGamePieceLocations =
        new Pose2d[] {
          AllianceFlipUtil.apply(
              new Pose2d(FieldConstants.GamePieces.wingPieces[0], new Rotation2d())),
          AllianceFlipUtil.apply(
              new Pose2d(FieldConstants.GamePieces.wingPieces[1], new Rotation2d())),
          AllianceFlipUtil.apply(
              new Pose2d(
                  FieldConstants.GamePieces.wingPieces[2].plus(new Translation2d(-0.25, 0)),
                  new Rotation2d()))
        };
    twoPieceStartingLocations =
        new Pose2d[] {
          AllianceFlipUtil.addToFlipped(wingGamePieceLocations[0], -Units.inchesToMeters(60)),
          AllianceFlipUtil.apply(
              new Pose2d(
                  FieldConstants.StartingZone.endX - Units.inchesToMeters(22),
                  FieldConstants.Speaker.speakerY,
                  new Rotation2d())),
          AllianceFlipUtil.addToFlipped(wingGamePieceLocations[2], -Units.inchesToMeters(60)),
          AllianceFlipUtil.addToFlipped(wingGamePieceLocations[2], -Units.inchesToMeters(65))
        };
    centerGamePieceLocations =
        new Pose2d[] {
          AllianceFlipUtil.apply(
              new Pose2d(
                  FieldConstants.GamePieces.centerPieces[0].plus(
                      new Translation2d(Units.inchesToMeters(6), Units.inchesToMeters(0.0))),
                  new Rotation2d())),
          AllianceFlipUtil.apply(
              new Pose2d(
                  FieldConstants.GamePieces.centerPieces[1].plus(
                      new Translation2d(Units.inchesToMeters(6), Units.inchesToMeters(0.0))),
                  new Rotation2d())),
          AllianceFlipUtil.apply(
              new Pose2d(
                  FieldConstants.GamePieces.centerPieces[2].plus(
                      new Translation2d(Units.inchesToMeters(6), Units.inchesToMeters(0.0))),
                  new Rotation2d())),
          AllianceFlipUtil.apply(
              new Pose2d(
                  FieldConstants.GamePieces.centerPieces[3].plus(
                      new Translation2d(Units.inchesToMeters(6), Units.inchesToMeters(0.0))),
                  new Rotation2d())),
          AllianceFlipUtil.apply(
              new Pose2d(
                  FieldConstants.GamePieces.centerPieces[4].plus(
                      new Translation2d(Units.inchesToMeters(6), Units.inchesToMeters(0.0))),
                  new Rotation2d())),
        };
    shootingPositions =
        new Pose2d[] {
          new Pose2d(new Translation2d(3.68, 5.80), new Rotation2d()),
          new Pose2d(new Translation2d(2.88, 5.54), new Rotation2d()),
          new Pose2d(new Translation2d(2.50, 3.49), new Rotation2d())
        };
    betweenZeroAndOne = AllianceFlipUtil.apply(new Pose2d(2.88, 6.28, new Rotation2d()));
    betweenOneandTwo = AllianceFlipUtil.apply(new Pose2d(2.88, 4.8, new Rotation2d()));
    farShootingPose =
        AllianceFlipUtil.apply(
            new Pose2d(
                FieldConstants.Wing.endX * 0.85,
                FieldConstants.Speaker.speakerY * 1.2,
                new Rotation2d()));
    pivotAngle =
        () -> {
          Pose2d robotPose = drive.getPose();
          var shotData = ShooterMath.calculateShotData(robotPose, drive.getFieldRelativeSpeeds());
          return shotData.pivotAngle();
        };
  }

  /**
   * Returns a command that resets the robot to a pose
   *
   * @param pose The unflipped pose to set
   * @return The command to run
   */
  private Command reset(Pose2d pose) {
    return Commands.runOnce(() -> drive.setPose(AllianceFlipUtil.apply(pose)));
  }

  /**
   * Returns a command that resets the robot to a pose
   *
   * @param pose The flipped pose to set
   * @return The command to run
   */
  private Command resetFlipped(Pose2d pose) {
    return Commands.runOnce(() -> drive.setPose(pose));
  }

  /**
   * Drives along the specified trajectory.
   *
   * @param waypoints The list of waypoints to path through
   */
  private Command path(List<Waypoint> waypoints) {
    return path(waypoints, List.of());
  }

  /** Drives along the specified trajectory. */
  private Command path(List<Waypoint> waypoints, List<TrajectoryConstraint> extraConstraints) {
    if (waypoints.size() == 2
        && waypoints.get(0).getDriveRotation().isEmpty()
        && waypoints.get(1).getDriveRotation().isEmpty()
        && waypoints.get(0).getTranslation().getDistance(waypoints.get(1).getTranslation()) < 0.5) {
      var driveToPose =
          new DriveToPose(
              drive,
              () ->
                  new Pose2d(
                      waypoints.get(1).getTranslation(),
                      waypoints.get(1).getHolonomicRotation().get()));
      return driveToPose.until(driveToPose::atGoal);
    }
    List<TrajectoryConstraint> allConstraints = new ArrayList<>();
    allConstraints.addAll(extraConstraints);
    return new DriveTrajectory(drive, waypoints, allConstraints, 0.0);
  }

  private Command path(Waypoint... waypoints) {
    return path(Arrays.asList(waypoints));
  }

  private Rotation2d driveRotation(Pose2d robotPose) {
    // FIXME: No alliance flip
    return ShooterMath.getGoalTheta(robotPose);
  }

  /**
   * A command that takes the starting position, and various parameters to automitically drive and
   * intake a game piece from the wing.
   *
   * @param startingPosition The starting pose of the robot.
   * @param fromCenter Wether or not the robot is picking up from center, effects wether or not the
   *     robot will rotate to get a game piece.
   * @param wrapToPickup Wether or not the robot will not pick up from center, and will wrap around
   *     the game piece to pick it up.
   * @param wingPosition The position of the game piece, left being 0, right being 2.
   * @return The command and pose.
   */
  private CommandWithPose driveAndIntakeWing(
      Pose2d startingPosition, boolean fromCenter, boolean wrapToPickup, int wingPosition) {
    List<Waypoint> waypoints = new ArrayList<>();
    waypoints.add(Waypoint.fromHolonomicPose(startingPosition));
    if (fromCenter) {
      if (wingPosition == 0) {
        var rotation = AllianceFlipUtil.apply(new Rotation2d(Units.degreesToRadians(100)));
        // We have to turn 180 on the red alliance so that we don't try to intake with
        // the wrong side of the robot
        if (AllianceFlipUtil.shouldFlip()) {
          rotation = rotation.rotateBy(Rotation2d.fromDegrees(180));
        }
        waypoints.add(
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(
                    wingGamePieceLocations[wingPosition].plus(new Transform2d(0.0, 0.0, rotation)),
                    Units.inchesToMeters(-8))));
      } else if (wingPosition == 2) {
        var rotation = AllianceFlipUtil.apply(new Rotation2d(Units.degreesToRadians(-90)));
        if (AllianceFlipUtil.shouldFlip()) {
          rotation = rotation.rotateBy(Rotation2d.fromDegrees(180));
        }
        waypoints.add(
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(
                    wingGamePieceLocations[wingPosition].plus(
                        new Transform2d(0.0, Units.inchesToMeters(-0), rotation)),
                    Units.inchesToMeters(-3))));
      } else {
        waypoints.add(Waypoint.fromHolonomicPose(wingGamePieceLocations[wingPosition]));
      }

    } else if (wrapToPickup) {
      if (wingPosition <= 1) {
        if (AllianceFlipUtil.apply(startingPosition.getX()) >= Units.inchesToMeters(116)) {
          waypoints.add(Waypoint.fromHolonomicPose(betweenZeroAndOne));
        }
        waypoints.add(
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(
                    wingGamePieceLocations[wingPosition].plus(
                        new Transform2d(0.0, -0.35, new Rotation2d())),
                    -0.75)));
      } else {
        if (AllianceFlipUtil.apply(startingPosition.getX()) >= Units.inchesToMeters(116)) {
          waypoints.add(Waypoint.fromHolonomicPose(betweenOneandTwo));
        }
        waypoints.add(
            Waypoint.fromHolonomicPose(
                AllianceFlipUtil.addToFlipped(
                    wingGamePieceLocations[wingPosition].plus(
                        new Transform2d(0.0, 0.35, new Rotation2d())),
                    -0.75)));
      }
      waypoints.add(
          Waypoint.fromHolonomicPose(
              wingGamePieceLocations[wingPosition].plus(
                  new Transform2d(0.0, Units.inchesToMeters(4.0), new Rotation2d()))));
    } else {
      waypoints.add(Waypoint.fromHolonomicPose(wingGamePieceLocations[wingPosition]));
    }
    var handoff = new Handoff(intake, superstructure, shooter).withTimeout(2.5);
    if (!shooter.hasGamePiece()) {
      return new CommandWithPose(
          UtilCommands.namedSequence(
              "Auto Intake Wing Progress",
              resetFlipped(startingPosition),
              superstructure.setMode(SuperstructureState.POSE),
              superstructure.setPose(Preset.HANDOFF).withTimeout(0.3),
              path(waypoints)
                  .alongWith(
                      Commands.waitUntil(
                              () -> {
                                return AllianceFlipUtil.apply(drive.getPose().getX())
                                    >= FieldConstants.StartingZone.endX;
                              })
                          .andThen(handoff))),
          new Pose2d(
              waypoints.get(waypoints.size() - 1).getTranslation(),
              waypoints.get(waypoints.size() - 1).getHolonomicRotation().get()));
    } else {
      return new CommandWithPose(Commands.none(), startingPosition);
    }
  }

  private Command prepareShooting(Pose2d targetPose) {
    final var shot = ShooterMath.calculateShotData(targetPose, new ChassisSpeeds());
    final double rpm = shot.rpm();
    final double angle = shot.pivotAngle();

    return superstructure
        .setPose(Preset.HANDOFF)
        .withTimeout(0.8)
        .andThen(
            new Handoff(intake, superstructure, shooter)
                .withTimeout(0.3)
                .andThen(
                    Commands.parallel(
                        superstructure.setMode(SuperstructureState.SHOOTING),
                        Commands.run(
                            () -> {
                              shooter.spinToSpeedVoid(rpm);
                            },
                            shooter),
                        Commands.run(() -> superstructure.setShootingAngleRad(angle)))));
  }

  private CommandWithPose autoShoot(
      Pose2d startingPose, boolean intakeFirst, boolean driveFirst, double timeout) {
    return autoShoot(startingPose, intakeFirst, driveFirst, false, timeout);
  }

  private CommandWithPose autoShoot(
      Pose2d startingPose,
      boolean intakeFirst,
      boolean driveFirst,
      boolean autoTurn,
      double timeout) {
    var startingWaypoint = Waypoint.fromHolonomicPose(startingPose);
    List<Waypoint> waypoints = new ArrayList<>();
    waypoints.add(startingWaypoint);
    // if (AllianceFlipUtil.apply(startingPose.getX()) > 4) {
    // waypoints.add(
    // Waypoint.fromHolonomicPose(
    // AllianceFlipUtil.apply(new Pose2d(4.0, 6.25, driveRotation(new Pose2d(4.0,
    // 6.25, new Rotation2d()))))));
    // } else {
    // waypoints.add(
    // Waypoint.fromHolonomicPose(
    // new Pose2d(startingPose.getTranslation(), driveRotation(startingPose))));
    // }
    waypoints.add(
        Waypoint.fromHolonomicPose(
            new Pose2d(startingPose.getTranslation(), driveRotation(startingPose))));
    var intakeCommand =
        intakeFirst
            ? new Handoff(intake, superstructure, shooter).withTimeout(0.25)
            : Commands.none();

    Supplier<Waypoint> rotationWaypoint =
        () -> {
          if (!autoTurn) {
            final var pose = AllianceFlipUtil.apply(new Pose2d(4.0, 6.25, new Rotation2d()));
            return Waypoint.fromHolonomicPose(new Pose2d(4.0, 6.25, driveRotation(pose)));
          } else {
            return Waypoint.fromHolonomicPose(
                new Pose2d(startingPose.getTranslation(), driveRotation(startingPose)));
          }
        };
    var driveToPose =
        new DriveToPose(
            drive,
            () ->
                new Pose2d(
                    rotationWaypoint.get().getTranslation(),
                    rotationWaypoint.get().getHolonomicRotation().get()));

    return new CommandWithPose(
        UtilCommands.namedSequence(
            "Autonomous Shoot Progress",
            resetFlipped(startingPose),
            intakeFirst ? intakeCommand : Commands.none(),
            superstructure.setMode(SuperstructureState.SHOOTING),
            Commands.deadline(
                    driveFirst ? driveToPose.withTimeout(timeout) : Commands.none(),
                    Commands.run(
                        () -> {
                          shooter.spinToSpeedVoid(ShooterMath.calculateShooterRPM(startingPose));
                        },
                        shooter),
                    Commands.run(() -> superstructure.setShootingAngleRad(pivotAngle)))
                .andThen(
                    Commands.waitUntil(
                        () -> superstructure.isAtSetpoint() && shooter.isAtSetpoint()))
                .andThen(shooter.setRawFeederVoltsCommand(-2))
                .andThen(Commands.waitSeconds(0.15).withTimeout(0.15))
                .andThen(shooter.setRawFeederVoltsCommand(0.0))
                .andThen(shooter.setSpeed(0.0))
                .andThen(superstructure.setMode(SuperstructureState.POSE))),
        new Pose2d(
            waypoints.get(waypoints.size() - 1).getTranslation(),
            waypoints.get(waypoints.size() - 1).getHolonomicRotation().get()));
  }

  /**
   * Drives to the specified game piece in the center of the field and intakes it.
   *
   * @return The command.
   */
  private CommandWithPose driveAndIntakeCenter(Pose2d startingPosition, int centerPosition) {
    List<Waypoint> waypoints = new ArrayList<>();
    waypoints.add(Waypoint.fromHolonomicPose(startingPosition));
    Command handoff = new Handoff(intake, superstructure, shooter);
    if (centerPosition <= 1) {
      if (startingPosition.getX() < 4) {
        waypoints.add(Waypoint.fromHolonomicPose(betweenZeroAndOne));
      }
      waypoints.add(Waypoint.fromHolonomicPose(centerGamePieceLocations[centerPosition]));
    } else if (centerPosition == 2) {
      if (startingPosition.getX() < 4) {
        waypoints.add(Waypoint.fromHolonomicPose(betweenZeroAndOne));
      }
      waypoints.add(Waypoint.fromHolonomicPose(centerGamePieceLocations[centerPosition]));
    }
    if (!shooter.hasGamePiece()) {
      return new CommandWithPose(
          Commands.sequence(
              superstructure.setMode(SuperstructureState.POSE),
              superstructure.setPose(Preset.HANDOFF).withTimeout(0.2),
              path(waypoints)
                  .alongWith(
                      Commands.waitUntil(() -> AllianceFlipUtil.apply(drive.getPose().getX()) > 4.5)
                          .andThen(handoff.withTimeout(2.5)))),
          centerGamePieceLocations[centerPosition]);
    } else {
      return new CommandWithPose(Commands.none(), startingPosition);
    }
  }

  /**
   * Drives the robot out to the center of the field. Used for the end of some autos so that drivers
   * are in a good position to start the teleop period
   *
   * @param currentPose The current pose of the robot
   * @return The command to run
   */
  private Command driveOutToCenter(Pose2d currentPose) {
    ArrayList<Waypoint> waypoints = new ArrayList<>();

    final Rotation2d rotation = AllianceFlipUtil.apply(new Rotation2d());

    double x = FieldConstants.midLineX - 2.5;
    double y = FieldConstants.midLineY / 3;
    // Choose the quicker side to go to based on where the robot is
    if (currentPose.getY() > FieldConstants.midLineY) {
      y = FieldConstants.fieldWidth - y;
      waypoints.add(Waypoint.fromHolonomicPose(betweenZeroAndOne));
    } else {
      // Create a transitory waypoint so that the robot doesn't go through the stage
      final double transitoryX = x / 4.0;
      final double transitoryY = y;
      waypoints.add(
          Waypoint.fromHolonomicPose(
              new Pose2d(AllianceFlipUtil.apply(transitoryX), transitoryY, rotation)));
    }

    waypoints.add(Waypoint.fromHolonomicPose(new Pose2d(AllianceFlipUtil.apply(x), y, rotation)));
    return path(waypoints);
  }

  /**
   * Get a transitory pose for pathing on the wing line, but on the side that your current pose's y
   * is on
   *
   * @param currentPose The current pose of the robot
   * @param desiredRotation The unflipped desired rotation for the pose
   * @return The pose
   */
  private Pose2d getWingLinePose(Pose2d currentPose, Rotation2d desiredRotation) {
    final double x = AllianceFlipUtil.apply(FieldConstants.Wing.endX);
    double y = FieldConstants.midLineY / 3;
    if (currentPose.getY() > FieldConstants.midLineY) {
      y = FieldConstants.fieldWidth - y;
    }
    return new Pose2d(x, y, AllianceFlipUtil.apply(desiredRotation));
  }

  /**
   * An auto that starts at one of the starting locations, scores a game piece, and then drives past
   * the line.
   *
   * @return The command.
   */
  private Command onePiece(int startingLocation) {
    Pose2d startingPose = startingLocations[startingLocation];
    var autoShoot = autoShoot(AllianceFlipUtil.apply(startingPose), true, false, false, 0.1);
    return UtilCommands.namedSequence(
        "Auto Progress", resetFlipped(startingPose), autoShoot.command());
  }

  public Command onePiece() {
    return Commands.select(
        Map.of(
            AutoQuestionResponse.AMP_SIDE,
            onePiece(0),
            AutoQuestionResponse.CENTER,
            onePiece(1),
            AutoQuestionResponse.WALL_SIDE,
            onePiece(2)),
        () -> responses.get().get(0));
  }

  private Command twoPiece(int startingLocation) {
    final Pose2d startingPose = twoPieceStartingLocations[startingLocation];
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
    final var autoShoot1 = autoShoot(driveToBetterSpot1.pose(), true, true, true, shootTimeout);

    // Intake the piece right behind us
    final var driveIntakeWing1 =
        driveAndIntakeWing(autoShoot1.pose(), false, false, startingLocation);
    // Drive to the better spot before the second shot for the wall side, so that we
    // don't run into the stage while targeting
    final var driveToBetterSpot2 =
        startingLocation == 2
            ? new CommandWithPose(
                new DriveToPose(drive, driveToBetterSpotPose2), driveToBetterSpotPose2)
            : new CommandWithPose(Commands.none(), driveIntakeWing1.pose());
    // Shoot the second piece
    final var autoShoot2 = autoShoot(driveToBetterSpot2.pose(), true, true, true, shootTimeout);
    return UtilCommands.namedSequence(
        "Auto Progress",
        resetFlipped(startingPose),
        driveToBetterSpot1.command(),
        autoShoot1.command(),
        driveIntakeWing1.command(),
        driveToBetterSpot2.command(),
        autoShoot2.command(),
        driveOutToCenter(autoShoot2.pose()));
  }

  public Command twoPiece() {
    return Commands.select(
        Map.of(
            AutoQuestionResponse.AMP_SIDE,
            twoPiece(0),
            AutoQuestionResponse.CENTER,
            twoPiece(1),
            AutoQuestionResponse.WALL_SIDE,
            twoPiece(2)),
        () -> responses.get().get(0));
  }

  public Command threePieceClose(boolean isRight) {
    final Pose2d startingPose = startingLocations[1];
    var autoShoot1 = autoShoot(startingPose, true, false, true, 0.1);
    System.out.println(autoShoot1.pose().getX());

    var intake1 = driveAndIntakeWing(autoShoot1.pose(), false, false, 1);
    var autoShoot2 = autoShoot(intake1.pose(), false, true, true, 2.2);
    var intake2 =
        isRight
            ? driveAndIntakeWing(autoShoot2.pose(), true, false, 2)
            : driveAndIntakeWing(autoShoot2.pose(), true, false, 0);
    var autoShoot3 = autoShoot(wingGamePieceLocations[1], false, true, true, 2.0);
    return UtilCommands.namedSequence(
        "Auto Progress",
        resetFlipped(startingPose),
        autoShoot1.command(),
        intake1.command(),
        autoShoot2.command(),
        intake2.command(),
        path(
            Waypoint.fromHolonomicPose(intake2.pose()),
            Waypoint.fromHolonomicPose(wingGamePieceLocations[1])),
        autoShoot3.command());
  }

  public Command threePieceClose() {
    return Commands.select(
        Map.of(
            AutoQuestionResponse.AMP_SIDE,
            threePieceClose(false),
            AutoQuestionResponse.WALL_SIDE,
            threePieceClose(true)),
        () -> responses.get().get(0));
  }

  public Command threePieceCenterWingOld() {
    final Pose2d startingPose = startingLocations[1];
    var autoShoot1 = autoShoot(startingPose, true, false, true, 0.1);
    var intake1 = driveAndIntakeWing(autoShoot1.pose(), false, false, 1);
    var autoShoot2 = autoShoot(intake1.pose(), false, true, true, 2.0);
    var intake2 = driveAndIntakeWing(autoShoot2.pose(), true, false, 0);
    var autoShoot3 = autoShoot(wingGamePieceLocations[1], false, true, true, 2.0);
    return Commands.sequence(
        autoShoot1.command(),
        intake1.command(),
        autoShoot2.command(),
        intake2.command(),
        path(
            Waypoint.fromHolonomicPose(intake2.pose()),
            Waypoint.fromHolonomicPose(wingGamePieceLocations[1])),
        autoShoot3.command());
  }

  public Command fourPieceCenterWing() {
    Pose2d startingPose = startingLocations[1];
    var autoShoot1 = autoShoot(startingPose, true, false, 0.1);
    var intake1 = driveAndIntakeWing(autoShoot1.pose(), false, false, 1);
    var autoShoot2 = autoShoot(intake1.pose(), false, true, 0.75);
    var intake2 = driveAndIntakeWing(autoShoot2.pose(), true, true, 0);
    var autoShoot3 = autoShoot(intake2.pose(), false, true, 1.3);
    var intake3 = driveAndIntakeWing(autoShoot3.pose(), true, false, 2);
    var autoShoot4 = autoShoot(wingGamePieceLocations[1], false, true, 1.25);
    return UtilCommands.namedSequence(
        "Auto Progress",
        autoShoot1.command(),
        intake1.command(),
        autoShoot2.command(),
        intake2.command(),
        autoShoot3.command(),
        intake3.command(),
        path(
            Waypoint.fromHolonomicPose(intake3.pose()),
            Waypoint.fromHolonomicPose(
                intake3
                    .pose()
                    .plus(new Transform2d(Units.inchesToMeters(-24), 0.0, new Rotation2d()))),
            Waypoint.fromHolonomicPose(wingGamePieceLocations[1])),
        autoShoot4.command());
  }

  public Command fourPieceLong() {
    var autoShoot1 = autoShoot(startingLocations[1], true, false, true, 0.3);
    var driveAndIntakeCenter1 = driveAndIntakeCenter(startingLocations[1], 0);
    final var robotAngle1 = ShooterMath.getGoalTheta(farShootingPose);
    final var shootPose1 = farShootingPose.plus(new Transform2d(0.0, 0.0, robotAngle1));
    final var move1 = new DriveToPose(drive, shootPose1);
    var autoShoot2 = autoShoot(shootPose1, false, false, false, 1.2);
    var driveAndIntakeCenter2 = driveAndIntakeCenter(autoShoot2.pose(), 1);
    final Pose2d shootPose2 =
        AllianceFlipUtil.addToFlipped(betweenZeroAndOne, -Units.inchesToMeters(30));
    final var robotAngle2 = ShooterMath.getGoalTheta(shootPose2);
    final var wingLine = getWingLinePose(driveAndIntakeCenter2.pose(), Rotation2d.fromDegrees(180));
    final var move2 =
        path(
            // Waypoint.fromHolonomicPose(wingLine, new Rotation2d()),
            Waypoint.fromHolonomicPose(shootPose2),
            Waypoint.fromHolonomicPose(shootPose2.plus(new Transform2d(0.0, 0.0, robotAngle2))));
    // final var move2 =
    var autoShoot3 = autoShoot(shootPose2, false, true, true, 1.2);
    var driveAndIntakeWing1 = driveAndIntakeWing(autoShoot3.pose(), false, true, 0);
    var autoShoot4 = autoShoot(driveAndIntakeWing1.pose(), false, true, true, 1.5);
    return UtilCommands.namedSequence(
        "Auto Progress",
        reset(startingLocations[1]),
        autoShoot1.command(),
        driveAndIntakeCenter1.command(),
        Commands.deadline(move1, prepareShooting(autoShoot2.pose())),
        autoShoot2.command(),
        driveAndIntakeCenter2.command(),
        Commands.deadline(move2, prepareShooting(autoShoot3.pose())),
        autoShoot3.command(),
        driveAndIntakeWing1.command(),
        autoShoot4.command());
  }

  private Command mobility(int startingPosition) {
    Pose2d startingPose = startingLocations[startingPosition];
    return Commands.sequence(
        reset(startingPose),
        path(
            Waypoint.fromHolonomicPose(startingPose),
            Waypoint.fromHolonomicPose(
                startingPose.plus(new Transform2d(0.9, 0, new Rotation2d())))));
  }

  public Command plow(int startingLocation) {
    final Pose2d startingPose =
        new Pose2d(
            twoPieceStartingLocations[0].getX(),
            FieldConstants.fieldWidth * (1.0 - 0.065),
            new Rotation2d());
    final Pose2d centerPose = centerGamePieceLocations[0];
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
        .raceWith(
            UtilCommands.namedSequence("Auto Progress", resetFlipped(startingPose), intake, plow));
  }

  public Command plow2(int startingLocation) {
    final Pose2d startingPose =
        new Pose2d(
            twoPieceStartingLocations[0].getX(),
            FieldConstants.fieldWidth * (0.05),
            new Rotation2d());
    final Pose2d centerPose = centerGamePieceLocations[0];
    final var intake = driveAndIntakeCenter2(startingPose, 4);
    double rv = Units.degreesToRadians(-90);
    if (AllianceFlipUtil.shouldFlip()) {
      rv *= -1;
    }
    final double rv2 = rv;
    final var plow =
        Commands.run(
                () ->
                    drive.runVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(0.0, -1.0, rv2, drive.getRotation())),
                drive)
            .raceWith(
                Commands.waitUntil(() -> drive.getPose().getY() < FieldConstants.midLineY * 1.37));
    return UtilCommands.namedSequence(
        "Auto Progress", resetFlipped(startingPose), intake.command(), plow);
  }

  /**
   * Returns the mobility auto, where the robot starts in one of the predefined positions, and then
   * drives forwards 0.9 meters.
   *
   * @return The command.
   */
  public Command mobility() {
    return Commands.select(
        Map.of(
            AutoQuestionResponse.AMP_SIDE,
            mobility(0),
            AutoQuestionResponse.CENTER,
            mobility(1),
            AutoQuestionResponse.WALL_SIDE,
            mobility(2)),
        () -> responses.get().get(0));
  }

  public Command pitTest() {
    return new PitTest(drive, superstructure, intake, shooter);
  }

  private CommandWithPose driveAndIntakeCenter2(Pose2d startingPosition, int centerPosition) {
    List<Waypoint> waypoints = new ArrayList<>();
    waypoints.add(Waypoint.fromHolonomicPose(startingPosition));
    Command handoff = Commands.none();
    if (centerPosition <= 1) {
      if (startingPosition.getX() < 4) {
        // waypoints.add(Waypoint.fromHolonomicPose(betweenZeroAndOne));
      }
      waypoints.add(Waypoint.fromHolonomicPose(centerGamePieceLocations[centerPosition]));
    } else if (centerPosition == 2) {
      if (startingPosition.getX() < 4) {
        // waypoints.add(Waypoint.fromHolonomicPose(betweenZeroAndOne));
      }
      waypoints.add(Waypoint.fromHolonomicPose(centerGamePieceLocations[centerPosition]));
    }
    if (!shooter.hasGamePiece()) {
      return new CommandWithPose(
          UtilCommands.namedSequence(
              "Intake Center 2 Progress",
              superstructure.setMode(SuperstructureState.POSE),
              // superstructure.setPose(Preset.HANDOFF).withTimeout(0.2),
              path(waypoints)
                  .alongWith(
                      Commands.waitUntil(() -> AllianceFlipUtil.apply(drive.getPose().getX()) > 4.5)
                          .andThen(handoff.withTimeout(2.5)))),
          centerGamePieceLocations[centerPosition]);
    } else {
      return new CommandWithPose(Commands.none(), startingPosition);
    }
  }
}
