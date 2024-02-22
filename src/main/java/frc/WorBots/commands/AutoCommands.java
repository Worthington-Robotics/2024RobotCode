// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.constraint.*;
import edu.wpi.first.math.util.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.*;
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
  private final Pose2d[] wingGamePieceLocations;
  private final Pose2d[] centerGamePieceLocations;
  private final Pose2d[] shootingPositions;
  private final Pose2d betweenZeroAndOne;
  private final Pose2d betweenOneandTwo;
  private double speakerOpeningHeightZ;
  private double speakerOpeningCenterY;

  // Other
  public static record CommandWithPose(Command command, Pose2d pose) {}

  private Supplier<Double> pivotAngle;
  private Supplier<Rotation2d> driveRotation;

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
    driveRotation =
        () -> {
          Pose2d robotPose = drive.getPose();
          return AllianceFlipUtil.apply(AutoShoot.getRobotRotationToShoot(robotPose));
        };
    startingLocations =
        new Pose2d[] {
          AllianceFlipUtil.apply(
              new Pose2d(
                  new Translation2d(0.69, 6.63), new Rotation2d(Units.degreesToRadians(60)))),
          AllianceFlipUtil.apply(
              new Pose2d(
                  FieldConstants.StartingZone.endX - Units.inchesToMeters(22),
                  FieldConstants.Speaker.speakerY,
                  new Rotation2d())),
          AllianceFlipUtil.apply(
              new Pose2d(
                  new Translation2d(0.69, 4.51), new Rotation2d(Units.degreesToRadians(-60)))),
        };
    wingGamePieceLocations =
        new Pose2d[] {
          new Pose2d(
              FieldConstants.GamePieces.wingPieces[0].plus(
                  new Translation2d(-Units.inchesToMeters(10), 0)),
              new Rotation2d()),
          new Pose2d(
              FieldConstants.GamePieces.wingPieces[1].plus(
                  new Translation2d(-Units.inchesToMeters(10), 0)),
              new Rotation2d()),
          new Pose2d(
              FieldConstants.GamePieces.wingPieces[2].plus(
                  new Translation2d(-Units.inchesToMeters(10), 0)),
              new Rotation2d())
        };
    centerGamePieceLocations =
        new Pose2d[] {
          new Pose2d(
              FieldConstants.GamePieces.centerPieces[0].plus(
                  new Translation2d(-Units.inchesToMeters(0), Units.inchesToMeters(12))),
              new Rotation2d()),
          new Pose2d(
              FieldConstants.GamePieces.centerPieces[1].plus(
                  new Translation2d(-Units.inchesToMeters(0), Units.inchesToMeters(12))),
              new Rotation2d()),
          new Pose2d(
              FieldConstants.GamePieces.centerPieces[2].plus(
                  new Translation2d(-Units.inchesToMeters(0), Units.inchesToMeters(12))),
              new Rotation2d()),
          new Pose2d(
              FieldConstants.GamePieces.centerPieces[3].plus(
                  new Translation2d(-Units.inchesToMeters(0), Units.inchesToMeters(12))),
              new Rotation2d()),
          new Pose2d(
              FieldConstants.GamePieces.centerPieces[4].plus(
                  new Translation2d(-Units.inchesToMeters(0), Units.inchesToMeters(12))),
              new Rotation2d()),
        };
    shootingPositions =
        new Pose2d[] {
          new Pose2d(new Translation2d(3.68, 5.80), new Rotation2d()),
          new Pose2d(new Translation2d(2.88, 5.54), new Rotation2d()),
          new Pose2d(new Translation2d(2.50, 3.49), new Rotation2d())
        };
    betweenZeroAndOne = new Pose2d(2.88, 6.28, new Rotation2d());
    betweenOneandTwo = new Pose2d(2.88, 4.8, new Rotation2d());
    speakerOpeningHeightZ =
        (FieldConstants.Speaker.openingHeightHigher - FieldConstants.Speaker.openingHeightLower)
            / 2;
    speakerOpeningCenterY = (FieldConstants.Speaker.speakerY);
    pivotAngle =
        () -> {
          Pose2d robotPose = drive.getPose();
          var shotData = ShooterMath.calculateShotData(robotPose, drive.getFieldRelativeSpeeds());
          return shotData.pivotAngle();
        };
  }

  private Command reset(Pose2d pose) {
    return Commands.runOnce(() -> drive.setPose(AllianceFlipUtil.apply(pose)));
  }

  /** Drives along the specified trajectory. */
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
                  AllianceFlipUtil.apply(
                      new Pose2d(
                          waypoints.get(1).getTranslation(),
                          waypoints.get(1).getHolonomicRotation().get())));
      return driveToPose.until(driveToPose::atGoal);
    }
    List<TrajectoryConstraint> allConstraints = new ArrayList<>();
    allConstraints.addAll(extraConstraints);
    return new DriveTrajectory(drive, waypoints, allConstraints, 0.0);
  }

  private Command path(Waypoint... waypoints) {
    return path(Arrays.asList(waypoints));
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
        waypoints.add(
            Waypoint.fromHolonomicPose(
                wingGamePieceLocations[wingPosition].plus(
                    new Transform2d(0.0, 0.0, new Rotation2d(Math.PI / 2)))));
      } else if (wingPosition == 2) {
        waypoints.add(
            Waypoint.fromHolonomicPose(
                wingGamePieceLocations[wingPosition].plus(
                    new Transform2d(0.0, 0.0, new Rotation2d(-Math.PI / 2)))));
      } else {
        waypoints.add(Waypoint.fromHolonomicPose(wingGamePieceLocations[wingPosition]));
      }

    } else if (wrapToPickup) {
      if (wingPosition <= 1) {
        if (startingPosition.getX() >= Units.inchesToMeters(116)) {
          waypoints.add(Waypoint.fromHolonomicPose(betweenZeroAndOne));
        }
        waypoints.add(
            Waypoint.fromHolonomicPose(
                wingGamePieceLocations[wingPosition].plus(
                    new Transform2d(-0.6, -0.35, new Rotation2d()))));
      } else {
        if (startingPosition.getX() >= Units.inchesToMeters(116)) {
          waypoints.add(Waypoint.fromHolonomicPose(betweenOneandTwo));
        }
        waypoints.add(
            Waypoint.fromHolonomicPose(
                wingGamePieceLocations[wingPosition].plus(
                    new Transform2d(-0.6, 0.35, new Rotation2d()))));
      }
    } else {
      waypoints.add(Waypoint.fromHolonomicPose(wingGamePieceLocations[wingPosition]));
    }
    var handoff = new Handoff(intake, superstructure, shooter);
    if (!shooter.hasGamePiece()) {
      return new CommandWithPose(
          Commands.sequence(
              superstructure.setMode(SuperstructureState.POSE),
              superstructure.setPose(Preset.HANDOFF),
              path(waypoints)
                  .alongWith(
                      Commands.waitUntil(
                              () -> {
                                return drive.getPose().getX() >= FieldConstants.StartingZone.endX;
                              })
                          .andThen(handoff))),
          new Pose2d(
              waypoints.get(waypoints.size() - 1).getTranslation(),
              waypoints.get(waypoints.size() - 1).getHolonomicRotation().get()));
    } else {
      return new CommandWithPose(Commands.none(), startingPosition);
    }
  }

  private CommandWithPose autoShoot(Pose2d startingPose, boolean intakeFirst, boolean driveFirst) {
    var startingWaypoint = Waypoint.fromHolonomicPose(startingPose);
    List<Waypoint> waypoints = new ArrayList<>();
    waypoints.add(startingWaypoint);
    if (AllianceFlipUtil.apply(startingPose.getX()) > 4) {
      waypoints.add(Waypoint.fromHolonomicPose(new Pose2d(4, 6.25, driveRotation.get())));
    } else {
      waypoints.add(
          Waypoint.fromHolonomicPose(
              new Pose2d(startingPose.getTranslation(), driveRotation.get())));
    }
    var intakeCommand =
        intakeFirst
            ? new Handoff(intake, superstructure, shooter).withTimeout(0.5)
            : Commands.none();

    Supplier<Waypoint> rotationWaypoint =
        () -> {
          if (startingPose.getX() > 3) {
            return Waypoint.fromHolonomicPose(new Pose2d(4, 6.25, driveRotation.get()));
          } else {
            return Waypoint.fromHolonomicPose(
                new Pose2d(startingPose.getTranslation(), driveRotation.get()));
          }
        };
    var driveToPose =
        new DriveToPose(
            drive,
            () ->
                AllianceFlipUtil.apply(
                    new Pose2d(
                        rotationWaypoint.get().getTranslation(),
                        rotationWaypoint.get().getHolonomicRotation().get())));

    return new CommandWithPose(
        Commands.sequence(
            reset(startingPose),
            intakeFirst ? intakeCommand : Commands.none(),
            superstructure.setMode(SuperstructureState.SHOOTING),
            Commands.deadline(
                    driveFirst ? driveToPose.withTimeout(0.1) : Commands.none(),
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

    } else {

    }
    if (!shooter.hasGamePiece()) {
      return new CommandWithPose(
          Commands.sequence(
              superstructure.setMode(SuperstructureState.POSE),
              superstructure.setPose(Preset.HANDOFF),
              path(waypoints)
                  .alongWith(
                      Commands.waitUntil(() -> drive.getPose().getX() > 4.5).andThen(handoff))),
          centerGamePieceLocations[centerPosition]);
    } else {
      return new CommandWithPose(Commands.none(), startingPosition);
    }
  }

  /**
   * An auto that starts at one of the starting locations, scores a game piece, and then drives past
   * the line.
   *
   * @return The command.
   */
  private Command onePiece(int startingLocation) {
    Pose2d startingPose = startingLocations[startingLocation];
    var autoShoot = autoShoot(startingPose, true, false);
    var driveIntakeWing1 = driveAndIntakeWing(startingPose, false, false, startingLocation);
    return Commands.sequence(
        autoShoot.command(),
        // path(
        //     Waypoint.fromHolonomicPose(startingPose),
        //     Waypoint.fromHolonomicPose(wingGamePieceLocations[startingLocation]))
        driveIntakeWing1.command());
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
    Pose2d startingPose = startingLocations[startingLocation];
    var autoShoot1 = autoShoot(startingPose, true, false);
    var driveIntakeWing1 = driveAndIntakeWing(startingPose, false, false, startingLocation);
    var autoShoot2 = autoShoot(driveIntakeWing1.pose(), true, true);
    return Commands.sequence(
        autoShoot1.command(), driveIntakeWing1.command(), autoShoot2.command());
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

  public Command threePieceCenterWing() {
    Pose2d startingPose = startingLocations[1];
    var autoShoot1 = autoShoot(startingPose, true, false);
    var intake1 = driveAndIntakeWing(autoShoot1.pose(), false, false, 1);
    var autoShoot2 = autoShoot(intake1.pose(), false, true);
    var intake2 = driveAndIntakeWing(autoShoot2.pose(), true, false, 0);
    var autoShoot3 = autoShoot(wingGamePieceLocations[1], false, true);
    return Commands.sequence(
        autoShoot1.command(),
        intake1.command(),
        autoShoot2.command(),
        intake2.command(),
        path(
            List.of(
                Waypoint.fromHolonomicPose(autoShoot2.pose()),
                Waypoint.fromHolonomicPose(wingGamePieceLocations[1]))),
        autoShoot3.command());
  }

  public Command fourPieceCenterWing() {
    Pose2d startingPose = startingLocations[1];
    var autoShoot1 = autoShoot(startingPose, true, false);
    var intake1 = driveAndIntakeWing(autoShoot1.pose(), false, false, 1);
    var autoShoot2 = autoShoot(intake1.pose(), false, true);
    var intake2 = driveAndIntakeWing(autoShoot2.pose(), true, true, 0);
    var autoShoot3 = autoShoot(intake2.pose(), false, true);
    var intake3 = driveAndIntakeWing(autoShoot3.pose(), true, false, 2);
    var autoShoot4 = autoShoot(intake3.pose(), false, true);
    return Commands.sequence(
        autoShoot1.command(),
        intake1.command(),
        autoShoot2.command(),
        intake2.command(),
        autoShoot3.command(),
        intake3.command(),
        autoShoot4.command());
  }

  public Command testLongAuto() {
    var autoShoot1 = autoShoot(startingLocations[0], true, false);
    var driveIntakeCenter1 = driveAndIntakeCenter(startingLocations[0], 0);
    var autoShoot2 = autoShoot(centerGamePieceLocations[0], false, true);
    var driveAndIntakeCenter2 = driveAndIntakeCenter(autoShoot2.pose(), 1);
    var autoShoot3 = autoShoot(driveAndIntakeCenter2.pose(), false, true);
    var driveAndIntakeWing1 = driveAndIntakeWing(autoShoot3.pose(), false, true, 0);
    return Commands.sequence(
        reset(startingLocations[0]),
        autoShoot1.command(),
        driveIntakeCenter1.command(),
        autoShoot2.command(),
        driveAndIntakeCenter2.command(),
        autoShoot3.command(),
        driveAndIntakeWing1.command());
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
}
