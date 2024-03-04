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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.WorBots.FieldConstants;
import frc.WorBots.commands.DriveToPose;
import frc.WorBots.commands.DriveTrajectory;
import frc.WorBots.commands.Handoff;
import frc.WorBots.commands.UtilCommands;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.intake.Intake;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.Preset;
import frc.WorBots.util.math.AllianceFlipUtil;
import frc.WorBots.util.math.ShooterMath;
import frc.WorBots.util.trajectory.Waypoint;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** Utility methods, commands, poses, and values for autos to use */
public class AutoUtil {
  // Constants
  public final Pose2d[] startingLocations;
  public final Pose2d[] twoPieceStartingLocations;
  public final Pose2d[] wingGamePieceLocations;
  public final Pose2d[] centerGamePieceLocations;
  public final Pose2d[] shootingPositions;
  public final Pose2d betweenZeroAndOne;
  public final Pose2d betweenOneandTwo;
  public final Pose2d farShootingPose;

  // Subsystems
  private final Drive drive;
  private final Superstructure superstructure;
  private final Intake intake;
  private final Shooter shooter;

  public AutoUtil(Drive drive, Superstructure superstructure, Intake intake, Shooter shooter) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.intake = intake;
    this.shooter = shooter;

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
    betweenZeroAndOne =
        AllianceFlipUtil.apply(
            new Pose2d(
                FieldConstants.GamePieces.wingX,
                (FieldConstants.GamePieces.wingPieces[0].getY()
                        + FieldConstants.GamePieces.wingPieces[1].getY())
                    / 2,
                new Rotation2d()));
    betweenOneandTwo =
        AllianceFlipUtil.apply(
            new Pose2d(
                FieldConstants.GamePieces.wingX,
                (FieldConstants.GamePieces.wingPieces[1].getY()
                        + FieldConstants.GamePieces.wingPieces[2].getY())
                    / 2,
                new Rotation2d()));
    farShootingPose =
        AllianceFlipUtil.apply(
            new Pose2d(
                FieldConstants.Wing.endX * 0.85,
                FieldConstants.Speaker.speakerY * 1.2,
                new Rotation2d()));
  }

  /**
   * Returns a command that resets the robot to an unflipped pose
   *
   * @param pose The unflipped pose to set
   * @return The command to run
   */
  public CommandWithPose resetUnflipped(Pose2d pose) {
    return reset(AllianceFlipUtil.apply(pose));
  }

  /**
   * Returns a command that resets the robot to a pose
   *
   * @param pose The flipped pose to set
   * @return The command to run
   */
  public CommandWithPose reset(Pose2d pose) {
    return new CommandWithPose(Commands.runOnce(() -> drive.setPose(pose)), pose);
  }

  /** Drives along the specified trajectory. */
  public CommandWithPose path(Waypoint... waypoints) {
    return path(Arrays.asList(waypoints));
  }

  /**
   * Drives along the specified trajectory.
   *
   * @param waypoints The list of waypoints to path through
   */
  public CommandWithPose path(List<Waypoint> waypoints) {
    return path(waypoints, List.of());
  }

  /** Drives along the specified trajectory. */
  public CommandWithPose path(
      List<Waypoint> waypoints, List<TrajectoryConstraint> extraConstraints) {
    if (waypoints.size() == 2
        && waypoints.get(0).getDriveRotation().isEmpty()
        && waypoints.get(1).getDriveRotation().isEmpty()
        && waypoints.get(0).getTranslation().getDistance(waypoints.get(1).getTranslation()) < 0.5) {
      return driveTo(waypoints.get(1).getPose());
    }
    List<TrajectoryConstraint> allConstraints = new ArrayList<>();
    allConstraints.addAll(extraConstraints);
    return new CommandWithPose(
        new DriveTrajectory(drive, waypoints, allConstraints, 0.0),
        waypoints.get(waypoints.size() - 1).getPose());
  }

  /** Drives to a single pose */
  public CommandWithPose driveTo(Pose2d pose) {
    final var driveToPose = new DriveToPose(drive, pose);
    return new CommandWithPose(driveToPose.until(driveToPose::atGoal), pose);
  }

  /**
   * Returns a command that prepares the robot for handoff
   *
   * @return The command
   */
  public Command prepareHandoff() {
    return superstructure.setPose(Preset.HANDOFF).withTimeout(0.3);
  }

  /**
   * Returns a command that runs handoff and intake while the robot is near a pose, then ends when
   * the robot is far enough away
   *
   * @param pose The pose to check for distance to
   * @param distance The distance in meters to start and stop intaking at
   * @return The command
   */
  public Command intakeWhenNear(Pose2d pose, double distance) {
    final BooleanSupplier isNearSupplier =
        () -> (drive.getPose().getTranslation().getDistance(pose.getTranslation()) < distance);
    final var handoff = new Handoff(intake, superstructure, shooter);
    return UtilCommands.namedSequence(
        "Intake When Near Progress",
        Commands.waitUntil(isNearSupplier),
        handoff.onlyWhile(isNearSupplier));
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
  public CommandWithPose driveAndIntakeWing(
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
    final var handoff = new Handoff(intake, superstructure, shooter).withTimeout(2.5);
    if (!shooter.hasGamePiece()) {
      return new CommandWithPose(
          UtilCommands.namedSequence(
              "Auto Intake Wing Progress",
              reset(startingPosition).command(),
              prepareHandoff(),
              path(waypoints)
                  .command()
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

  /**
   * Drives to the specified game piece in the center of the field and intakes it.
   *
   * @return The command.
   */
  public CommandWithPose driveAndIntakeCenter(Pose2d startingPosition, int centerPosition) {
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
              prepareHandoff(),
              path(waypoints)
                  .command()
                  .alongWith(
                      Commands.waitUntil(() -> AllianceFlipUtil.apply(drive.getPose().getX()) > 4.5)
                          .andThen(handoff.withTimeout(2.5)))),
          centerGamePieceLocations[centerPosition]);
    } else {
      return new CommandWithPose(Commands.none(), startingPosition);
    }
  }

  public CommandWithPose moveAndShoot(
      Pose2d startingPose, boolean intakeFirst, boolean driveFirst, double timeout) {
    return moveAndShoot(startingPose, intakeFirst, driveFirst, false, timeout);
  }

  public CommandWithPose moveAndShoot(
      Pose2d startingPose,
      boolean intakeFirst,
      boolean driveFirst,
      boolean autoTurn,
      double timeout) {
    var startingWaypoint = Waypoint.fromHolonomicPose(startingPose);
    List<Waypoint> waypoints = new ArrayList<>();
    waypoints.add(startingWaypoint);
    waypoints.add(
        Waypoint.fromHolonomicPose(
            new Pose2d(startingPose.getTranslation(), ShooterMath.getGoalTheta(startingPose))));
    final var intakeCommand =
        intakeFirst
            ? new Handoff(intake, superstructure, shooter).withTimeout(0.25)
            : Commands.none();

    Supplier<Waypoint> rotationWaypoint =
        () -> {
          if (!autoTurn) {
            final var pose = AllianceFlipUtil.apply(new Pose2d(4.0, 6.25, new Rotation2d()));
            return Waypoint.fromHolonomicPose(
                new Pose2d(pose.getX(), pose.getY(), ShooterMath.getGoalTheta(pose)));
          } else {
            return Waypoint.fromHolonomicPose(
                new Pose2d(startingPose.getTranslation(), ShooterMath.getGoalTheta(startingPose)));
          }
        };
    final var driveToPose =
        driveFirst
            ? new DriveToPose(
                    drive,
                    () ->
                        new Pose2d(
                            rotationWaypoint.get().getTranslation(),
                            rotationWaypoint.get().getHolonomicRotation().get()))
                .withTimeout(timeout)
            : Commands.none();

    return new CommandWithPose(
        UtilCommands.namedSequence(
            "Autonomous Shoot Progress",
            reset(startingPose).command().alongWith(intakeCommand),
            Commands.runOnce(
                () -> {
                  superstructure.setModeVoid(SuperstructureState.SHOOTING);
                  shooter.spinToSpeedVoid(ShooterMath.calculateShooterRPM(startingPose));
                  superstructure.setShootingAngleRad(ShooterMath.calculatePivotAngle(startingPose));
                },
                shooter,
                superstructure),
            driveToPose.alongWith(
                Commands.waitUntil(() -> superstructure.isAtSetpoint() && shooter.isAtSetpoint())),
            shooter.setRawFeederVoltsCommand(-2),
            Commands.waitSeconds(0.2).withTimeout(0.2),
            Commands.runOnce(
                () -> {
                  shooter.setRawFeederVolts(0.0);
                  shooter.spinToSpeedVoid(0.0);
                  superstructure.setModeVoid(SuperstructureState.POSE);
                },
                shooter,
                superstructure)),
        waypoints.get(waypoints.size() - 1).getPose());
  }

  public Command prepareShooting(Pose2d targetPose) {
    final var shot = ShooterMath.calculateShotData(targetPose, new ChassisSpeeds());
    final double rpm = shot.rpm();
    final double angle = shot.pivotAngle();

    return UtilCommands.namedSequence(
        "Prepare Shooting Progress",
        prepareHandoff().withTimeout(0.8),
        new Handoff(intake, superstructure, shooter).withTimeout(0.3),
        Commands.parallel(
            superstructure.setMode(SuperstructureState.SHOOTING),
            Commands.runOnce(
                () -> {
                  shooter.spinToSpeedVoid(rpm);
                },
                shooter),
            Commands.runOnce(() -> superstructure.setShootingAngleRad(angle))));
  }

  /**
   * Drives the robot out to the center of the field. Used for the end of some autos so that drivers
   * are in a good position to start the teleop period
   *
   * @param currentPose The current pose of the robot
   * @return The command to run
   */
  public CommandWithPose driveOutToCenter(Pose2d currentPose) {
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
  public Pose2d getWingLinePose(Pose2d currentPose, Rotation2d desiredRotation) {
    final double x = AllianceFlipUtil.apply(FieldConstants.Wing.endX);
    double y = FieldConstants.midLineY / 3;
    if (currentPose.getY() > FieldConstants.midLineY) {
      y = FieldConstants.fieldWidth - y;
    }
    return new Pose2d(x, y, AllianceFlipUtil.apply(desiredRotation));
  }

  public Pose2d getAutoShootPose(Pose2d targetPose) {
    final Rotation2d robotAngle =
        AllianceFlipUtil.flipRotation(ShooterMath.getGoalTheta(targetPose));
    return targetPose.plus(new Transform2d(0.0, 0.0, robotAngle));
  }

  /** A command with a pose that it will end at */
  public static record CommandWithPose(Command command, Pose2d pose) {}
}
