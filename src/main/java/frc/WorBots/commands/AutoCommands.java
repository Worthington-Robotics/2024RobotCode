// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.constraint.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.*;
import frc.WorBots.AutoSelector.AutoQuestionResponse;
import frc.WorBots.subsystems.drive.*;
import frc.WorBots.subsystems.intake.*;
import frc.WorBots.subsystems.shooter.*;
import frc.WorBots.subsystems.superstructure.*;
import frc.WorBots.util.debug.Logger;
import frc.WorBots.util.math.AllianceFlipUtil;
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
  private final double shootingLineX = Units.inchesToMeters(110);

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
          new Pose2d(
              FieldConstants.StartingZone.regionCorners[3].plus(
                  new Translation2d(-Units.inchesToMeters(22), -0.8)),
              new Rotation2d()),
          new Pose2d(
              FieldConstants.StartingZone.endX - Units.inchesToMeters(22),
              FieldConstants.Speaker.speakerY,
              new Rotation2d()),
          new Pose2d(
              FieldConstants.StartingZone.endX - Units.inchesToMeters(22),
              FieldConstants.Stage.foot1Center.getY(),
              new Rotation2d()),
        };
    wingGamePieceLocations =
        new Pose2d[] {
          new Pose2d(
              FieldConstants.GamePieces.wingPieces[0].plus(
                  new Translation2d(-Units.inchesToMeters(12), 0)),
              new Rotation2d()),
          new Pose2d(
              FieldConstants.GamePieces.wingPieces[1].plus(
                  new Translation2d(-Units.inchesToMeters(12), 0)),
              new Rotation2d()),
          new Pose2d(
              FieldConstants.GamePieces.wingPieces[2].plus(
                  new Translation2d(-Units.inchesToMeters(12), 0)),
              new Rotation2d())
        };
    centerGamePieceLocations =
        new Pose2d[] {
          new Pose2d(
              FieldConstants.GamePieces.centerPieces[0].plus(
                  new Translation2d(-Units.inchesToMeters(12), 0)),
              new Rotation2d()),
          new Pose2d(
              FieldConstants.GamePieces.centerPieces[1].plus(
                  new Translation2d(-Units.inchesToMeters(12), 0)),
              new Rotation2d()),
          new Pose2d(
              FieldConstants.GamePieces.centerPieces[2].plus(
                  new Translation2d(-Units.inchesToMeters(12), 0)),
              new Rotation2d()),
          new Pose2d(
              FieldConstants.GamePieces.centerPieces[3].plus(
                  new Translation2d(-Units.inchesToMeters(12), 0)),
              new Rotation2d()),
          new Pose2d(
              FieldConstants.GamePieces.centerPieces[4].plus(
                  new Translation2d(-Units.inchesToMeters(12), 0)),
              new Rotation2d()),
        };
    shootingPositions =
        new Pose2d[] {
          new Pose2d(new Translation2d(3.68, 5.80), new Rotation2d()),
          new Pose2d(new Translation2d(2.88, 5.54), new Rotation2d()),
          new Pose2d(new Translation2d(2.50, 3.49), new Rotation2d())
        };
    SmartDashboard.putNumberArray("Starting", Logger.pose2dToArray(startingLocations[0]));
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
   * Drives to the specified game piece in the wing and intakes it.
   *
   * @return The command.
   */
  private Command driveAndIntakeWing(int wingPosition) {
    return Commands.none();
  }

  /**
   * Drives to the specified game piece in the center of the field and intakes it.
   *
   * @return The command.
   */
  private Command driveAndIntakeCenter(int centerPosition) {
    return Commands.none();
  }

  /**
   * Drives to a specified safe shooting position and shoots a game piece in the speaker.
   *
   * @return The command.
   */
  private Command driveAndShoot(Pose2d startingPose, int position) {
    List<Waypoint> waypoints = new ArrayList<>();
    Pose2d shootingPose = shootingPositions[position];
    waypoints.add(Waypoint.fromHolonomicPose(startingPose));
    waypoints.add(Waypoint.fromHolonomicPose(shootingPose));
    return Commands.sequence(path(waypoints));
  }

  /**
   * An auto that starts at one of the starting locations, scores a game piece, and then drives past
   * the line.
   *
   * @return The command.
   */
  private Command onePiece(int startingLocation) {
    Pose2d startingPose = startingLocations[startingLocation];
    // return Commands.sequence(reset(startingPose), driveAndShoot(startingPose, startingLocation));
    AutoShoot autoShoot = new AutoShoot(superstructure, drive, shooter, () -> 0.0, () -> 0.0);
    return Commands.sequence(
        // reset(startingPose),
        autoShoot
            .alongWith(Commands.waitSeconds(2.0).andThen(shooter.setRawFeederVoltsCommand(-2)))
            .finallyDo(() -> shooter.setRawFeederVolts(0.0)));
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

  private Command mobility(int startingPosition) {
    Pose2d startingPose = startingLocations[startingPosition];
    return Commands.sequence(
        reset(startingPose),
        path(
            Waypoint.fromHolonomicPose(startingPose),
            Waypoint.fromDifferentialPose(
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
