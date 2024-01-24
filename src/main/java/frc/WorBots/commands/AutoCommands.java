// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
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
import frc.WorBots.util.*;
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

  // Poses
  private final Pose2d[] startingLocations;
  private final Pose2d[] wingGamePieceLocations;
  private final Pose2d[] centerGamePieceLocations;

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

  private Command driveAndIntakeWing() {
    return Commands.none();
  }

  private Command driveAndIntakeCenter() {
    return Commands.none();
  }

  private Command driveAndShoot() {
    return Commands.none();
  }

  public Command onePiece() {
    Pose2d startingPose = startingLocations[0];
    return Commands.sequence(
        reset(startingPose),
        path(
            Waypoint.fromHolonomicPose(startingPose),
            Waypoint.fromHolonomicPose(wingGamePieceLocations[0]),
            Waypoint.fromHolonomicPose(startingPose),
            Waypoint.fromHolonomicPose(centerGamePieceLocations[1])));
  }
}
