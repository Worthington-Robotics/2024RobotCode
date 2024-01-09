package frc.WorBots.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.constraint.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.*;
import frc.WorBots.subsystems.drive.*;
import frc.WorBots.util.*;
import frc.WorBots.util.trajectory.*;

public class AutoCommands extends Command {
  // Subsystems
  private final Drive drive;

  // Poses
  private final Pose2d[] startingLocations;

  public AutoCommands(Drive drive) {
    this.drive = drive;
    startingLocations = new Pose2d[] {
        new Pose2d(FieldConstants.StartingZone.regionCorners[0].plus(new Translation2d(-1, 1)), new Rotation2d())
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
      var driveToPose = new DriveToPose(
          drive,
          () -> AllianceFlipUtil.apply(
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
}
