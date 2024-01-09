package frc.WorBots.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.util.AllianceFlipUtil;
import frc.WorBots.util.trajectory.Waypoint;

public class AutoCommands extends Command {
  private final Drive drive;

  public AutoCommands(Drive drive) {
    this.drive = drive;
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

}
