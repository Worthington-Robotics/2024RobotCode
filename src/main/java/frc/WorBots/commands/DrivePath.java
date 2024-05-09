// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.util.debug.Logger;
import frc.WorBots.util.trajectory.CustomPathGenerator;
import frc.WorBots.util.trajectory.Waypoint;
import java.util.List;

/** Drives along a PathPlanner path */
public class DrivePath extends Command {
  private final Drive drive;
  private final PathPlannerTrajectory trajectory;
  private final PathFollowingController controller;
  private final Pose2d[] loggableTrajectory;
  private final Timer pathTimer = new Timer();

  public DrivePath(
      Drive drive,
      List<Waypoint> waypoints,
      List<TrajectoryConstraint> extraConstraints,
      double startVelocity) {
    this.drive = drive;

    final double maxVelocityMetersPerSec = Units.inchesToMeters(160.0);
    final double maxAccelerationMetersPerSec2 = Units.inchesToMeters(600.0);

    final TrajectoryConfig config =
        new TrajectoryConfig(maxVelocityMetersPerSec, maxAccelerationMetersPerSec2)
            .setKinematics(new SwerveDriveKinematics(drive.getModuleTranslations()))
            .setStartVelocity(startVelocity)
            .setEndVelocity(0.0)
            .addConstraints(extraConstraints);
    final CustomPathGenerator generator = new CustomPathGenerator();
    generator.generate(config, waypoints);
    trajectory = generator.getDriveTrajectory();

    // Create loggable trajectory
    loggableTrajectory = new Pose2d[trajectory.getStates().size()];
    for (int i = 0; i < trajectory.getStates().size(); i++) {
      loggableTrajectory[i] =
          new Pose2d(trajectory.getStates().get(i).positionMeters, new Rotation2d());
    }

    // Create controller
    final PIDConstants driveGains =
        Constants.getSim() ? new PIDConstants(2.7, 0.0, 0.0) : new PIDConstants(8.9, 0, 0);
    final PIDConstants turnGains =
        Constants.getSim() ? new PIDConstants(6.5, 0.0, 0.0) : new PIDConstants(6.4, 0, 0.03);
    this.controller =
        new PPHolonomicDriveController(
            driveGains,
            turnGains,
            Constants.ROBOT_PERIOD,
            drive.getMaxLinearSpeedMetersPerSec(),
            Drive.WHEELBASE);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    controller.reset(drive.getPose(), drive.getRobotRelativeSpeeds());
    pathTimer.restart();

    // Log trajectory
    Logger.getInstance().setDriveTrajectory(loggableTrajectory);
  }

  @Override
  public void execute() {
    final State state = trajectory.sample(pathTimer.get());

    final ChassisSpeeds speeds = controller.calculateRobotRelativeSpeeds(drive.getPose(), state);
    drive.runVelocity(speeds);
  }

  @Override
  public boolean isFinished() {
    return pathTimer.hasElapsed(trajectory.getTotalTimeSeconds())
        && Math.abs(controller.getPositionalError()) < Units.inchesToMeters(2.7);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
