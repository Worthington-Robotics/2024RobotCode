// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.util.debug.Logger;
import frc.WorBots.util.debug.TunableDouble;
import frc.WorBots.util.trajectory.CustomPathGenerator;
import frc.WorBots.util.trajectory.Waypoint;
import java.util.List;

/** Drives along a trajectory with a new method */
public class DriveTrajectory2 extends Command {
  // Constants

  /** Lookahead amount in number of states */
  private static final TunableDouble LOOKAHEAD =
      new TunableDouble("Tuning", "Pathing", "Lookahead", 7);

  /** Scaling factor for feedforwards based on distance. Lower values create tighter tolerances. */
  private static final TunableDouble FF_DISTANCE_SCALING =
      new TunableDouble("Tuning", "Pathing", "FF Distance Scaling", 0.1);

  /** Threshold for path points to say we have finished them */
  private static final TunableDouble DISTANCE_THRESHOLD =
      new TunableDouble("Tuning", "Pathing", "Distance Threshold", 0.1);

  /** Threshold for theta */
  private static final TunableDouble THETA_THRESHOLD =
      new TunableDouble("Tuning", "Pathing", "Theta Threshold", Units.degreesToRadians(1.5));

  private final Drive drive;
  private final PathPlannerTrajectory trajectory;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;
  private final Pose2d[] loggableTrajectory;

  /** Current index in the path */
  private int index = 0;

  private final Timer pathTimer = new Timer();

  public DriveTrajectory2(
      Drive drive,
      List<Waypoint> waypoints,
      List<TrajectoryConstraint> extraConstraints,
      double startVelocity) {
    this.drive = drive;

    final double maxVelocityMetersPerSec = Units.inchesToMeters(160.0);
    final double maxAccelerationMetersPerSec2 = Units.inchesToMeters(200.0);

    xController = new PIDController(2.9, 0.0, 0.0, Constants.ROBOT_PERIOD);
    yController = new PIDController(2.9, 0.0, 0.0, Constants.ROBOT_PERIOD);
    thetaController = new PIDController(6.4, 0.0, 0.0, Constants.ROBOT_PERIOD);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController.setTolerance(DISTANCE_THRESHOLD.get());
    yController.setTolerance(DISTANCE_THRESHOLD.getCurrent());
    thetaController.setTolerance(THETA_THRESHOLD.get());

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

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    index = 0;
    pathTimer.restart();

    xController.reset();
    yController.reset();
    thetaController.reset();

    // Log trajectory
    Logger.getInstance().setDriveTrajectory(loggableTrajectory);
  }

  @Override
  public void execute() {
    // Find out what states we are sampling from
    final int lookahead = (int) (LOOKAHEAD.get());
    final int lookaheadStateIndex =
        MathUtil.clamp(index + lookahead, 0, trajectory.getStates().size() - 1);
    final int stateIndex = MathUtil.clamp(index, 0, trajectory.getStates().size() - 1);
    final State lookaheadState = trajectory.getState(lookaheadStateIndex);
    final State state = trajectory.getState(stateIndex);
    Logger.getInstance()
        .setDriveTrajSetpoint(
            state.getTargetHolonomicPose(), lookaheadState.getTargetHolonomicPose());

    final double stateDistance = drive.getPose().getTranslation().getDistance(state.positionMeters);
    // final double lookaheadDistance =
    //     drive.getPose().getTranslation().getDistance(lookaheadState.positionMeters);

    // Calculate feedforward from the time-parameterized trajectory at the lookahead
    // state
    final double ffCoefficient = scaleFeedforward(stateDistance);
    final Translation2d ffTranslation =
        new Translation2d(lookaheadState.heading.getCos(), lookaheadState.heading.getSin())
            .times(lookaheadState.velocityMps)
            .times(ffCoefficient);
    final double ffRotation =
        lookaheadState.holonomicAngularVelocityRps.orElse(0.0) * ffCoefficient;

    SmartDashboard.putNumberArray(
        "Pathing/Feedforward",
        Logger.pose2dToArray(new Pose2d(ffTranslation, new Rotation2d(ffRotation))));

    // Calculate feedback from PID controllers at the lookahead state
    final double xFeedback =
        xController.calculate(drive.getPose().getX(), lookaheadState.positionMeters.getX());
    final double yFeedback =
        yController.calculate(drive.getPose().getY(), lookaheadState.positionMeters.getY());
    final double thetaFeedback =
        thetaController.calculate(
            drive.getPose().getRotation().getRadians(),
            lookaheadState.targetHolonomicRotation.getRadians());

    // Create final ChassisSpeeds
    final ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            ffTranslation.getX() + xFeedback,
            ffTranslation.getY() + yFeedback,
            ffRotation + thetaFeedback,
            drive.getRotation());

    drive.runVelocity(speeds);

    // Check if we are close to either the target or lookahead state to advance the
    // path
    DISTANCE_THRESHOLD.update();
    for (int i = 0; i < lookahead; i++) {
      if (i + index >= trajectory.getStates().size()) {
        break;
      }

      final double distance =
          drive
              .getPose()
              .getTranslation()
              .getDistance(trajectory.getState(i + index).positionMeters);
      if (distance < DISTANCE_THRESHOLD.getCurrent()) {
        index = index + i + 1;
        break;
      }
    }
    // if (lookaheadDistance < DISTANCE_THRESHOLD.get()) {
    //   index = lookaheadStateIndex + 1;
    // } else if (distance < DISTANCE_THRESHOLD.getCurrent()) {
    //   index++;
    // }
  }

  @Override
  public boolean isFinished() {
    return index >= trajectory.getStates().size()
        && xController.atSetpoint()
        && yController.atSetpoint()
        && thetaController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  /**
   * Returns a scale factor for trajectory state feedforward values based on the robot's distance to
   * that state's pose
   */
  private static double scaleFeedforward(double distance) {
    return 1.0 - MathUtil.clamp(distance / FF_DISTANCE_SCALING.get(), 0.0, 1.0);
  }
}
