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
import frc.WorBots.util.math.GeomUtil;
import frc.WorBots.util.trajectory.CustomPathGenerator;
import frc.WorBots.util.trajectory.Waypoint;
import java.util.List;

/** Drives along a trajectory with a newer method */
public class DriveTrajectory3 extends Command {
  // Constants

  /** Lookahead amount in time */
  private static final TunableDouble LOOKAHEAD =
      new TunableDouble("Tuning", "Pathing 2", "Lookahead", 0.15);

  /** Scaling factor for feedforwards based on distance. Lower values create tighter tolerances. */
  private static final TunableDouble FF_DISTANCE_SCALING =
      new TunableDouble("Tuning", "Pathing 2", "FF Distance Scaling", 0.2);

  /** Threshold for path points to say we have finished them */
  private static final TunableDouble DISTANCE_THRESHOLD =
      new TunableDouble("Tuning", "Pathing 2", "Distance Threshold", 0.2);

  /** Threshold for path points to say we have finished them */
  private static final TunableDouble TRACKING_THRESHOLD =
      new TunableDouble("Tuning", "Pathing 2", "Tracking Threshold", 0.05);

  /** Threshold for theta */
  private static final TunableDouble THETA_THRESHOLD =
      new TunableDouble("Tuning", "Pathing 2", "Theta Threshold", Units.degreesToRadians(1.5));

  private final Drive drive;
  private final PathPlannerTrajectory trajectory;
  private final PIDController trackingController;
  private final PIDController pathingController;
  private final PIDController thetaController;
  private final Pose2d[] loggableTrajectory;

  /** Current time in the path */
  private double t = 0.0;

  private final Timer pathTimer = new Timer();

  public DriveTrajectory3(
      Drive drive,
      List<Waypoint> waypoints,
      List<TrajectoryConstraint> extraConstraints,
      double startVelocity) {
    this.drive = drive;

    final double maxVelocityMetersPerSec = Units.inchesToMeters(160.0);
    final double maxAccelerationMetersPerSec2 = Units.inchesToMeters(200.0);

    if (Constants.getSim()) {
      trackingController = new PIDController(6.8, 0.0, 0.0, Constants.ROBOT_PERIOD);
      pathingController = new PIDController(5.0, 0.0, 0.0, Constants.ROBOT_PERIOD);
      thetaController = new PIDController(6.4, 0.0, 0.0, Constants.ROBOT_PERIOD);
    } else {
      trackingController = new PIDController(2.0, 0.0, 0.0, Constants.ROBOT_PERIOD);
      pathingController = new PIDController(2.0, 0.0, 0.0, Constants.ROBOT_PERIOD);
      thetaController = new PIDController(6.4, 0.0, 0.0, Constants.ROBOT_PERIOD);
    }
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    trackingController.setTolerance(TRACKING_THRESHOLD.get());
    pathingController.setTolerance(DISTANCE_THRESHOLD.getCurrent());
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
    t = 0.0;
    pathTimer.restart();

    trackingController.reset();
    pathingController.reset();
    thetaController.reset();

    // Log trajectory
    Logger.getInstance().setDriveTrajectory(loggableTrajectory);
  }

  @Override
  public void execute() {
    // Find out what states we are sampling from
    final double lookahead = LOOKAHEAD.get();
    final double stateT = MathUtil.clamp(t, 0.0, trajectory.getTotalTimeSeconds());
    final State state = trajectory.sample(stateT);
    final double lookaheadStateT =
        MathUtil.clamp(stateT + lookahead * state.velocityMps, 0, trajectory.getTotalTimeSeconds());
    final State lookaheadState = trajectory.sample(lookaheadStateT);
    Logger.getInstance()
        .setDriveTrajSetpoint(
            state.getTargetHolonomicPose(), lookaheadState.getTargetHolonomicPose());

    final double stateDistance = drive.getPose().getTranslation().getDistance(state.positionMeters);
    final double lookaheadDistance =
        drive.getPose().getTranslation().getDistance(lookaheadState.positionMeters);

    // Calculate feedforward from the time-parameterized trajectory at the lookahead
    // state
    final double ffCoefficient = scaleFeedforward(stateDistance) * 0.0;
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
    final Translation2d stateVector = new Translation2d(1.0, lookaheadState.heading);
    final double pathingMeasurement = GeomUtil.dot(drive.getPose().getTranslation(), stateVector);
    final double pathingFeedback =
        trackingController.calculate(
            pathingMeasurement, GeomUtil.dot(lookaheadState.positionMeters, stateVector));
    final Translation2d pathingFeedbackVector = stateVector.times(pathingFeedback);
    final Translation2d perpendicularVector = stateVector.rotateBy(Rotation2d.fromDegrees(90));
    final double trackingMeasurement =
        GeomUtil.dot(drive.getPose().getTranslation(), perpendicularVector);
    final double trackingFeedback =
        trackingController.calculate(
            trackingMeasurement, GeomUtil.dot(lookaheadState.positionMeters, perpendicularVector));
    final Translation2d trackingFeedbackVector = perpendicularVector.times(trackingFeedback);

    final Translation2d xUnit = new Translation2d(1.0, 0.0);
    final Translation2d yUnit = new Translation2d(0.0, 1.0);
    final double xFeedback =
        GeomUtil.dot(pathingFeedbackVector, xUnit) + GeomUtil.dot(trackingFeedbackVector, xUnit);
    final double yFeedback =
        GeomUtil.dot(pathingFeedbackVector, yUnit) + GeomUtil.dot(trackingFeedbackVector, yUnit);

    // final double xFeedback = trackingController.calculate(drive.getPose().getX(),
    //         lookaheadState.positionMeters.getX());
    // final double yFeedback = pathingController.calculate(drive.getPose().getY(),
    //         lookaheadState.positionMeters.getY());
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
    // if ()
    // for (int i = 0; i < lookahead; i++) {
    // if (i + index >= trajectory.getStates().size()) {
    // break;
    // }

    // final double distance = drive
    // .getPose()
    // .getTranslation()
    // .getDistance(trajectory.getState(i + index).positionMeters);
    // if (distance < DISTANCE_THRESHOLD.getCurrent()) {
    // index = index + i + 1;
    // break;
    // }
    // }
    final double distance =
        GeomUtil.pointToLineSegmentDistance(
            drive.getPose().getTranslation(), state.positionMeters, lookaheadState.positionMeters);
    if (distance < DISTANCE_THRESHOLD.getCurrent() && trackingController.atSetpoint()) {
      t += 0.15;
    }
    // if (lookaheadDistance < DISTANCE_THRESHOLD.get()) {
    // t = lookaheadStateT + 0.05;
    // } else if (stateDistance < DISTANCE_THRESHOLD.getCurrent()) {
    // t += 0.05;
    // }
  }

  @Override
  public boolean isFinished() {
    return t >= trajectory.getTotalTimeSeconds()
        && trackingController.atSetpoint()
        && pathingController.atSetpoint()
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
