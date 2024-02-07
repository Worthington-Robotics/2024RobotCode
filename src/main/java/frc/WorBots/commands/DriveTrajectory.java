// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.util.AllianceFlipUtil;
import frc.WorBots.util.Logger;
import frc.WorBots.util.trajectory.*;
import java.util.List;
import java.util.function.Supplier;

public class DriveTrajectory extends Command {
  private static boolean supportedRobot = true;
  private static double maxVelocityMetersPerSec;
  private static double maxAccelerationMetersPerSec2;
  private static double maxCentripetalAccelerationMetersPerSec2;

  private PIDController xController = new PIDController(2.5, 0.0, 0.0);
  private PIDController yController = new PIDController(2.5, 0.0, 0.0);
  private PIDController thetaController = new PIDController(5.0, 0.0, 0.0);

  private final CustomHolonomicDriveController customHolonomicDriveController =
      new CustomHolonomicDriveController(xController, yController, thetaController);

  private final Drive drive;
  private final Timer timer = new Timer();

  private Supplier<List<Waypoint>> waypointsSupplier = null;
  private Supplier<List<TrajectoryConstraint>> constraintsSupplier = null;
  private Supplier<Double> startVelocitySupplier = null;
  private CustomTrajectoryGenerator customGenerator = new CustomTrajectoryGenerator();

  /** Creates a DriveTrajectory command with a dynamic set of waypoints. */
  public DriveTrajectory(Drive drive, Supplier<List<Waypoint>> waypointsSupplier) {
    this(drive, waypointsSupplier, () -> List.of(), () -> 0.0);
  }

  /** Creates a DriveTrajectory command with a dynamic set of waypoints and constraints. */
  public DriveTrajectory(
      Drive drive,
      Supplier<List<Waypoint>> waypointsSupplier,
      Supplier<List<TrajectoryConstraint>> constraintsSupplier,
      Supplier<Double> startVelocitySupplier) {
    this.drive = drive;
    addRequirements(drive);
    this.waypointsSupplier = waypointsSupplier;
    this.constraintsSupplier = constraintsSupplier;
    this.startVelocitySupplier = startVelocitySupplier;
  }

  /** Creates a DriveTrajectory command with a static set of waypoints. */
  public DriveTrajectory(Drive drive, List<Waypoint> waypoints) {
    this(drive, waypoints, List.of(), 0.0);
  }

  /** Creates a DriveTrajectory command with a static set of waypoints and constraints. */
  public DriveTrajectory(
      Drive drive,
      List<Waypoint> waypoints,
      List<TrajectoryConstraint> constraints,
      double startVelocity) {
    this.drive = drive;
    addRequirements(drive);
    if (Constants.getSim()) {
      maxVelocityMetersPerSec = Units.inchesToMeters(160.0);
      maxAccelerationMetersPerSec2 = Units.inchesToMeters(105.0);
      maxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(150.0);

      xController = new PIDController(2.5, 0, 0.0);
      yController = new PIDController(2.5, 0, 0.0);
      thetaController = new PIDController(6.0, 0, 0.0);
    } else {
      maxVelocityMetersPerSec = Units.inchesToMeters(160.0);
      maxAccelerationMetersPerSec2 = Units.inchesToMeters(105.0);
      maxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(150.0);

      xController = new PIDController(2.5, 0, 0.0);
      yController = new PIDController(2.5, 0, 0.0);
      thetaController = new PIDController(5.0, 0, 0.0);
    }
    customHolonomicDriveController.setTolerance(
        new Pose2d(new Translation2d(0.02, 0.02), new Rotation2d(0.02)));
    generate(waypoints, constraints, startVelocity, true);
  }

  /** Generates the trajectory. */
  private void generate(
      List<Waypoint> waypoints,
      List<TrajectoryConstraint> constraints,
      double startVelocity,
      boolean alertOnFail) {
    // Set up trajectory configuration
    TrajectoryConfig config =
        new TrajectoryConfig(maxVelocityMetersPerSec, maxAccelerationMetersPerSec2)
            .setKinematics(new SwerveDriveKinematics(drive.getModuleTranslations()))
            .setStartVelocity(startVelocity)
            .setEndVelocity(0.0)
            .addConstraint(
                new CentripetalAccelerationConstraint(maxCentripetalAccelerationMetersPerSec2))
            .addConstraints(constraints);

    // Generate trajectory
    customGenerator = new CustomTrajectoryGenerator(); // Reset generator
    try {
      customGenerator.generate(config, waypoints);
    } catch (Exception exception) {
      if (supportedRobot && alertOnFail) {
        exception.printStackTrace();
      }
    }
  }

  @Override
  public void initialize() {
    // Generate trajectory if supplied
    if (waypointsSupplier != null || constraintsSupplier != null) {
      generate(
          waypointsSupplier.get(), constraintsSupplier.get(), startVelocitySupplier.get(), false);
    }

    // Log trajectory
    Logger.getInstance()
        .setDriveTrajectory(
            customGenerator.getDriveTrajectory().getStates().stream()
                .map(state -> AllianceFlipUtil.apply(state.poseMeters))
                .toArray(Pose2d[]::new));

    // Reset all controllers
    timer.reset();
    timer.start();
    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  @Override
  public void execute() {
    // Exit if trajectory generation failed
    if (customGenerator.getDriveTrajectory().getStates().size() <= 1) {
      return;
    }

    // Get setpoint
    Trajectory.State driveState =
        AllianceFlipUtil.apply(customGenerator.getDriveTrajectory().sample(timer.get()));
    RotationSequence.State holonomicRotationState =
        AllianceFlipUtil.apply(customGenerator.getHolonomicRotationSequence().sample(timer.get()));

    // Log Setpoint
    Logger.getInstance()
        .setDriveTrajSetpoint(
            new Pose2d(driveState.poseMeters.getTranslation(), holonomicRotationState.position));

    // Log Errors
    SmartDashboard.putNumberArray("Traj Errors", customHolonomicDriveController.getErrors());

    // Calculate velocity
    ChassisSpeeds nextDriveState =
        customHolonomicDriveController.calculate(
            drive.getPose(), driveState, holonomicRotationState);
    drive.runVelocity(nextDriveState);
  }

  @Override
  public void end(boolean interrupted) {
    Logger.getInstance().setDriveTrajectory(new Pose2d());
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(customGenerator.getDriveTrajectory().getTotalTimeSeconds());
  }
}
