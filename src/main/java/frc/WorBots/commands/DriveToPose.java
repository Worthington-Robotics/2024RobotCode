// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.util.debug.Logger;
import frc.WorBots.util.math.GeomUtil;
import java.util.function.Supplier;

public class DriveToPose extends Command {
  private final Drive drive;
  private final boolean slowMode;
  private final Supplier<Pose2d> poseSupplier;

  private boolean running = false;
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(2.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private Translation2d lastSetpointTranslation;
  private double ffMinRadius = 0.02;
  private double ffMaxRadius = 0.06;

  /** Drives to the specified pose under full software control. */
  public DriveToPose(Drive drive, Pose2d pose) {
    this(drive, false, pose);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(Drive drive, boolean slowMode, Pose2d pose) {
    this(drive, slowMode, () -> pose);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(Drive drive, Supplier<Pose2d> poseSupplier) {
    this(drive, false, poseSupplier);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(Drive drive, boolean slowMode, Supplier<Pose2d> poseSupplier) {
    this.drive = drive;
    this.slowMode = slowMode;
    this.poseSupplier = poseSupplier;
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    driveController.setP(2.0);
    driveController.setD(0.0);
    driveController.setConstraints(
        new Constraints(
            slowMode ? Units.inchesToMeters(50.0) : Units.inchesToMeters(150.0),
            Units.inchesToMeters(90.0)));
    driveController.setTolerance(slowMode ? 0.06 : 0.01);
    thetaController.setP(5.0);
    thetaController.setD(0.0);
    thetaController.setConstraints(
        new Constraints(
            slowMode ? Units.degreesToRadians(50.0) : Units.degreesToRadians(360.0),
            Units.degreesToRadians(720.0)));
    thetaController.setTolerance(
        slowMode ? Units.degreesToRadians(3.0) : Units.degreesToRadians(1.0));
    // Reset all controllers
    var currentPose = drive.getPose();
    driveController.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(drive.getFieldVelocity().dx, drive.getFieldVelocity().dy)
                .rotateBy(
                    poseSupplier
                        .get()
                        .getTranslation()
                        .minus(drive.getPose().getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(currentPose.getRotation().getRadians(), drive.getYawVelocity());
    lastSetpointTranslation = drive.getPose().getTranslation();
  }

  @Override
  public void execute() {
    running = true;

    // Get current and target pose
    var currentPose = drive.getPose();
    var targetPose = poseSupplier.get();

    // Calculate drive speed
    double currentDistance =
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    // Command speeds
    var driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
            .getTranslation();
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    // Log data
    SmartDashboard.putNumber("DriveToPose/CurrentDistance", currentDistance);
    SmartDashboard.putNumberArray(
        "DriveToPose/DriveToPoseSetpoint",
        Logger.pose2dToArray(
            new Pose2d(
                lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position))));
    SmartDashboard.putNumberArray(
        "DriveToPose/DistanceSetopoint", Logger.pose2dToArray(targetPose));
    // Logger.getInstance().recordOutput("DriveToPose/DistanceMeasured",
    // currentDistance);
    // Logger.getInstance()
    // .recordOutput("DriveToPose/DistanceSetpoint",
    // driveController.getSetpoint().position);
    // Logger.getInstance()
    // .recordOutput("DriveToPose/ThetaMeasured",
    // currentPose.getRotation().getRadians());
    // Logger.getInstance()
    // .recordOutput("DriveToPose/ThetaSetpoint",
    // thetaController.getSetpoint().position);
    Logger.getInstance()
        .setDriveTrajSetpoint(
            new Pose2d(
                lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
    // Logger.getInstance().recordOutput("Odometry/DriveToPoseGoal", targetPose);
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }

  /** Returns whether the command is actively running. */
  public boolean isRunning() {
    return running;
  }
}
