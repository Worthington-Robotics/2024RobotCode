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
import frc.WorBots.Constants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.util.debug.Logger;
import frc.WorBots.util.math.GeomUtil;
import java.util.function.Supplier;

/** Command that drives to a pose on the field. Credit to team 6328. */
public class DriveToPose extends Command {
  private final Drive drive;
  private final boolean slowMode;
  private boolean enableDriving;
  private boolean checkTheta = true;
  private final Supplier<Pose2d> poseSupplier;

  private boolean running = false;
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          2.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.ROBOT_PERIOD);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          6.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.ROBOT_PERIOD);
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private Translation2d lastSetpointTranslation;
  private double ffMinRadius = 0.02;
  private double ffMaxRadius = 0.06;
  private double ffScaleFactor = 1.0;

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
    this.enableDriving = true;
    this.poseSupplier = poseSupplier;
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public static DriveToPose withoutDriving(Drive drive, Supplier<Pose2d> poseSupplier) {
    var out = new DriveToPose(drive, poseSupplier);
    out.enableDriving = false;
    return out;
  }

  public static DriveToPose ignoreTurn(Drive drive, Supplier<Pose2d> poseSupplier) {
    var out = new DriveToPose(drive, poseSupplier);
    out.checkTheta = false;
    return out;
  }

  @Override
  public void initialize() {
    driveController.setP(2.4);
    driveController.setD(0.01);
    driveController.setConstraints(
        new Constraints(
            slowMode ? Units.inchesToMeters(50.0) : Units.inchesToMeters(140.0),
            Units.inchesToMeters(90.0)));
    driveController.setTolerance(slowMode ? Units.inchesToMeters(2) : Units.inchesToMeters(3.0));
    thetaController.setP(4.2);
    thetaController.setD(0.05);
    thetaController.setConstraints(
        new Constraints(
            slowMode ? Units.degreesToRadians(50.0) : Units.degreesToRadians(170.0),
            Units.degreesToRadians(820.0)));
    thetaController.setTolerance(
        slowMode ? Units.degreesToRadians(2.0) : Units.degreesToRadians(1.0));
    // Reset all controllers
    final Pose2d currentPose = drive.getPose();
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
    SmartDashboard.putBoolean("DriveToPose/Enable Driving", enableDriving);
  }

  @Override
  public void execute() {
    running = true;

    // Get current and target pose
    final Pose2d currentPose = drive.getPose();
    Pose2d targetPose = poseSupplier.get();
    if (!enableDriving) {
      targetPose = new Pose2d(currentPose.getX(), currentPose.getY(), targetPose.getRotation());
    }

    // Calculate drive speed
    final double currentDistance =
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    final double ffScalar =
        MathUtil.clamp(
            (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius) * ffScaleFactor,
            0.0,
            1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScalar
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
        thetaController.getSetpoint().velocity * ffScalar
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    // Command speeds
    final Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
            .getTranslation();
    final double x = enableDriving ? driveVelocity.getX() : 0.0;
    final double y = enableDriving ? driveVelocity.getY() : 0.0;
    final ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(x, y, thetaVelocity, currentPose.getRotation());
    drive.runVelocity(speeds);

    // Log data
    SmartDashboard.putNumber("DriveToPose/CurrentDistance", currentDistance);
    SmartDashboard.putNumberArray(
        "DriveToPose/DriveToPoseSetpoint",
        Logger.pose2dToArray(
            new Pose2d(
                lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position))));
    SmartDashboard.putNumberArray("DriveToPose/DistanceSetpoint", Logger.pose2dToArray(targetPose));
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
    boolean out = running;
    if (enableDriving) {
      out &= driveController.atGoal();
    }
    if (checkTheta) {
      out &= thetaController.atGoal();
    }
    out |= withinTolerance(Units.inchesToMeters(3.0), Rotation2d.fromDegrees(1.0));
    return out;
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
