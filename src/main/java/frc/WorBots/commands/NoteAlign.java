// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.subsystems.lights.Lights.LightsMode;
import frc.WorBots.subsystems.vision.Vision;
import frc.WorBots.util.control.DriveController;
import frc.WorBots.util.debug.Logger;
import frc.WorBots.util.debug.TunableDouble;
import frc.WorBots.util.debug.TunablePIDController.TunablePIDGains;
import frc.WorBots.util.debug.TunablePIDController.TunableProfiledPIDController;
import frc.WorBots.util.debug.TunablePIDController.TunableTrapezoidConstraints;
import frc.WorBots.util.math.GeomUtil;
import frc.WorBots.util.math.InterpolatingTable;
import java.util.Optional;
import java.util.function.*;

/** Command for automatic speaker targeting during teleop */
public class NoteAlign extends Command {
  // Constants
  /** Speed reduction for the driver while in this mode. Higher values reduce their speed more. */
  private static final double DRIVE_SPEED_REDUCTION = 1.5;

  /** Tolerance for the theta controller */
  private static final double THETA_TOLERANCE = Units.degreesToRadians(0.65);

  private final TunableDouble lookaheadFactor;

  private static boolean ENABLE_3D_TRACKING = true;

  /** Lookup table to guess the distance of the note */
  private static final InterpolatingTable DISTANCE_TABLE =
      new InterpolatingTable(
          new double[][] {
            {2.6, Units.inchesToMeters(48)},
            {5.4, Units.inchesToMeters(66)},
            {7.1, Units.inchesToMeters(72)},
            {12.5, Units.inchesToMeters(84)},
          });

  private Optional<Translation2d> noteLocation;

  private final Drive drive;
  private final Vision vision;

  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;

  private boolean hasTargeted = false;

  private final TunableProfiledPIDController thetaController =
      new TunableProfiledPIDController(
          new TunablePIDGains("Vision", "Note Align Gains"),
          new TunableTrapezoidConstraints("Vision", "Note Align Constraints"));

  private final TunableProfiledPIDController thetaController2 =
      new TunableProfiledPIDController(
          new TunablePIDGains("Vision", "Note Align Gains 2"),
          new TunableTrapezoidConstraints("Vision", "Note Align Constraints 2"));

  private final DriveController driveController = new DriveController();

  public NoteAlign(
      Drive drive, Vision vision, Supplier<Double> leftXSupplier, Supplier<Double> leftYSupplier) {
    this.drive = drive;
    this.vision = vision;
    addRequirements(drive);
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    thetaController.setGains(4.0, 0.00, 0.0);
    thetaController.setConstraints(Units.degreesToRadians(150.0), Units.degreesToRadians(700.0));
    thetaController2.setGains(2.0, 0.00, 0.0);
    thetaController2.setConstraints(Units.degreesToRadians(150.0), Units.degreesToRadians(700.0));
    lookaheadFactor = new TunableDouble("Vision", "Tuning", "Note Align Lookahead", 40.0);

    thetaController2.pid.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.pid.setTolerance(THETA_TOLERANCE);
  }

  @Override
  public void initialize() {
    Lights.getInstance().setSolid(Color.kOrangeRed);
    this.hasTargeted = false;
    noteLocation = Optional.empty();
    // Since the whole tracking system is relative to the note, we actually don't want vision
    // or the lack of it messing up our pose
    drive.enableVisionUpdates(false);
  }

  @Override
  public void execute() {
    thetaController.update();
    thetaController2.update();

    // Run driving
    final double x = leftXSupplier.get();
    final double y = leftYSupplier.get();

    final ChassisSpeeds driveSpeeds =
        driveController.getSpeeds(
            x,
            y,
            0.0,
            drive.getYaw(),
            drive.getMaxLinearSpeedMetersPerSec() / DRIVE_SPEED_REDUCTION);

    // Calculate turn
    if (!hasTargeted && vision.hasNoteTarget()) {
      final double thetaVelocity = thetaController.pid.calculate(-vision.getNoteTheta(), 0.0);

      // Stop turning once we get within tolerance
      if (!thetaController.pid.atGoal()) {
        // Don't turn there if we already have a lock
        if (!ENABLE_3D_TRACKING || (ENABLE_3D_TRACKING && noteLocation.isEmpty())) {
          driveSpeeds.omegaRadiansPerSecond = thetaVelocity;
        }
      } else {
        // Once we are within tolerance, we can try to guesstimate the note position
        if (ENABLE_3D_TRACKING && noteLocation.isEmpty()) {
          final double distance = DISTANCE_TABLE.get(vision.getNoteRatio());
          if (distance >= 0 && distance <= 4.0) {
            final Transform2d transform = new Transform2d(distance, 0.0, new Rotation2d());
            final Translation2d translated = drive.getPose().plus(transform).getTranslation();
            noteLocation = Optional.of(translated);
            Lights.getInstance().setSolid(Color.kGreen);
            thetaController2.pid.reset(drive.getRotation().getRadians());
          }
        }
      }
    }
    // Fill in using the note position
    if (ENABLE_3D_TRACKING && noteLocation.isPresent()) {
      Pose2d robot = drive.getPose();
      // Apply lookahead
      robot =
          GeomUtil.applyChassisSpeeds(
              robot,
              drive.getFieldRelativeSpeeds(),
              Constants.ROBOT_PERIOD * lookaheadFactor.get());

      // Negative so that we aim with our intake instead of our shooter
      final double desiredYaw =
          -Math.atan2(
                  noteLocation.get().getX() - robot.getX(),
                  noteLocation.get().getY() - robot.getY())
              + Units.degreesToRadians(90);
      SmartDashboard.putNumberArray(
          "thing",
          Logger.pose2dToArray(new Pose2d(robot.getX(), robot.getY(), new Rotation2d(desiredYaw))));
      final double thetaVelocity =
          thetaController2.pid.calculate(drive.getRotation().getRadians(), desiredYaw);
      driveSpeeds.omegaRadiansPerSecond = thetaVelocity;
    }

    if (noteLocation.isPresent()) {
      SmartDashboard.putNumberArray(
          "Note Position",
          Logger.translation3dToArray(GeomUtil.translation2dTo3d(noteLocation.get())));
    }

    driveController.drive(drive, driveSpeeds);

    // Output info
    SmartDashboard.putNumber(
        "Note Align Theta Controller Error", thetaController.pid.getPositionError());

    SmartDashboard.putNumber(
        "Note Align Theta Controller Error 2", thetaController2.pid.getPositionError());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Lights.getInstance().setMode(LightsMode.Delivery);
    drive.enableVisionUpdates(true);
  }
}
