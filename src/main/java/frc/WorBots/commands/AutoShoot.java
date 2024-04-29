// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.subsystems.lights.Lights.LightsMode;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.Preset;
import frc.WorBots.util.control.DriveController;
import frc.WorBots.util.math.ShooterMath;
import frc.WorBots.util.math.ShooterMath.ShotData;
import java.util.function.*;

/** Command for automatic speaker targeting during teleop */
public class AutoShoot extends Command {
  // Constants
  /** Speed reduction for the driver while in this mode. Higher values reduce their speed more. */
  private static final double DRIVE_SPEED_REDUCTION = 3.0;

  /** Tolerance for the theta controller */
  private static final double THETA_TOLERANCE = Units.degreesToRadians(1.3);

  private final Drive drive;
  private final Superstructure superstructure;
  private final Shooter shooter;

  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          6.5,
          0.001,
          0.032,
          new TrapezoidProfile.Constraints(
              Units.degreesToRadians(300.0), Units.degreesToRadians(4000.0)),
          Constants.ROBOT_PERIOD);

  private final DriveController driveController = new DriveController();

  /**
   * This command automatically drives to a known safe shooting location and shoots a game piece.
   *
   * @param superstructure The superstructure subsystem.
   * @param drive The drive subsystem.
   */
  public AutoShoot(
      Superstructure superstructure,
      Drive drive,
      Shooter shooter,
      Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.shooter = shooter;
    addRequirements(superstructure, drive, shooter);
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);
  }

  @Override
  public void initialize() {
    thetaController.reset(drive.getPose().getRotation().getRadians());
    superstructure.setModeVoid(SuperstructureState.SHOOTING);
    Lights.getInstance().setMode(LightsMode.Shooting);
  }

  @Override
  public void execute() {
    final Pose2d robotPose = drive.getPose();

    // Run shot
    final ShotData shot = ShooterMath.calculateShotData(robotPose, drive.getFieldRelativeSpeeds());
    shooter.setSpeedVoid(shot.rpm());
    superstructure.setShootingAngleRad(shot.pivotAngle());

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
    final double setpointAngle = shot.robotAngle().getRadians();
    final double thetaVelocity =
        thetaController.calculate(robotPose.getRotation().getRadians(), setpointAngle);

    driveSpeeds.omegaRadiansPerSecond = thetaVelocity;

    driveController.drive(drive, driveSpeeds);

    // Output info
    SmartDashboard.putNumber("Goal Range", ShooterMath.getGoalDistance(drive.getPose()));
    SmartDashboard.putNumber(
        "Autoshoot Theta Controller Error", thetaController.getPositionError());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.setPose(Preset.STOW);
    Lights.getInstance().setMode(LightsMode.Delivery);
    shooter.idle();
  }
}
