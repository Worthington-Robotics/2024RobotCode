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
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.WorBots.util.Cache;
import frc.WorBots.util.control.DriveController;
import frc.WorBots.util.math.ShooterMath;
import frc.WorBots.util.math.ShooterMath.ShotData;
import java.util.function.*;

public class AutoShoot extends SequentialCommandGroup {
  // Constants
  private static final double DRIVE_SPEED_REDUCTION = 3.0;
  private static final double THETA_TOLERANCE = 1.3;
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          3.9,
          0.001,
          0.032,
          new TrapezoidProfile.Constraints(
              Units.degreesToRadians(150.0), Units.degreesToRadians(700.0)),
          Constants.ROBOT_PERIOD);
  private DriveController driveController = new DriveController();

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
    addRequirements(superstructure, drive, shooter);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(THETA_TOLERANCE));

    final Supplier<ShotData> supplier =
        () -> ShooterMath.calculateShotData(drive.getPose(), drive.getFieldRelativeSpeeds());
    final Cache<ShotData> shotSupplier = new Cache<ShotData>(supplier);
    final Supplier<ChassisSpeeds> speedsSupplier =
        () -> {
          SmartDashboard.putNumber(
              "Autoshoot Theta Controller Error", thetaController.getPositionError());
          final Pose2d robotPose = drive.getPose();
          final double x = leftXSupplier.get();
          final double y = leftYSupplier.get();

          ChassisSpeeds speeds =
              driveController.getSpeeds(
                  x,
                  y,
                  0.0,
                  drive.getRotation(),
                  drive.getMaxLinearSpeedMetersPerSec() / DRIVE_SPEED_REDUCTION);

          // Calculate turn
          final double setpointAngle = shotSupplier.get().robotAngle().getRadians();
          double thetaVelocity =
              thetaController.calculate(robotPose.getRotation().getRadians(), setpointAngle);
          // double thetaErrorAbs =
          // Math.abs(robotPose.getRotation().minus(driveAngleSupplier.get()).getRadians());
          // if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity =
          // 0.0;

          speeds.omegaRadiansPerSecond = thetaVelocity;

          return speeds;
        };
    addCommands(
        superstructure.setMode(SuperstructureState.SHOOTING),
        Commands.run(
            () -> {
              shotSupplier.update();
              shooter.setSpeedVoid(shotSupplier.get().rpm());
              driveController.drive(drive, speedsSupplier.get());
              superstructure.setShootingAngleRad(shotSupplier.get().pivotAngle());
              SmartDashboard.putNumber("Goal Range", ShooterMath.getGoalDistance(drive.getPose()));
            },
            shooter,
            drive),
        Commands.waitUntil(() -> false)
            .finallyDo(
                () -> {
                  superstructure.setModeVoid(SuperstructureState.DISABLED);
                }));
  }
}
