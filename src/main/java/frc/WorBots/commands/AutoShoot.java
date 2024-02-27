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
import frc.WorBots.util.control.DriveController;
import frc.WorBots.util.math.ShooterMath;
import java.util.function.*;

public class AutoShoot extends SequentialCommandGroup {
  // Locations
  private Supplier<Double> leftXSupplier;
  private Supplier<Double> leftYSupplier;
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
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(1.3));

    Supplier<Double> pivotAngle =
        () -> {
          Pose2d robotPose = drive.getPose();
          var shotData = ShooterMath.calculateShotData(robotPose, drive.getFieldRelativeSpeeds());
          SmartDashboard.putNumber("Shot Angle", shotData.pivotAngle());
          return shotData.pivotAngle();
        };
    Supplier<Rotation2d> driveAngleSupplier =
        () -> {
          Pose2d robotPose = drive.getPose();
          double robotAngle =
              ShooterMath.calculateRobotAngle(robotPose, drive.getFieldRelativeSpeeds())
                  .getRadians();
          return new Rotation2d(robotAngle);
        };
    Supplier<Double> shooterSpeedSupplier =
        () -> {
          Pose2d robotPose = drive.getPose();
          return ShooterMath.calculateShooterRPM(robotPose);
        };
    Supplier<ChassisSpeeds> speedsSupplier =
        () -> {
          SmartDashboard.putNumber("Controller Error", thetaController.getPositionError());
          Pose2d robotPose = drive.getPose();
          double x = leftXSupplier.get();
          double y = leftYSupplier.get();

          ChassisSpeeds speeds =
              driveController.getSpeeds(
                  x, y, 0.0, drive.getRotation(), drive.getMaxLinearSpeedMetersPerSec() / 3.0);

          // Calculate turn
          double thetaVelocity =
              thetaController.calculate(
                  robotPose.getRotation().getRadians(), driveAngleSupplier.get().getRadians());
          // double thetaErrorAbs =
          //     Math.abs(robotPose.getRotation().minus(driveAngleSupplier.get()).getRadians());
          // if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

          speeds.omegaRadiansPerSecond = thetaVelocity;

          return speeds;
        };
    var shooting =
        Commands.run(
            () -> {
              superstructure.setShootingAngleRad(pivotAngle);
            },
            superstructure);
    addCommands(
        superstructure.setMode(SuperstructureState.SHOOTING),
        shooting
            .alongWith(
                Commands.run(() -> shooter.spinToSpeedVoid(shooterSpeedSupplier.get()), shooter))
            .alongWith(
                Commands.run(() -> driveController.drive(drive, speedsSupplier.get()), drive))
            .alongWith(
                Commands.run(
                    () ->
                        SmartDashboard.putNumber(
                            "Goal Range", ShooterMath.getGoalDistance(drive.getPose())))),
        Commands.waitUntil(() -> false)
            .finallyDo(
                () -> {
                  superstructure.setModeVoid(SuperstructureState.DISABLED);
                }));
  }
}
