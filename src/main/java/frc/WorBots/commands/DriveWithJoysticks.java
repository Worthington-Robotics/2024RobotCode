// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.drive.Drive;
import java.util.function.Supplier;

/** Command for teleop that drives the robot using controllers */
public class DriveWithJoysticks extends Command {
  private Drive drive;
  private Supplier<Double> leftXSupplier;
  private Supplier<Double> leftYSupplier;
  private Supplier<Double> rightXSupplier;
  private Supplier<Double> rightYSupplier;

  public DriveWithJoysticks(
      Drive drive,
      Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier,
      Supplier<Double> rightXSupplier,
      Supplier<Double> rightYSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightXSupplier = rightXSupplier;
    this.rightYSupplier = rightYSupplier;
  }

  @Override
  public void execute() {
    // Get values from double suppliers
    double leftX = leftXSupplier.get();
    double leftY = leftYSupplier.get();
    double rightX = rightXSupplier.get();
    double rightY = rightYSupplier.get();

    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(leftX, leftY);
    Rotation2d linearDirection = new Rotation2d(leftX, leftY);

    // Apply deadband
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, 0.065);
    rightY = MathUtil.applyDeadband(rightY, 0.065);

    // Apply squaring
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
    rightY = Math.copySign(rightY * rightY, rightY);

    // Calcaulate new linear components
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(
                new Transform2d(
                    new Translation2d(linearMagnitude, new Rotation2d()), new Rotation2d()))
            .getTranslation();

    // Convert to meters per second
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            rightY * 10.0);

    // Convert from field relative
    var driveRotation = drive.getRotation();
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      driveRotation = driveRotation.plus(new Rotation2d(Math.PI));
    }
    speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            driveRotation);

    // Send to drive
    // var driveTranslation = FlipFieldUtil.apply(drive.getPose().getTranslation());
    if (Math.abs(speeds.vxMetersPerSecond) < 1e-3
        && Math.abs(speeds.vyMetersPerSecond) < 1e-3
        && Math.abs(speeds.omegaRadiansPerSecond) < 1e-3) {
      drive.stop();
    } else {
      drive.runVelocity(speeds);
    }
  }
}
