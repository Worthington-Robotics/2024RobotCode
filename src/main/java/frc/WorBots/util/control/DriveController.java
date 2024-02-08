// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.util.math.GeneralMath;

/** Controller for teleoperated driving */
public class DriveController {
  // Constants
  /** The percentage of the max drive speed that the robot will drive at */
  public static final double driveSpeedMultiplier = 0.75;

  /** The max rotational speed in radians per update that the robot will drive at */
  public static final double rotationalSpeed = 10.0;

  /** The amount of input deadband to apply */
  public static final double deadband = 0.065;

  /** The minimum speed output that will make the drive stop */
  public static final double minimumSpeed = 1e-3;

  /** The amount of curving to apply to the drive inputs */
  public static final double driveCurveAmount = 2.0;

  /** The amount of curving to apply to the turn input */
  public static final double turnCurveAmount = 2.0;

  /** Construct a new DriveController */
  public DriveController() {}

  /**
   * Drive the drivetrain using controller inputs
   *
   * @param drive The drive subsystem to drive
   * @param x The desired x velocity, from -1 to 1
   * @param y The desired y velocity, from -1 to 1
   * @param theta The desired rotational velocity, from -1 to 1
   */
  public void drive(Drive drive, double x, double y, double theta) {
    final ChassisSpeeds speeds =
        getSpeeds(x, y, theta, drive.getRotation(), drive.getMaxLinearSpeedMetersPerSec());

    // Send to drive
    if (Math.abs(speeds.vxMetersPerSecond) < minimumSpeed
        && Math.abs(speeds.vyMetersPerSecond) < minimumSpeed
        && Math.abs(speeds.omegaRadiansPerSecond) < minimumSpeed) {
      drive.stop();
    } else {
      drive.runVelocity(speeds);
    }
  }

  /**
   * Get the base chassis speeds for driving
   *
   * @param x The desired x velocity, from -1 to 1
   * @param y The desired y velocity, from -1 to 1
   * @param theta The desired rotational velocity, from -1 to 1
   * @param robotRotation The current absolute rotation of the robot
   * @param maxSpeed The maximum speed in m/s that the robot can drive at
   * @return The calculated drive speeds, relative to the field
   */
  private static ChassisSpeeds getSpeeds(
      double x, double y, double theta, Rotation2d robotRotation, double maxSpeed) {
    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(x, y);
    Rotation2d linearDirection = new Rotation2d(x, y);

    // Apply deadband
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, deadband);
    theta = MathUtil.applyDeadband(theta, deadband);

    // Apply squaring
    linearMagnitude = GeneralMath.curve(linearMagnitude, driveCurveAmount);
    theta = GeneralMath.curve(theta, turnCurveAmount);

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
            linearVelocity.getX() * maxSpeed * driveSpeedMultiplier,
            linearVelocity.getY() * maxSpeed * driveSpeedMultiplier,
            theta * rotationalSpeed);

    // Convert to field relative based on the alliance
    var driveRotation = robotRotation;
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
    return speeds;
  }
}
