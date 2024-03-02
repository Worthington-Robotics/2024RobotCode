// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.util.math.GeneralMath;

/** Controller for teleoperated driving */
public class DriveController {
  // Constants
  /** The percentage of the max drive speed that the robot will drive at */
  public static final double driveSpeedMultiplier = 0.66;

  /** The max rotational speed in radians per update that the robot will drive at */
  public static final double rotationalSpeed = 5.0;

  /** The amount of input deadband to apply */
  public static final double deadband = 0.1;

  /** The minimum speed output that will make the drive stop */
  public static final double minimumSpeed = 1e-3;

  /** The amount of curving to apply to the drive inputs */
  public static final double driveCurveAmount = 2.0;

  /** The amount of curving to apply to the turn input */
  public static final double turnCurveAmount = 2.0;

  /** The amount of time in seconds after which to force brake */
  public static final double brakeDelay = 0.2;

  /** The amount of time in seconds after which to apply stop locking */
  public static final double stopLockDelay = 0.5;

  private static final LinearFilter driveFilter = LinearFilter.movingAverage(10);
  private static final LinearFilter turnFilter = LinearFilter.movingAverage(1);
  private static final LinearFilter maxSpeedFilter = LinearFilter.movingAverage(1);

  private Timer stopLockTimer = new Timer();
  private Timer stopTimer = new Timer();

  /** Construct a new DriveController */
  public DriveController() {
    stopLockTimer.restart();
    stopTimer.restart();
  }

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
        getSpeeds(x, y, theta, drive.getYaw(), drive.getMaxLinearSpeedMetersPerSec());

    drive(drive, speeds);
  }

  /**
   * Drive the drivetrain using existing chassisspeeds. This is useful if you run getSpeeds and want
   * to modify them
   *
   * @param drive The drive subsystem to drive
   * @param speeds The desired chassis speeds
   */
  public void drive(Drive drive, ChassisSpeeds speeds) {
    // Send to drive
    if (Math.abs(speeds.vxMetersPerSecond) < minimumSpeed
        && Math.abs(speeds.vyMetersPerSecond) < minimumSpeed
        && Math.abs(speeds.omegaRadiansPerSecond) < minimumSpeed) {
      drive.stop();
    } else {
      stopLockTimer.restart();
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
  public ChassisSpeeds getSpeeds(
      double x, double y, double theta, Rotation2d robotRotation, double maxSpeed) {
    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(x, y);
    SmartDashboard.putNumber("Drive Magnitude", linearMagnitude);
    Rotation2d linearDirection = new Rotation2d(x, y);

    // Apply deadband
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, deadband);
    if (linearMagnitude == 0.0) {
      driveFilter.reset();
    } else {
      stopTimer.reset();
    }
    theta = MathUtil.applyDeadband(theta, deadband);
    if (theta == 0.0) {
      turnFilter.reset();
    }

    // Apply squaring
    linearMagnitude = GeneralMath.curve(linearMagnitude, driveCurveAmount);
    theta = GeneralMath.curve(theta, turnCurveAmount);

    // Apply filtering
    linearMagnitude = driveFilter.calculate(linearMagnitude);
    theta = turnFilter.calculate(theta);

    // Calcaulate new linear components
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(
                new Transform2d(
                    new Translation2d(linearMagnitude, new Rotation2d()), new Rotation2d()))
            .getTranslation();

    // Convert to meters per second

    // Filter max speed to prevent large speed changes
    final double maximumSpeed = maxSpeedFilter.calculate(maxSpeed * driveSpeedMultiplier);
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * maximumSpeed,
            linearVelocity.getY() * maximumSpeed,
            theta * rotationalSpeed);

    // Convert to field relative based on the alliance
    var driveRotation = robotRotation;
    // final var alliance = AllianceCache.getInstance().get();
    // if (alliance.isPresent() && alliance.get() == Alliance.Red) {
    //   driveRotation = driveRotation.plus(new Rotation2d(Math.PI));
    // }
    speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            driveRotation);
    return speeds;
  }

  public double driveSingleAxis(double axis, double maxSpeed) {
    final double maximumSpeed = maxSpeedFilter.calculate(maxSpeed * driveSpeedMultiplier);
    return driveFilter.calculate(axis) * maximumSpeed;
  }
}
