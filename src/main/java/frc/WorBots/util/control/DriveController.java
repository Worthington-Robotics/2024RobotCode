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
import frc.WorBots.util.debug.TunableDouble;
import frc.WorBots.util.math.GeneralMath;

/** Controller for teleoperated driving */
public class DriveController {
  // Constants
  /** The percentage of the max drive speed that the robot will drive at */
  public static final double DRIVE_SPEED_MULTIPLIER = 0.70;

  /** The max rotational speed in radians per update that the robot will drive at */
  public static final double ROTATIONAL_SPEED = 7.5;

  /** The amount of input deadband to apply */
  public static final double DEADBAND = 0.1;

  /** The minimum speed output that will make the drive stop */
  public static final double MINIMUM_SPEED = 1e-3;

  /** The amount of curving to apply to the drive inputs */
  public static final double DRIVE_CURVE_AMOUNT = 2.0;

  /** The amount of curving to apply to the turn input */
  public static final double TURN_CURVE_AMOUNT = 2.0;

  /** The amount of time in seconds after which to force brake */
  public static final TunableDouble BRAKE_DELAY =
      new TunableDouble("Tuning", "Drive", "Brake Delay", 0.3);

  private static final LinearFilter driveFilter = LinearFilter.movingAverage(13);
  private static final LinearFilter turnFilter = LinearFilter.movingAverage(3);
  private static final LinearFilter maxSpeedFilter = LinearFilter.movingAverage(24);

  private Timer stopTimer = new Timer();

  /**
   * Last demanded vector of the drive, used to have smooth braking while moving in the same
   * direction
   */
  private Translation2d lastVector = new Translation2d();

  /** Construct a new DriveController */
  public DriveController() {
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
    if (Math.abs(speeds.vxMetersPerSecond) < MINIMUM_SPEED
        && Math.abs(speeds.vyMetersPerSecond) < MINIMUM_SPEED
        && Math.abs(speeds.omegaRadiansPerSecond) < MINIMUM_SPEED) {
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
   * @param robotRotation The current rotation of the robot relative to the driver station wall
   * @param maxSpeed The maximum speed in m/s that the robot can drive at
   * @return The calculated drive speeds, relative to the field
   */
  public ChassisSpeeds getSpeeds(
      double x, double y, double theta, Rotation2d robotRotation, double maxSpeed) {
    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(x, y);
    Rotation2d linearDirection = new Rotation2d(x, y);
    SmartDashboard.putNumber("Drive Magnitude", linearMagnitude);

    // Apply deadband
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, DEADBAND);
    // Stopping
    if (linearMagnitude == 0.0) {
      if (stopTimer.hasElapsed(BRAKE_DELAY.get())) {
        driveFilter.reset();
      } else {
        // Smooth stop from our last vector
        // linearDirection = lastVector.getAngle();
        // linearMagnitude = (1.0 - stopTimer.get() / BRAKE_DELAY.getCurrent()) *
        // lastVector.getNorm();
        // SmartDashboard.putNumber("Linear Magnitude", linearMagnitude);
      }
    } else {
      stopTimer.reset();
    }
    theta = MathUtil.applyDeadband(theta, DEADBAND);
    if (theta == 0.0) {
      turnFilter.reset();
    }

    // Apply squaring
    linearMagnitude = GeneralMath.curve(linearMagnitude, DRIVE_CURVE_AMOUNT);
    theta = GeneralMath.curve(theta, TURN_CURVE_AMOUNT);

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

    lastVector = linearVelocity;

    // Filter max speed to prevent large speed changes
    final double maximumSpeed = maxSpeedFilter.calculate(maxSpeed * DRIVE_SPEED_MULTIPLIER);
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * maximumSpeed,
            linearVelocity.getY() * maximumSpeed,
            theta * ROTATIONAL_SPEED);

    // Convert to field relative based on the alliance
    final var driveRotation = robotRotation;
    speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            driveRotation);
    return speeds;
  }

  public double driveSingleAxis(double axis, double maxSpeed) {
    final double maximumSpeed = maxSpeedFilter.calculate(maxSpeed * DRIVE_SPEED_MULTIPLIER);
    return driveFilter.calculate(axis) * maximumSpeed;
  }
}
