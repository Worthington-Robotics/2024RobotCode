// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.subsystems.drive.GyroIO.GyroIOInputs;
import frc.WorBots.util.*;
import frc.WorBots.util.PoseEstimator.*;
import java.util.List;

public class Drive extends SubsystemBase {
  private final Module[] modules = new Module[4];
  private ChassisSpeeds setpoint = new ChassisSpeeds();

  private final GyroIO gyroIO;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();
  private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};
  private SwerveModuleState[] lastSetpointStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };
  private Rotation2d lastGyroYaw = new Rotation2d();
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private PoseEstimator poseEstimator = new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));
  private NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private Twist2d fieldVelocity = new Twist2d();
  private Translation2d centerOfRotation = new Translation2d();

  private NetworkTable driveTable = instance.getTable("Drive");
  private DoubleArrayPublisher speedSetpointPublisher =
      driveTable.getDoubleArrayTopic("Speed Setpoint").publish();
  private DoubleArrayPublisher setpointPublisher =
      driveTable.getDoubleArrayTopic("Setpoint").publish();
  private DoubleArrayPublisher optimizedPublisher =
      driveTable.getDoubleArrayTopic("Optimized").publish();
  private DoubleArrayPublisher measuredPublisher =
      driveTable.getDoubleArrayTopic("Measured").publish();
  private DoubleArrayPublisher posePublisher =
      driveTable.getDoubleArrayTopic("Pose Estimator").publish();

  /**
   * The main swerve drive subsystem.
   *
   * @param gyroIO The IO to use for the gyro.
   * @param flModule The IO to use for the front left module.
   * @param frModule The IO to use for the front right module.
   * @param blModule The IO to use for the back left module.
   * @param brModule The IO to use for the back right module.
   */
  public Drive(
      GyroIO gyroIO, ModuleIO flModule, ModuleIO frModule, ModuleIO blModule, ModuleIO brModule) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModule, 0);
    modules[1] = new Module(frModule, 1);
    modules[2] = new Module(blModule, 2);
    modules[3] = new Module(brModule, 3);
    StatusPage.reportStatus(StatusPage.DRIVE_SUBSYSTEM, true);
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    StatusPage.reportStatus(StatusPage.GYROSCOPE, gyroInputs.connected);

    for (Module module : modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      for (Module module : modules) {
        module.stop();
      }
    } else {
      var setpointTwist =
          new Pose2d()
              .log(
                  new Pose2d(
                      new Translation2d(
                          setpoint.vxMetersPerSecond * 0.02, setpoint.vyMetersPerSecond * 0.02),
                      new Rotation2d(setpoint.omegaRadiansPerSecond * 0.02)));
      var adjustedSpeeds =
          new ChassisSpeeds(
              setpointTwist.dx / 0.02, setpointTwist.dy / 0.02, setpointTwist.dtheta / 0.02);
      speedSetpointPublisher.set(Logger.chassisSpeedsToArray(adjustedSpeeds));

      SwerveModuleState[] setpointStates =
          kinematics.toSwerveModuleStates(adjustedSpeeds, centerOfRotation);
      // Desaturate speeds to ensure we don't go faster than is possible
      SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, getMaxLinearSpeedMetersPerSec());

      lastSetpointStates = setpointStates;

      setpointPublisher.set(Logger.statesToArray(setpointStates));

      SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        optimizedStates[i] = modules[i].optimizeState(setpointStates[i]);
      }

      // Desaturate the states again
      SwerveDriveKinematics.desaturateWheelSpeeds(optimizedStates, getMaxLinearSpeedMetersPerSec());
      optimizedPublisher.set(Logger.statesToArray(optimizedStates));

      // Run the states on the modules
      for (int i = 0; i < 4; i++) {
        modules[i].runState(optimizedStates[i]);
      }
    }

    SwerveModuleState[] measuredStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = modules[i].getState();
    }
    measuredPublisher.set(Logger.statesToArray(measuredStates));

    // Update twist
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] =
          new SwerveModulePosition(
              (modules[i].getPositionMeters() - lastModulePositionsMeters[i]),
              modules[i].getAngle());
      lastModulePositionsMeters[i] = modules[i].getPositionMeters();
    }

    var twist = kinematics.toTwist2d(wheelDeltas);
    var gyroYaw = new Rotation2d(gyroInputs.yawPositionRad);
    if (gyroInputs.connected) {
      twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
    }
    lastGyroYaw = gyroYaw;
    poseEstimator.addDriveData(Timer.getFPGATimestamp(), twist);

    posePublisher.set(Logger.pose2dToArray(getPose()));

    // Update field velocity
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(measuredStates);
    Translation2d linearFieldVelocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(getRotation());
    // Update for simulated gyro
    gyroIO.setExpectedYawVelocity(chassisSpeeds.omegaRadiansPerSecond);
    fieldVelocity =
        new Twist2d(
            linearFieldVelocity.getX(),
            linearFieldVelocity.getY(),
            gyroInputs.connected
                ? gyroInputs.yawVelocityRadPerSec
                : chassisSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Gets the last swerve module states.
   *
   * @return The last measured swerve module states.
   */
  public SwerveModuleState[] getLastSwerveModuleStates() {
    return lastSetpointStates;
  }

  /**
   * Adds vision data to the drive subsystem.
   *
   * @param updates The vision updates to be added.
   */
  public void addVisionData(List<TimestampedVisionUpdate> updates) {
    poseEstimator.addVisionData(updates);
  }

  /**
   * Returns the maximum linear speed (free speed) that the drive train can physically attain.
   *
   * @return The value in meters per second.
   */
  public double getMaxLinearSpeedMetersPerSec() {
    return 4.5;
  }

  /**
   * Gets the current rotation of the drive base.
   *
   * @return The current yaw of the robot.
   */
  public Rotation2d getRotation() {
    return poseEstimator.getLatestPose().getRotation();
  }

  /**
   * Gets the current pose of the robot.
   *
   * @return The X,Y, and \theta
   */
  public Pose2d getPose() {
    return poseEstimator.getLatestPose();
  }

  /**
   * Gets the current velocity on the field.
   *
   * @return Returns the velocity as a twist.
   */
  public Twist2d getFieldVelocity() {
    return fieldVelocity;
  }

  /**
   * Gets the current yaw velocity.
   *
   * @return The yaw velocity in rads per second.
   */
  public double getYawVelocity() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  /**
   * Gets the current pitch velocity.
   *
   * @return The pitch velocity in rads per second.
   */
  public double getPitchVelocity() {
    return gyroInputs.pitchVelocityRadPerSec;
  }

  /**
   * Gets the current roll velocity.
   *
   * @return The roll velocity in rads per second.
   */
  public double getRollVelocity() {
    return gyroInputs.rollVelocityRadPerSec;
  }

  /**
   * Gets the current pitch.
   *
   * @return The pitch as a rotation.
   */
  public Rotation2d getPitch() {
    return new Rotation2d(gyroInputs.pitchPositionRad);
  }

  /**
   * Gets the current yaw.
   *
   * @return The pitch as a rotation.
   */
  public Rotation2d getYaw() {
    return new Rotation2d(gyroInputs.yawPositionRad);
  }

  /**
   * Gets the current roll.
   *
   * @return The roll as a rotation.
   */
  public Rotation2d getRoll() {
    return new Rotation2d(gyroInputs.rollPositionRad);
  }

  /**
   * Sets the pose of the pose estimator, used on starting auto.
   *
   * @param pose The pose to be set.
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  /** Resets the robot heading */
  public void resetHeading() {
    gyroIO.resetHeading();
  }

  /**
   * Sets the center of rotation to orbit around.
   *
   * @param center The center of rotation.
   */
  public void setCenterOfRotation(Translation2d center) {
    this.centerOfRotation = center;
  }

  /**
   * Gets the most recent swerve module positions.
   *
   * @return The positions.
   */
  public SwerveModulePosition[] getLastSwerveModulePositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(
          lastSetpointStates[0].speedMetersPerSecond, lastSetpointStates[0].angle),
      new SwerveModulePosition(
          lastSetpointStates[1].speedMetersPerSecond, lastSetpointStates[1].angle),
      new SwerveModulePosition(
          lastSetpointStates[2].speedMetersPerSecond, lastSetpointStates[2].angle),
      new SwerveModulePosition(
          lastSetpointStates[3].speedMetersPerSecond, lastSetpointStates[3].angle)
    };
  }

  /**
   * Runs the provided ChassisSpeeds.
   *
   * @param speeds The speeds to be run.
   */
  public void runVelocity(ChassisSpeeds speeds) {
    setpoint = speeds;
  }

  /** Stops the drive train by clearing the chassis speeds. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** Stops the drive train in an X pattern, locking it in place. */
  public void stopWithLock() {
    stop();
    for (int i = 0; i < 4; i++) {
      lastSetpointStates[i] =
          new SwerveModuleState(
              lastSetpointStates[i].speedMetersPerSecond, getModuleTranslations()[i].getAngle());
    }
  }

  /**
   * Gets the module translation, basically where they are relative to the robots center.
   *
   * @return The translations.
   */
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(Units.inchesToMeters(13), Units.inchesToMeters(13)),
      new Translation2d(Units.inchesToMeters(13), -Units.inchesToMeters(13)),
      new Translation2d(-Units.inchesToMeters(13), Units.inchesToMeters(13)),
      new Translation2d(-Units.inchesToMeters(13), -Units.inchesToMeters(13))
    };
  }
}
