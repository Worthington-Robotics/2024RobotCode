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
import frc.WorBots.util.debug.Logger;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.util.math.AllianceFlipUtil;
import frc.WorBots.util.math.GeomUtil;
import frc.WorBots.util.math.PoseEstimator;
import frc.WorBots.util.math.PoseEstimator.*;
import java.util.List;

public class Drive extends SubsystemBase {
  // Constants
  public static final double WHEELBASE = Units.inchesToMeters(10);

  /** The drift rate of the robot when driving, in radians per second */
  private static final double DRIFT_RATE = 0.0;

  private final Module[] modules = new Module[4];
  private final GyroIO gyroIO;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private final PoseEstimator poseEstimator =
      new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));

  /** The setpoint speeds for the drivetrain */
  private ChassisSpeeds setpointSpeeds = new ChassisSpeeds();

  /** The last field velocity */
  private Twist2d fieldVelocity = new Twist2d();

  /** The last yaw of the gyro, used for delta calculation */
  private Rotation2d lastGyroYaw = new Rotation2d();

  /** The last positions of the modules, used for delta calculations */
  private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};

  /** The last setpoint states for the modules */
  private SwerveModuleState[] lastSetpointStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private static final String driveTableName = "Drive";
  private final NetworkTable driveTable = instance.getTable(driveTableName);
  private final DoubleArrayPublisher speedSetpointPublisher =
      driveTable.getDoubleArrayTopic("Speed Setpoint").publish();
  private final DoubleArrayPublisher setpointPublisher =
      driveTable.getDoubleArrayTopic("Setpoint").publish();
  private final DoubleArrayPublisher optimizedPublisher =
      driveTable.getDoubleArrayTopic("Optimized").publish();
  private final DoubleArrayPublisher measuredPublisher =
      driveTable.getDoubleArrayTopic("Measured").publish();
  private final DoubleArrayPublisher posePublisher =
      driveTable.getDoubleArrayTopic("Pose Estimator").publish();

  /**
   * The main swerve drive subsystem
   *
   * @param gyroIO The IO to use for the gyro
   * @param flModule The IO to use for the front left module
   * @param frModule The IO to use for the front right module
   * @param blModule The IO to use for the back left module
   * @param brModule The IO to use for the back right module
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

    for (Module module : modules) {
      module.periodic();
    }
    // The timestamp that is closest to when we get our inputs
    final double inputTimestamp = Timer.getFPGATimestamp();

    StatusPage.reportStatus(StatusPage.GYROSCOPE, gyroInputs.connected);

    if (DriverStation.isDisabled()) {
      for (Module module : modules) {
        module.stop();
      }
      stop();
    } else {
      speedSetpointPublisher.set(Logger.chassisSpeedsToArray(setpointSpeeds));

      SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(setpointSpeeds);
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

    // Get measured states from modules
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = modules[i].getState();
    }
    measuredPublisher.set(Logger.statesToArray(measuredStates));

    // Update twist
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      final Module module = modules[i];
      wheelDeltas[i] =
          new SwerveModulePosition(
              (module.getPositionMeters() - lastModulePositionsMeters[i]), module.getAngle());
      lastModulePositionsMeters[i] = module.getPositionMeters();
    }

    // Do inverse kinematics to get the robot twist
    final Twist2d twist = kinematics.toTwist2d(wheelDeltas);

    // If the gyro is connected, use it's dtheta as it is more accurate
    final Rotation2d gyroYaw = getYaw();
    if (gyroInputs.connected) {
      twist.dtheta = gyroYaw.minus(lastGyroYaw).getRadians();
    }
    lastGyroYaw = gyroYaw;

    // Add to pose estimator
    poseEstimator.addDriveData(inputTimestamp, twist);
    posePublisher.set(Logger.pose2dToArray(getPose()));

    // Update field velocity
    final ChassisSpeeds measuredChassisSpeeds = kinematics.toChassisSpeeds(measuredStates);
    final Translation2d linearFieldVelocity =
        new Translation2d(
                measuredChassisSpeeds.vxMetersPerSecond, measuredChassisSpeeds.vyMetersPerSecond)
            .rotateBy(getRotation());

    // Update for simulated gyro
    gyroIO.setExpectedYawVelocity(measuredChassisSpeeds.omegaRadiansPerSecond);

    // Update field velocity twist
    fieldVelocity =
        new Twist2d(
            linearFieldVelocity.getX(),
            linearFieldVelocity.getY(),
            gyroInputs.connected
                ? gyroInputs.yawVelocityRadPerSec
                : measuredChassisSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Gets the last swerve module states
   *
   * @return The last measured swerve module states
   */
  public SwerveModuleState[] getLastSwerveModuleStates() {
    return lastSetpointStates;
  }

  /**
   * Adds vision data to the drive subsystem
   *
   * @param updates The vision updates to be added
   */
  public void addVisionData(List<TimestampedVisionUpdate> updates) {
    poseEstimator.addVisionData(updates);
  }

  /**
   * Returns the maximum linear speed (free speed) that the drive train can physically attain
   *
   * @return The value in meters per second
   */
  public double getMaxLinearSpeedMetersPerSec() {
    return 4.5;
  }

  /**
   * Gets the current rotation of the drive base
   *
   * @return The current yaw of the robot
   */
  public Rotation2d getRotation() {
    return poseEstimator.getLatestPose().getRotation();
  }

  /**
   * Gets the current pose of the robot
   *
   * @return The field-relative robot pose
   */
  public Pose2d getPose() {
    return poseEstimator.getLatestPose();
  }

  /**
   * Gets the current velocity on the field
   *
   * @return Returns the velocity as a twist
   */
  public Twist2d getFieldVelocity() {
    return fieldVelocity;
  }

  /**
   * Gets the setpoint field-relative ChassisSpeeds of the robot
   *
   * @return The speed of the robot
   */
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return setpointSpeeds;
  }

  /**
   * Gets the current yaw velocity
   *
   * @return The yaw velocity in rads per second
   */
  public double getYawVelocity() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  /**
   * Gets the current yaw
   *
   * @return The yaw as a rotation
   */
  public Rotation2d getYaw() {
    return new Rotation2d(gyroInputs.yawPositionRad);
  }

  /**
   * Sets the pose of the pose estimator, used on starting auto
   *
   * @param pose The pose to be set
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  /** Resets the robot heading */
  public void resetHeading(Rotation2d heading) {
    gyroIO.resetHeading(heading);
    final Pose2d currentPose = poseEstimator.getLatestPose();
    poseEstimator.resetPose(
        new Pose2d(currentPose.getX(), currentPose.getY(), AllianceFlipUtil.apply(heading)));
  }

  /**
   * Gets the most recent swerve module positions
   *
   * @return The positions
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
   * Gets the distance driven of each of the modules in radians
   *
   * @return The distances
   */
  public double[] getModuleDistances() {
    return new double[] {
      modules[0].getPositionRads(),
      modules[1].getPositionRads(),
      modules[2].getPositionRads(),
      modules[3].getPositionRads()
    };
  }

  /**
   * Runs the provided ChassisSpeeds
   *
   * @param speeds The speeds to be run
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Adjust the velocity only once here to reduce calculations
    final ChassisSpeeds adjusted = GeomUtil.driftCorrectChassisSpeeds(speeds, DRIFT_RATE);
    setpointSpeeds = adjusted;
  }

  /** Stops the drive train by clearing the chassis speeds */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** Stops the drive train in an X pattern, locking it in place */
  public void stopWithLock() {
    stop();
    for (int i = 0; i < 4; i++) {
      lastSetpointStates[i] =
          new SwerveModuleState(
              lastSetpointStates[i].speedMetersPerSecond, getModuleTranslations()[i].getAngle());
    }
  }

  /**
   * Gets the module translations relative to the robot's center
   *
   * @return The translations
   */
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(WHEELBASE, WHEELBASE),
      new Translation2d(WHEELBASE, -WHEELBASE),
      new Translation2d(-WHEELBASE, WHEELBASE),
      new Translation2d(-WHEELBASE, -WHEELBASE)
    };
  }
}
