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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.drive.GyroIO.GyroIOInputs;
import frc.WorBots.util.debug.Logger;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.util.debug.TunablePIDController.TunablePIDGains;
import frc.WorBots.util.debug.TunablePIDController.TunableProfiledPIDController;
import frc.WorBots.util.debug.TunablePIDController.TunableTrapezoidConstraints;
import frc.WorBots.util.math.PoseEstimator;
import frc.WorBots.util.math.PoseEstimator.*;
import java.util.List;

public class Drive extends SubsystemBase {
  // Constants
  public static final double WHEELBASE = Units.inchesToMeters(10);

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
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private final PoseEstimator poseEstimator =
      new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));
  private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private Twist2d fieldVelocity = new Twist2d();

  /** Theta setpoint. Can be null */
  private Rotation2d thetaSetpoint = null;

  /** PID controller for the theta setpoint */
  private final TunableProfiledPIDController thetaController =
      new TunableProfiledPIDController(
          new TunablePIDGains(driveTableName, "Theta Gains"),
          new TunableTrapezoidConstraints(driveTableName, "Theta Constraints"));

  /** Whether or not to automatically remove the theta setpoint when it is reached */
  private boolean autoRemoveThetaSetpoint;

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

    if (RobotBase.isReal()) {
      thetaController.setGains(3.9, 0.000, 0.0);
      thetaController.setConstraints(9.5, 3.6);
    } else {
      thetaController.setGains(20.0, 0.08, 0.0);
      thetaController.setConstraints(3.0, 2.0);
    }
    thetaController.pid.enableContinuousInput(0, 2 * Math.PI);
    thetaController.pid.setTolerance(Units.degreesToRadians(2.5));

    StatusPage.reportStatus(StatusPage.DRIVE_SUBSYSTEM, true);
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    StatusPage.reportStatus(StatusPage.GYROSCOPE, gyroInputs.connected);

    thetaController.update();

    for (Module module : modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      for (Module module : modules) {
        module.stop();
      }
      stop();
      removeThetaSetpoint();
    } else {
      double setpointRadsPerSec = setpoint.omegaRadiansPerSecond;

      // Apply additional rotation to get to theta setpoint
      SmartDashboard.putBoolean("Drive/Theta Setpoint Exists", thetaSetpoint != null);
      if (thetaSetpoint != null) {
        SmartDashboard.putNumber("Drive/Theta Setpoint", thetaSetpoint.getRadians());
        if (autoRemoveThetaSetpoint && thetaController.pid.atGoal()) {
          removeThetaSetpoint();
        } else {
          final double additional =
              thetaController.pid.calculate(getRotation().getRadians(), thetaSetpoint.getRadians());
          SmartDashboard.putNumber("Drive/Theta Additional Demand", additional);
          setpointRadsPerSec += additional;
        }
      }

      final var setpointTwist =
          new Pose2d()
              .log(
                  new Pose2d(
                      new Translation2d(
                          setpoint.vxMetersPerSecond * Constants.ROBOT_PERIOD,
                          setpoint.vyMetersPerSecond * Constants.ROBOT_PERIOD),
                      new Rotation2d(setpointRadsPerSec * Constants.ROBOT_PERIOD)));
      final var adjustedSpeeds =
          new ChassisSpeeds(
              setpointTwist.dx / Constants.ROBOT_PERIOD,
              setpointTwist.dy / Constants.ROBOT_PERIOD,
              setpointTwist.dtheta / Constants.ROBOT_PERIOD);
      speedSetpointPublisher.set(Logger.chassisSpeedsToArray(adjustedSpeeds));

      SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(adjustedSpeeds);
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
    final var gyroYaw = new Rotation2d(gyroInputs.yawPositionRad);
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
   * Resets the pose estimator to the latest pose from vision, in teleop mode
   *
   * @param pose The pose from the vision
   */
  public void setLastVisionPose(Pose2d pose) {
    if (DriverStation.isTeleop()) {
      this.poseEstimator.resetPose(pose);
    }
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
   * Gets the setpoint field-relative ChassisSpeeds of the robot
   *
   * @return The speed of the robot
   */
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return setpoint;
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
   * @return The yaw as a rotation.
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
   * Sets a continuous theta setpoint that will not clear automatically
   *
   * @param setpoint The setpoint
   */
  public void setContinuousThetaSetpoint(Rotation2d setpoint) {
    removeThetaSetpoint();
    this.thetaSetpoint = setpoint;
    autoRemoveThetaSetpoint = false;
  }

  /**
   * Sets a single theta setpoint that will automatically clear when it is reached
   *
   * @param setpoint The setpoint
   */
  public void setSingleThetaSetpoint(Rotation2d setpoint) {
    removeThetaSetpoint();
    this.thetaSetpoint = setpoint;
    autoRemoveThetaSetpoint = true;
  }

  /** Removes the existing theta setpoint */
  public void removeThetaSetpoint() {
    this.thetaSetpoint = null;
    autoRemoveThetaSetpoint = false;
    thetaController.pid.reset(getRotation().getRadians());
  }

  /**
   * Returns a command that will set a single theta setpoint
   *
   * @param setpoint The setpoint
   * @return The command
   */
  public Command setSingleThetaSetpointCommand(Rotation2d setpoint) {
    return this.runOnce(() -> setSingleThetaSetpoint(setpoint));
  }

  /**
   * Returns a command that will remove the theta setpoint
   *
   * @return The command
   */
  public Command removeThetaSetpointCommand() {
    return this.runOnce(() -> removeThetaSetpoint());
  }

  /**
   * Gets the module translation, basically where they are relative to the robots center.
   *
   * @return The translations.
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
