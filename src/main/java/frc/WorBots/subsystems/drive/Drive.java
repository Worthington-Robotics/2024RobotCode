package frc.WorBots.subsystems.drive;

import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.subsystems.drive.GyroIO.GyroIOInputs;
import frc.WorBots.util.*;
import frc.WorBots.util.PoseEstimator.TimestampedVisionUpdate;

public class Drive extends SubsystemBase {
  private final Module[] modules = new Module[4];
  private ChassisSpeeds setpoint = new ChassisSpeeds();

  private final GyroIO gyroIO;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();
  private double[] lastModulePositionsMeters = new double[] { 0.0, 0.0, 0.0, 0.0 };
  private SwerveModuleState[] lastSetpointStates = new SwerveModuleState[] {
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
  private NetworkTable driveTable = instance.getTable("Drive");
  private DoubleArrayPublisher setpointPublisher = driveTable.getDoubleArrayTopic("Setpoint").publish();
  private DoubleArrayPublisher measuredPublisher = driveTable.getDoubleArrayTopic("Measured").publish();
  private DoubleArrayPublisher posePublisher = driveTable.getDoubleArrayTopic("Pose Estimator").publish();

  public Drive(GyroIO gyroIO, ModuleIO flModule, ModuleIO frModule, ModuleIO blModule, ModuleIO brModule) {
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
      var setpointTwist = new Pose2d()
          .log(new Pose2d(new Translation2d(setpoint.vxMetersPerSecond * 0.02, setpoint.vyMetersPerSecond * 0.02),
              new Rotation2d(setpoint.omegaRadiansPerSecond * 0.02)));
      var adjustedSpeeds = new ChassisSpeeds(setpointTwist.dx / 0.02, setpointTwist.dy / 0.02,
          setpointTwist.dtheta / 0.02);
      SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(adjustedSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, 4.5);

      lastSetpointStates = setpointStates;

      setpointPublisher.set(Logger.statesToArray(setpointStates));

      SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        optimizedStates[i] = modules[i].runState(setpointStates[i]);
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
      wheelDeltas[i] = new SwerveModulePosition((modules[i].getPositionMeters() - lastModulePositionsMeters[i]),
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
    Translation2d linearFieldVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond)
        .rotateBy(getRotation());
    // Update for simulated gyro
    gyroIO.setExpectedYawVelocity(chassisSpeeds.omegaRadiansPerSecond);
    fieldVelocity = new Twist2d(
        linearFieldVelocity.getX(),
        linearFieldVelocity.getY(),
        gyroInputs.connected
            ? gyroInputs.yawVelocityRadPerSec
            : chassisSpeeds.omegaRadiansPerSecond);
  }

  public SwerveModuleState[] getLastSwerveModuleStates() {
    return lastSetpointStates;
  }

  public void addVisionData(List<TimestampedVisionUpdate> updates) {
    poseEstimator.addVisionData(updates);
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return 4.0;
  }

  public Rotation2d getRotation() {
    return poseEstimator.getLatestPose().getRotation();
  }

  public Pose2d getPose() {
    return poseEstimator.getLatestPose();
  }

  public Twist2d getFieldVelocity() {
    return fieldVelocity;
  }

  public double getYawVelocity() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  public double getPitchVelocity() {
    return gyroInputs.pitchVelocityRadPerSec;
  }

  public double getRollVelocity() {
    return gyroInputs.rollVelocityRadPerSec;
  }

  public Rotation2d getPitch() {
    return new Rotation2d(gyroInputs.pitchPositionRad);
  }

  public Rotation2d getYaw() {
    return new Rotation2d(gyroInputs.yawPositionRad);
  }

  public Rotation2d getRoll() {
    return new Rotation2d(gyroInputs.rollPositionRad);
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  public SwerveModulePosition[] getLastSwerveModulePositions() {
    return new SwerveModulePosition[] {
        new SwerveModulePosition(lastSetpointStates[0].speedMetersPerSecond, lastSetpointStates[0].angle),
        new SwerveModulePosition(lastSetpointStates[1].speedMetersPerSecond, lastSetpointStates[1].angle),
        new SwerveModulePosition(lastSetpointStates[2].speedMetersPerSecond, lastSetpointStates[2].angle),
        new SwerveModulePosition(lastSetpointStates[3].speedMetersPerSecond, lastSetpointStates[3].angle)
    };
  }

  public void runVelocity(ChassisSpeeds speeds) {
    setpoint = speeds;
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithLock() {
    stop();
    for (int i = 0; i < 4; i++) {
      lastSetpointStates[i] = new SwerveModuleState(lastSetpointStates[i].speedMetersPerSecond,
          getModuleTranslations()[i].getAngle());
    }
  }

  public Rotation2d getYawRotation() {
    return new Rotation2d(gyroInputs.yawPositionRad);
  }

  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
        new Translation2d(0.41, 0.41),
        new Translation2d(0.41, -0.41),
        new Translation2d(-0.41, 0.41),
        new Translation2d(-0.41, -0.41)
    };
  }
}
