package frc.WorBots.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.WorBots.util.trajectory.RotationSequence;

public class Logger {
  private static Logger instance = new Logger();

  public static Logger getInstance() {
    return instance;
  }

  NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();

  NetworkTable driveTable = defaultInstance.getTable("Drive");
  DoubleArrayPublisher trajectoryPublisher = driveTable.getDoubleArrayTopic("Trajectory").publish();
  DoubleArrayPublisher trajectorySetpointPublisher = driveTable.getDoubleArrayTopic("Trajectory Setpoint").publish();

  NetworkTable visionTable = defaultInstance.getTable("Vision");
  DoubleArrayPublisher robotPosesPublisher = visionTable.getDoubleArrayTopic("RobotPoses").publish();
  DoubleArrayPublisher tagsPosesPublisher = visionTable.getDoubleArrayTopic("TagPoses").publish();
  DoubleArrayPublisher robotPoses3dPublisher = visionTable.getDoubleArrayTopic("RobotPoses3d").publish();

  public void logPose3d(String tableName, String topicName, Pose3d pose) {
    NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();
    NetworkTable table = defaultInstance.getTable(tableName);
    DoubleArrayPublisher topic = table.getDoubleArrayTopic(topicName).publish();
    topic.set(new double[] {
        pose.getX(), pose.getY(), pose.getZ(), pose.getRotation().getQuaternion().getW(),
        pose.getRotation().getQuaternion().getX(), pose.getRotation().getQuaternion().getY(),
        pose.getRotation().getQuaternion().getZ()
    });
    topic.close();
  }

  public void logPose2d(String tableName, String topicName, Pose2d pose) {
    NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();
    NetworkTable table = defaultInstance.getTable(tableName);
    DoubleArrayPublisher topic = table.getDoubleArrayTopic(topicName).publish();
    topic.set(new double[] {
        pose.getX(), pose.getY(), pose.getRotation().getRadians()
    });
    topic.close();
  }

  public void logTrajectory(String tableName, String topicName, Trajectory trajectory) {
    NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();
    NetworkTable table = defaultInstance.getTable(tableName);
    DoubleArrayPublisher topic = table.getDoubleArrayTopic(topicName).publish();
    List<Double> points = new ArrayList<>();
    for (Trajectory.State state : trajectory.getStates()) {
      points.add(state.poseMeters.getX());
      points.add(state.poseMeters.getY());
      points.add(state.poseMeters.getRotation().getRadians());
    }
    topic.set(points.stream().mapToDouble(Double::doubleValue).toArray());
    topic.close();
  }

  public void logSwerveModuleState(String tableName, String topicName, SwerveModuleState state) {

  }

  public void logSwerveModuleStates(String tableName, String topicName, SwerveModuleState[] state) {
    NetworkTable table = defaultInstance.getTable(tableName);
    DoubleArrayPublisher topic = table.getDoubleArrayTopic(topicName).publish();
    topic.set(new double[] {
        state[0].angle.getRadians(), state[0].speedMetersPerSecond,
        state[1].angle.getRadians(), state[1].speedMetersPerSecond,
        state[2].angle.getRadians(), state[2].speedMetersPerSecond,
        state[3].angle.getRadians(), state[3].speedMetersPerSecond
    });
    topic.close();
  }

  public static double[] statesToArray(SwerveModuleState[] state) {
    return new double[] {
        state[0].angle.getRadians(), state[0].speedMetersPerSecond,
        state[1].angle.getRadians(), state[1].speedMetersPerSecond,
        state[2].angle.getRadians(), state[2].speedMetersPerSecond,
        state[3].angle.getRadians(), state[3].speedMetersPerSecond
    };
  }

  public static double[] holonomicTrajectoryToArray(Trajectory trajectory, RotationSequence rotations) {
    List<Double> doubles = new ArrayList<>();
    for (double i = 0.0; i < trajectory.getTotalTimeSeconds(); i += 0.02) {
      doubles.add(trajectory.sample(i).poseMeters.getX());
      doubles.add(trajectory.sample(i).poseMeters.getY());
      doubles.add(rotations.sample(i).position.getRadians());
    }
    return doubles.stream().mapToDouble(Double::doubleValue).toArray();
  }

  public static double[] trajectoryToArray(Trajectory trajectory) {
    List<Double> doubles = new ArrayList<>();
    for (double i = 0.0; i < trajectory.getTotalTimeSeconds(); i += 0.02) {
      doubles.add(trajectory.sample(i).poseMeters.getX());
      doubles.add(trajectory.sample(i).poseMeters.getY());
      doubles.add(trajectory.sample(i).poseMeters.getRotation().getRadians());
    }
    return doubles.stream().mapToDouble(Double::doubleValue).toArray();
  }

  public static double[] pose2dToArray(Pose2d pose) {
    return new double[] {
        pose.getX(), pose.getY(), pose.getRotation().getRadians()
    };
  }

  public static double[] pose3dToArray(Pose3d pose) {
    return new double[] {
        pose.getX(), pose.getY(), pose.getZ(), pose.getRotation().getQuaternion().getW(),
        pose.getRotation().getQuaternion().getX(), pose.getRotation().getQuaternion().getY(),
        pose.getRotation().getQuaternion().getZ()
    };
  }

  public static double[] translation2dToArray(Translation2d translation) {
    return new double[] {
        translation.getX(), translation.getY(), translation.getAngle().getRadians()
    };
  }

  public static double[] translation3dToArray(Translation3d translation) {
    return new double[] {
      translation.getX(), translation.getY(), translation.getZ(), -0.7071067811865475, 0.0, 0.0, 0.7071067811865476
    };
  }

  public void setRobotPoses(Pose2d... value) {
    double[] data = new double[value.length * 3];
    for (int i = 0; i < value.length; i++) {
      data[i * 3] = value[i].getX();
      data[i * 3 + 1] = value[i].getY();
      data[i * 3 + 2] = value[i].getRotation().getRadians();
    }
    robotPosesPublisher.accept(data);
  }

  public void setTagPoses(Pose3d... value) {
    double[] data = new double[value.length * 7];
    for (int i = 0; i < value.length; i++) {
      data[i * 7] = value[i].getX();
      data[i * 7 + 1] = value[i].getY();
      data[i * 7 + 2] = value[i].getZ();
      data[i * 7 + 3] = value[i].getRotation().getQuaternion().getW();
      data[i * 7 + 4] = value[i].getRotation().getQuaternion().getX();
      data[i * 7 + 5] = value[i].getRotation().getQuaternion().getY();
      data[i * 7 + 6] = value[i].getRotation().getQuaternion().getZ();
    }
    tagsPosesPublisher.accept(data);
  }

  public void setRobotPoses3d(Pose3d... value) {
    double[] data = new double[value.length * 7];
    for (int i = 0; i < value.length; i++) {
      data[i * 7] = value[i].getX();
      data[i * 7 + 1] = value[i].getY();
      data[i * 7 + 2] = value[i].getZ();
      data[i * 7 + 3] = value[i].getRotation().getQuaternion().getW();
      data[i * 7 + 4] = value[i].getRotation().getQuaternion().getX();
      data[i * 7 + 5] = value[i].getRotation().getQuaternion().getY();
      data[i * 7 + 6] = value[i].getRotation().getQuaternion().getZ();
    }
    robotPoses3dPublisher.set(data);
  }

  public void setDriveTrajectory(Pose2d... value) {
    double[] data = new double[value.length * 3];
    for (int i = 0; i < value.length; i++) {
      data[i * 3] = value[i].getX();
      data[i * 3 + 1] = value[i].getY();
      data[i * 3 + 2] = value[i].getRotation().getRadians();
    }
    trajectoryPublisher.set(data);
  }

  public void setDriveTrajSetpoint(Pose2d... value) {
    double[] data = new double[value.length * 3];
    for (int i = 0; i < value.length; i++) {
      data[i * 3] = value[i].getX();
      data[i * 3 + 1] = value[i].getY();
      data[i * 3 + 2] = value[i].getRotation().getRadians();
    }
    trajectorySetpointPublisher.set(data);
  }
}
