// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.debug;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.superstructure.SuperstructureIO.SuperstructureIOInputs;
import frc.WorBots.util.trajectory.RotationSequence;
import java.util.ArrayList;
import java.util.List;

/**
 * A collection of methods to output WPILib classes to NetworkTables, for use with AdvantageScope
 */
public class Logger {
  private static Logger instance = new Logger();

  public static Logger getInstance() {
    return instance;
  }

  NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();

  NetworkTable driveTable = defaultInstance.getTable("Drive");
  DoubleArrayPublisher trajectoryPublisher = driveTable.getDoubleArrayTopic("Trajectory").publish();
  DoubleArrayPublisher trajectorySetpointPublisher =
      driveTable.getDoubleArrayTopic("Trajectory Setpoint").publish();

  NetworkTable superstructureTable = defaultInstance.getTable("Superstructure");
  StringPublisher modePublisher = superstructureTable.getStringTopic("Mode").publish();
  DoublePublisher setpointPositionPivot =
      superstructureTable.getDoubleTopic("Pivot Setpoint Rad").publish();
  DoublePublisher setpointPositionElevator =
      superstructureTable.getDoubleTopic("Elevator Setpoint Meters").publish();
  DoublePublisher setpointVoltagePivot =
      superstructureTable.getDoubleTopic("Pivot Setpoint Voltage").publish();
  DoublePublisher setpointVoltageElevator =
      superstructureTable.getDoubleTopic("Elevator Setpoint Voltage").publish();
  DoubleArrayPublisher pose3dPublisher =
      superstructureTable.getDoubleArrayTopic("3d Poses").publish();
  BooleanPublisher isAtSetpointPublisher =
      superstructureTable.getBooleanTopic("At Setpoint").publish();
  BooleanPublisher inHandoffPublisher = superstructureTable.getBooleanTopic("In Handoff").publish();

  NetworkTable pivotTable = superstructureTable.getSubTable("Pivot");
  DoublePublisher pivotPositionRel = pivotTable.getDoubleTopic("Position Rad Rel").publish();
  DoublePublisher pivotPositionAbs = pivotTable.getDoubleTopic("Position Rad Abs").publish();
  DoublePublisher pivotFusedAngleRad = pivotTable.getDoubleTopic("Fused Rad").publish();
  DoublePublisher pivotVelocity = pivotTable.getDoubleTopic("Velocity Rad Per Sec").publish();
  DoublePublisher pivotVoltage = pivotTable.getDoubleTopic("Voltage Applied").publish();
  DoublePublisher pivotTemp = pivotTable.getDoubleTopic("Temp Celsius").publish();

  NetworkTable elevatorTable = superstructureTable.getSubTable("Elevator");
  DoublePublisher elevatorPositionRel =
      elevatorTable.getDoubleTopic("Position Meters Rel").publish();
  DoublePublisher elevatorPositionAbs =
      elevatorTable.getDoubleTopic("Position Meters Abs").publish();
  DoublePublisher elevatorVelocity =
      elevatorTable.getDoubleTopic("Velocity Meters Per Sec").publish();
  DoublePublisher elevatorVoltage = elevatorTable.getDoubleTopic("Voltage Applied").publish();
  DoublePublisher elevatorTemp = elevatorTable.getDoubleTopic("Temp Celsius").publish();
  DoublePublisher elevatorPercentageRaised =
      elevatorTable.getDoubleTopic("Percentage Raised").publish();

  public void logPose3d(String tableName, String topicName, Pose3d pose) {
    NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();
    NetworkTable table = defaultInstance.getTable(tableName);
    DoubleArrayPublisher topic = table.getDoubleArrayTopic(topicName).publish();
    topic.set(
        new double[] {
          pose.getX(),
          pose.getY(),
          pose.getZ(),
          pose.getRotation().getQuaternion().getW(),
          pose.getRotation().getQuaternion().getX(),
          pose.getRotation().getQuaternion().getY(),
          pose.getRotation().getQuaternion().getZ()
        });
    topic.close();
  }

  public void logPose2d(String tableName, String topicName, Pose2d pose) {
    NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();
    NetworkTable table = defaultInstance.getTable(tableName);
    DoubleArrayPublisher topic = table.getDoubleArrayTopic(topicName).publish();
    topic.set(new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()});
    topic.close();
  }

  public void logTranslation2d(String tableName, String topicName, Translation2d pose) {
    NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();
    NetworkTable table = defaultInstance.getTable(tableName);
    DoubleArrayPublisher topic = table.getDoubleArrayTopic(topicName).publish();
    topic.set(new double[] {pose.getX(), pose.getY()});
    topic.close();
  }

  public void logTranslation3d(String tableName, String topicName, Translation3d pose) {
    NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();
    NetworkTable table = defaultInstance.getTable(tableName);
    DoubleArrayPublisher topic = table.getDoubleArrayTopic(topicName).publish();
    topic.set(new double[] {pose.getX(), pose.getY(), pose.getZ()});
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

  public void logSwerveModuleState(String tableName, String topicName, SwerveModuleState state) {}

  public void logSwerveModuleStates(String tableName, String topicName, SwerveModuleState[] state) {
    NetworkTable table = defaultInstance.getTable(tableName);
    DoubleArrayPublisher topic = table.getDoubleArrayTopic(topicName).publish();
    topic.set(
        new double[] {
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

  public static double[] holonomicTrajectoryToArray(
      Trajectory trajectory, RotationSequence rotations) {
    List<Double> doubles = new ArrayList<>();
    for (double i = 0.0; i < trajectory.getTotalTimeSeconds(); i += Constants.ROBOT_PERIOD) {
      doubles.add(trajectory.sample(i).poseMeters.getX());
      doubles.add(trajectory.sample(i).poseMeters.getY());
      doubles.add(rotations.sample(i).position.getRadians());
    }
    return doubles.stream().mapToDouble(Double::doubleValue).toArray();
  }

  public static double[] trajectoryToArray(Trajectory trajectory) {
    List<Double> doubles = new ArrayList<>();
    for (double i = 0.0; i < trajectory.getTotalTimeSeconds(); i += Constants.ROBOT_PERIOD) {
      doubles.add(trajectory.sample(i).poseMeters.getX());
      doubles.add(trajectory.sample(i).poseMeters.getY());
      doubles.add(trajectory.sample(i).poseMeters.getRotation().getRadians());
    }
    return doubles.stream().mapToDouble(Double::doubleValue).toArray();
  }

  public static double[] pose2dToArray(Pose2d pose) {
    return new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
  }

  public static double[] pose3dToArray(Pose3d pose) {
    return new double[] {
      pose.getX(),
      pose.getY(),
      pose.getZ(),
      pose.getRotation().getQuaternion().getW(),
      pose.getRotation().getQuaternion().getX(),
      pose.getRotation().getQuaternion().getY(),
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
      translation.getX(),
      translation.getY(),
      translation.getZ(),
      -0.7071067811865475,
      0.0,
      0.0,
      0.7071067811865476
    };
  }

  public static double[] chassisSpeedsToArray(ChassisSpeeds speeds) {
    return new double[] {
      speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond
    };
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

  public void setSuperstructureInputs(SuperstructureIOInputs inputs) {
    pivotPositionRel.set(inputs.pivotPositionRelRad);
    pivotPositionAbs.set(inputs.pivotPositionAbsRad);
    pivotVelocity.set(inputs.pivot.velocityRadsPerSec);
    pivotVoltage.set(inputs.pivot.appliedPowerVolts);
    pivotTemp.set(inputs.pivot.temperatureCelsius);
    elevatorPositionRel.set(inputs.elevatorPositionMeters);
    elevatorPositionAbs.set(0.0);
    elevatorVelocity.set(inputs.elevatorVelocityMetersPerSec);
    elevatorVoltage.set(inputs.elevator.appliedPowerVolts);
    elevatorTemp.set(inputs.elevator.temperatureCelsius);
    elevatorPercentageRaised.set(inputs.elevatorPercentageRaised);
  }

  public void setSuperstructureSetpoints(
      String mode,
      double pivotPos,
      double elevatorPos,
      double pivotVoltage,
      double elevatorVoltage) {
    modePublisher.set(mode);
    setpointPositionElevator.set(elevatorPos);
    setpointPositionPivot.set(pivotPos);
    setpointVoltageElevator.set(elevatorVoltage);
    setpointVoltagePivot.set(pivotVoltage);
  }

  public void setSuperstructureMode(String mode) {
    modePublisher.set(mode);
  }

  public void setSuperstructurePivotFusedRad(double value) {
    pivotFusedAngleRad.set(value);
  }

  public void setSuperstructurePivotPosSetpoint(double value) {
    setpointPositionPivot.set(value);
  }

  public void setSuperstructureElevatorPosSetpoint(double value) {
    setpointPositionElevator.set(value);
  }

  public void setSuperstructureElevatorVoltageSetpoint(double value) {
    setpointVoltageElevator.set(value);
  }

  public void setSuperstructurePivotVoltageSetpoint(double value) {
    setpointVoltagePivot.set(value);
  }

  public void setSuperstructurePoses3d(Pose3d... value) {
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
    pose3dPublisher.set(data);
  }

  public void setSuperstructureAtSetpoint(boolean value) {
    isAtSetpointPublisher.set(value);
  }

  public void setSuperstructureInHandoff(boolean value) {
    inHandoffPublisher.set(value);
  }
}
