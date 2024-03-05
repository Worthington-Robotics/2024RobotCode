// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.vision.VisionIO.VisionIOInputs;
import frc.WorBots.util.debug.*;
import frc.WorBots.util.math.PoseEstimator.TimestampedVisionUpdate;
import java.util.*;
import java.util.function.*;

/**
 * This subsystem manages all of the camera inputs from our robot, reads them, decides which ones to
 * use, or throw out, logs them, adds the to the pose estimator, and repeats.
 */
public class Vision extends SubsystemBase {
  private final VisionIO[] io;
  private final VisionIOInputs[] inputs;

  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};
  private Consumer<Pose2d> lastPoseConsumer = (x) -> {};
  private Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  /** The transforms for the cameras to robot center */
  private static final Transform3d[] CAMERA_TRANSFORMS =
      new Transform3d[] {
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-11), Units.inchesToMeters(11), Units.inchesToMeters(-9)),
            new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(90 + 43.745)))),
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-11), Units.inchesToMeters(-11), Units.inchesToMeters(-9)),
            new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(180 + 43.745))))
      };

  /** Detection weights for each camera */
  private static final double[] CAMERA_WEIGHTS = new double[] {1.0, 0.9};

  /** How much influence XY data has on the robot pose */
  private static final double xyStdDevCoefficient = 0.025;

  /** How much influence theta data has on the robot pose */
  private static final double thetaStdDevCoefficient = 0.015;

  /** The margin inside the field border to accept vision poses from */
  private static final double fieldBorderMargin = 0.5;

  /** The margin in the z-axis from 0m for a pose to be considered valid, in meters */
  private static final double Z_MARGIN = Units.inchesToMeters(20);

  /** Weights for different tags on the field to be chosen */
  private static final double[] TAG_WEIGHTS =
      new double[] {
        0.95, // Source
        0.95, // Source
        1.15, // Speaker
        1.15, // Speaker
        1.0, // Amp
        0.9, // Stage
        0.9, // Stage
        0.9, // Stage
        0.95, // Source
        0.95, // Source
        1.15, // Speaker
        1.15, // Speaker
        1.0, // Amp
        0.9, // Stage
        0.9, // Stage
        0.9, // Stage
      };

  private static final double targetLogTimeSecs = 0.1;

  private Optional<Pose2d> lastPose = Optional.empty();
  private double lastPoseTime = 0.0;

  public Vision(VisionIO... io) {
    this.io = io;
    inputs = new VisionIOInputs[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputs();
    }
    StatusPage.reportStatus(StatusPage.VISION_SUBSYSTEM, true);
  }

  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      StatusPage.reportStatus(StatusPage.CAM_PREFIX + i, inputs[i].isConnected);
    }

    // Loop over instances
    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses3d = new ArrayList<>();
    List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();

    for (int camIndex = 0; camIndex < io.length; camIndex++) {
      for (int frame = 0; frame < inputs[camIndex].timestamps.length; frame++) {
        final var timestamp = inputs[camIndex].timestamps[frame];
        final var values = inputs[camIndex].frames[frame];

        if (values.length == 0 || values[0] == 0) {
          continue;
        }

        Pose3d cameraPose = null;
        Pose3d robotPose3d = null;

        // Switch based on the number of detections
        switch ((int) values[0]) {
          case 1:
            cameraPose =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            robotPose3d = cameraPose.transformBy(CAMERA_TRANSFORMS[camIndex]);
            break;
          case 2:
            final double error0 = values[1];
            final double error1 = values[9];

            final Pose3d cameraPose0 =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            final Pose3d cameraPose1 =
                new Pose3d(
                    values[10],
                    values[11],
                    values[12],
                    new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
            final Pose3d robotPose3d0 = cameraPose0.transformBy(CAMERA_TRANSFORMS[camIndex]);
            final Pose3d robotPose3d1 = cameraPose1.transformBy(CAMERA_TRANSFORMS[camIndex]);

            // Select pose using scores
            final double score0 = scoreDetection(error0, camIndex, -1);
            final double score1 = scoreDetection(error1, camIndex, -1);
            if (score0 < score1) {
              cameraPose = cameraPose0;
              robotPose3d = robotPose3d0;
            } else if (score1 < score0) {
              cameraPose = cameraPose1;
              robotPose3d = robotPose3d1;
            }
            break;
        }

        // Exit if no data
        if (cameraPose == null || robotPose3d == null) {
          continue;
        }

        // Exit if the pose is invalid
        if (!isPoseValid(robotPose3d)) {
          continue;
        }

        // Get 2D robot pose
        final Pose2d robotPose = robotPose3d.toPose2d();

        // Get tag poses and update last detection times
        List<Pose3d> tagPoses = new ArrayList<>();
        for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i++) {
          final int tagId = (int) values[i];
          lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
          final Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose(tagId);
          if (tagPose.isPresent()) {
            tagPoses.add(tagPose.get());
          }
        }
        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPoses) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        final double avgDistance = totalDistance / tagPoses.size();

        // Add to vision updates
        final double xyStdDev = xyStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();
        final double thetaStdDev =
            thetaStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();
        visionUpdates.add(
            new TimestampedVisionUpdate(
                timestamp, robotPose, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
        allRobotPoses.add(robotPose);
        allRobotPoses3d.add(robotPose3d);
      }

      Logger.getInstance().setRobotPoses(allRobotPoses.toArray(new Pose2d[allRobotPoses.size()]));
      List<Pose3d> allTagPoses = new ArrayList<>();
      for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
        if (Timer.getFPGATimestamp() - detectionEntry.getValue() < targetLogTimeSecs) {
          allTagPoses.add(FieldConstants.aprilTags.getTagPose(detectionEntry.getKey()).get());
        }
      }
      if (DriverStation.isTeleop()) {
        if (allRobotPoses.size() > 0) {
          lastPose = Optional.of(allRobotPoses.get(0));
          lastPoseTime = Timer.getFPGATimestamp();
          SmartDashboard.putNumber("Last Vision Pose Time", lastPoseTime);
        }
      }
      Logger.getInstance().setTagPoses(allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
      Logger.getInstance()
          .setRobotPoses3d(allRobotPoses3d.toArray(new Pose3d[allRobotPoses3d.size()]));
      visionConsumer.accept(visionUpdates);
      if (lastPose.isPresent()) {
        lastPoseConsumer.accept(lastPose.get());
      }
      // lastPose = Optional.empty();
    }
  }

  /**
   * This function accepts the interfaces in and out of the vision system, such as giving out vision
   * updates, and recieving poses.
   */
  public void setDataInterfaces(
      Consumer<List<TimestampedVisionUpdate>> visionConsumer,
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> foo) {
    this.visionConsumer = visionConsumer;
    this.lastPoseConsumer = foo;
  }

  public double getLastPoseTime() {
    return this.lastPoseTime;
  }

  /**
   * Gets whether a 3D pose from vision should be considered valid
   *
   * @param pose The pose from vision
   * @return Whether the pose should be used
   */
  private static boolean isPoseValid(Pose3d pose) {
    if (pose.getX() < -fieldBorderMargin
        || pose.getX() > FieldConstants.fieldLength + fieldBorderMargin
        || pose.getY() < -fieldBorderMargin
        || pose.getY() > FieldConstants.fieldWidth + fieldBorderMargin
        || pose.getZ() < -Z_MARGIN
        || pose.getZ() > Z_MARGIN) {
      return false;
    }

    return true;
  }

  /**
   * Scores a single tag detection
   *
   * @param error The reprojection error of the detection
   * @param camID The ID of the camera making the detection
   * @param tagID The ID of the AprilTag (starting from 1). If -1, the tag score will be ignored
   * @return The score. This value is unitless and relative to other scores only
   */
  private static double scoreDetection(double error, int camID, int tagID) {
    double score = 1.0;

    // A higher error decreases the score
    score /= error;

    // Weigh based on the camera
    final double camWeight = CAMERA_WEIGHTS[camID];
    score *= camWeight;

    // Weigh based on the tag
    if (tagID > 0) {
      final double tagWeight = TAG_WEIGHTS[tagID - 1];
      score *= tagWeight;
    }

    return score;
  }
}
