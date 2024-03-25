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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.vision.VisionIO.VisionIOInputs;
import frc.WorBots.util.cache.Cache.TimeCache;
import frc.WorBots.util.debug.*;
import frc.WorBots.util.math.GeneralMath;
import frc.WorBots.util.math.PoseEstimator.TimestampedVisionUpdate;
import java.util.*;
import java.util.function.*;

/**
 * This subsystem manages all of the camera inputs from our robot, reads them, decides which ones to
 * use, or throw out, logs them, adds the to the pose estimator, and repeats.
 *
 * <p>Modified from team 6328.
 */
public class Vision extends SubsystemBase {
  private final VisionIO[] io;
  private final VisionIOInputs[] inputs;

  /** Consumer that receives vision updates out of the subsystem */
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};

  /** Times of last detections for tags */
  private Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  /** The amount of detections that have been made */
  private int detectionCount = 0;

  /** Transform for the inward-facing camera on the BR module */
  private static final Transform3d RIGHT_SWERVE_MODULE_TRANSFORM =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-11), Units.inchesToMeters(-11), Units.inchesToMeters(-9)),
          new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
              .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(180 + 43.745))));

  /** Transform for the center-mounted camera */
  private static final Transform3d CENTER_TRANSFORM =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-12.5), Units.inchesToMeters(0), Units.inchesToMeters(-16.0)),
          new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(180 - 0.0)));

  /** The transforms for the cameras to robot center */
  private static final Transform3d[] CAMERA_TRANSFORMS =
      new Transform3d[] {RIGHT_SWERVE_MODULE_TRANSFORM, CENTER_TRANSFORM};

  /** Detection weights for each camera */
  private static final double[] CAMERA_WEIGHTS = new double[] {0.9, 1.0};

  /** How much influence XY data has on the robot pose. Smaller values increase influence */
  private static final double XY_STD_DEV_COEFFICIENT = 0.00015;

  /** How much influence theta data has on the robot pose. Smaller values increase influence */
  private static final double THETA_STD_DEV_COEFFICIENT = 0.00025;

  /**
   * The exponent for the distance scoring formula. Larger values make scores fall off harder with
   * distance
   */
  private static final double DISTANCE_GAIN = 2.8;

  /** The margin inside the field border to accept vision poses from */
  private static final double FIELD_BORDER_MARGIN = 0.5;

  /** The margin in the z-axis from 0m for a pose to be considered valid, in meters */
  private static final double Z_MARGIN = Units.inchesToMeters(20);

  /** Weights for different tags on the field to be chosen */
  private static final double[] TAG_WEIGHTS =
      new double[] {
        0.95, // Source
        0.95, // Source
        1.25, // Speaker
        1.0, // Speaker
        0.9, // Amp
        0.8, // Stage
        0.8, // Stage
        0.8, // Stage
        0.95, // Source
        0.95, // Source
        1.25, // Speaker
        1.0, // Speaker
        0.9, // Amp
        0.8, // Stage
        0.8, // Stage
        0.8, // Stage
      };

  /** The amount of time to log tag poses for */
  private static final double TARGET_LOG_TIME_SECS = 0.1;

  private final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");
  private final DoubleArrayPublisher robotPosesPublisher =
      visionTable.getDoubleArrayTopic("RobotPoses").publish();
  private final DoubleArrayPublisher tagsPosesPublisher =
      visionTable.getDoubleArrayTopic("TagPoses").publish();
  private final DoubleArrayPublisher robotPoses3dPublisher =
      visionTable.getDoubleArrayTopic("RobotPoses3d").publish();
  private final DoublePublisher error0Publisher = visionTable.getDoubleTopic("Error 0").publish();
  private final DoublePublisher error1Publisher = visionTable.getDoubleTopic("Error 1").publish();
  private final DoublePublisher finalScorePublisher =
      visionTable.getDoubleTopic("Final Score").publish();
  private final BooleanPublisher isPoseValidPublisher =
      visionTable.getBooleanTopic("Is Pose Valid").publish();
  private final DoubleArrayPublisher invalidPosePublisher =
      visionTable.getDoubleArrayTopic("Invalid Pose").publish();
  private final IntegerPublisher detectionCountPublisher =
      visionTable.getIntegerTopic("Detection Count").publish();

  public Vision(VisionIO... io) {
    this.io = io;
    inputs = new VisionIOInputs[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputs();
    }
    StatusPage.reportStatus(StatusPage.VISION_SUBSYSTEM, true);
  }

  public void periodic() {
    // Update inputs
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      StatusPage.reportStatus(StatusPage.CAM_PREFIX + i, inputs[i].isConnected);
    }

    // Loop over instances
    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses3d = new ArrayList<>();
    List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();

    for (int camIndex = 0; camIndex < io.length; camIndex++) {
      final VisionIOInputs camInputs = inputs[camIndex];
      for (int frame = 0; frame < camInputs.timestamps.length; frame++) {
        final double timestamp = camInputs.timestamps[frame];
        final double[] values = camInputs.frames[frame];

        if (values.length == 0 || values[0] == 0) {
          continue;
        }

        Pose3d cameraPose = null;
        Pose3d robotPose3d = null;

        // Switch based on the number of detections
        switch ((int) values[0]) {
          case 1:
            // Multiple tags
            cameraPose =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            break;
          case 2:
            // One tag that needs to be disambiguated
            final double error0 = values[1];
            final double error1 = values[9];

            error0Publisher.set(error0);
            error1Publisher.set(error1);

            // Select pose using reprojection error
            if (error0 < error1) {
              cameraPose =
                  new Pose3d(
                      values[2],
                      values[3],
                      values[4],
                      new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            } else {
              cameraPose =
                  new Pose3d(
                      values[10],
                      values[11],
                      values[12],
                      new Rotation3d(
                          new Quaternion(values[13], values[14], values[15], values[16])));
            }
            break;
        }

        // Set robot pose
        if (cameraPose != null) {
          robotPose3d = cameraPose.transformBy(CAMERA_TRANSFORMS[camIndex]);
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
        List<Integer> tagIds = new ArrayList<>();
        for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i++) {
          final int tagId = (int) values[i];
          tagIds.add(tagId);

          lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());

          final Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose(tagId);
          if (tagPose.isPresent()) {
            tagPoses.add(tagPose.get());
          }
        }

        // Calculate average score from all tag detections
        double averageScore = 0.0;
        for (int i = 0; i < tagPoses.size(); i++) {
          final double dist =
              tagPoses.get(i).getTranslation().getDistance(cameraPose.getTranslation());
          final double score = scoreDetection(1.0, dist, camIndex, tagIds.get(i));
          averageScore += score;
        }
        averageScore /= tagPoses.size();
        finalScorePublisher.set(averageScore);

        // Add to vision updates with the calculated standard deviation of the update
        final double xyStdDev = XY_STD_DEV_COEFFICIENT / averageScore;
        final double thetaStdDev = THETA_STD_DEV_COEFFICIENT / averageScore;
        visionUpdates.add(
            new TimestampedVisionUpdate(
                timestamp, robotPose, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));

        allRobotPoses.add(robotPose);
        allRobotPoses3d.add(robotPose3d);

        detectionCount++;
        detectionCountPublisher.set(detectionCount);
      }

      // Collect all tag poses
      List<Pose3d> allTagPoses = new ArrayList<>();
      for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
        if (TimeCache.getInstance().get() - detectionEntry.getValue() < TARGET_LOG_TIME_SECS) {
          allTagPoses.add(FieldConstants.aprilTags.getTagPose(detectionEntry.getKey()).get());
        }
      }

      // Log poses
      setRobotPoses(allRobotPoses);
      setRobotPoses3d(allRobotPoses3d);
      setTagPoses(allTagPoses);

      // Send vision data to consumer
      visionConsumer.accept(visionUpdates);
    }
  }

  /**
   * This function accepts the interfaces in and out of the vision system, such as giving out vision
   * updates, and recieving poses.
   */
  public void setDataInterfaces(Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
    this.visionConsumer = visionConsumer;
  }

  /**
   * Gets whether a 3D pose from vision should be considered valid
   *
   * @param pose The pose from vision
   * @return Whether the pose should be used
   */
  private boolean isPoseValid(Pose3d pose) {
    if (pose.getX() < -FIELD_BORDER_MARGIN
        || pose.getX() > FieldConstants.fieldLength + FIELD_BORDER_MARGIN
        || pose.getY() < -FIELD_BORDER_MARGIN
        || pose.getY() > FieldConstants.fieldWidth + FIELD_BORDER_MARGIN
        || pose.getZ() < -Z_MARGIN
        || pose.getZ() > Z_MARGIN) {
      invalidPosePublisher.set(Logger.pose3dToArray(pose));
      isPoseValidPublisher.set(false);
      return false;
    }
    isPoseValidPublisher.set(true);

    return true;
  }

  /**
   * Scores a single tag detection
   *
   * @param error The reprojection error of the detection
   * @param distance The distance to the AprilTag
   * @param camID The ID of the camera making the detection
   * @param tagID The ID of the AprilTag (starting from 1). If -1, the tag score will be ignored
   * @return The score. This value is unitless and relative to other scores only
   */
  private static double scoreDetection(double error, double distance, int camID, int tagID) {
    double score = 1.0;

    // A higher error decreases the score
    score /= error;

    // A higher distance decreases the score
    score /= GeneralMath.curve(distance, DISTANCE_GAIN);

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

  /** Set the robot poses publisher */
  private void setRobotPoses(List<Pose2d> poses) {
    double[] data = new double[poses.size() * 3];
    for (int i = 0; i < poses.size(); i++) {
      final Pose2d pose = poses.get(i);
      data[i * 3] = pose.getX();
      data[i * 3 + 1] = pose.getY();
      data[i * 3 + 2] = pose.getRotation().getRadians();
    }
    robotPosesPublisher.set(data);
  }

  /** Set the 3D robot poses publisher */
  private void setRobotPoses3d(List<Pose3d> poses) {
    double[] data = new double[poses.size() * 7];
    for (int i = 0; i < poses.size(); i++) {
      final Pose3d pose = poses.get(i);
      data[i * 7] = pose.getX();
      data[i * 7 + 1] = pose.getY();
      data[i * 7 + 2] = pose.getZ();
      data[i * 7 + 3] = pose.getRotation().getQuaternion().getW();
      data[i * 7 + 4] = pose.getRotation().getQuaternion().getX();
      data[i * 7 + 5] = pose.getRotation().getQuaternion().getY();
      data[i * 7 + 6] = pose.getRotation().getQuaternion().getZ();
    }
    robotPoses3dPublisher.set(data);
  }

  /** Set the tag poses publisher */
  private void setTagPoses(List<Pose3d> poses) {
    double[] data = new double[poses.size() * 7];
    for (int i = 0; i < poses.size(); i++) {
      final Pose3d pose = poses.get(i);
      data[i * 7] = pose.getX();
      data[i * 7 + 1] = pose.getY();
      data[i * 7 + 2] = pose.getZ();
      data[i * 7 + 3] = pose.getRotation().getQuaternion().getW();
      data[i * 7 + 4] = pose.getRotation().getQuaternion().getX();
      data[i * 7 + 5] = pose.getRotation().getQuaternion().getY();
      data[i * 7 + 6] = pose.getRotation().getQuaternion().getZ();
    }
    tagsPosesPublisher.set(data);
  }
}
