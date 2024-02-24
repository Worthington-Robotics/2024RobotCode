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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.vision.VisionIO.VisionIOInputs;
import frc.WorBots.util.debug.*;
import frc.WorBots.util.math.GeomUtil;
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
  private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
  private final Pose3d[] cameraPoses;
  // How much influence XY data has on the robot pose
  private final double xyStdDevCoefficient;
  // How much influence theta data has on the robot pose
  private final double thetaStdDevCoefficient;
  private Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();
  private static final double ambiguityThreshold = 0.3;
  private static final double fieldBorderMargin = 0.5;
  private static final double zMargin = 0.75;
  private static final double targetLogTimeSecs = 0.1;

  public Vision(VisionIO... io) {
    this.io = io;
    inputs = new VisionIOInputs[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputs();
    }
    cameraPoses =
        new Pose3d[] {
          new Pose3d(
              new Translation3d(
                  Units.inchesToMeters(-11), Units.inchesToMeters(11), Units.inchesToMeters(-9)),
              new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(90 + 43.745)))),
          new Pose3d(
              new Translation3d(
                  Units.inchesToMeters(-11), Units.inchesToMeters(-11), Units.inchesToMeters(-9)),
              new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(180 + 43.745))))
        };
    SmartDashboard.putNumberArray("Camera Pose 1", Logger.pose3dToArray(cameraPoses[1]));
    xyStdDevCoefficient = 0.01;
    thetaStdDevCoefficient = 0.015;
    StatusPage.reportStatus(StatusPage.VISION_SUBSYSTEM, true);
  }

  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
    }

    // Loop over instances
    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses3d = new ArrayList<>();
    List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();

    for (int index = 0; index < io.length; index++) {
      for (int frame = 0; frame < inputs[index].timestamps.length; frame++) {
        var timestamp = inputs[index].timestamps[frame];
        var values = inputs[index].frames[frame];

        if (values.length == 0 || values[0] == 0) {
          continue;
        }

        Pose3d cameraPose = null;
        Pose3d robotPose3d = null;

        switch ((int) values[0]) {
          case 1:
            cameraPose =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            robotPose3d = cameraPose.transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[index]));
            break;
          case 2:
            double error0 = values[1];
            double error1 = values[9];

            Pose3d cameraPose0 =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            Pose3d cameraPose1 =
                new Pose3d(
                    values[10],
                    values[11],
                    values[12],
                    new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
            Pose3d robotPose3d0 =
                cameraPose0.transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[index]));
            Pose3d robotPose3d1 =
                cameraPose1.transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[index]));

            // Select pose using projection errors
            if (error0 < error1 * ambiguityThreshold) {
              cameraPose = cameraPose0;
              robotPose3d = robotPose3d0;
            } else if (error1 < error0 * ambiguityThreshold) {
              cameraPose = cameraPose1;
              robotPose3d = robotPose3d1;
            }
            break;
        }

        // Exit if no data
        if (cameraPose == null || robotPose3d == null) {
          continue;
        }

        // Exit if robot pose is off the field
        if (robotPose3d.getX() < -fieldBorderMargin
            || robotPose3d.getX() > FieldConstants.fieldLength + fieldBorderMargin
            || robotPose3d.getY() < -fieldBorderMargin
            || robotPose3d.getY() > FieldConstants.fieldWidth + fieldBorderMargin
            || robotPose3d.getZ() < -zMargin
            || robotPose3d.getZ() > zMargin) {
          continue;
        }
        // Get 2D robot pose
        Pose2d robotPose = robotPose3d.toPose2d();

        // Get tag poses and update last detection times
        List<Pose3d> tagPoses = new ArrayList<>();
        for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i++) {
          int tagId = (int) values[i];
          lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
          Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose((int) values[i]);
          if (tagPose.isPresent()) {
            tagPoses.add(tagPose.get());
          }
        }
        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPoses) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        double avgDistance = totalDistance / tagPoses.size();

        // Add to vision updates
        double xyStdDev = xyStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();
        double thetaStdDev = thetaStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();
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
      Logger.getInstance().setTagPoses(allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
      Logger.getInstance()
          .setRobotPoses3d(allRobotPoses3d.toArray(new Pose3d[allRobotPoses3d.size()]));
      visionConsumer.accept(visionUpdates);
    }
  }

  /**
   * This function accepts the interfaces in and out of the vision system, such as giving out vision
   * updates, and recieving poses.
   */
  public void setDataInterfaces(
      Consumer<List<TimestampedVisionUpdate>> visionConsumer, Supplier<Pose2d> poseSupplier) {
    this.visionConsumer = visionConsumer;
    this.poseSupplier = poseSupplier;
  }
}
