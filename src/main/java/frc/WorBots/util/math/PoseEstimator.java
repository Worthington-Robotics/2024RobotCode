// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.WorBots.Constants;
import frc.WorBots.FieldConstants;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * A class to estimate the absolute position and rotation of the robot on the field using a
 * combination of robot odometry and vision data. Used for both real operation and simulation.
 *
 * <p>Modified from team 6328.
 */
public class PoseEstimator {
  /** The length of time to keep updates for in seconds */
  private static final double HISTORY_LENGTH = 0.3;

  /** Vision pose distance threshold under which to apply anti-jitter */
  private static final double ANTI_JITTER_THRESHOLD = Units.inchesToMeters(4.5);

  /** The amount of anti-jitter to apply. Lower values increase the effect. */
  private static final double ANTI_JITTER_FACTOR = 0.20;

  private Pose2d basePose = new Pose2d();
  private Pose2d latestPose = new Pose2d();
  private final NavigableMap<Double, PoseUpdate> updates = new TreeMap<>();
  private final Matrix<N3, N1> q = new Matrix<>(Nat.N3(), Nat.N1());

  /**
   * Create a PoseEstimator with standard deviation noise
   *
   * @param stateStdDevs A matrix of the standard deviation parameters
   */
  public PoseEstimator(Matrix<N3, N1> stateStdDevs) {
    for (int i = 0; i < 3; ++i) {
      q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
    }
  }

  /**
   * Returns the latest robot pose based on drive and vision data.
   *
   * @return The latest pose of the robot
   */
  public Pose2d getLatestPose() {
    return latestPose;
  }

  /**
   * Resets the odometry to a known pose.
   *
   * @param pose The pose to reset the robot to
   */
  public void resetPose(Pose2d pose) {
    basePose = pose;
    updates.clear();
    update();
  }

  /**
   * Records a new drive movement.
   *
   * @param timestamp The timestamp in seconds where the event occurred
   * @param twist The twist of the robot that represents the movement
   */
  public void addDriveData(double timestamp, Twist2d twist) {
    updates.put(timestamp, new PoseUpdate(twist, new ArrayList<>()));
    update();
  }

  /**
   * Records a new set of vision updates.
   *
   * @param visionData A list of timestamped vision updates
   */
  public void addVisionData(List<TimestampedVisionUpdate> visionData) {
    for (TimestampedVisionUpdate timestampedVisionUpdate : visionData) {
      final double timestamp = timestampedVisionUpdate.timestamp();
      final VisionUpdate visionUpdate =
          new VisionUpdate(timestampedVisionUpdate.pose(), timestampedVisionUpdate.stdDevs());

      if (updates.containsKey(timestamp)) {
        // There was already an update at this timestamp, add to it
        final ArrayList<VisionUpdate> oldVisionUpdates = updates.get(timestamp).visionUpdates();
        oldVisionUpdates.add(visionUpdate);
        oldVisionUpdates.sort(VisionUpdate.compareDescStdDev);
      } else {
        // Insert a new update
        final Entry<Double, PoseUpdate> prevUpdate = updates.floorEntry(timestamp);
        final Entry<Double, PoseUpdate> nextUpdate = updates.ceilingEntry(timestamp);
        if (prevUpdate == null || nextUpdate == null) {
          // Outside the range of existing data
          return;
        }

        // Create partial twists (prev -> vision, vision -> next)
        final double dt = (nextUpdate.getKey() - prevUpdate.getKey());
        final Twist2d twist0 =
            GeomUtil.multiplyTwist(
                nextUpdate.getValue().twist(), (timestamp - prevUpdate.getKey()) / dt);
        final Twist2d twist1 =
            GeomUtil.multiplyTwist(
                nextUpdate.getValue().twist(), (nextUpdate.getKey() - timestamp) / dt);

        // Add new pose updates
        final ArrayList<VisionUpdate> newVisionUpdates = new ArrayList<VisionUpdate>();
        newVisionUpdates.add(visionUpdate);
        newVisionUpdates.sort(VisionUpdate.compareDescStdDev);
        updates.put(timestamp, new PoseUpdate(twist0, newVisionUpdates));
        updates.put(
            nextUpdate.getKey(), new PoseUpdate(twist1, nextUpdate.getValue().visionUpdates()));
      }
    }

    // Recalculate latest pose once
    update();
  }

  /** Clears old data and calculates the latest pose. */
  private void update() {
    // Clear old data and update base pose
    while (updates.size() > 1 && updates.firstKey() < Timer.getFPGATimestamp() - HISTORY_LENGTH) {
      final Entry<Double, PoseUpdate> update = updates.pollFirstEntry();
      basePose = update.getValue().apply(basePose, q);
    }

    // Update latest pose
    latestPose = basePose;
    for (Entry<Double, PoseUpdate> updateEntry : updates.entrySet()) {
      latestPose = updateEntry.getValue().apply(latestPose, q);
    }
  }

  /**
   * Represents a sequential update to a pose estimate, with a twist (drive movement) and list of
   * vision updates.
   */
  private static record PoseUpdate(Twist2d twist, ArrayList<VisionUpdate> visionUpdates) {
    public Pose2d apply(Pose2d lastPose, Matrix<N3, N1> q) {
      // Apply drive twist
      Pose2d pose = lastPose.exp(twist);

      // Apply vision updates
      for (VisionUpdate visionUpdate : visionUpdates) {
        // Calculate Kalman gains based on std devs
        // (https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/estimator/)
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
          r[i] = visionUpdate.stdDevs().get(i, 0) * visionUpdate.stdDevs().get(i, 0);
        }
        for (int row = 0; row < 3; ++row) {
          if (q.get(row, 0) == 0.0) {
            visionK.set(row, row, 0.0);
          } else {
            visionK.set(
                row, row, q.get(row, 0) / (q.get(row, 0) + Math.sqrt(q.get(row, 0) * r[row])));
          }
        }

        // Scale twists that are within a distance down to reduce jitter
        final double distance =
            pose.getTranslation().getDistance(visionUpdate.pose().getTranslation());
        double scaleFactor = 1.0;
        if (distance < ANTI_JITTER_THRESHOLD) {
          scaleFactor = ANTI_JITTER_FACTOR;
        }

        // Calculate twist between current and vision pose
        final Twist2d visionTwist = pose.log(visionUpdate.pose());

        // Multiply by Kalman gain matrix
        final Matrix<N3, N1> twistMatrix =
            visionK.times(
                VecBuilder.fill(
                    visionTwist.dx * scaleFactor,
                    visionTwist.dy * scaleFactor,
                    visionTwist.dtheta));

        // Apply twist to the current pose
        pose =
            pose.exp(
                new Twist2d(twistMatrix.get(0, 0), twistMatrix.get(1, 0), twistMatrix.get(2, 0)));
      }

      // Clamp the pose within the field bounds
      pose = clampPose(pose);

      return pose;
    }
  }

  /** Represents a single vision pose with associated standard deviations. */
  public static record VisionUpdate(Pose2d pose, Matrix<N3, N1> stdDevs) {
    public static final Comparator<VisionUpdate> compareDescStdDev =
        (VisionUpdate a, VisionUpdate b) -> {
          return -Double.compare(
              a.stdDevs().get(0, 0) + a.stdDevs().get(1, 0),
              b.stdDevs().get(0, 0) + b.stdDevs().get(1, 0));
        };
  }

  /** Represents a single vision pose with a timestamp and associated standard deviations. */
  public static record TimestampedVisionUpdate(
      double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {}

  /**
   * Clamps a pose within the bounds of the field
   *
   * @param pose The pose to clamp
   * @return The clamped pose
   */
  private static Pose2d clampPose(Pose2d pose) {
    // Get the shorter dimension of the robot so that we can clamp poses with the
    // robot inside the wall
    final double robotSize = Math.min(Constants.ROBOT_WIDTH, Constants.ROBOT_LENGTH) / 2;

    // Clamp poses off the field
    pose =
        new Pose2d(
            MathUtil.clamp(pose.getX(), 0.0 + robotSize, FieldConstants.fieldLength - robotSize),
            MathUtil.clamp(pose.getY(), 0.0 + robotSize, FieldConstants.fieldWidth - robotSize),
            pose.getRotation());
    return pose;
  }
}
