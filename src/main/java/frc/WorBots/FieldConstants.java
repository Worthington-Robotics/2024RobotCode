package frc.WorBots;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final boolean isWPIField = true; // Red alliance

  public static final double fieldLength = Units.inchesToMeters(651.25);
  public static final double fieldWidth = Units.inchesToMeters(323.25);
  public static final double tapeWidth = Units.inchesToMeters(2.0);

  // April Tags
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  public static final AprilTagFieldLayout aprilTags = isWPIField ? new AprilTagFieldLayout(
      List.of(
          new AprilTag(1,
              new Pose3d(
                  15.079471999999997,
                  0.24587199999999998,
                  1.355852,
                  new Rotation3d(
                      new Quaternion(0.5, 0.0, 0.0, 0.8660254037844386)))),
          new AprilTag(2,
              new Pose3d(
                  16.185134,
                  0.883666,
                  1.355852,
                  new Rotation3d(
                      new Quaternion(0.5, 0.0, 0.0, 0.8660254037844386)))),
          new AprilTag(3,
              new Pose3d(
                  16.579342,
                  4.982717999999999,
                  1.4511020000000001,
                  new Rotation3d(
                      new Quaternion(0.0, 0.0, 0.0, 1.0)))),
          new AprilTag(4,
              new Pose3d(
                  16.579342,
                  5.547867999999999,
                  1.4511020000000001, new Rotation3d(
                      new Quaternion(0.0, 0.0, 0.0, 1.0)))),
          new AprilTag(5,
              new Pose3d(
                  14.7,
                  8.2042,
                  1.355852,
                  new Rotation3d(
                      new Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865476)))),
          new AprilTag(6,
              new Pose3d(
                  1.8415,
                  8.2042,
                  1.355852,
                  new Rotation3d(
                      new Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865476)))),
          new AprilTag(7,
              new Pose3d(
                  -0.038099999999999995,
                  5.547867999999999,
                  1.4511020000000001,
                  new Rotation3d(
                      new Quaternion(1.0, 0.0, 0.0, 0.0)))),
          new AprilTag(8,
              new Pose3d(
                  -0.038099999999999995,
                  4.982717999999999,
                  1.4511020000000001,
                  new Rotation3d(
                      new Quaternion(1.0, 0.0, 0.0, 0.0)))),
          new AprilTag(9,
              new Pose3d(
                  0.356108,
                  0.883666,
                  1.355852,
                  new Rotation3d(
                      new Quaternion(0.8660254037844387, 0.0, 0.0, 0.5)))),
          new AprilTag(10,
              new Pose3d(
                  1.4615159999999998,
                  0.24587199999999998,
                  1.355852,
                  new Rotation3d(
                      new Quaternion(0.8660254037844387, 0.0, 0.0, 0.5))))),
      fieldLength, fieldWidth) : null;
}
