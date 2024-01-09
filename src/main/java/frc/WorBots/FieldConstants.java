package frc.WorBots;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final boolean isWPIField = true; // Red alliance

  public static final double fieldLength = Units.inchesToMeters(651.25); // 16.451 m
  public static final double fieldWidth = Units.inchesToMeters(323.25); // 8.211 m
  public static final double tapeWidth = Units.inchesToMeters(2.0);

  public static final class Wing {

  }

  public static final class Amp {
    public static final double openingBottomZ = Units.inchesToMeters(26);
    public static final double openingTopZ = openingBottomZ + Units.inchesToMeters(18);
    public static final double faceStartX = Units.inchesToMeters(49.5);
    public static final double openingStartX = faceStartX + Units.inchesToMeters(11);
    public static final double openingEndX = openingStartX + Units.feetToMeters(2);
    public static final Translation3d test = new Translation3d(openingStartX, fieldWidth, openingBottomZ);

    public static final double bottomY = fieldWidth;
    public static final double topY = bottomY - Units.inchesToMeters(10);
    public static final double startX = 0;
    public static final double endX = Units.feetToMeters(10);
    public static final Translation2d[] regionCorners = new Translation2d[] {
        new Translation2d(startX, bottomY),
        new Translation2d(startX, topY),
        new Translation2d(endX, topY),
        new Translation2d(endX, bottomY)
    };
  }

  public static final class Speaker {

  }

  public static final class Source {

  }

  public static final class Stage {

  }

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
                      new Quaternion(0.8660254037844387, 0.0, 0.0, 0.5)))),
          new AprilTag(11,
              new Pose3d(
                  11.904726,
                  3.7132259999999997,
                  1.3208,
                  new Rotation3d(
                      new Quaternion(-0.8660254037844387, 0.0, 0.0, 0.5)))),
          new AprilTag(12,
              new Pose3d(
                  11.904726,
                  4.49834,
                  1.3208,
                  new Rotation3d(
                      new Quaternion(0.8660254037844387, 0.0, 0.0, 0.5)))),
          new AprilTag(13,
              new Pose3d(
                  11.220196,
                  4.105148,
                  1.3208,
                  new Rotation3d(
                      new Quaternion(0.0, 0.0, 0.0, 1.0)))),
          new AprilTag(14,
              new Pose3d(
                  5.320792,
                  4.105148,
                  1.3208,
                  new Rotation3d(
                      new Quaternion(1.0, 0.0, 0.0, 0.0)))),
          new AprilTag(15,
              new Pose3d(
                  4.641342,
                  4.49834,
                  1.3208,
                  new Rotation3d(
                      new Quaternion(0.5, 0.0, 0.0, 0.8660254037844386)))),
          new AprilTag(16,
              new Pose3d(
                  4.641342,
                  3.7132259999999997,
                  1.3208,
                  new Rotation3d(
                      new Quaternion(-0.5, 0.0, 0.0, 0.8660254037844387))))),
      fieldLength, fieldWidth) : null;
}
