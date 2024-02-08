// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots;

import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.WorBots.util.debug.Logger;
import java.io.IOException;

/**
 * Physical constants for the field.
 *
 * <p>Values are sourced from the layout diagram, field element drawings, AprilTag positions, and
 * physical measurements
 *
 * <p>Units are in meters and radians. Some measurements in the field manual are incorrect,
 * reference drawings
 */
public class FieldConstants {
  public static final boolean isWPIField = false;

  public static final double fieldLength = Units.inchesToMeters(651.25); // 16.451 m
  public static final double fieldWidth = Units.inchesToMeters(323.25); // 8.211 m
  public static final double tapeWidth = Units.inchesToMeters(2.0);
  public static final double midLineY = fieldWidth / 2.0;
  public static final double midLineX = fieldLength / 2.0;

  /** Logs the current poses of field pieces that are defined in code */
  public static void testField() {
    Logger.getInstance()
        .logTranslation2d("Field", "Starting note wing 1", GamePieces.wingPieces[0]);
    Logger.getInstance()
        .logTranslation2d("Field", "Starting note wing 2", GamePieces.wingPieces[1]);
    Logger.getInstance()
        .logTranslation2d("Field", "Starting note wing 3", GamePieces.wingPieces[2]);
    Logger.getInstance()
        .logTranslation2d("Field", "Starting note far center", GamePieces.centerPieces[0]);
    Logger.getInstance()
        .logTranslation2d("Field", "Starting note far high 1", GamePieces.centerPieces[1]);
    Logger.getInstance()
        .logTranslation2d("Field", "Starting note far high 2", GamePieces.centerPieces[2]);
    Logger.getInstance()
        .logTranslation2d("Field", "Starting note far low 1", GamePieces.centerPieces[3]);
    Logger.getInstance()
        .logTranslation2d("Field", "Starting note far low 2", GamePieces.centerPieces[4]);
    Logger.getInstance().logTranslation2d("Field", "Stage foot 1 center", Stage.foot1Center);
    Logger.getInstance().logTranslation2d("Field", "Stage foot 2 center", Stage.foot2Center);
    Logger.getInstance().logTranslation2d("Field", "Stage foot 3 center", Stage.foot3Center);
    Logger.getInstance().logTranslation2d("Field", "Stage center", Stage.center);
    Logger.getInstance().logTranslation2d("Field", "Subwoofer corner 1", Speaker.regionCorners[0]);
    Logger.getInstance().logTranslation2d("Field", "Subwoofer corner 2", Speaker.regionCorners[1]);
    Logger.getInstance().logTranslation2d("Field", "Subwoofer corner 3", Speaker.regionCorners[2]);
    Logger.getInstance().logTranslation2d("Field", "Subwoofer corner 4", Speaker.regionCorners[3]);
    Logger.getInstance()
        .logTranslation3d("Field", "Speaker opening corner 1", Speaker.openingCorners[0]);
    Logger.getInstance()
        .logTranslation3d("Field", "Speaker opening corner 2", Speaker.openingCorners[1]);
    Logger.getInstance()
        .logTranslation3d("Field", "Speaker opening corner 3", Speaker.openingCorners[2]);
    Logger.getInstance()
        .logTranslation3d("Field", "Speaker opening corner 4", Speaker.openingCorners[3]);
    Logger.getInstance().logTranslation2d("Field", "Source center", Source.center);
    for (var tag : aprilTags.getTags()) {
      Logger.getInstance().logPose3d("Field", "AprilTag " + tag.ID, tag.pose);
    }
  }

  /** The wing of the game field. */
  public static final class Wing {
    public static final double startX = 0;
    public static final double endX = Units.inchesToMeters(231.2);
    public static final double startY = fieldWidth;
    public static final double endY = 0;

    public static final Translation2d[] regionCorners =
        new Translation2d[] {
          new Translation2d(startX, startY),
          new Translation2d(startX, endY),
          new Translation2d(endX, endY),
          new Translation2d(endX, startY)
        };
  }

  /** The region in which robots start, also houses the amp and speaker. */
  public static final class StartingZone {
    public static final double startX = 0;
    public static final double endX = Units.inchesToMeters(76.1);
    public static final double startY = fieldWidth - Units.inchesToMeters(18);
    public static final double midY = startY - Units.inchesToMeters((12.0 * 23.0) + 8.125);
    public static final double endY = 0;

    /** The corners of the starting zone. */
    public static final Translation2d[] regionCorners =
        new Translation2d[] {
          new Translation2d(startX, startY),
          new Translation2d(startX, midY),
          new Translation2d(endX, endY),
          new Translation2d(endX, startY)
        };
  }

  /** The constants of the Amp */
  public static final class Amp {
    /** Source: Game Manual */
    public static final double openingBottomZ = Units.inchesToMeters(26);

    public static final double openingTopZ = openingBottomZ + Units.inchesToMeters(18);
    public static final double faceStartX = Units.inchesToMeters(49.5);
    public static final double openingStartX = faceStartX + Units.inchesToMeters(11);
    public static final double openingEndX = openingStartX + Units.feetToMeters(2);
    public static final Translation3d test =
        new Translation3d(openingStartX, fieldWidth, openingBottomZ);

    public static final double bottomY = fieldWidth;
    public static final double topY = bottomY - Units.inchesToMeters(18);
    public static final double startX = 0;
    public static final double endX = Units.feetToMeters(10);
    public static final Translation2d[] regionCorners =
        new Translation2d[] {
          new Translation2d(startX, bottomY),
          new Translation2d(startX, topY),
          new Translation2d(endX, topY),
          new Translation2d(endX, bottomY)
        };
  }

  public static final class Speaker {
    public static final double speakerY = fieldWidth - Units.inchesToMeters(103.875);
    public static final Translation2d position = new Translation2d(0, speakerY);
    public static final double subwooferDepth = Units.inchesToMeters(36.37);
    public static final double subwooferSideLength = Units.inchesToMeters(41);
    public static final double subwooferBackLength = Units.inchesToMeters(77.96);
    public static final Translation2d[] regionCorners =
        new Translation2d[] {
          new Translation2d(subwooferDepth, speakerY + subwooferSideLength / 2),
          new Translation2d(subwooferDepth, speakerY - subwooferSideLength / 2),
          new Translation2d(0, speakerY + subwooferBackLength / 2),
          new Translation2d(0, speakerY - subwooferBackLength / 2),
        };
    public static final double subwooferBaseHeight = Units.inchesToMeters(8.375);

    public static final double openingHeightLower = Units.inchesToMeters(78);
    public static final double openingHeightHigher = Units.inchesToMeters(82.875);
    public static final double openingHeightCenter = (openingHeightHigher - openingHeightLower) / 2;
    public static final double openingWidth = Units.inchesToMeters(41.375);
    // Distance from the wall to the end of the opening. Acts as the base of the
    // right triangle formed by the opening. Game manual is wrong and says 1'16"
    public static final double openingDepth = Units.inchesToMeters(18);
    public static final Translation3d[] openingCorners =
        new Translation3d[] {
          // Back corners
          new Translation3d(0, speakerY - openingWidth / 2, openingHeightLower),
          new Translation3d(0, speakerY + openingWidth / 2, openingHeightLower),
          // Front corners
          new Translation3d(openingDepth, speakerY - openingWidth / 2, openingHeightHigher),
          new Translation3d(openingDepth, speakerY + openingWidth / 2, openingHeightHigher),
        };
  }

  public static final class Source {
    // The angle of the source wall to the field wall
    public static final double wallAngle = 2.0943951;
    public static final double chuteBottomHeight = Units.inchesToMeters(36.71);

    // Midpoint of both source apriltags
    public static final Translation2d center =
        new Translation2d(
            (15.079471999999997 + 16.185134) / 2,
            (fieldWidth - 0.24587199999999998 + fieldWidth - 0.883666) / 2);
  }

  public static final class Stage {
    public static final double footWidth = Units.inchesToMeters(24.0);
    public static final double trussToTrussDistance = Units.inchesToMeters(85.9);
    public static final double trussDistanceAtAngle = Units.feetToMeters((Math.sqrt(3) + 1) / 2);
    public static final double footToFootDistance = trussToTrussDistance + trussDistanceAtAngle;
    // The altitude of the equilateral triangle formed by the feet
    private static final double temp = footToFootDistance / 2 * Math.sqrt(3);

    public static final Translation2d foot1Center =
        new Translation2d(Units.inchesToMeters(121.0) + footWidth / 2, fieldWidth / 2);
    public static final Translation2d foot2Center =
        foot1Center.plus(new Translation2d(temp, footToFootDistance / 2));
    public static final Translation2d foot3Center =
        foot1Center.plus(new Translation2d(temp, footToFootDistance / -2));

    public static final Translation2d center = foot1Center.plus(new Translation2d(temp / 2, 0));

    public static final double verticalClearance = Units.inchesToMeters(27.83);
    public static final double trapOpeningBottomHeight = Units.inchesToMeters(56.5);
  }

  public static final class GamePieces {
    public static final double startX = 0;
    public static final double wingX = startX + Units.inchesToMeters(114);
    public static final double secondPieceY = midLineY + Units.inchesToMeters(57);
    public static final double thirdPieceY = secondPieceY + Units.inchesToMeters(57);

    public static final double firstMidGamePieceY = fieldWidth - Units.inchesToMeters(29.64);
    public static final double secondMidGamePieceY = firstMidGamePieceY - Units.inchesToMeters(66);
    public static final double thirdMidGamePieceY = secondMidGamePieceY - Units.inchesToMeters(66);
    public static final double fourthMidGamePieceY = thirdMidGamePieceY - Units.inchesToMeters(66);
    public static final double fifthMidGamePieceY = fourthMidGamePieceY - Units.inchesToMeters(66);

    public static final Translation2d[] wingPieces =
        new Translation2d[] {
          new Translation2d(wingX, thirdPieceY),
          new Translation2d(wingX, secondPieceY),
          new Translation2d(wingX, midLineY),
        };

    public static final Translation2d[] centerPieces =
        new Translation2d[] {
          new Translation2d(midLineX, firstMidGamePieceY),
          new Translation2d(midLineX, secondMidGamePieceY),
          new Translation2d(midLineX, thirdMidGamePieceY),
          new Translation2d(midLineX, fourthMidGamePieceY),
          new Translation2d(midLineX, fifthMidGamePieceY)
        };
  }

  public static final class Note {
    public static double insideDiameter = Units.inchesToMeters(10);
    public static double outsideDiameter = Units.inchesToMeters(14);
    public static double thickness = Units.inchesToMeters(2);
  }

  // April Tags
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  public static final AprilTagFieldLayout aprilTags;

  static {
    try {
      aprilTags = AprilTagFieldLayout.loadFromResource(k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}
