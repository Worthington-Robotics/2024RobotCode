// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.util.*;
import java.util.function.*;

public class AutoShoot extends SequentialCommandGroup {
  // Locations
  private double speakerOpeningHeightZ;
  private double speakerOpeningCenterY;
  private double forwardLineX;

  public AutoShoot(Superstructure superstructure, Drive drive) {
    addRequirements(superstructure);
    speakerOpeningHeightZ =
        (FieldConstants.Speaker.openingHeightHigher - FieldConstants.Speaker.openingHeightLower)
            / 2;
    speakerOpeningCenterY = (FieldConstants.Speaker.speakerY);
    forwardLineX = FieldConstants.Stage.foot1Center.getX();

    Supplier<Pose2d> driveTargetSupplier =
        () -> {
          // Robot Pose
          Pose2d robotPose = drive.getPose();
          Pose2d flippedRobotPose = AllianceFlipUtil.apply(robotPose);
          Alliance currentAlliance = DriverStation.getAlliance().get();

          // Calculates the shooting angle
          double adjascent = robotPose.getX();
          double opposite = FieldConstants.Speaker.openingHeightLower + speakerOpeningHeightZ;
          superstructure.setShootingAngleRad(() -> Math.atan2(opposite, adjascent));

          // Calculates the robot shooting rotation
          double robotAngle;
          double robotY = robotPose.getY();
          if (currentAlliance == Alliance.Red) {
            robotAngle =
                Math.atan2(
                    robotY - (speakerOpeningCenterY), adjascent - FieldConstants.fieldLength);
          } else {
            robotAngle = Math.atan2(robotY - (speakerOpeningCenterY), adjascent);
          }

          if (flippedRobotPose.getX()
              > FieldConstants.Wing.endX) { // if robot is outside of the wing
            if (flippedRobotPose.getY()
                < FieldConstants.Stage.center.getY()) { // if its on the right side from blue
              return new Pose2d(
                  AllianceFlipUtil.apply(FieldConstants.Wing.endX),
                  robotY,
                  new Rotation2d(robotAngle));
            } else { // if its on the left side relative to blue
              return new Pose2d(
                  AllianceFlipUtil.apply(FieldConstants.Wing.endX),
                  robotY,
                  new Rotation2d(robotAngle));
            }
          } else {
            return new Pose2d(robotPose.getX(), robotPose.getY(), new Rotation2d(robotAngle));
          }
        };
    var driveToPose = new DriveToPose(drive, driveTargetSupplier);
    addCommands(driveToPose);
  }
}
