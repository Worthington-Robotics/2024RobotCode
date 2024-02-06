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
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.WorBots.util.*;
import java.util.function.*;

public class AutoShoot extends SequentialCommandGroup {
  // Locations
  private double speakerOpeningHeightZ;
  private double speakerOpeningCenterY;
  private static final double shootingLineX = 3.0;

  /**
   * This command automatically drives to a known safe shooting location and shoots a game piece.
   *
   * @param superstructure The superstructure subsystem.
   * @param drive The drive subsystem.
   */
  public AutoShoot(Superstructure superstructure, Drive drive) {
    addRequirements(superstructure);
    speakerOpeningHeightZ =
        (FieldConstants.Speaker.openingHeightHigher - FieldConstants.Speaker.openingHeightLower)
            / 2;
    speakerOpeningCenterY = (FieldConstants.Speaker.speakerY);

    Supplier<Pose2d> driveTargetSupplier =
        () -> {
          // Robot Pose
          Pose2d robotPose = drive.getPose();
          Pose2d flippedRobotPose = AllianceFlipUtil.apply(robotPose);

          // Calculates the shooting angle
          double opposite =
              (FieldConstants.Speaker.openingHeightLower + speakerOpeningHeightZ)
                  - superstructure.getShooterHeightMeters();
          superstructure.setShootingAngleRad(() -> Math.atan2(opposite, flippedRobotPose.getX()));

          // Calculates the robot shooting rotation
          double robotAngle = getRobotRotationToShoot(robotPose).getRadians();

          if (flippedRobotPose.getX()
              > FieldConstants.Wing.endX) { // if robot is outside of the wing
            if (flippedRobotPose.getY()
                < FieldConstants.Stage.foot3Center.getY()) { // if robot is right of stage
              return new Pose2d(
                  AllianceFlipUtil.apply(FieldConstants.Wing.endX), 1, new Rotation2d(robotAngle));
            } else if (flippedRobotPose.getY() < FieldConstants.Stage.foot2Center.getY()
                && flippedRobotPose.getY()
                    > FieldConstants.Stage.foot3Center
                        .getY()) { // if robot is right of center, but not right of stage
              return new Pose2d(
                  AllianceFlipUtil.apply(FieldConstants.Wing.endX),
                  FieldConstants.Stage.center.getY(),
                  new Rotation2d(robotAngle));
            } else { // if robot is left of stage
              return new Pose2d(
                  AllianceFlipUtil.apply(FieldConstants.Wing.endX), 7, new Rotation2d(robotAngle));
            }
          } else {
            return new Pose2d(robotPose.getX(), robotPose.getY(), new Rotation2d(robotAngle));
          }
        };
    var driveToPose = new DriveToPose(drive, driveTargetSupplier);
    addCommands(
        superstructure.setMode(SuperstructureState.SHOOTING),
        driveToPose,
        Commands.waitUntil(() -> false)
            .finallyDo(() -> superstructure.setModeVoid(SuperstructureState.POSE)));
  }

  /**
   * Calculates the needed robot rotation in order to shoot into the speaker for a given pose,
   * automatically factors in alliance.
   *
   * @param robotPose The desired pose to be shot from, ignores rotation.
   * @return The rotation of the robot needed to shoot at the provided pose.
   */
  public Rotation2d getRobotRotationToShoot(Pose2d robotPose) {
    Alliance currentAlliance = DriverStation.getAlliance().get();
    double translatedX =
        (currentAlliance == Alliance.Red
            ? robotPose.getX() - FieldConstants.fieldLength
            : robotPose.getX());
    double robotY = robotPose.getY();
    double robotAngle = Math.atan2(robotY - (speakerOpeningCenterY), translatedX);
    return new Rotation2d(robotAngle);
  }
}
