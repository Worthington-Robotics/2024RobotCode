package frc.WorBots.commands;

import java.util.function.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.WorBots.util.*;

public class AutoShoot extends SequentialCommandGroup {
  // Locations
  private double speakerOpeningHeightZ;
  private double speakerOpeningCenterY;
  private double forwardLineY;

  public AutoShoot(Superstructure superstructure, Drive drive) {
    addRequirements(superstructure);
    speakerOpeningHeightZ = (FieldConstants.Speaker.openingHeightHigher - FieldConstants.Speaker.openingHeightLower)
        / 2;
    speakerOpeningCenterY = (FieldConstants.Speaker.speakerY);
    forwardLineY = FieldConstants.Stage.foot1Center.getY();

    Supplier<Pose2d> driveTargetSupplier = () -> {
      // Robot Pose
      Pose2d robotPose = drive.getPose();

      // Calculates the shooting angle
      double adjascent = AllianceFlipUtil.apply(robotPose).getX();
      double opposite = FieldConstants.Speaker.openingHeightLower + speakerOpeningHeightZ;
      superstructure.setShootingAngleRad(() -> Math.atan2(opposite, adjascent));

      // Calculates the robot shooting rotation
      double robotAngle;
      double robotY = robotPose.getY();
      robotAngle = Math.atan2(robotY - (speakerOpeningCenterY), adjascent) + Math.PI;

      if (robotPose.getY() > forwardLineY) {

      }
      return new Pose2d(robotPose.getX(), robotPose.getY(), new Rotation2d(robotAngle));
    };
    var driveToPose = new DriveToPose(drive, driveTargetSupplier);
    addCommands(driveToPose);
  }
}
