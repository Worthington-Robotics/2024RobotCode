package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;

public class AutoShoot extends Command {
  private Supplier<Pose2d> robotPose;
  private Supplier<Double> joystickLeftX;
  private Supplier<Double> joystickLeftY;
  private Superstructure superstructure;
  private Drive drive;
  private double openingHeight;

  public AutoShoot(Supplier<Pose2d> robotPose, Supplier<Double> joystickLeftX, Supplier<Double> joystickLeftY,
      Superstructure superstructure, Drive drive) {
    addRequirements(drive, superstructure);
    this.robotPose = robotPose;
    this.superstructure = superstructure;
    this.drive = drive;
    this.joystickLeftX = joystickLeftX;
    this.joystickLeftY = joystickLeftY;
    openingHeight = (FieldConstants.Speaker.openingHeightHigher - FieldConstants.Speaker.openingHeightLower) / 2;
  }

  @Override
  public void initialize() {
    superstructure.setMode(SuperstructureState.SHOOTING);
  }

  public void execute() {
    double adjascent = robotPose.get().getX();
    double opposite = FieldConstants.Speaker.openingHeightLower + openingHeight;
    superstructure.setShootingAngleRad(() -> Math.atan2(opposite, adjascent));

    double robotError = robotPose.get().getY() - FieldConstants.Speaker.openingCorners[0].getY();
    double robotAngle = Math.hypot(adjascent, robotError);

    double leftX = joystickLeftX.get();
    double leftY = joystickLeftY.get();

    double linearMagnitude = Math.hypot(leftX, leftY);
    Rotation2d linearDirection = new Rotation2d(leftX, leftY);

    MathUtil.applyDeadband(linearMagnitude, 0.05);

    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);

    Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(new Translation2d(linearMagnitude, new Rotation2d()), new Rotation2d()))
        .getTranslation();

    ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(), 0.0);
  }

  public void end(boolean interrupted) {
    superstructure.setMode(SuperstructureState.POSE);
  }
}
