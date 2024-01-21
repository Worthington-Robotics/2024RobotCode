package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
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
    double opposite = FieldConstants.Speaker.openingHeightLower;
    superstructure.setShootingAngleRad(() -> Math.atan2(opposite, adjascent));

    double robotError = robotPose.get().getY() - FieldConstants.Speaker.openingCorners[0].getY();
    double robotAngle = Math.hypot(adjascent, robotError);
  }

  public void end(boolean interrupted) {
    superstructure.setMode(SuperstructureState.POSE);
  }
}
