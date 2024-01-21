package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;

public class AutoShoot extends Command {
  private Supplier<Pose2d> robotPose;
  private Superstructure superstructure;
  private double openingHeight;

  public AutoShoot(Supplier<Pose2d> robotPose, Superstructure superstructure) {
    this.robotPose = robotPose;
    this.superstructure = superstructure;
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
  }

  @Override
  public void end(boolean interrupted) {
    superstructure.setMode(SuperstructureState.POSE);
  }
}
