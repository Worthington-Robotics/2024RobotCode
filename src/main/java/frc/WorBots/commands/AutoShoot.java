package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.superstructure.Superstructure;

public class AutoShoot extends Command {
  private Supplier<Pose2d> robotPose;
  private double openingHeight;

  public AutoShoot(Supplier<Pose2d> robotPose, Superstructure superstructure) {
    this.robotPose = robotPose;
    openingHeight = (FieldConstants.Speaker.openingHeightHigher - FieldConstants.Speaker.openingHeightLower) / 2;
  }

  public void execute() {
    double adjascent = robotPose.get().getX();
    double opposite = openingHeight;
    
  }
}
