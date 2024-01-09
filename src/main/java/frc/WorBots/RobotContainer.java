// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.WorBots;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.WorBots.commands.DriveWithJoysticks;
import frc.WorBots.subsystems.drive.*;
import frc.WorBots.subsystems.vision.*;
import frc.WorBots.util.GeomUtil;
import frc.WorBots.util.Logger;
import frc.WorBots.util.StatusPage;

public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;

  // Joysticks
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandJoystick operator = new CommandJoystick(0);
  
  public RobotContainer() {
    if (!Constants.getSim()) { // Real Robot
      drive = new Drive(new GyroIOPigeon2(), new ModuleIOKraken(0), new ModuleIOKraken(1), new ModuleIOKraken(2), new ModuleIOKraken(3));
      vision = new Vision(new VisionIOCustom(0), new VisionIOCustom(1));
    } else { // Sim
      drive = new Drive(new GyroIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
      vision = new Vision(new VisionIOCustom(0));
    }
    vision.setDataInterfaces(drive::addVisionData, drive::getPose);
    bindControls();
    SmartDashboard.putNumberArray("POSE0", Logger.pose2dToArray(GeomUtil.translationToPose(FieldConstants.GamePieces.wingPieces[1])));
    SmartDashboard.putNumberArray("POSE1", Logger.pose2dToArray(GeomUtil.translationToPose(FieldConstants.StartingZone.regionCorners[1])));
    SmartDashboard.putNumberArray("POSE2", Logger.pose2dToArray(GeomUtil.translationToPose(FieldConstants.StartingZone.regionCorners[2])));
    SmartDashboard.putNumberArray("POSE3", Logger.pose2dToArray(GeomUtil.translationToPose(FieldConstants.StartingZone.regionCorners[3])));
  }

  private void bindControls() {
    StatusPage.reportStatus(StatusPage.DRIVE_CONTROLLER, driver.getHID().isConnected());
    drive.setDefaultCommand(new DriveWithJoysticks(drive, 
    () -> -driver.getLeftY(), 
    () -> -driver.getLeftX(), 
    () -> -driver.getRightX()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
