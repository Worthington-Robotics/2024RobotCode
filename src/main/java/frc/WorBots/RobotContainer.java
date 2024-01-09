// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.WorBots;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.WorBots.commands.DriveWithJoysticks;
import frc.WorBots.subsystems.drive.*;
import frc.WorBots.util.Logger;
import frc.WorBots.util.StatusPage;

public class RobotContainer {
  // Subsystems
  private Drive drive;

  // Joysticks
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandJoystick operator = new CommandJoystick(0);
  
  public RobotContainer() {
    if (!Constants.getSim()) { // Real Robot
      drive = new Drive(new GyroIOPigeon2(), new ModuleIOKraken(0), new ModuleIOKraken(1), new ModuleIOKraken(2), new ModuleIOKraken(3));
    } else { // Sim
      drive = new Drive(new GyroIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
    }
    bindControls();
    SmartDashboard.putNumberArray("April Pose 1", Logger.pose3dToArray(FieldConstants.aprilTags.getTagPose(1).get()));
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
