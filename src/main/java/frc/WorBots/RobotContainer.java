// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.WorBots;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.WorBots.commands.*;
import frc.WorBots.subsystems.drive.*;
import frc.WorBots.subsystems.superstructure.*;
import frc.WorBots.subsystems.vision.*;
import frc.WorBots.util.*;

public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;
  private Superstructure superstructure;

  // Joysticks
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    if (!Constants.getSim()) { // Real Robot
      drive = new Drive(new GyroIOPigeon(), new ModuleIOKraken(0), new ModuleIOKraken(1), new ModuleIOKraken(2),
          new ModuleIOKraken(3));
      vision = new Vision(new VisionIOCustom(0), new VisionIOCustom(1));
      superstructure = new Superstructure(new SuperstructureIOTalon());
    } else { // Sim
      drive = new Drive(new GyroIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
      vision = new Vision(new VisionIOCustom(0));
      superstructure = new Superstructure(new SuperstructureIOSim());
    }

    var autoCommands = new AutoCommands(drive);

    vision.setDataInterfaces(drive::addVisionData, drive::getPose);
    bindControls();
  }

  private void bindControls() {
    StatusPage.reportStatus(StatusPage.DRIVE_CONTROLLER, driver.getHID().isConnected());
    drive.setDefaultCommand(new DriveWithJoysticks(drive,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> -driver.getRightX()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("AAAAAAAAAA");
  }
}
