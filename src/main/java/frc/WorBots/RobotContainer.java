// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.WorBots.commands.*;
import frc.WorBots.subsystems.drive.*;
import frc.WorBots.subsystems.intake.*;
import frc.WorBots.subsystems.shooter.*;
import frc.WorBots.subsystems.superstructure.*;
import frc.WorBots.subsystems.vision.*;
import frc.WorBots.util.*;
import java.util.*;

public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;
  private Superstructure superstructure;
  private Intake intake;
  private Shooter shooter;
  private AutoSelector selector;

  // Joysticks
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    if (!Constants.getSim()) { // Real Robot
      drive =
          new Drive(
              new GyroIOPigeon(),
              new ModuleIOKraken(0),
              new ModuleIOKraken(1),
              new ModuleIOKraken(2),
              new ModuleIOKraken(3));
      vision = new Vision(new VisionIOCustom(0), new VisionIOCustom(1));
      superstructure = new Superstructure(new SuperstructureIOTalon());
      intake = new Intake(new IntakeIOTalon());
      shooter = new Shooter(new ShooterIOTalon());
    } else { // Sim
      drive =
          new Drive(
              new GyroIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim());
      vision = new Vision(new VisionIOCustom(0));
      superstructure = new Superstructure(new SuperstructureIOSim());
      intake = new Intake(new IntakeIOSim());
      shooter = new Shooter(new ShooterIOSim());
    }
    selector = new AutoSelector("Auto Selector");
    var autoCommands =
        new AutoCommands(drive, superstructure, intake, shooter, selector::getResponses);

    selector.addRoutine("TESTING", List.of(), autoCommands.onePiece());

    vision.setDataInterfaces(drive::addVisionData, drive::getPose);
    bindControls();
  }

  /** Bind driver controls to commands */
  private void bindControls() {
    StatusPage.reportStatus(StatusPage.DRIVE_CONTROLLER, driver.getHID().isConnected());
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    driver.a().toggleOnTrue(new AutoShoot(superstructure, drive));
    operator
        .a()
        .toggleOnTrue(
            new DriverShoot(
                drive,
                superstructure,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -operator.getLeftY(),
                () -> -operator.getLeftX()));
  }

  public Command getAutonomousCommand() {
    return selector.getCommand();
  }
}