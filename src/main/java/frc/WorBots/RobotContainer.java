// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.WorBots.AutoSelector.*;
import frc.WorBots.commands.*;
import frc.WorBots.subsystems.drive.*;
import frc.WorBots.subsystems.intake.*;
import frc.WorBots.subsystems.shooter.*;
import frc.WorBots.subsystems.superstructure.*;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.Preset;
import frc.WorBots.subsystems.vision.*;
import frc.WorBots.util.debug.StatusPage;
import java.util.*;

public class RobotContainer {
  // Subsystems
  public Drive drive;
  private Vision vision;
  public Superstructure superstructure;
  public Intake intake;
  public Shooter shooter;
  private AutoSelector selector;

  // Joysticks
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  /**
   * The robot container houses the joystics, and subsystems of the robot, as well as getting the
   * autonomous command.
   */
  public RobotContainer() {
    if (!Constants.getSim()) { // Real Robot
      drive =
          new Drive(
              new GyroIOPigeon2(),
              new ModuleIOTalon(0),
              new ModuleIOTalon(1),
              new ModuleIOTalon(2),
              new ModuleIOTalon(3));
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

    selector.addRoutine(
        "One Piece",
        List.of(
            new AutoQuestion(
                "Starting Location?",
                List.of(
                    AutoQuestionResponse.AMP_SIDE,
                    AutoQuestionResponse.CENTER,
                    AutoQuestionResponse.WALL_SIDE))),
        autoCommands.onePiece());

    selector.addRoutine(
        "Mobility",
        List.of(
            new AutoQuestion(
                "Starting Location?",
                List.of(
                    AutoQuestionResponse.AMP_SIDE,
                    AutoQuestionResponse.CENTER,
                    AutoQuestionResponse.WALL_SIDE))),
        autoCommands.mobility());

    selector.addRoutine("Do Nothing", List.of(), Commands.none());

    vision.setDataInterfaces(drive::addVisionData, drive::getPose);
    bindControls();
  }

  /** Bind driver controls to commands */
  private void bindControls() {
    StatusPage.reportStatus(StatusPage.DRIVE_CONTROLLER, driver.getHID().isConnected());
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    // superstructure.setDefaultCommand(new SuperstructureManual(superstructure, () ->
    // -operator.getLeftY(), () -> -operator.getRightY()));

    driver
        .leftTrigger()
        .whileTrue(intake.spitRaw().alongWith(shooter.setRawFeederVoltsCommand(1.2)))
        .onFalse(shooter.setRawFeederVoltsCommand(0.0));
    driver.leftBumper().onTrue(superstructure.setPose(Preset.STOW));
    driver.rightTrigger().whileTrue(new Handoff(intake, superstructure, shooter));
    driver.rightBumper().onTrue(superstructure.setPose(Preset.HANDOFF));
    driver.a().onTrue(superstructure.setPose(Preset.PIVOTTOTOP));
    driver.povUp().onTrue(shooter.spinToSpeed(5800)).onFalse(shooter.spinToSpeed(0));
    driver.povRight().onTrue(shooter.spinToSpeed(2500)).onFalse(shooter.spinToSpeed(0));
    driver
        .povDown()
        .toggleOnTrue(
            new SuperstructureManual(
                superstructure, () -> -operator.getLeftY(), () -> -operator.getRightY()));
    operator.b().onTrue(superstructure.setPose(Preset.STOW));
    operator.y().onTrue(superstructure.setPose(Preset.HANDOFF));
    operator.a().onTrue(superstructure.setPose(Preset.TRAP));
    operator
        .x()
        .whileTrue(superstructure.setPose(Preset.AMP).alongWith(shooter.spinToSpeed(500)))
        .onFalse(shooter.spinToSpeed(0.0));
    operator
        .rightBumper()
        .whileTrue(
            new AutoShoot(
                superstructure, drive, shooter, () -> -driver.getLeftY(), () -> -driver.getLeftX()))
        .onFalse(shooter.spinToSpeed(0.0));
    operator
        .leftBumper()
        .onTrue(shooter.setRawFeederVoltsCommand(-2))
        .onFalse(shooter.setRawFeederVoltsCommand(0.0));
  }

  public Command getAutonomousCommand() {
    return selector.getCommand();
  }
}
