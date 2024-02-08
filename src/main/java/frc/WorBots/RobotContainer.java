// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
        "Mobility",
        List.of(
            new AutoQuestion(
                "Starting Location?",
                List.of(
                    AutoQuestionResponse.AMP_SIDE,
                    AutoQuestionResponse.CENTER,
                    AutoQuestionResponse.WALL_SIDE))),
        autoCommands.mobility());

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

    vision.setDataInterfaces(drive::addVisionData, drive::getPose);
    bindControls();
  }

  /** Bind driver controls to commands */
  private void bindControls() {
    StatusPage.reportStatus(StatusPage.DRIVE_CONTROLLER, driver.getHID().isConnected());
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightY(),
            () -> -driver.getRightX()));
    driver.a().whileTrue(intake.intakeRaw());
    driver
        .b()
        .whileTrue(
            intake
                .spitRaw()
                .alongWith(
                    Commands.run(
                        () -> {
                          drive.runVelocity(new ChassisSpeeds(-0.5, 0.0, 0.0));
                        },
                        drive)));
    // driver.b().whileTrue(intake.spitRaw());
    driver.y().onTrue(Commands.runOnce(() -> drive.resetHeading(), drive));
    driver.x().toggleOnTrue(new DriverClimb(superstructure, () -> -driver.getRightY()));
    driver.leftBumper().onTrue(superstructure.setPose(Preset.HOME));
    driver.rightBumper().onTrue(PoseCommands.amp(drive, superstructure));
    // driver.rightBumper().onTrue(superstructure.setPose(Preset.AMP));
    driver.povUp().onTrue(superstructure.autoZero());
    // driver.leftBumper().whileTrue(elevator.setDemandCommand(-0.5));
    // driver.rightBumper().whileTrue(elevator.setDemandCommand(0.5));
    // driver.a().toggleOnTrue(new AutoShoot(superstructure, drive));
    // operator
    // .a()
    // .toggleOnTrue(
    // new DriverShoot(
    // drive,
    // superstructure,
    // () -> -driver.getLeftY(),
    // () -> -driver.getLeftX(),
    // () -> -operator.getLeftY(),
    // () -> -operator.getLeftX()));
  }

  public Command getAutonomousCommand() {
    return selector.getCommand();
  }
}
