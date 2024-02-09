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
import frc.WorBots.util.debug.StatusPage;
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
    superstructure.setDefaultCommand(
        new SuperstructureManual(
            superstructure, () -> -operator.getLeftY(), () -> -operator.getRightY()));

    driver.leftBumper().onTrue(new Turn90(drive, false));
    driver.rightBumper().onTrue(new Turn90(drive, true));
    driver.leftTrigger().whileTrue(intake.intakeRaw());
    driver
        .rightTrigger()
        .whileTrue(
            intake
                .spitRaw()
                .alongWith(
                    Commands.run(
                        () -> {
                          drive.runVelocity(new ChassisSpeeds(-0.65, 0.0, 0.0));
                        },
                        drive)));
    driver.b().whileTrue(intake.spitRaw());
    driver.y().onTrue(Commands.runOnce(() -> drive.resetHeading(), drive));
    driver.povRight().whileTrue(PoseCommands.fullZero(drive, superstructure));
    // operator
    // .y()
    // .toggleOnTrue(
    // new DriverShootingTest(
    // superstructure,
    // shooter,
    // () -> -operator.getLeftY(),
    // () -> -operator.getRightY(),
    // () -> operator.rightTrigger().getAsBoolean()));
    // operator
    // .x()
    // .toggleOnTrue(
    // Commands.startEnd(() -> inClimbingMode = true, () -> inClimbingMode = false)
    // .raceWith(new DriverClimb(superstructure, () -> -operator.getRightY())));
    operator.a().onTrue(superstructure.setPose(Preset.HOME));
    operator.b().onTrue(PoseCommands.amp(drive, superstructure));
    operator.x().onTrue(PoseCommands.slide(drive, superstructure));

    operator
        .leftTrigger()
        .whileTrue(
            new DriverShoot(
                drive,
                superstructure,
                () -> -driver.getLeftX(),
                () -> -driver.getLeftY(),
                () -> -operator.getLeftY(),
                () -> -operator.getRightY()));
    HashMap<String, Command> shootMap = new HashMap<String, Command>();
    shootMap.put("shoot", shooter.shootCommand(2000));
    shootMap.put("amp", shooter.shootCommand(500));
    shootMap.put("slide", shooter.spinToSpeed(-300, -300));
    shootMap.put("raw", shooter.spinToSpeed(2000, 2000));
    operator
        .rightTrigger()
        .whileTrue(
            Commands.select(
                shootMap,
                () -> {
                  if (superstructure.isInPose(Preset.AMP)) {
                    return "shoot";
                  }
                  if (superstructure.isInPose(Preset.SLIDE)) {
                    return "slide";
                  }
                  return "shoot";
                }));
    // driver.rightBumper().onTrue(superstructure.setPose(Preset.AMP));
    // driver.povLeft().whileTrue(superstructure.autoZero());
    operator.povUp().onTrue(PoseCommands.autoClimb(drive, superstructure).onlyIf(() -> true));
    operator.povDown().onTrue(PoseCommands.climbDown(drive, superstructure).onlyIf(() -> true));
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
