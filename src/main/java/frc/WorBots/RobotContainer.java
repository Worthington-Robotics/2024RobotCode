// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.WorBots.auto.AutoSelector;
import frc.WorBots.auto.AutoSelector.*;
import frc.WorBots.auto.Autos;
import frc.WorBots.auto.DebugRoutines;
import frc.WorBots.commands.*;
import frc.WorBots.subsystems.drive.*;
import frc.WorBots.subsystems.intake.*;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.subsystems.lights.Lights.LightsMode;
import frc.WorBots.subsystems.shooter.*;
import frc.WorBots.subsystems.superstructure.*;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.WorBots.subsystems.superstructure.SuperstructurePose.Preset;
import frc.WorBots.subsystems.vision.*;
import frc.WorBots.util.RobotSimulator;
import frc.WorBots.util.math.AllianceFlipUtil;
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
  public final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController operator = new CommandXboxController(1);

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
              new ModuleIOSim(0),
              new ModuleIOSim(1),
              new ModuleIOSim(2),
              new ModuleIOSim(3));
      vision = new Vision(new VisionIOCustom(0));
      superstructure = new Superstructure(new SuperstructureIOSim());
      intake = new Intake(new IntakeIOSim());
      shooter = new Shooter(new ShooterIOSim());
    }
    selector = new AutoSelector("Auto Selector 2");
    final Autos autos = new Autos(drive, superstructure, intake, shooter, selector::getResponses);

    selector.addRoutine(
        "One Piece",
        List.of(
            new AutoQuestion(
                "Starting Location?",
                List.of(
                    AutoQuestionResponse.AMP_SIDE,
                    AutoQuestionResponse.CENTER,
                    AutoQuestionResponse.WALL_SIDE))),
        autos.onePiece());

    selector.addRoutine(
        "Two Piece",
        List.of(
            new AutoQuestion(
                "Starting Location?",
                List.of(
                    AutoQuestionResponse.AMP_SIDE,
                    AutoQuestionResponse.CENTER,
                    AutoQuestionResponse.WALL_SIDE))),
        autos.twoPiece());

    selector.addRoutine(
        "Close Three",
        List.of(
            new AutoQuestion(
                "Direction?",
                List.of(AutoQuestionResponse.AMP_SIDE, AutoQuestionResponse.WALL_SIDE))),
        autos.threePieceClose());

    selector.addRoutine("Four From Middle", List.of(), autos.fourFromMiddle());
    selector.addRoutine("Long Four", List.of(), autos.fourPieceLong());
    selector.addRoutine("Long Five", List.of(), autos.fivePieceLong());

    selector.addRoutine(
        "Mobility",
        List.of(
            new AutoQuestion(
                "Starting Location?",
                List.of(
                    AutoQuestionResponse.AMP_SIDE,
                    AutoQuestionResponse.CENTER,
                    AutoQuestionResponse.WALL_SIDE))),
        autos.mobility());

    selector.addRoutine("Plow", List.of(), autos.plow());

    selector.addRoutine(
        "Drive Straight 10s",
        List.of(),
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0)), drive)
            .withTimeout(10));

    selector.addRoutine("Do Nothing", List.of(), Commands.none());

    if (Constants.ENABLE_DEBUG_ROUTINES) {
      final DebugRoutines routines = new DebugRoutines(drive, superstructure, intake, shooter);
      selector.addRoutine("Characterize Odometry", List.of(), routines.characterizeOdometry());
      selector.addRoutine("Pit Test", List.of(), routines.pitTest());
    }

    vision.setDataInterfaces(drive::addVisionData, drive::getPose, drive::setLastVisionPose);
    Lights.getInstance()
        .setDataInterfaces(
            drive::getPose,
            drive::getFieldRelativeSpeeds,
            superstructure::inHandoff,
            intake::hasGamePiece,
            shooter::hasGamePiece,
            superstructure::getElevatorPercentageRaised);
    Lights.getInstance()
        .setTargetedSupplier(() -> superstructure.isAtSetpoint() && shooter.isAtSetpoint());
    RobotSimulator.getInstance().setDrivePoseInterface(drive::getPose);
    RobotSimulator.getInstance().setSuperstructure(superstructure);
    bindControls();
  }

  /** Bind driver controls to commands */
  private void bindControls() {
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    shooter.setDefaultCommand(shooter.idleCommand());

    driver
        .leftTrigger()
        .whileTrue(intake.spitRaw().alongWith(shooter.setRawFeederVoltsCommand(-1.2)))
        .onFalse(shooter.setRawFeederVoltsCommand(0.0));
    driver.leftBumper().onTrue(superstructure.goToPose(Preset.STOW));
    driver.rightTrigger().whileTrue(new Handoff(intake, superstructure, shooter));
    driver.rightBumper().onTrue(superstructure.goToPose(Preset.HANDOFF));
    driver
        .povDown()
        .toggleOnTrue(
            new SuperstructureManual(
                superstructure, () -> -operator.getLeftY(), () -> -operator.getRightY()));
    driver.y().onTrue(Commands.runOnce(drive::resetHeading));
    // driver.a().whileTrue(new AmpAlign(drive, () -> -driver.getLeftX()));
    Pose2d ampPose =
        new Pose2d(
            AllianceFlipUtil.apply(FieldConstants.Amp.x),
            FieldConstants.fieldWidth - Units.inchesToMeters(10),
            Rotation2d.fromDegrees(90));
    driver.a().whileTrue(new DriveToPose(drive, ampPose));
    driver
        .povLeft()
        .onTrue(Commands.runOnce(() -> drive.setPose(new Pose2d(0.0, 0.0, new Rotation2d()))));
    operator.b().onTrue(superstructure.goToPose(Preset.STOW));
    operator.y().onTrue(superstructure.goToPose(Preset.HANDOFF));
    operator.a().onTrue(superstructure.goToPose(Preset.TRAP));
    operator.x().onTrue(superstructure.goToPose(Preset.AMP));
    operator.povUp().onTrue(superstructure.goToPose(Preset.SUBWOOFER_SHOOT));
    operator
        .rightBumper()
        .whileTrue(
            new AutoShoot(
                    superstructure,
                    drive,
                    shooter,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX())
                .alongWith(
                    Commands.runOnce(
                        () -> {
                          Lights.getInstance().setMode(LightsMode.Shooting);
                        }))
                .finallyDo(() -> superstructure.setModeVoid(SuperstructureState.DISABLED)))
        .onFalse(
            shooter
                .spinToSpeed(0.0)
                .alongWith(
                    Commands.runOnce(() -> Lights.getInstance().setMode(LightsMode.Delivery)))
                .andThen(Commands.waitSeconds(1.0))
                .andThen(new StowAfterShoot(superstructure)));
    operator
        .leftBumper()
        .whileTrue(shooter.feed())
        .onFalse(
            Commands.waitSeconds(1.0)
                // We only want the stow after shoot for the feeder button to apply if we are
                // not in shooting mode, since that should only stow after the targeting is
                // ended,
                // not when the feeding stops
                .andThen(
                    new StowAfterShoot(superstructure).onlyIf(() -> !superstructure.isShooting())));

    // Contextual shooting
    HashMap<String, Command> shootMap = new HashMap<>();
    shootMap.put("amp", shooter.setSpeedContinuous(2250));
    shootMap.put("trap", shooter.setSpeedContinuous(2000));
    shootMap.put("subwoofer_shoot", shooter.setSpeedContinuous(3200));
    shootMap.put("raw", shooter.setSpeedContinuous(3000));

    operator
        .rightTrigger()
        .whileTrue(
            Commands.select(
                shootMap,
                () -> {
                  if (superstructure.isInPose(Preset.AMP)) {
                    return "amp";
                  }
                  if (superstructure.isInPose(Preset.TRAP)) {
                    return "trap";
                  }
                  if (superstructure.isInPose(Preset.SUBWOOFER_SHOOT)) {
                    return "subwoofer_shoot";
                  }
                  return "raw";
                }));
  }

  public Command getAutonomousCommand() {
    return selector.getCommand();
  }
}
