// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.subsystems.lights.Lights.LightsMode;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.WorBots.util.MatchTime;
import frc.WorBots.util.RobotSimulator;
import frc.WorBots.util.cache.Cache.AllianceCache;
import frc.WorBots.util.cache.Cache.TimeCache;
import frc.WorBots.util.debug.StatusPage;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  private PowerDistribution pdp;

  @Override
  public void robotInit() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    Lights.getInstance();
    pdp = new PowerDistribution();

    robotContainer = new RobotContainer();
    StatusPage.reportStatus(StatusPage.ROBOT_CODE, true);

    if (Constants.getSim()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    } else {
      DriverStation.silenceJoystickConnectionWarning(false);
    }

    this.addPeriodic(
        () -> {
          // Simple status updates
          StatusPage.periodic(pdp);
          StatusPage.reportStatus(
              StatusPage.DRIVE_CONTROLLER, robotContainer.driver.getHID().isConnected());
          StatusPage.reportStatus(
              StatusPage.OPERATOR_CONTROLLER, robotContainer.operator.getHID().isConnected());
        },
        kDefaultPeriod);

    // Run our robot code at a higher frequency
    this.addPeriodic(this::periodicFunction, Constants.ROBOT_PERIOD);

    // Ensure caches are updated at the beginning
    TimeCache.getInstance().update();
    AllianceCache.getInstance().update();
  }

  @Override
  public void robotPeriodic() {}

  /** Periodic function for the robot */
  private void periodicFunction() {
    // Update caches
    TimeCache.getInstance().update();
    AllianceCache.getInstance().update();

    // Run the command scheduler
    CommandScheduler.getInstance().run();

    // Update simulator
    RobotSimulator.getInstance().periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if (Constants.IS_COMP) {
      Lights.getInstance().setMode(LightsMode.Status);
    } else {
      Lights.getInstance().setMode(LightsMode.WorbotsFlame);
    }
    robotContainer.checkAutos();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    MatchTime.getInstance().startAuto();

    // Run the autonomous command
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

    stopSubsystems();

    if (Constants.IS_COMP) {
      Lights.getInstance().setMode(LightsMode.Alliance);
    } else {
      Lights.getInstance().setMode(LightsMode.MatchTime);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
    MatchTime.getInstance().startTeleop();

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    stopSubsystems();

    Lights.getInstance().setMode(LightsMode.Delivery);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

  /** Stops subsystems for robot state changes */
  private void stopSubsystems() {
    robotContainer.drive.stop();
    robotContainer.intake.setVolts(0.0);
    robotContainer.shooter.stopFlywheels();
    robotContainer.shooter.setIdlingDisabled(false);
    robotContainer.shooter.setRawFeederVolts(0.0);
    robotContainer.superstructure.setModeVoid(SuperstructureState.DISABLED);
    robotContainer.superstructure.setClimbLocked(false);
  }
}
