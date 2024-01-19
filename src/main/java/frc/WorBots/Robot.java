// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.WorBots;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.util.StatusPage;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private PowerDistribution pdp;

  @Override
  public void robotInit() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    Lights.getInstance();
    pdp = new PowerDistribution();

    m_robotContainer = new RobotContainer();
    StatusPage.reportStatus(StatusPage.ROBOT_CODE, true);

    this.addPeriodic(() -> {
      // Simple status updates
      StatusPage.reportStatus(StatusPage.NETWORK_TABLES, NetworkTableInstance.getDefault().isConnected());
      StatusPage.reportStatus(StatusPage.DRIVER_STATION, DriverStation.isDSAttached());
      StatusPage.reportStatus(StatusPage.FMS, DriverStation.isFMSAttached());
      StatusPage.reportStatus(StatusPage.BATTERY, pdp.getVoltage() > 11.0);
      StatusPage.reportStatus(StatusPage.IDEAL_BATTERY, pdp.getVoltage() > 11.6);
      StatusPage.reportStatus(StatusPage.BROWNOUT, !HAL.getBrownedOut());
      StatusPage.reportStatus(StatusPage.DRIVE_CONTROLLER,
          DriverStation.isJoystickConnected(0) && DriverStation.getJoystickIsXbox(0));
      StatusPage.reportStatus(StatusPage.OPERATOR_CONTROLLER,
          DriverStation.isJoystickConnected(1) && !DriverStation.getJoystickIsXbox(1));
      StatusPage.reportStatus(StatusPage.NOT_ESTOPPED, !DriverStation.isEStopped());
      var pdpFaults = pdp.getFaults();
      boolean breakerFault = pdpFaults.Channel0BreakerFault ||
          pdpFaults.Channel1BreakerFault ||
          pdpFaults.Channel2BreakerFault ||
          pdpFaults.Channel3BreakerFault ||
          pdpFaults.Channel4BreakerFault ||
          pdpFaults.Channel5BreakerFault ||
          pdpFaults.Channel6BreakerFault ||
          pdpFaults.Channel7BreakerFault ||
          pdpFaults.Channel8BreakerFault ||
          pdpFaults.Channel9BreakerFault ||
          pdpFaults.Channel10BreakerFault ||
          pdpFaults.Channel11BreakerFault ||
          pdpFaults.Channel12BreakerFault ||
          pdpFaults.Channel13BreakerFault ||
          pdpFaults.Channel14BreakerFault ||
          pdpFaults.Channel15BreakerFault ||
          pdpFaults.Channel16BreakerFault ||
          pdpFaults.Channel17BreakerFault ||
          pdpFaults.Channel18BreakerFault ||
          pdpFaults.Channel19BreakerFault ||
          pdpFaults.Channel20BreakerFault ||
          pdpFaults.Channel21BreakerFault ||
          pdpFaults.Channel22BreakerFault ||
          pdpFaults.Channel23BreakerFault;
      StatusPage.reportStatus(StatusPage.PDP_BREAKERS, !breakerFault);
      StatusPage.reportStatus(StatusPage.CAN_WARNING, !pdpFaults.CanWarning);
      StatusPage.reportStatus(StatusPage.PDP_HARDWARE, !pdpFaults.HardwareFault);

      // Statuses for different clients
      boolean cam0 = false;
      boolean cam1 = false;
      boolean launchpad = false;
      for (var connection : NetworkTableInstance.getDefault().getConnections()) {
        if (connection.remote_id.contains("VisionModule0")) {
          cam0 = true;
        }
        if (connection.remote_id.contains("VisionModule1")) {
          cam1 = true;
        }
        if (connection.remote_id.contains("Launchpad")) {
          launchpad = true;
        }
      }
      if(DriverStation.isFMSAttached()) {
        StatusPage.reportMetadata();
      }
      StatusPage.reportStatus(StatusPage.CAM0, cam0);
      StatusPage.reportStatus(StatusPage.CAM1, cam1);
      StatusPage.reportStatus(StatusPage.LAUNCHPAD, launchpad);

      // Robot information
      SmartDashboard.putNumber("System/Battery Voltage", pdp.getVoltage());
      SmartDashboard.putNumber("System/PDP Current", pdp.getTotalCurrent());
      SmartDashboard.putNumber("System/PDP Temperature", pdp.getTemperature());
    }, kDefaultPeriod);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // FieldConstants.testField();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
