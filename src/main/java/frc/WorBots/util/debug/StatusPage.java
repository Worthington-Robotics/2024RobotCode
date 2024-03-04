// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.debug;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.WorBots.util.BuildConstants;
import java.util.HashMap;

/** A utility class which shows the status of systems on the robot to NetworkTables */
public class StatusPage {
  private ShuffleboardTab tab = Shuffleboard.getTab("Status");
  private HashMap<String, GenericEntry> entries = new HashMap<>();
  private static boolean hasBeenStarted = false;

  // System name constants
  public static final String ALL_AUTO_QUESTIONS = "All Auto Questions";
  public static final String AUTO_CHOSEN = "Auto Chosen";
  public static final String ROBOT_CODE = "Robot Code";
  public static final String DRIVE_CONTROLLER = "Drive Controller";
  public static final String SUPERSTRUCTURE_SUBSYSTEM = "Superstructure Subsystem";
  public static final String DRIVE_SUBSYSTEM = "Drive Subsystem";
  public static final String INTAKE_SUBSYSTEM = "Intake Subsystem";
  public static final String LIGHTS_SUBSYSTEM = "Lights Subsystem";
  public static final String VISION_SUBSYSTEM = "Vision Subsystem";
  public static final String SHOOTER_SUBSYSTEM = "Shooter Subsystem";
  public static final String SHOOTER_CONNECTED = "Shooter Connected";
  public static final String PIVOT_CONNECTED = "Pivot Connected";
  public static final String ELEVATOR_CONNECTED = "Elevator Connected";
  public static final String GYROSCOPE = "Gyroscope";
  public static final String SMODULE_PREFIX = "SModule";
  public static final String NETWORK_TABLES = "Network Tables";
  public static final String DRIVER_STATION = "Driver Station";
  public static final String FMS = "FMS";
  public static final String BATTERY = "Battery";
  public static final String IDEAL_BATTERY = "Ideal Battery";
  public static final String INTAKE_CONNECTED = "Intake Connected";
  public static final String BROWNOUT = "Brownout";
  public static final String OPERATOR_CONTROLLER = "Operator Controller";
  public static final String NOT_ESTOPPED = "Not EStopped";
  public static final String PDP_BREAKERS = "PDP Breakers";
  public static final String CAN_WARNING = "CAN Warning";
  public static final String PDP_HARDWARE = "PDP Hardware";
  public static final String CAM_PREFIX = "Cam";
  public static final String LAUNCHPAD = "Launchpad";
  public static final String DRIVER_CAM = "Driver Cam";

  // Sort in order of priority, from highest to lowest
  /** All systems that the StatusPage reports */
  public static final String[] ALL_SYSTEMS = {
    AUTO_CHOSEN,
    DRIVE_CONTROLLER,
    OPERATOR_CONTROLLER,
    SHOOTER_CONNECTED,
    ALL_AUTO_QUESTIONS,
    PIVOT_CONNECTED,
    ELEVATOR_CONNECTED,
    INTAKE_CONNECTED,
    GYROSCOPE,
    CAM_PREFIX + "0",
    CAM_PREFIX + "1",
    BATTERY,
    IDEAL_BATTERY,
    NETWORK_TABLES,
    SMODULE_PREFIX + "0",
    SMODULE_PREFIX + "1",
    SMODULE_PREFIX + "2",
    SMODULE_PREFIX + "3",
    ROBOT_CODE,
    SUPERSTRUCTURE_SUBSYSTEM,
    DRIVE_SUBSYSTEM,
    INTAKE_SUBSYSTEM,
    VISION_SUBSYSTEM,
    SHOOTER_SUBSYSTEM,
    DRIVER_CAM,
    DRIVER_STATION,
    FMS,
    BROWNOUT,
    PDP_BREAKERS,
    CAN_WARNING,
    PDP_HARDWARE,
    LAUNCHPAD,
    LIGHTS_SUBSYSTEM,
    NOT_ESTOPPED,
  };

  private StatusPage() {
    // Start with all systems down
    for (String system : ALL_SYSTEMS) {
      GenericEntry entry = getEntry(system);
      entry.setBoolean(false);
    }
  }

  private static StatusPage instance = new StatusPage();

  /**
   * Get the singleton instance of the StatusPage. This shouldn't be necessary as most methods work
   * on the singleton anyways
   */
  public static StatusPage getInstance() {
    return instance;
  }

  /**
   * Report the status of a system
   *
   * @param system The system to report. One of the static constants provided by this class should
   *     be used
   * @param status The status of the system to set. True represents a working system
   */
  public static void reportStatus(String system, boolean status) {
    StatusPage instance = getInstance();
    GenericEntry entry = instance.getEntry(system);
    entry.setBoolean(status);
  }

  /**
   * Get the current status of a system from NetworkTables
   *
   * @param system The system to check. Should be one of the system constants
   * @return Whether the system is up or down
   */
  public static boolean getStatus(String system) {
    StatusPage instance = getInstance();
    GenericEntry entry = instance.getEntry(system);
    return entry.getBoolean(false);
  }

  /**
   * Periodic method to run from the robot base to report common statuses
   *
   * @param pdp The PowerDistribution panel
   */
  public static void periodic(PowerDistribution pdp) {
    // Connection of main robot systems
    StatusPage.reportStatus(
        StatusPage.NETWORK_TABLES, NetworkTableInstance.getDefault().isConnected());
    StatusPage.reportStatus(StatusPage.DRIVER_STATION, DriverStation.isDSAttached());
    StatusPage.reportStatus(StatusPage.FMS, DriverStation.isFMSAttached());

    // Power
    StatusPage.reportStatus(StatusPage.BATTERY, pdp.getVoltage() > 11.0);
    StatusPage.reportStatus(StatusPage.IDEAL_BATTERY, pdp.getVoltage() > 11.6);
    StatusPage.reportStatus(StatusPage.BROWNOUT, !HAL.getBrownedOut());

    // Controllers
    StatusPage.reportStatus(
        StatusPage.DRIVE_CONTROLLER,
        DriverStation.isJoystickConnected(0) && DriverStation.getJoystickIsXbox(0));
    StatusPage.reportStatus(
        StatusPage.OPERATOR_CONTROLLER,
        DriverStation.isJoystickConnected(1) && DriverStation.getJoystickIsXbox(1));
    StatusPage.reportStatus(StatusPage.NOT_ESTOPPED, !DriverStation.isEStopped());

    // PDP
    var pdpFaults = pdp.getFaults();
    // boolean breakerFault =
    //     pdpFaults.Channel0BreakerFault
    //         || pdpFaults.Channel1BreakerFault
    //         || pdpFaults.Channel2BreakerFault
    //         || pdpFaults.Channel3BreakerFault
    //         || pdpFaults.Channel4BreakerFault
    //         || pdpFaults.Channel5BreakerFault
    //         || pdpFaults.Channel6BreakerFault
    //         || pdpFaults.Channel7BreakerFault
    //         || pdpFaults.Channel8BreakerFault
    //         || pdpFaults.Channel9BreakerFault
    //         || pdpFaults.Channel10BreakerFault
    //         || pdpFaults.Channel11BreakerFault
    //         || pdpFaults.Channel12BreakerFault
    //         || pdpFaults.Channel13BreakerFault
    //         || pdpFaults.Channel14BreakerFault
    //         || pdpFaults.Channel15BreakerFault
    //         || pdpFaults.Channel16BreakerFault
    //         || pdpFaults.Channel17BreakerFault
    //         || pdpFaults.Channel18BreakerFault
    //         || pdpFaults.Channel19BreakerFault
    //         || pdpFaults.Channel20BreakerFault
    //         || pdpFaults.Channel21BreakerFault
    //         || pdpFaults.Channel22BreakerFault
    //         || pdpFaults.Channel23BreakerFault;
    // StatusPage.reportStatus(StatusPage.PDP_BREAKERS, !breakerFault);
    // StatusPage.reportStatus(StatusPage.CAN_WARNING, !pdpFaults.CanWarning);
    StatusPage.reportStatus(StatusPage.PDP_HARDWARE, !pdpFaults.HardwareFault);

    // Statuses for different clients
    // boolean cam0 = false;
    // boolean cam1 = false;
    boolean launchpad = false;
    for (var connection : NetworkTableInstance.getDefault().getConnections()) {
      // if (connection.remote_id.contains("VisionModule0")) {
      //   cam0 = true;
      // }
      // if (connection.remote_id.contains("VisionModule1")) {
      //   cam1 = true;
      // }
      if (connection.remote_id.contains("Launchpad")) {
        launchpad = true;
      }
    }
    // StatusPage.reportStatus(StatusPage.CAM0, cam0);
    // StatusPage.reportStatus(StatusPage.CAM1, cam1);
    StatusPage.reportStatus(StatusPage.LAUNCHPAD, launchpad);

    // Robot information
    SmartDashboard.putNumber("System/Battery Voltage", pdp.getVoltage());
    SmartDashboard.putNumber("System/PDP Current", pdp.getTotalCurrent());
    SmartDashboard.putNumber("System/PDP Temperature", pdp.getTemperature());

    // Report metadata
    StatusPage.reportMetadata();
  }

  /** Report metadata for AdvantageScope to use. Also starts the WPILib DataLog */
  public static void reportMetadata() {
    if (!hasBeenStarted) {
      // Build metadata
      var nt = NetworkTableInstance.getDefault();
      var table = nt.getTable("Metadata");
      var repo = table.getEntry("Repository");
      repo.setString(BuildConstants.MAVEN_NAME);
      var gitBranch = table.getEntry("Git Branch");
      gitBranch.setString(BuildConstants.GIT_BRANCH);
      var gitSHA = table.getEntry("Git SHA");
      gitSHA.setString(BuildConstants.GIT_SHA);
      var gitDate = table.getEntry("Git Date");
      gitDate.setString(BuildConstants.GIT_DATE);
      var buildDate = table.getEntry("Build Date");
      buildDate.setString(BuildConstants.BUILD_DATE);
      var dirty = table.getEntry("Is Dirty");
      dirty.setInteger(BuildConstants.DIRTY);

      // Match metadata can be put only in the WPILog
      // and only when we are using a real robot
      if (DriverStation.isFMSAttached()) {
        DataLog log = DataLogManager.getLog();
        StringLogEntry name = new StringLogEntry(log, "/Metadata/Event Name");
        name.append(DriverStation.getEventName());
        IntegerLogEntry number = new IntegerLogEntry(log, "/Metadata/Match Number");
        number.append(DriverStation.getMatchNumber());
        StringLogEntry type = new StringLogEntry(log, "/Metadata/Match Type");
        type.append(DriverStation.getMatchType().toString());
        StringLogEntry alliance = new StringLogEntry(log, "/Metadata/Alliance");
        alliance.append(
            (DriverStation.getAlliance().isPresent()
                ? DriverStation.getAlliance().get().name()
                : "Not present"));
        IntegerLogEntry stationNumber = new IntegerLogEntry(log, "/Metadata/Alliance Station");
        stationNumber.append(DriverStation.getLocation().getAsInt());
      }
    }
    hasBeenStarted = true;
  }

  private GenericEntry getEntry(String system) {
    return entries.computeIfAbsent(system, k -> tab.add(system, false).getEntry());
  }
}
