package frc.WorBots.util;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class StatusPage {
  private ShuffleboardTab tab = Shuffleboard.getTab("Status");
  private HashMap<String, GenericEntry> entries = new HashMap<>();

  // System name constants
  public static final String AUTO_RUNNING = "Auto Running";
  public static final String ALL_AUTO_QUESTIONS = "All Auto Questions";
  public static final String AUTO_CHOSEN = "Auto Chosen";
  public static final String ROBOT_CODE = "Robot Code";
  public static final String DRIVE_CONTROLLER = "Drive Controller";
  public static final String ARM_SUBSYSTEM = "Arm Subsystem";
  public static final String DRIVE_SUBSYSTEM = "Drive Subsystem";
  public static final String INTAKE_SUBSYSTEM = "Intake Subsystem";
  public static final String LIGHTS_SUBSYSTEM = "Lights Subsystem";
  public static final String VISION_SUBSYSTEM = "Vision Subsystem";
  public static final String GYROSCOPE = "Gyroscope";
  public static final String SMODULE_PREFIX = "SModule";
  public static final String NODE_SELECTOR = "Node Selector";
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
  public static final String CAM0 = "Cam 0";
  public static final String CAM1 = "Cam 1";
  public static final String LAUNCHPAD = "Launchpad";

  public static final String[] ALL_SYSTEMS = {
      AUTO_RUNNING,
      ALL_AUTO_QUESTIONS,
      AUTO_CHOSEN,
      ROBOT_CODE,
      DRIVE_CONTROLLER,
      ARM_SUBSYSTEM,
      DRIVE_SUBSYSTEM,
      INTAKE_SUBSYSTEM,
      LIGHTS_SUBSYSTEM,
      VISION_SUBSYSTEM,
      GYROSCOPE,
      SMODULE_PREFIX + "0",
      SMODULE_PREFIX + "1",
      SMODULE_PREFIX + "2",
      SMODULE_PREFIX + "3",
      NODE_SELECTOR,
      NETWORK_TABLES,
      DRIVER_STATION,
      FMS,
      BATTERY,
      IDEAL_BATTERY,
      INTAKE_CONNECTED,
      BROWNOUT,
      OPERATOR_CONTROLLER,
      NOT_ESTOPPED,
      PDP_BREAKERS,
      CAN_WARNING,
      PDP_HARDWARE,
      CAM0,
      CAM1,
      LAUNCHPAD,
  };

  private StatusPage() {
    // Start with all systems down
    for (String system : ALL_SYSTEMS) {
      GenericEntry entry = getEntry(system);
      entry.setBoolean(false);
    }
  }

  private static StatusPage instance = new StatusPage();

  public static StatusPage getInstance() {
    return instance;
  }

  public static void reportStatus(String system, boolean status) {
    StatusPage instance = getInstance();
    GenericEntry entry = instance.getEntry(system);
    entry.setBoolean(status);
  }

  public static boolean getStatus(String system) {
    StatusPage instance = getInstance();
    GenericEntry entry = instance.getEntry(system);
    return entry.getBoolean(false);
  }

  private GenericEntry getEntry(String system) {
    return entries.computeIfAbsent(system, k -> tab.add(system, false).getEntry());
  }
}
