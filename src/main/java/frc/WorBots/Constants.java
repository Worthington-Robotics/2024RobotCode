package frc.WorBots;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Miscellaneous global constants for the robot and code
 */
public class Constants {
  /**
   * Gets whether the robot is running in a simulation or is real
   * 
   * @return True if the robot is simulated, false otherwise
   */
  public static final boolean getSim() {
    return RobotBase.isSimulation();
  }
}
