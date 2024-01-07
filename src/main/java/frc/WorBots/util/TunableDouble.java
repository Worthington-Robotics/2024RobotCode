package frc.WorBots.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Simple utility that allows you to have a double value that is modifiable
 * at runtime using the dashboard.
 */
public class TunableDouble {
  // Set this to false to disable tuning and improve performance.
  // All tunables will just use their default value
  private static final boolean enableTuning = true;

  private double value;
  // We have to keep this around since we need to supply a default
  // value when getting from the NT entry
  private double defaultValue;
  private NetworkTableEntry entry;

  /**
   * Creates a new TunableDouble with a name and the value set to 0.0
   */
  public TunableDouble(String table, String subtable, String name) {
    this(table, subtable, name, 0.0);
  }

  /**
   * Creates a new TunableDouble with a name and a default value
   */
  public TunableDouble(String table, String subtable, String name, double defaultValue) {
    value = defaultValue;
    this.defaultValue = defaultValue;
    entry = NetworkTableInstance.getDefault().getTable(table).getSubTable(subtable).getEntry(name);
  }

  /**
   * Update the value to the latest version and log that value
   */
  public void update() {
    if (enableTuning) {
      value = entry.getDouble(defaultValue);
    } else {
      value = defaultValue;
    }
  }

  /**
   * Update the current value and then get it.
   */
  public double get() {
    update();
    return value;
  }

  /**
   * Set the value and default value
   */
  public void set(double value) {
    defaultValue = value;
    if (enableTuning) {
      entry.setDouble(value);
    }
  }

  /**
   * Get the default value
   */
  public double getDefault() {
    return defaultValue;
  }
}
