// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.debug;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.WorBots.Constants;

/**
 * Simple utility that allows you to have a double value that is modifiable at runtime using the
 * dashboard.
 */
public class TunableDouble {
  // Set this to false to disable tuning and improve performance.
  // All tunables will just use their default value
  private static final boolean ENABLE_TUNING = !Constants.IS_COMP && false;

  private double value;
  // We have to keep this around since we need to supply a default
  // value when getting from the NT entry
  private double defaultValue;
  private NetworkTableEntry entry;

  /**
   * Creates a new TunableDouble with a name and the value set to 0.0
   *
   * @param table The main NT table
   * @param subtable The NT subtable
   * @param name The NT key
   */
  public TunableDouble(String table, String subtable, String name) {
    this(table, subtable, name, 0.0);
  }

  /**
   * Creates a new TunableDouble with a name and a default value
   *
   * @param table The main NT table
   * @param subtable The NT subtable
   * @param name The NT key
   * @param defaultValue The default value for the tunable to have before it is updated or if the
   *     update fails
   */
  public TunableDouble(String table, String subtable, String name, double defaultValue) {
    value = defaultValue;
    this.defaultValue = defaultValue;
    if (ENABLE_TUNING) {
      entry =
          NetworkTableInstance.getDefault().getTable(table).getSubTable(subtable).getEntry(name);
    }
  }

  /** Update the value to the latest version and log that value */
  public void update() {
    if (ENABLE_TUNING) {
      value = entry.getDouble(defaultValue);
    }
  }

  /**
   * Update the current value and then get it.
   *
   * @return The latest updated value from NetworkTables
   */
  public double get() {
    update();
    return value;
  }

  /**
   * Get the current value without updating
   *
   * @return The value from the last update
   */
  public double getCurrent() {
    return value;
  }

  /**
   * Set the value and default value
   *
   * @param value The value to set
   */
  public void set(double value) {
    defaultValue = value;
    if (ENABLE_TUNING) {
      entry.setDouble(value);
    }
  }

  /**
   * Get the default value
   *
   * @return The default value of the tunable
   */
  public double getDefault() {
    return defaultValue;
  }
}
