// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.debug;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class DebugValue {
  /** A debug value that is logged either to NT or the log file */
  public static class DebugDouble {
    private DoublePublisher publisher = null;
    private DoubleLogEntry logEntry = null;

    public DebugDouble(String table, String key, boolean useNT) {
      if (useNT) {
        publisher = NetworkTableInstance.getDefault().getTable(table).getDoubleTopic(key).publish();
      } else {
        logEntry = new DoubleLogEntry(DataLogManager.getLog(), "/" + table + "/" + key);
      }
    }

    public void set(double value) {
      if (publisher == null) {
        if (logEntry != null) {
          logEntry.append(value, (long) (Timer.getFPGATimestamp() * 1000000.0));
        }
      } else {
        publisher.set(value);
      }
    }
  }

  /** A debug value that is logged either to NT or the log file */
  public static class DebugBool {
    private BooleanPublisher publisher = null;
    private BooleanLogEntry logEntry = null;

    public DebugBool(String table, String key, boolean useNT) {
      if (useNT) {
        publisher =
            NetworkTableInstance.getDefault().getTable(table).getBooleanTopic(key).publish();
      } else {
        logEntry = new BooleanLogEntry(DataLogManager.getLog(), "/" + table + "/" + key);
      }
    }

    public void set(boolean value) {
      if (publisher == null) {
        if (logEntry != null) {
          logEntry.append(value, (long) (Timer.getFPGATimestamp() * 1000000.0));
        }
      } else {
        publisher.set(value);
      }
    }
  }

  /** Returns a logged double that is stored in the log when connected to an FMS */
  public static DebugDouble compDouble(String table, String key) {
    return new DebugDouble(table, key, !DriverStation.isFMSAttached());
  }

  /** Returns a logged boolean that is stored in the log when connected to an FMS */
  public static DebugBool compBool(String table, String key) {
    return new DebugBool(table, key, !DriverStation.isFMSAttached());
  }
}
