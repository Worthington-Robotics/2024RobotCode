// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.debug;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class DebugValue {
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

  public static DebugDouble compDouble(String table, String key) {
    return new DebugDouble(table, key, !DriverStation.isFMSAttached());
  }
}
