// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util;

import com.playingwithfusion.TimeOfFlight;

/** Utility functions for working with hardware devices */
public class DeviceUtils {
  /**
   * Checks if a time of flight device is connected
   *
   * @param tof The time of flight
   * @return True if the device is working properly, false if it is not
   */
  public static boolean getTimeOfFlightStatus(TimeOfFlight tof) {
    final TimeOfFlight.Status status = tof.getStatus();
    return status == TimeOfFlight.Status.Valid;
  }
}
