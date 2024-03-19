// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class MatchTime {
  private static MatchTime instance = new MatchTime();

  public static MatchTime getInstance() {
    return instance;
  }

  private final Timer timer = new Timer();

  public void startTeleop() {
    timer.restart();
  }

  public void startAuto() {
    timer.restart();
  }

  public double getTime() {
    if (DriverStation.isFMSAttached()) {
      return DriverStation.getMatchTime();
    }

    return timer.get();
  }
}
