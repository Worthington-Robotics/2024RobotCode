// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class UtilCommands {
  public static Command named(Command command, String key) {
    return command.raceWith(
        Commands.startEnd(
            () -> SmartDashboard.putBoolean(key, true),
            () -> SmartDashboard.putBoolean(key, false)));
  }

  public static Command namedSequence(String key, Command... commands) {
    for (int i = 0; i < commands.length; i++) {
      final int finalI = i;
      commands[i] =
          commands[i].alongWith(Commands.run(() -> SmartDashboard.putNumber(key, finalI)));
    }
    return Commands.parallel(commands);
  }

  public static Command waitForDriverstationButton() {
    return Commands.waitUntil(() -> SmartDashboard.getBoolean("DB/Button 1", false))
        .andThen(() -> SmartDashboard.putBoolean("DB/Button 1", false));
  }
}
