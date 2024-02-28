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
  /**
   * Returns a command with a SmartDashboard entry that displays whether it is running or not
   *
   * @param command The command to decorate
   * @param key The SmartDashboard key to use
   * @return The command to run
   */
  public static Command named(Command command, String key) {
    return command.raceWith(
        Commands.startEnd(
            () -> SmartDashboard.putBoolean(key, true),
            () -> SmartDashboard.putBoolean(key, false)));
  }

  /**
   * Returns a command sequence with a SmartDashboard entry that displays progress
   *
   * @param key The SmartDashboard key to use
   * @param commands The commands to run
   * @return The command
   */
  public static Command namedSequence(String key, Command... commands) {
    for (int i = 0; i < commands.length; i++) {
      final int finalI = i;
      commands[i] =
          commands[i].alongWith(Commands.runOnce(() -> SmartDashboard.putNumber(key, finalI)));
    }
    return Commands.sequence(commands);
  }

  /**
   * Creates a command that waits for Button 1 on the Driverstation to be pressed
   *
   * @return The command to run
   */
  public static Command waitForDriverstationButton() {
    return Commands.waitUntil(() -> SmartDashboard.getBoolean("DB/Button 1", false))
        .andThen(() -> SmartDashboard.putBoolean("DB/Button 1", false));
  }
}
