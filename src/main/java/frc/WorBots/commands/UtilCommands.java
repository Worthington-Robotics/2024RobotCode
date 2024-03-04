// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.wpilibj.Timer;
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
          Commands.runOnce(() -> SmartDashboard.putNumber(key, finalI)).andThen(commands[i]);
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

  /**
   * Returns a command that times how long the command inside it takes
   *
   * @param name The key to put the time in SmartDashboard
   * @param command The command to time
   * @return The wrapped command
   */
  public static Command timer(String name, Command command) {
    return new TimedCommand(command, name);
  }

  public static class TimedCommand extends Command {
    private Command command;
    private final Timer timer = new Timer();
    private final String name;

    public TimedCommand(Command command, String name) {
      this.command = command;
      this.name = name;
    }

    @Override
    public void initialize() {
      timer.restart();
      if (command != null) {
        command.schedule();
      }
    }

    @Override
    public void end(boolean interrupted) {
      if (interrupted) {
        command.cancel();
      }
      command = null;
      SmartDashboard.putNumber(name, timer.get());
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
      return command == null || !command.isScheduled();
    }

    @Override
    public boolean runsWhenDisabled() {
      return command.runsWhenDisabled();
    }
  }
}
