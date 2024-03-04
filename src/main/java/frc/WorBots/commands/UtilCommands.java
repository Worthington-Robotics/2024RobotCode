// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;

public class UtilCommands {
  /**
   * Returns a command with a SmartDashboard entry that displays whether it is running or not
   *
   * @param command The command to decorate
   * @param key The SmartDashboard key to use
   * @return The command to run
   */
  public static Command named(Command command, String key) {
    return Commands.startEnd(
            () -> SmartDashboard.putBoolean(key, true), () -> SmartDashboard.putBoolean(key, false))
        .raceWith(command);
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
          optimalSequence(
              Commands.runOnce(() -> SmartDashboard.putNumber(key, finalI)), commands[i]);
    }
    return optimalSequence(commands);
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

  /**
   * Returns a command that runs a sequence of commands optimally with no delay
   *
   * @param commands The commands to run in the sequence
   * @return The command
   */
  public static Command optimalSequence(Command... commands) {
    return new OptimalSequentialCommandGroup(commands);
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

  /**
   * A command composition that runs a list of commands in sequence, with optimal execution. Unlike
   * the normal SequentialCommandGroup, when this command runs a subcommand that is already
   * finished, it will immediately start the next one in the sequence without the usual
   * CommandScheduler overhead.
   */
  public static class OptimalSequentialCommandGroup extends Command {
    private final List<Command> commands = new ArrayList<>();
    private int currentCommandIndex = -1;
    private boolean runWhenDisabled = true;
    private InterruptionBehavior interruptBehavior = InterruptionBehavior.kCancelIncoming;

    /**
     * Creates a new SequentialCommandGroup. The given commands will be run sequentially, with the
     * composition finishing when the last command finishes.
     *
     * @param commands the commands to include in this composition.
     */
    public OptimalSequentialCommandGroup(Command... commands) {
      addCommands(commands);
    }

    /**
     * Adds the given commands to the group.
     *
     * @param commands Commands to add, in order of execution.
     */
    public final void addCommands(Command... commands) {
      if (currentCommandIndex != -1) {
        throw new IllegalStateException(
            "Commands cannot be added to a composition while it's running");
      }

      CommandScheduler.getInstance().registerComposedCommands(commands);

      for (Command command : commands) {
        this.commands.add(command);
        m_requirements.addAll(command.getRequirements());
        runWhenDisabled &= command.runsWhenDisabled();
        if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
          interruptBehavior = InterruptionBehavior.kCancelSelf;
        }
      }
    }

    @Override
    public final void initialize() {
      currentCommandIndex = 0;

      if (!commands.isEmpty()) {
        commands.get(0).initialize();
      }
    }

    @Override
    public final void execute() {
      if (commands.isEmpty()) {
        return;
      }

      while (true) {
        if (currentCommandIndex >= commands.size()) {
          break;
        }

        final Command currentCommand = commands.get(currentCommandIndex);

        currentCommand.execute();
        if (currentCommand.isFinished()) {
          currentCommand.end(false);
          currentCommandIndex++;
          if (currentCommandIndex < commands.size()) {
            commands.get(currentCommandIndex).initialize();
          }
        } else {
          break;
        }
      }
    }

    @Override
    public final void end(boolean interrupted) {
      if (interrupted
          && !commands.isEmpty()
          && currentCommandIndex > -1
          && currentCommandIndex < commands.size()) {
        commands.get(currentCommandIndex).end(true);
      }
      currentCommandIndex = -1;
    }

    @Override
    public final boolean isFinished() {
      return currentCommandIndex == commands.size();
    }

    @Override
    public boolean runsWhenDisabled() {
      return runWhenDisabled;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
      return interruptBehavior;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);

      builder.addIntegerProperty("index", () -> currentCommandIndex, null);
    }
  }
}
