// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.util.StatusPage;
import frc.WorBots.util.SwitchableChooser;
import java.util.ArrayList;
import java.util.List;

public class AutoSelector extends SubsystemBase {
  public static final int maxQuestions = 4;
  private static final AutoRoutine defaultRoutine =
      new AutoRoutine("Do Nothing", List.of(), Commands.none());

  private SwitchableChooser routineChooser;
  private final List<StringPublisher> questionPublishers;
  private final List<SwitchableChooser> questionChoosers;

  private AutoRoutine lastRoutine;
  private List<String> names = new ArrayList<>();
  private List<AutoRoutine> routines = new ArrayList<>();
  private List<AutoQuestionResponse> lastResponses;

  /**
   * The auto selector is logged onto SmartDashboard and allows for the drivers to set the desired
   * auto before a match.
   *
   * @param key The SmartDashboard table to be logged under.
   */
  public AutoSelector(String key) {
    StatusPage.reportStatus(StatusPage.AUTO_RUNNING, false);

    routineChooser = new SwitchableChooser(key);
    lastRoutine = defaultRoutine;

    questionPublishers = new ArrayList<>();
    questionChoosers = new ArrayList<>();
    for (int i = 0; i < maxQuestions; i++) {
      var publisher =
          NetworkTableInstance.getDefault()
              .getStringTopic("/SmartDashboard/" + key + "/Question #" + Integer.toString(i + 1))
              .publish();
      publisher.set("NA");
      questionPublishers.add(publisher);
      questionChoosers.add(
          new SwitchableChooser(key + "/Question #" + Integer.toString(i + 1) + " Chooser"));
    }
  }

  /**
   * Adds a auto the the auto selector.
   *
   * @param name The name of the auto.
   * @param questions The auto questions to be asked.
   * @param command The command that needs to be ran.
   */
  public void addRoutine(String name, List<AutoQuestion> questions, Command command) {
    if (questions.size() > maxQuestions) {
      throw new RuntimeException(
          "Auto routine contained more than "
              + Integer.toString(maxQuestions)
              + " questions: "
              + name);
    }
    names.add(name);
    routines.add(new AutoRoutine(name, questions, createRoutineCommand(command)));
    routineChooser.setOptions(names.toArray(new String[0]));
  }

  public AutoRoutine getRoutineFromName(String name) {
    AutoRoutine returnRoutine = null;
    for (AutoRoutine routine : routines) {
      if (routine.name.equalsIgnoreCase(name)) {
        returnRoutine = routine;
      }
    }
    return returnRoutine;
  }

  /**
   * Gets the current chosen auto.
   *
   * @return The responses in the form of a list.
   */
  public List<AutoQuestionResponse> getResponses() {
    return lastResponses;
  }

  public void periodic() {
    if (DriverStation.isAutonomousEnabled() && lastRoutine != null && lastResponses == null) {
      return;
    }
    routineChooser.periodic();

    var selectedRoutine = getRoutineFromName(routineChooser.get());
    if (selectedRoutine == null) {
      return;
    }
    for (SwitchableChooser chooser : questionChoosers) {
      chooser.periodic();
    }
    if (!selectedRoutine.equals(lastRoutine)) {
      var questions = selectedRoutine.questions();
      for (int i = 0; i < maxQuestions; i++) {
        if (i < questions.size()) {
          questionPublishers.get(i).set(questions.get(i).question());
          questionChoosers
              .get(i)
              .setOptions(
                  questions.get(i).responses().stream()
                      .map((AutoQuestionResponse response) -> response.toString())
                      .toArray(String[]::new));
        } else {
          questionPublishers.get(i).set("");
          questionChoosers.get(i).setOptions(new String[] {});
        }
      }
    }

    lastRoutine = selectedRoutine;
    lastResponses = new ArrayList<>();
    boolean allQuestionsChosen = true;
    for (int i = 0; i < lastRoutine.questions().size(); i++) {
      String responseString = questionChoosers.get(i).get();
      if (responseString == null) {
        allQuestionsChosen = false;
      }
      lastResponses.add(
          responseString == null
              ? lastRoutine.questions().get(i).responses().get(0)
              : AutoQuestionResponse.valueOf(responseString));
    }
    StatusPage.reportStatus(StatusPage.ALL_AUTO_QUESTIONS, allQuestionsChosen);
    StatusPage.reportStatus(StatusPage.AUTO_CHOSEN, lastRoutine != null);
  }

  /**
   * Return the selected auto command.
   *
   * @return The command.
   */
  public Command getCommand() {
    return lastRoutine.command();
  }

  /**
   * We have to do this once at the beginning when the command is registered otherwise WPILib gives
   * us errors
   */
  private static Command createRoutineCommand(Command command) {
    return Commands.deadline(
        command,
        Commands.startEnd(
            () -> {
              StatusPage.reportStatus(StatusPage.AUTO_RUNNING, true);
            },
            () -> {
              StatusPage.reportStatus(StatusPage.AUTO_RUNNING, false);
            }));
  }

  private static final record AutoRoutine(
      String name, List<AutoQuestion> questions, Command command) {}

  public static record AutoQuestion(String question, List<AutoQuestionResponse> responses) {}

  /** The potential answers to auto questions */
  public static enum AutoQuestionResponse {
    YES,
    NO,
    WALL_SIDE,
    CENTER,
    AMP_SIDE,
  }
}
