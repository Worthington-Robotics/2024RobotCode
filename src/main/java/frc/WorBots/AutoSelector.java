// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.util.debug.SwitchableChooser;
import java.util.ArrayList;
import java.util.List;

public class AutoSelector extends SubsystemBase {
  public static final int maxQuestions = 4;

  /** Whether to use the LabVIEW dashboard instead of multiple question choosers */
  private static final boolean useDriverStation = true;

  private static final AutoRoutine defaultRoutine =
      new AutoRoutine("Do Nothing", List.of(), Commands.none());

  private SwitchableChooser routineChooser;
  private List<StringPublisher> questionPublishers;
  private List<SwitchableChooser> questionChoosers;

  private AutoRoutine lastRoutine;
  private List<String> names = new ArrayList<>();
  private List<AutoRoutine> routines = new ArrayList<>();
  private List<AutoQuestionResponse> lastResponses;
  private List<String> answers = new ArrayList<>();

  /**
   * The auto selector is logged onto SmartDashboard and allows for the drivers to set the desired
   * auto before a match.
   *
   * @param key The SmartDashboard table to be logged under.
   */
  public AutoSelector(String key) {
    lastRoutine = defaultRoutine;

    if (!useDriverStation) {
      routineChooser = new SwitchableChooser(key);
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
  }

  /**
   * Adds an auto to the auto selector.
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
    if (useDriverStation) {
      createAnswerList();
    } else {
      routineChooser.setOptions(names.toArray(new String[0]));
    }
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

  private void createAnswerList() {
    answers.clear();
    for (AutoRoutine routine : routines) {
      if (routine.questions.size() > 0) {
        createAnswerPermutations(routine.name, 0, routine.questions);
      } else {
        answers.add(routine.name);
      }
    }
  }

  // Recursive thing for creating answers
  private void createAnswerPermutations(String answer, int index, List<AutoQuestion> questions) {
    AutoQuestion question = questions.get(index);
    for (AutoQuestionResponse response : question.responses) {
      final String modified = answer + "; " + response;
      if (index < questions.size() - 1) {
        createAnswerPermutations(modified, index + 1, questions);
      } else {
        answers.add(modified);
      }
    }
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
    if (DriverStation.isAutonomousEnabled()
        || DriverStation.isTeleopEnabled() && lastRoutine != null && lastResponses == null) {
      return;
    }
    if (useDriverStation) {
      SmartDashboard.putStringArray("Auto List", answers.toArray(new String[0]));
      String answer = SmartDashboard.getString("Auto Selector", "Do Nothing");
      if (answer == null) {
        return;
      }
      String[] items = answer.split("; ");
      var selectedRoutine = getRoutineFromName(items[0]);
      if (selectedRoutine == null || selectedRoutine.equals(lastRoutine)) {
        return;
      }

      lastRoutine = selectedRoutine;
      SmartDashboard.putString("Actually Selected Auto", lastRoutine.name);

      lastResponses = new ArrayList<>();
      for (int i = 0; i < lastRoutine.questions().size(); i++) {
        String responseString = items[i + 1];
        lastResponses.add(
            responseString == null
                ? lastRoutine.questions().get(i).responses().get(0)
                : AutoQuestionResponse.valueOf(responseString));
      }

      StatusPage.reportStatus(StatusPage.ALL_AUTO_QUESTIONS, true);
    } else {
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
    }
    StatusPage.reportStatus(StatusPage.AUTO_CHOSEN, lastRoutine != null);
  }

  /**
   * Return the selected auto command.
   *
   * @return The command.
   */
  public Command getCommand() {
    SmartDashboard.putString("Auto Attempted", lastRoutine.name);
    return lastRoutine.command();
  }

  /**
   * We have to do this once at the beginning when the command is registered otherwise WPILib gives
   * us errors
   */
  private static Command createRoutineCommand(Command command) {
    return Commands.deadline(command);
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
