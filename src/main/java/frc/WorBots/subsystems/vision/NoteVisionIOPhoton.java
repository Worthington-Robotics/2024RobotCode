// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.vision;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NoteVisionIOPhoton implements NoteVisionIO {
  private final NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();
  private final DoubleSubscriber noteThetaSub;
  private final DoubleSubscriber notePitchSub;
  private final DoubleSubscriber noteAreaSub;
  private final DoubleSubscriber notePixelsXSub;
  private final DoubleSubscriber notePixelsYSub;
  private final BooleanSubscriber hasTargetSub;
  private final LinearFilter noteThetaFilter = LinearFilter.movingAverage(6);
  private final LinearFilter notePitchFilter = LinearFilter.movingAverage(6);
  private final LinearFilter noteAreaFilter = LinearFilter.movingAverage(6);
  private final LinearFilter notePixelsXFilter = LinearFilter.movingAverage(6);
  private final LinearFilter notePixelsYFilter = LinearFilter.movingAverage(6);

  public NoteVisionIOPhoton() {
    final NetworkTable table =
        defaultInstance.getTable("photonvision").getSubTable("Camera_Module_v1");
    noteThetaSub = table.getDoubleTopic("targetYaw").subscribe(0.0);
    notePitchSub = table.getDoubleTopic("targetPitch").subscribe(0.0);
    noteAreaSub = table.getDoubleTopic("targetArea").subscribe(0.0);
    notePixelsXSub = table.getDoubleTopic("targetPixelsX").subscribe(0.0);
    notePixelsYSub = table.getDoubleTopic("targetPixelsY").subscribe(0.0);
    hasTargetSub = table.getBooleanTopic("hasTarget").subscribe(false);
  }

  public void updateInputs(NoteVisionIOInputs inputs) {
    // Flip the sign to make it CCW positive
    inputs.noteTheta = noteThetaFilter.calculate(-Units.degreesToRadians(noteThetaSub.get()));
    inputs.notePitch = notePitchFilter.calculate(Units.degreesToRadians(notePitchSub.get()));
    inputs.noteArea = noteAreaFilter.calculate(Units.degreesToRadians(noteAreaSub.get()));
    inputs.noteRatio =
        notePixelsXFilter.calculate(notePixelsXSub.get())
            / notePixelsYFilter.calculate(notePixelsYSub.get());
    if (Double.isNaN(inputs.noteRatio)) {
      inputs.noteRatio = 0.0;
    }
    inputs.hasTarget = hasTargetSub.get();
  }
}
