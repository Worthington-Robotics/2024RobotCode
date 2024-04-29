// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.vision;

public interface NoteVisionIO {
  public static class NoteVisionIOInputs {
    public double noteTheta = 0.0;
    public double notePitch = 0.0;
    public double noteArea = 0.0;
    public double noteRatio = 0.0;
    public boolean hasTarget = false;
  }

  /**
   * Updates the inputs of the vision subsystem.
   *
   * @param inputs The inputs to be updated.
   */
  public default void updateInputs(NoteVisionIOInputs inputs) {}
}
