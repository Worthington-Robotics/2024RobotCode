// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.vision;

public interface VisionIO {
  /** The inputs from the Orange Pi's running our 2024 Vision code. */
  public static class VisionIOInputs {
    public double[][] frames = new double[][] {};
    public double timestamps[] = new double[] {};
    public double fps = 0.0;
    public boolean isConnected = false;
  }

  /**
   * Updates the inputs of the vision subsystem.
   *
   * @param inputs The inputs to be updated.
   */
  public default void updateInputs(VisionIOInputs inputs) {}
}
