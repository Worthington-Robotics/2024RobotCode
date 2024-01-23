// Copyright (c) 2024 FRC 4145
// http://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.vision;

public interface VisionIO {
  public static class VisionIOInputs {
    public double[][] frames = new double[][] {};
    public double timestamps[] = new double[] {};
    public double fps = 0.0;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
